/*
Main functionality of the UTME Weeble drone: main.cpp

Hardware:
  ESP32 dev board (PlatformIO)
  2 Brushless motors (coaxial, counter-rotating) with tachometers
  2-axis thrust gimbal (pitch + roll), each a servo driving a four-bar linkage
  IMU on I2C (SDA 21, SCL 22, address 0x68)

Control structure (three layers):
  PROTOCOLS     WHAT to do: an ordered list of Steps (flight state + parameters)
  STATES        HOW to do it: applyStep() sets the motors / gimbal for the current state
  VERIFICATION  DID it happen: verifyStep() checks the IMU and tachometers

  Every step:
    1. enter the step's flight state and act on it every tick
    2. after minTimeMs, start checking the verification condition
       (it must hold for kVerifyHoldMs in a row, so one noisy reading can't pass it)
    3. verified           -> keep running for holdMs, then go to the next step
       not verified by timeoutMs -> jump to the step's fallback state

  Safety checks (tilt limit, IMU failure, kill) sit ABOVE the protocols and can stop any of them.

  Protocols are listed in kProtocols (PROTOCOL REGISTRY section). Adding one = a make...()
  function, a build...() function, and one line in the table.

Serial commands (115200 baud, press Enter after each):
  unlock / lock             safety lock (only while idle in Rest)
  list                      show available protocols (from kProtocols)
  run <protocol> [args]     build and start a protocol, e.g. "run bench", "run hop 10 90"
  plan <protocol> [args]    build and print it without flying
    bench                   spin-up test - PROPS OFF
    hop <ft> [heading]      hop <ft> feet; heading in deg, 0 = forward (+pitch)
  abort                     end the protocol and land safely
  kill                      cut all motors immediately (any state)
  reset                     leave Damaged and go back to Rest
  status                    print one status line

!! Requires the corrected gimblecontrol.h and servo.h (GimbleAdj returns std::optional,
!! Servo takes an LEDC channel) and C++17 in platformio.ini.
!! Every number marked TODO is a placeholder. Test with props off, then on a tether.
*/
#include <vector>
#include <cmath>
#include <cstring>
#include <cstdlib>
#include <cctype>
#include <Arduino.h>
#include <espcontroller.h>
#include <imu.h>
#include <servo.h>
#include <gimblecontrol.h>

using FS = FlightState;   // short nickname: FS::Ascending instead of FlightState::Ascending

// ===================================================== CONFIG ==========================
namespace config {
  constexpr int      kSdaPin     = 21;
  constexpr int      kSclPin     = 22;
  constexpr uint32_t kI2CClockHz = 400000;   // 400 kHz (uint16_t only goes to 65535)

  // Pins / LEDC channels. On core 2.x, channels 0/1, 2/3... share a timer, so the
  // ESCs (16-bit) use 0/1 and the servos (14-bit) use 2/3.
  constexpr uint8_t kEscPinTop      = 18, kTachPinTop    = 4,  kEscChTop    = 0;
  constexpr uint8_t kEscPinBottom   = 19, kTachPinBottom = 16, kEscChBottom = 1;
  constexpr uint8_t kServoPitchPin  = 25, kServoPitchCh  = 2;
  constexpr uint8_t kServoRollPin   = 26, kServoRollCh   = 3;

  // Loop timing
  constexpr uint32_t kControlPeriodUs   = 5000;  // 200Hz attitude/state loop
  constexpr uint32_t kTelemetryPeriodMs = 100;   // 10Hz serial telemetry

  // Safety
  constexpr float   kMaxTiltDeg       = 45.0f;   // cut motors beyond this while flying
  constexpr uint8_t kMaxImuFailStreak = 10;      // Consecutive failed IMU reads --> fault
  constexpr float   kLaunchleveltoDeg = 5.0f;    // must sit this level to launch

  // Verification (assumes the IMU reports accel in g and rates in deg/s)
  constexpr uint32_t kVerifyHoldMs    = 150;     // a condition must hold this long to count
  constexpr float    kStillRateDps    = 3.0f;    // "stationary": all rotation rates below this
  constexpr float    kStillAccelTolG  = 0.10f;   //   ...and accel within 1 g +/- this
  constexpr float    kLiftoffAccelG   = 1.10f;   // climbing: thrust/weight reading above this
  constexpr float    kDescentAccelG   = 0.92f;   // descending: reading below this
  constexpr float    kAmpUpMaxTiltDeg = 10.0f;   // must not tip while spinning up
  constexpr float    kTouchdownAccelG = 1.6f;    // impact spike = touchdown

  // Flight profile (defaults - protocols can use different values per step)
  constexpr float kAscendRPM  = 7000.0f;         // Tune these based on drone weights and payload
  constexpr float kDescendRPM = 5000.0f;
  constexpr float kLandingRPM = 4500.0f;
  constexpr float kBenchRPM   = 2000.0f;         // bench test, props OFF
  constexpr float kAmpupRpmTolerance = 0.05f;    // within 5% of target
  constexpr float kStoppedRpm = 300.0f;

  // Step timing defaults (all measured from the moment the step starts)
  constexpr uint32_t kLaunchSettleMs   = 500;    // let the attitude re-seed before checking
  constexpr uint32_t kLaunchTimeoutMs  = 3000;
  constexpr uint32_t kAmpUpRampMs      = 2000;
  constexpr uint32_t kAmpUpTimeoutMs   = 5000;
  constexpr uint32_t kLiftoffTimeoutMs = 2000;   // Ascending must see liftoff within this
  constexpr uint32_t kDescentTimeoutMs = 2000;
  constexpr uint32_t kLandingMinMs     = 300;    // ignore the first moments of Landing
  constexpr uint32_t kLandingTimeoutMs = 5000;
  constexpr uint32_t kAmpDownRampMs    = 1500;
  constexpr uint32_t kAmpDownTimeoutMs = 4000;
  constexpr uint32_t kRestTimeoutMs    = 5000;   // a Weeble wobbles - give it time to settle

  // Hop model - TODO: measure all of these
  constexpr float kMassKg           = 1.2f;      // TODO: weigh the drone with battery
  constexpr float kThrustCoeff      = 1.5e-7f;   // TODO: N per RPM^2 per motor (thrust stand)
  constexpr float kIdleFraction     = 0.6f;      // AmpUp spins to 60% of hover RPM (stays grounded)
  constexpr float kHopLeanDeg       = 10.0f;     // lean angle while climbing
  constexpr float kMaxLeanDeg       = 20.0f;
  constexpr float kLeanRateDps      = 30.0f;     // how fast the lean target may change
  constexpr float kMaxHopFt         = 30.0f;
  constexpr float kMaxHopHeightM    = 3.0f;      // refuse any plan that peaks higher
  constexpr float kMaxBurnS         = 4.0f;
  constexpr float kHopDistanceScale = 1.0f;      // tune from test hops (see planHop)
  constexpr float kDescendFraction  = 0.7f;      // share of predicted descent spent in Descending

  // Gimbal
  // BodyLean:     the protocol sets a target lean; the PID uses the gimbal to hold it.
  // GimbalOffset: the PID holds the body level; the lean is added straight to the gimbal.
  enum class TiltMode { BodyLean, GimbalOffset };
  constexpr TiltMode kTiltMode = TiltMode::BodyLean;   // CHANGE depending on props vs CG
  constexpr float kMaxGinbalTiltDeg = 15.0f;           // Tommy said this is the max tilt of the gimbal in either direction
  constexpr float kTh2InitialDeg    = 25.0f;           // CHANGE once the gimbal design is finalized
  // {L1, L2, L3, G}. TODO: G (the 4th value) is a placeholder - the solver needs all 4
  const std::vector<float> LinkageLengths = {2.5f, 3.0f, 4.1f, 2.0f};

  // Yaw (differential RPM between the coaxial props)
  constexpr float kYawRateGain   = 2.0f;         // RPM per deg/s of yaw rate
  constexpr float kMaxYawDiffRpm = 500.0f;

  // =============================================== HARDWARE ===========================================================
  // ki/kd have no defaults in espcontroller.h, so every field is set here.
  static EscController::Config EscConfig(uint8_t escPin, uint8_t tachPin, uint8_t ledcChannel) {
    EscController::Config c{};
    c.escPin       = escPin;
    c.tachPin      = tachPin;
    c.pulsesPerRev = 2;           // TODO: blades (optical) or pole pairs (ESC rpm wire)
    c.motorKv      = 920.0f;      // TODO: from the motor spec
    c.battVolt     = 11.1f;       // FULL PACK voltage. Check this logic, there should be a way to monitor the battery voltage
    c.maxRpm       = 9000.0f;     // TODO
    c.ledcChannel  = ledcChannel;
    c.pwnFreq      = 50;
    c.controlPeriodMs       = 5;
    c.stallTimeoutMs        = 200;
    c.maxThrottleslewpersec = 2.0f;
    c.kp = 0.00005f;
    c.ki = 0.00005f;
    c.kd = 0.000001f;
    return c;
  }
}  // namespace config

// ===================================================== HARDWARE OBJECTS ================
EscController escs[] = {
  EscController(config::EscConfig(config::kEscPinTop,    config::kTachPinTop,    config::kEscChTop)),
  EscController(config::EscConfig(config::kEscPinBottom, config::kTachPinBottom, config::kEscChBottom)),
};
constexpr size_t kNumMotors = sizeof(escs) / sizeof(escs[0]);

IMUdefine imu(Wire, 0x68);

Servo pitchServo(config::kServoPitchPin, 500.0f, 2500.0f, PI, config::kServoPitchCh);
Servo rollServo (config::kServoRollPin,  500.0f, 2500.0f, PI, config::kServoRollCh);

// Simple PID; the derivative uses the gyro rate directly (no derivative kick).
struct Pid {
  float kp, ki, kd;
  float iLimit;     // integral clamp (deg*s)
  float outLimit;   // output clamp (deg of gimbal tilt)
  float integral = 0;

  void  reset() { integral = 0; }
  float step(float error, float rate, float dt) {
    integral = constrain(integral + error * dt, -iLimit, iLimit);
    return constrain(kp * error + ki * integral - kd * rate, -outLimit, outLimit);
  }
};

struct GimbalAxis {
  const char*        name;
  Servo*             servo;
  std::vector<float> links;          // {L1, L2, L3, G}
  float              direction;      // +1 / -1: flip if the servo moves the wrong way
  float              servoCenterRad; // shaft angle when the gimbal sits at home (45 deg)
  Pid                pid;
  float              th2HomeDeg = 0; // solved at boot
  float              th2Deg     = 0; // last solved servo-arm angle
  uint32_t           solveFails = 0;
};

GimbalAxis pitchAxis{"pitch", &pitchServo, config::LinkageLengths, +1.0f, PI / 2,
                     {0.8f, 0.2f, 0.10f, 10.0f, config::kMaxGinbalTiltDeg}};
GimbalAxis rollAxis {"roll",  &rollServo,  config::LinkageLengths, +1.0f, PI / 2,
                     {0.8f, 0.2f, 0.10f, 10.0f, config::kMaxGinbalTiltDeg}};

// ===================================================== PROTOCOL TYPES ==================
// One step of a protocol.
//   minTimeMs: verification isn't checked before this (settling / ramp time)
//   timeoutMs: must be verified before this, or the step falls back
//   holdMs:    keep running this step this long AFTER it is verified
struct Step {
  FlightState state      = FS::Rest;
  float       rpm        = 0;   // motor target for this step
  float       leanDeg    = 0;   // lean (or gimbal offset) during this step
  float       headingDeg = 0;   // direction of the lean, 0 = forward (+pitch)
  uint32_t    minTimeMs  = 0;
  uint32_t    holdMs     = 0;
  uint32_t    timeoutMs  = 0;
  FlightState fallback   = FS::Rest;
};

// A protocol is a fixed-size list of steps (no memory allocation while flying).
constexpr size_t kMaxSteps = 12;
struct Protocol {
  char    name[16]  = "";
  char    info[128] = "";   // optional summary a builder can fill in (printed by run / plan)
  Step    steps[kMaxSteps];
  uint8_t count = 0;

  explicit Protocol(const char* n = "") {
    strncpy(name, n, sizeof(name) - 1);
    name[sizeof(name) - 1] = '\0';
  }
  bool add(const Step& s) {
    if (count >= kMaxSteps) return false;
    steps[count++] = s;
    return true;
  }
};

// What happened to each step (printed when the protocol ends, for tuning)
struct StepResult {
  bool     ran        = false;
  bool     verified   = false;
  bool     fellBack   = false;
  bool     aborted    = false;
  uint32_t verifyAtMs = 0;   // time from step start to verification
  uint32_t durationMs = 0;
};

// Walks through the active protocol
struct Runner {
  Protocol   proto;
  bool       active       = false;
  uint8_t    index        = 0;
  uint32_t   protoStartMs = 0;
  uint32_t   stepStartMs  = 0;
  bool       condHolding  = false;   // verification condition currently true?
  uint32_t   condSinceMs  = 0;
  bool       verified     = false;
  uint32_t   verifiedAtMs = 0;
  float      stepStartRpm = 0;       // RPM when the step began (AmpDown ramps from here)
  StepResult results[kMaxSteps];

  const Step& step() const { return proto.steps[index]; }
};

// ===================================================== STATE ===========================
FlightState state  = FS::Rest;
Armed       safety = Armed::SafetyLockEngaged;
Runner      runner;

uint8_t     imuFailStreak = 0;
uint32_t    lastControlUs = 0;
float       baseRpm       = 0;
StateValues att;

// Lean: the step sets a desired value; the current value moves toward it smoothly
float desiredPitchLean = 0, desiredRollLean = 0;
float pitchLean = 0, rollLean = 0;

// ===================================================== HELPERS =========================
const char* stateName(FlightState s) {
  switch (s) {
    case FS::Rest:       return "Rest";
    case FS::Launch:     return "Launch";
    case FS::AmpUp:      return "AmpUp";
    case FS::Ascending:  return "Ascending";
    case FS::Descending: return "Descending";
    case FS::Landing:    return "Landing";
    case FS::AmpDown:    return "AmpDown";
    case FS::Damaged:    return "Damaged";
    case FS::Recharge:   return "Recharge";
  }
  return "?";
}

bool isUnlocked() { return safety == Armed::NoSafetyLockEngaged; }

bool inFlight() {   // motors may be spinning
  return state == FS::AmpUp || state == FS::Ascending || state == FS::Descending ||
         state == FS::Landing || state == FS::AmpDown;
}

bool gimbalActive() {   // gimbal stabilizes attitude
  return state == FS::Ascending || state == FS::Descending || state == FS::Landing;
}

void enterState(FlightState next, const char* reason) {
  Serial.printf("[STATE] %s -> %s (%s)\n", stateName(state), stateName(next), reason);
  if (next == FS::Rest || next == FS::Damaged) safety = Armed::SafetyLockEngaged;
  state = next;
}

// ---------------- sensor checks ----------------
float accelMagnitudeG() {
  const RawIMUData raw = imu.getRaw();
  return std::sqrt(raw.ax * raw.ax + raw.ay * raw.ay + raw.az * raw.az);
}

bool isLevel(float tolDeg) {
  return std::fabs(att.roll) < tolDeg && std::fabs(att.pitch) < tolDeg;
}

bool isStationary() {
  return std::fabs(att.rollrate)  < config::kStillRateDps &&
         std::fabs(att.pitchrate) < config::kStillRateDps &&
         std::fabs(att.yawrate)   < config::kStillRateDps &&
         std::fabs(accelMagnitudeG() - 1.0f) < config::kStillAccelTolG;
}

// ---------------- motors ----------------
void stopAllMotors() {
  for (auto& e : escs) e.stop();
  baseRpm = 0;
}

// yawDiff is added to even-index motors and subtracted from odd-index motors.
void setMotorRpm(float rpm, float yawDiff = 0) {
  if (!isUnlocked()) {          // hard interlock: locked means no thrust, ever
    stopAllMotors();
    return;
  }
  baseRpm = rpm;
  for (size_t i = 0; i < kNumMotors; ++i) {
    const float sign = (i % 2 == 0) ? 1.0f : -1.0f;
    escs[i].setTargetRPM(rpm + sign * yawDiff);
  }
}

bool allMotorsNear(float target, float tol) {
  for (auto& e : escs) {
    if (std::fabs(e.rpm() - target) > target * tol) return false;
  }
  return true;
}

bool allMotorsStopped() {
  for (auto& e : escs) {
    if (e.rpm() > config::kStoppedRpm) return false;
  }
  return true;
}

float yawDiffRpm() {
  if (kNumMotors < 2) return 0;
  return constrain(-config::kYawRateGain * att.yawrate,
                   -config::kMaxYawDiffRpm, config::kMaxYawDiffRpm);
}

// ---------------- gimbal ----------------
bool driveGimbalAxis(GimbalAxis& ax, float tiltDeg) {
  tiltDeg = constrain(tiltDeg, -config::kMaxGinbalTiltDeg, config::kMaxGinbalTiltDeg);
  auto th2 = GimbleAdj(ax.links, ax.th2Deg, gimbal::kGimbalHomeDeg + tiltDeg);
  if (!th2) {                   // unreachable: hold the last position
    ax.solveFails++;
    return false;
  }
  ax.th2Deg = *th2;
  const float delta = gimbal::wrapToPi((ax.th2Deg - ax.th2HomeDeg) * gimbal::kDegToRad);
  ax.servo->setAngle(ax.servoCenterRad + ax.direction * delta);
  return true;
}

bool initGimbalAxis(GimbalAxis& ax) {
  if (!ax.servo->begin()) return false;
  auto home = GimbleAdj(ax.links, config::kTh2InitialDeg, gimbal::kGimbalHomeDeg);
  if (!home) return false;
  ax.th2HomeDeg = ax.th2Deg = *home;
  ax.servo->setAngle(ax.servoCenterRad);
  Serial.printf("  %s gimbal home: theta2 = %.2f deg\n", ax.name, ax.th2HomeDeg);
  return true;
}

void centerGimbal() {
  pitchAxis.pid.reset();
  rollAxis.pid.reset();
  pitchLean = rollLean = 0;
  driveGimbalAxis(pitchAxis, 0);
  driveGimbalAxis(rollAxis, 0);
}

// Move the current lean toward the desired lean at a limited rate (no sudden jerks)
void updateLean(float dt) {
  const float maxStep = config::kLeanRateDps * dt;
  pitchLean += constrain(desiredPitchLean - pitchLean, -maxStep, maxStep);
  rollLean  += constrain(desiredRollLean  - rollLean,  -maxStep, maxStep);
}

void runGimbal(float dt) {
  // BodyLean: the PID's target is the lean angle.
  // GimbalOffset: the PID targets level and the lean is added on top of its output.
  const bool bodyLean = (config::kTiltMode == config::TiltMode::BodyLean);
  const float pitchTarget = bodyLean ? pitchLean : 0.0f;
  const float rollTarget  = bodyLean ? rollLean  : 0.0f;
  const float pitchOffset = bodyLean ? 0.0f : pitchLean;
  const float rollOffset  = bodyLean ? 0.0f : rollLean;

  // Flip axis.direction if a correction makes the tilt worse.
  const float pitchCmd = pitchAxis.pid.step(pitchTarget - att.pitch, att.pitchrate, dt);
  const float rollCmd  = rollAxis.pid.step (rollTarget  - att.roll,  att.rollrate,  dt);
  driveGimbalAxis(pitchAxis, pitchCmd + pitchOffset);
  driveGimbalAxis(rollAxis,  rollCmd  + rollOffset);
}

// ---------------- safety ----------------
void emergencyStop(const char* reason) {
  runner.active = false;        // cancel any protocol
  stopAllMotors();
  desiredPitchLean = desiredRollLean = 0;
  centerGimbal();
  if (state != FS::Damaged) enterState(FS::Damaged, reason);
}

[[noreturn]] void fatal(const char* msg) {
  stopAllMotors();
  while (true) {
    Serial.printf("[FATAL] %s\n", msg);
    delay(1000);
  }
}

// ===================================================== STEP BUILDERS ===================
// Each sets the default timing and the fallback from the table in imu.h.
// Field order: state, rpm, lean, heading, minTimeMs, holdMs, timeoutMs, fallback
Step stepLaunch() {
  return {FS::Launch, 0, 0, 0, config::kLaunchSettleMs, 0, config::kLaunchTimeoutMs, FS::Rest};
}
Step stepAmpUp(float rpm, uint32_t holdMs) {
  return {FS::AmpUp, rpm, 0, 0, config::kAmpUpRampMs, holdMs, config::kAmpUpTimeoutMs, FS::AmpDown};
}
Step stepAscend(float rpm, float leanDeg, float headingDeg, uint32_t holdMs) {
  return {FS::Ascending, rpm, leanDeg, headingDeg, 0, holdMs, config::kLiftoffTimeoutMs, FS::Descending};
}
Step stepDescend(float rpm, uint32_t holdMs) {
  return {FS::Descending, rpm, 0, 0, 0, holdMs, config::kDescentTimeoutMs, FS::Landing};
}
Step stepLanding(float rpm) {
  return {FS::Landing, rpm, 0, 0, config::kLandingMinMs, 0, config::kLandingTimeoutMs, FS::AmpDown};
}
Step stepAmpDown() {
  return {FS::AmpDown, 0, 0, 0, config::kAmpDownRampMs, 0, config::kAmpDownTimeoutMs, FS::Rest};
}
Step stepRest() {
  return {FS::Rest, 0, 0, 0, 0, 0, config::kRestTimeoutMs, FS::Damaged};
}

// ===================================================== PROTOCOLS =======================
// Bench test (PROPS OFF): spin up, hold 5 s, spin down. Checks motors, tachs and the runner.
Protocol makeBenchProtocol() {
  Protocol p("bench");
  p.add(stepLaunch());
  p.add(stepAmpUp(config::kBenchRPM, 5000));
  p.add(stepAmpDown());
  p.add(stepRest());
  return p;
}

// ---------------- hop model ----------------
// Open-loop physics: no position sensor, so the IMU confirms each phase HAPPENED,
// not where the drone ended up. Tune kHopDistanceScale from real test hops:
//   new scale = old scale * (commanded distance / measured distance)
struct HopPlan {
  float hoverRpm = 0, idleRpm = 0;
  float burnS = 0, descentS = 0;
  float peakM = 0, distM = 0;
};

float totalThrustN(float rpm) { return kNumMotors * config::kThrustCoeff * rpm * rpm; }

// Phase 1 (Ascending, burnS): thrust at the lean angle -> climbs and accelerates sideways.
// Phase 2 (Descending/Landing): level, less thrust -> falls back while coasting sideways.
bool simulateHop(float burnS, HopPlan& plan) {
  const float g    = 9.81f;
  const float lean = config::kHopLeanDeg * DEG_TO_RAD;
  const float a1   = totalThrustN(config::kAscendRPM) / config::kMassKg;
  const float aH   = a1 * std::sin(lean);                          // sideways accel
  const float aV1  = a1 * std::cos(lean) - g;                      // climb accel
  const float aV2  = totalThrustN(config::kDescendRPM) / config::kMassKg - g;   // descent accel (negative)
  if (aV1 <= 0.0f || aV2 >= 0.0f) return false;

  const float h1 = 0.5f * aV1 * burnS * burnS;    // height at end of burn
  const float v1 = aV1 * burnS;                   // climb speed at end of burn
  // Time to fall back to the ground: h1 + v1*t + 0.5*aV2*t^2 = 0
  const float a = 0.5f * aV2;
  plan.descentS = (-v1 - std::sqrt(v1 * v1 - 4.0f * a * h1)) / (2.0f * a);
  plan.peakM    = h1 + v1 * v1 / (2.0f * -aV2);
  plan.distM    = 0.5f * aH * burnS * burnS + aH * burnS * plan.descentS;
  plan.burnS    = burnS;
  return true;
}

bool planHop(float distFt, HopPlan& plan, const char*& error) {
  const float g = 9.81f;
  plan.hoverRpm = std::sqrt(config::kMassKg * g / (kNumMotors * config::kThrustCoeff));
  plan.idleRpm  = config::kIdleFraction * plan.hoverRpm;

  if (config::kHopLeanDeg > config::kMaxLeanDeg) { error = "kHopLeanDeg above kMaxLeanDeg"; return false; }

  HopPlan longest;
  if (!simulateHop(config::kMaxBurnS, longest)) {
    error = "model says climb thrust can't lift the drone, or descent thrust can't let it fall";
    return false;
  }
  const float targetM = distFt * 0.3048f * config::kHopDistanceScale;
  if (longest.distM < targetM) { error = "too far for kMaxBurnS"; return false; }

  // Distance grows with burn time, so bisect for the burn time that hits the target
  float lo = 0.0f, hi = config::kMaxBurnS;
  for (int i = 0; i < 40; ++i) {
    const float mid = 0.5f * (lo + hi);
    simulateHop(mid, plan);
    if (plan.distM < targetM) lo = mid; else hi = mid;
  }
  simulateHop(hi, plan);

  if (plan.peakM > config::kMaxHopHeightM) { error = "predicted peak above kMaxHopHeightM"; return false; }
  return true;
}

// Rest -> Launch -> AmpUp -> Ascending(lean) -> Descending -> Landing -> AmpDown -> Rest
bool makeHopProtocol(float distFt, float headingDeg, Protocol& out, HopPlan& plan, const char*& error) {
  if (!planHop(distFt, plan, error)) return false;
  out = Protocol("hop");
  out.add(stepLaunch());
  out.add(stepAmpUp(plan.idleRpm, 500));          // spin up but stay on the ground
  out.add(stepAscend(config::kAscendRPM, config::kHopLeanDeg, headingDeg,
                     static_cast<uint32_t>(plan.burnS * 1000.0f)));
  out.add(stepDescend(config::kDescendRPM,
                      static_cast<uint32_t>(plan.descentS * config::kDescendFraction * 1000.0f)));
  out.add(stepLanding(config::kLandingRPM));
  out.add(stepAmpDown());
  out.add(stepRest());
  snprintf(out.info, sizeof(out.info),
           "hover %.0f rpm, idle %.0f rpm, burn %.2f s, descent %.2f s, peak %.2f m, distance %.2f m",
           plan.hoverRpm, plan.idleRpm, plan.burnS, plan.descentS, plan.peakM, plan.distM);
  return true;
}

// Safe way down from wherever we are (used by "abort" and missing fallbacks)
Protocol makeAbortProtocol(FlightState from) {
  Protocol p("abort");
  switch (from) {
    case FS::Ascending:
    case FS::Descending:
      p.add(stepDescend(config::kDescendRPM, 1000));
      p.add(stepLanding(config::kLandingRPM));
      p.add(stepAmpDown());
      p.add(stepRest());
      break;
    case FS::Landing:
      p.add(stepLanding(config::kLandingRPM));
      p.add(stepAmpDown());
      p.add(stepRest());
      break;
    case FS::AmpUp:
    case FS::AmpDown:
      p.add(stepAmpDown());
      p.add(stepRest());
      break;
    default:          // Rest / Launch / Damaged: nothing is spinning
      break;
  }
  return p;
}

void printProtocol(const Protocol& p) {
  Serial.printf("Protocol '%s' (%u steps)\n", p.name, (unsigned)p.count);
  if (p.info[0] != '\0') Serial.printf("  %s\n", p.info);
  for (uint8_t i = 0; i < p.count; ++i) {
    const Step& s = p.steps[i];
    Serial.printf("  %u. %-10s rpm %5.0f lean %4.1f@%3.0f  min %4lu hold %4lu timeout %4lu -> fallback %s\n",
                  (unsigned)(i + 1), stateName(s.state), s.rpm, s.leanDeg, s.headingDeg,
                  (unsigned long)s.minTimeMs, (unsigned long)s.holdMs, (unsigned long)s.timeoutMs,
                  stateName(s.fallback));
  }
}

// ===================================================== PROTOCOL REGISTRY ===============
// Every runnable protocol has a builder with the SAME signature:
//   argc / argv : the words typed after the protocol name ("run hop 10 90" -> {"10", "90"})
//   out         : the protocol to fill in
//   error       : set this to a message and return false to reject the request
// Builders only describe a flight: no motors, IMU, servos or Serial in here.
using BuildFn = bool (*)(int argc, char** argv, Protocol& out, const char*& error);

struct ProtocolEntry {
  const char* name;          // what you type after "run" / "plan"
  const char* usage;         // shown by "list" and "help"
  const char* description;
  BuildFn     build;
};

// ---- builders (argument parsing + validation, then call the make...() function) ----
bool buildBench(int argc, char** argv, Protocol& out, const char*& error) {
  (void)argv;
  if (argc != 0) { error = "bench takes no arguments"; return false; }
  out = makeBenchProtocol();
  return true;
}

bool buildHop(int argc, char** argv, Protocol& out, const char*& error) {
  if (argc < 1 || argc > 2) { error = "usage: hop <ft> [heading]"; return false; }
  const float distFt  = atof(argv[0]);
  const float heading = (argc == 2) ? atof(argv[1]) : 0.0f;
  if (distFt <= 0.0f || distFt > config::kMaxHopFt) {
    error = "hop distance must be above 0 and at most kMaxHopFt";
    return false;
  }
  HopPlan plan;
  return makeHopProtocol(distFt, heading, out, plan, error);
}

// ---- the table: add one line per protocol ----
const ProtocolEntry kProtocols[] = {
  {"bench", "bench",              "spin-up test, PROPS OFF",               buildBench},
  {"hop",   "hop <ft> [heading]", "hop <ft> feet; heading 0 = forward",    buildHop},
  // {"name", "name <args>",     "what it does",                       buildName},
};
constexpr size_t kNumProtocols = sizeof(kProtocols) / sizeof(kProtocols[0]);

/* Adding a protocol:
     1. write  Protocol makeNameProtocol(...)  using the step builders (stepLaunch(), ...)
     2. write  bool buildName(int argc, char** argv, Protocol& out, const char*& error)
        that checks the arguments and calls makeNameProtocol()
     3. add one line to kProtocols
   The runner, verification, safety checks and serial commands need no changes. */

const ProtocolEntry* findProtocol(const char* name) {
  for (const auto& entry : kProtocols) {
    if (!strcmp(entry.name, name)) return &entry;
  }
  return nullptr;
}

// ===================================================== STATES (act) ====================
// HOW each state behaves. Called every tick for the current step.
void applyStep(const Step& s, uint32_t t) {
  desiredPitchLean = desiredRollLean = 0;

  switch (s.state) {
    case FS::Rest:
    case FS::Launch:
    case FS::Damaged:
    case FS::Recharge:
      stopAllMotors();
      break;

    case FS::AmpUp: {          // ramp from 0 to the step's RPM; gimbal stays centered
      const float frac = constrain(t / float(config::kAmpUpRampMs), 0.0f, 1.0f);
      setMotorRpm(s.rpm * frac);
      break;
    }

    case FS::Ascending:
    case FS::Descending:
    case FS::Landing: {
      setMotorRpm(s.rpm, yawDiffRpm());
      // The lean only starts once liftoff is confirmed, so it can't drag on the ground
      const bool airborne = (s.state != FS::Ascending) || runner.verified;
      if (airborne) {
        const float lean = constrain(s.leanDeg, -config::kMaxLeanDeg, config::kMaxLeanDeg);
        const float hdg  = s.headingDeg * DEG_TO_RAD;
        desiredPitchLean = lean * std::cos(hdg);
        desiredRollLean  = lean * std::sin(hdg);
      }
      break;
    }

    case FS::AmpDown: {        // ramp from the RPM we entered with down to zero
      const float frac = constrain(t / float(config::kAmpDownRampMs), 0.0f, 1.0f);
      if (frac < 1.0f) setMotorRpm(runner.stepStartRpm * (1.0f - frac));
      else             stopAllMotors();
      break;
    }
  }
}

// ===================================================== VERIFICATION ====================
// DID the state happen? 'instant' = a one-off event (an impact spike) that passes
// immediately instead of having to hold for kVerifyHoldMs.
bool verifyStep(const Step& s, bool& instant) {
  instant = false;
  const float accelG = accelMagnitudeG();

  switch (s.state) {
    case FS::Rest:        // still, motors stopped
      return isStationary() && allMotorsStopped();

    case FS::Launch:      // level, still, IMU healthy, safety off
      return isUnlocked() && imuFailStreak == 0 &&
             isLevel(config::kLaunchleveltoDeg) && isStationary();

    case FS::AmpUp:       // at target RPM and not tipping
      return allMotorsNear(s.rpm, config::kAmpupRpmTolerance) &&
             isLevel(config::kAmpUpMaxTiltDeg);

    case FS::Ascending:   // liftoff: thrust > weight, so accel reads above 1 g
      return accelG > config::kLiftoffAccelG;

    case FS::Descending:  // thrust < weight, so accel reads below 1 g
      return accelG < config::kDescentAccelG;

    case FS::Landing:     // impact spike, or sitting still on the ground
      if (accelG > config::kTouchdownAccelG) { instant = true; return true; }
      return isStationary();

    case FS::AmpDown:     // props stopped
      return allMotorsStopped();

    default:
      return false;
  }
}

// ===================================================== PROTOCOL RUNNER =================
void beginStep(uint8_t i, const char* reason) {
  runner.index        = i;
  runner.stepStartMs  = millis();
  runner.condHolding  = false;
  runner.verified     = false;
  runner.stepStartRpm = baseRpm;
  runner.results[i]   = StepResult{};
  runner.results[i].ran = true;

  const Step& s = runner.step();
  if (s.state == FS::Launch) imu.resetAttitude();
  Serial.printf("[STEP %u/%u] %s\n", (unsigned)(i + 1), (unsigned)runner.proto.count, stateName(s.state));
  enterState(s.state, reason);
}

void printSummary() {
  Serial.printf("---- '%s' summary (%.1f s) ----\n", runner.proto.name,
                (millis() - runner.protoStartMs) / 1000.0f);
  for (uint8_t i = 0; i < runner.proto.count; ++i) {
    const StepResult& r = runner.results[i];
    const char* name = stateName(runner.proto.steps[i].state);
    if (!r.ran)           Serial.printf("  %u. %-10s skipped\n", (unsigned)(i + 1), name);
    else if (r.aborted)   Serial.printf("  %u. %-10s ABORTED after %lu ms\n",
                                        (unsigned)(i + 1), name, (unsigned long)r.durationMs);
    else if (r.fellBack)  Serial.printf("  %u. %-10s NOT verified -> fallback after %lu ms\n",
                                        (unsigned)(i + 1), name, (unsigned long)r.durationMs);
    else                  Serial.printf("  %u. %-10s verified at %lu ms, ran %lu ms\n",
                                        (unsigned)(i + 1), name, (unsigned long)r.verifyAtMs,
                                        (unsigned long)r.durationMs);
  }
}

void finishProtocol(const char* reason) {
  stopAllMotors();
  printSummary();
  runner.active = false;
  if (state != FS::Rest) enterState(FS::Rest, reason);
  else Serial.printf("[PROTOCOL] done (%s)\n", reason);
  safety = Armed::SafetyLockEngaged;
}

// Start a protocol. 'force' skips the Rest/unlock checks (used by abort mid-flight).
bool startProtocol(const Protocol& p, bool force = false) {
  if (!force && (state != FS::Rest || runner.active)) {
    Serial.println("[PROTOCOL] can only start from Rest with no protocol running");
    return false;
  }
  if (!force && !isUnlocked()) {
    Serial.println("[PROTOCOL] send 'unlock' first");
    return false;
  }
  if (p.count == 0) return false;

  runner.proto        = p;
  runner.active       = true;
  runner.protoStartMs = millis();
  for (auto& r : runner.results) r = StepResult{};
  Serial.printf("[PROTOCOL] starting '%s'\n", p.name);
  beginStep(0, p.name);
  return true;
}

void abortProtocol(const char* reason) {
  Serial.printf("[PROTOCOL] abort: %s\n", reason);
  const Protocol p = makeAbortProtocol(state);
  if (p.count == 0) {           // nothing spinning: just stop
    if (runner.active) {
      runner.results[runner.index].aborted    = true;
      runner.results[runner.index].durationMs = millis() - runner.stepStartMs;
      finishProtocol(reason);
    }
    return;
  }
  if (runner.active) {
    StepResult& r = runner.results[runner.index];
    r.aborted    = true;
    r.durationMs = millis() - runner.stepStartMs;
    printSummary();
  }
  startProtocol(p, /*force=*/true);
}

void stepFallback() {
  const Step& s = runner.step();
  StepResult& r = runner.results[runner.index];
  r.fellBack   = true;
  r.durationMs = millis() - runner.stepStartMs;
  Serial.printf("[VERIFY] %s NOT confirmed within %lu ms -> fallback %s\n",
                stateName(s.state), (unsigned long)s.timeoutMs, stateName(s.fallback));

  if (s.fallback == FS::Damaged) {
    emergencyStop("verification failed");
    return;
  }
  // Jump forward to the next step that uses the fallback state
  for (uint8_t j = runner.index + 1; j < runner.proto.count; ++j) {
    if (runner.proto.steps[j].state == s.fallback) {
      beginStep(j, "fallback");
      return;
    }
  }
  // Protocol has no such step: use the safe landing sequence instead
  abortProtocol("no fallback step in protocol");
}

void advanceStep() {
  StepResult& r = runner.results[runner.index];
  r.durationMs = millis() - runner.stepStartMs;
  if (runner.index + 1 >= runner.proto.count) {
    finishProtocol("protocol complete");
  } else {
    beginStep(runner.index + 1, "step complete");
  }
}

// One tick of the active protocol: act, verify, decide
void runProtocol() {
  if (!runner.active) return;
  const Step&    s   = runner.step();
  const uint32_t now = millis();
  const uint32_t t   = now - runner.stepStartMs;

  // 1. Act
  applyStep(s, t);

  // 2. Verify (only after minTimeMs, and the condition must hold for kVerifyHoldMs)
  if (!runner.verified && t >= s.minTimeMs) {
    bool instant = false;
    const bool cond = verifyStep(s, instant);
    bool confirmed = false;
    if (cond && instant) {
      confirmed = true;
    } else if (cond) {
      if (!runner.condHolding) { runner.condHolding = true; runner.condSinceMs = now; }
      confirmed = (now - runner.condSinceMs >= config::kVerifyHoldMs);
    } else {
      runner.condHolding = false;
    }
    if (confirmed) {
      runner.verified     = true;
      runner.verifiedAtMs = now;
      runner.results[runner.index].verified   = true;
      runner.results[runner.index].verifyAtMs = t;
      Serial.printf("[VERIFY] %s confirmed at %lu ms\n", stateName(s.state), (unsigned long)t);
    }
  }

  // 3. Decide
  if (runner.verified) {
    if (now - runner.verifiedAtMs >= s.holdMs) advanceStep();
  } else if (t >= s.timeoutMs) {
    stepFallback();
  }
}

// ===================================================== CONTROL STEP ====================
void controlStep(float dt) {
  if (imu.update()) {
    imuFailStreak = 0;
    att = imu.getAttitude();
  } else if (imuFailStreak < 255) {
    imuFailStreak++;
  }

  // Safety layer - above every protocol
  if (imuFailStreak >= config::kMaxImuFailStreak && state != FS::Damaged) {
    emergencyStop("IMU not responding");
  } else if (inFlight() && !isLevel(config::kMaxTiltDeg)) {
    emergencyStop("tilt limit exceeded");
  }

  if (runner.active) {
    runProtocol();
  } else {
    stopAllMotors();            // idle in Rest or Damaged
    desiredPitchLean = desiredRollLean = 0;
  }

  if (gimbalActive()) {
    updateLean(dt);
    runGimbal(dt);
  } else {
    centerGimbal();
  }
}

// ===================================================== SERIAL ==========================
void printStatus() {
  Serial.printf("%-10s %s", stateName(state), isUnlocked() ? "LIVE" : "LOCK");
  if (runner.active) {
    Serial.printf(" | %s %u/%u %s", runner.proto.name, (unsigned)(runner.index + 1),
                  (unsigned)runner.proto.count, runner.verified ? "OK" : "...");
  }
  Serial.printf(" | roll %6.1f pitch %6.1f lean %5.1f/%5.1f | rpm",
                att.roll, att.pitch, pitchLean, rollLean);
  for (auto& e : escs) Serial.printf(" %5.0f/%5.0f", e.rpm(), e.targetRPM());
  Serial.printf(" | %.2fg\n", accelMagnitudeG());
}

void printTelemetry() {
  static uint32_t last = 0;
  if (millis() - last < config::kTelemetryPeriodMs) return;
  last = millis();
  printStatus();
}

void printHelp() {
  Serial.println("commands: unlock lock list status abort kill reset");
  Serial.println("          run <protocol> [args]   plan <protocol> [args] (preview, no flight)");
  for (const auto& entry : kProtocols) Serial.printf("  protocol: %s\n", entry.usage);
}

void printProtocolList() {
  Serial.printf("%u protocols:\n", (unsigned)kNumProtocols);
  for (const auto& entry : kProtocols) {
    Serial.printf("  %-22s %s\n", entry.usage, entry.description);
  }
}

// "run <name> [args]" or "plan <name> [args]": look the name up in kProtocols,
// build the protocol, print it, and (for run) start it.
void runOrPlan(bool run, int argc, char** argv) {
  if (argc < 1) { printHelp(); return; }
  const ProtocolEntry* entry = findProtocol(argv[0]);
  if (!entry) {
    Serial.printf("unknown protocol '%s' - type 'list'\n", argv[0]);
    return;
  }
  Protocol p;
  const char* error = "unknown error";
  if (!entry->build(argc - 1, argv + 1, p, error)) {
    Serial.printf("[PLAN] %s rejected: %s\n", entry->name, error);
    return;
  }
  printProtocol(p);
  if (run) startProtocol(p);
}

void handleCommand(char* line) {
  // Split the line into words, like line.split() in Python
  constexpr int kMaxArgs = 8;
  char* argv[kMaxArgs];
  int   argc = 0;
  for (char* tok = strtok(line, " "); tok && argc < kMaxArgs; tok = strtok(nullptr, " ")) {
    argv[argc++] = tok;
  }
  if (argc == 0) return;
  const char* cmd = argv[0];

  if (!strcmp(cmd, "kill")) {
    emergencyStop("kill command");
  } else if (!strcmp(cmd, "abort")) {
    if (runner.active) abortProtocol("abort command");
    else Serial.println("no protocol running");
  } else if (!strcmp(cmd, "status")) {
    printStatus();
  } else if (!strcmp(cmd, "unlock")) {
    if (state == FS::Rest && !runner.active) {
      safety = Armed::NoSafetyLockEngaged;
      Serial.println("[SAFETY] lock DISENGAGED - motors can spin");
    } else {
      Serial.println("[SAFETY] unlock is only allowed while idle in Rest");
    }
  } else if (!strcmp(cmd, "lock")) {
    if (!runner.active) {
      safety = Armed::SafetyLockEngaged;
      Serial.println("[SAFETY] lock engaged");
    } else {
      Serial.println("[SAFETY] protocol running: use 'abort' or 'kill'");
    }
  } else if (!strcmp(cmd, "reset")) {
    if (state == FS::Damaged && allMotorsStopped()) enterState(FS::Rest, "reset command");
    else Serial.println("reset only from Damaged with motors stopped");
  } else if (!strcmp(cmd, "list")) {
    printProtocolList();
  } else if (!strcmp(cmd, "run")) {
    runOrPlan(true, argc - 1, argv + 1);
  } else if (!strcmp(cmd, "plan")) {
    runOrPlan(false, argc - 1, argv + 1);
  } else {
    printHelp();
  }
}

void handleSerial() {
  static char    buf[48];
  static uint8_t len = 0;
  while (Serial.available()) {
    const char ch = static_cast<char>(Serial.read());
    if (ch == '\n' || ch == '\r') {
      if (len == 0) continue;
      buf[len] = '\0';
      len = 0;
      handleCommand(buf);
    } else if (len < sizeof(buf) - 1) {
      buf[len++] = static_cast<char>(tolower(static_cast<unsigned char>(ch)));
    }
  }
}

// ===================================================== SETUP / LOOP ====================
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n=== UTME Weeble boot ===");

  // 1) ESCs first so they see a valid min-throttle pulse as early as possible
  for (auto& e : escs) e.begin();

  // 2) IMU (keep the drone still during calibration)
  if (!imu.begin(config::kSdaPin, config::kSclPin, config::kI2CClockHz)) fatal("IMU not found");
  Serial.println("Calibrating IMU - keep the drone still...");
  if (!imu.calibrate(1000)) fatal("IMU calibration failed");

  // 3) Gimbal servos to home
  if (!initGimbalAxis(pitchAxis) || !initGimbalAxis(rollAxis)) {
    fatal("gimbal has no solution at home - check LinkageLengths / servo channels");
  }

  // 4) Arm ESCs. Only the last call waits; the others keep receiving min pulse meanwhile.
  Serial.println("Arming ESCs...");
  for (size_t i = 0; i < kNumMotors; ++i) escs[i].arm(i + 1 == kNumMotors ? 3000 : 0);

  lastControlUs = micros();
  Serial.println("Ready. State: Rest, safety LOCKED.");
  printHelp();
}

void loop() {
  handleSerial();

  const uint32_t nowUs = micros();
  if (nowUs - lastControlUs >= config::kControlPeriodUs) {
    const float dt = (nowUs - lastControlUs) * 1e-6f;
    lastControlUs = nowUs;
    controlStep(dt);
  }

  for (auto& e : escs) e.update();   // RPM PID (rate-limited internally)

  printTelemetry();
}
