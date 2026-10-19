/*
Main functionality of the UTME Weeble drone: main.cpp

Hardware:
  ESP32 dev board (PlatformIO)
  2 Brushless motors
  IMU on 2IC (SDA 21, SCL 22, address 0x68)

Control structure (three layers):
  PROTOCOLS     WHAT to do: an ordered list of Steps (flight state + parameters)
  STATES        HOW to do it: applyStep() sets the motors / gimbal for the current state
  VERIFICATION  DID it happen: verifyStep() checks the IMU and tachometers
 
  Every step:
    1. enter the step's flight state and act on it every tick
    2. after minTimeMs, start checking the verification condition
       (it must hold for kVerifyHoldMs in a row, so one noisy reading can't pass it)
    3. verified                  -> keep running for holdMs, then go to the next step
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

*/

#include <vector>
#include <Arduino.h>
#include <espcontroller.h>
#include <imu.h>
#include <servo.h>
#include <gimblecontrol.h>

using FS = FlightState;   // short nickname: FS::Ascending instead of FlightState::Ascending

// ===================================================== CONFIG ==========================================================
namespace config {
  constexpr int kSdaPin = 21;
  constexpr int kSclPin = 22;
  constexpr uint16_t kI2CClockHz = 400000;                //400 kHz

  // Loop timing
  constexpr uint32_t kControlPeriodUs = 5000;             // 200Hz attitude/state loop
  constexpr uint32_t kTelemetryPeriodMs = 100;            // 10Hz serial telemetry

  // Safety
  constexpr float kMaxTiltDeg = 45.0f;                    // IDK what this value is tbh
  constexpr uint8_t kMaxImuFailStreak = 10;               // Consecutive failed IMU reads --> fault
  constexpr float kLaunchleveltoDeg = 5.0f;

  // Flight profile
  constexpr float kAscendRPM = 7000.0f;                   // Tune these based on drone weights and payload
  constexpr float kDescendRPM = 5000.0f;
  constexpr float kLandingRPM = 4500.0f;

  constexpr uint32_t kLaunchTimeoutMs = 3000;
  constexpr uint32_t kAmpUpRampMs = 2000;
  constexpr uint32_t kAmpUpTimeoutMs = 5000;
  constexpr uint32_t kAmpupRpmTolerance = 0.05f;
  constexpr uint32_t kAmpUpSettleMs = 300;

  constexpr uint32_t kLandingTimeoutMs = 5000;
  constexpr float kTouchdownAccelG = 1.6f;
  constexpr uint32_t kAmpDownRampMs    = 1500;
  constexpr uint32_t kAmpDownTimeoutMs = 4000;
  constexpr float kStoppedRpm = 300.0f;

  // Gimbal
  constexpr float kMaxGinbalTiltDeg = 15.0f;                  // Tommy said this is the max tilt of the gimbal in either direction
  constexpr float kTh2InitialDeg = 25.0f;                     // CHANGE once the gimbal design is finalized
  const std::vector<float> LinkageLengths = {2.5, 3.0, 4.1};  // Investigate this further, this is the set of linkage lengths

  // Yaw



  // =============================================== HARDWARE ===========================================================

  static EscController::Config makeEscConfig(uint8_t escPin, uint8_t tachPin, uint8_t ledcChannel) {

    EscController::Config c{};
    c.escPin = escPin;                        // 
    c.tachPin = tachPin;                      //
    c.pulsesPerRev = 2;                       //
    c.motorKv = 920.0f;                       //
    c.battVolt = 11.1f;                       // Check this logic, there should be a way to monitor the battery voltage 
    c.maxRpm = 9000.0f;                       //
    c.ledcChannel = ledcChannel;              //
    c.pwnFreq = 50;                           //

    c.kp = 0.00005f;                          //
    c.ki = 0.00005f;                          //
    c.kp = 0.00005f;                          //
    return c;

  };
};

// ===================================================== HARDWARE OBJECTS ================
EscController escs[2] = {
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

// =============================================== PROTOCOL TYPES ================================================================

/*
One Step of a protocol
minTimeMs: verification isn't checked before this (settling / ramp time)
timeoutMs: must be verified before this, or the step falls back
holdMs:    keep running this step this long AFTER it is verified
*/

struct Step {
  FlightState state;
  float rpm = 0;
  float leanDeg = 0;
  float headingDeg = 0;
  uint32_t minTimeMs = 0;
  uint32_t holdMs = 0;
  uint32_t timeoutMs = 0;
  FlightState fallback = (FlightState - 1); // This will need some work
};

// A Protocol is a fixed-size list of steps (no memory allocation while flying).
constexpr size_t kMaxSteps = 12;
struct Protocol {
  char name[16] = "";
  char info [128] = "";
  Step steps;
  uint8_t count = 0;

  explicit Protocol(const char* n = "") {
    strncpy(name, n, sizeof(name) - 1);
    name[sizeof(name) - 1] = '\0';
  }
  bool add(const Step& s) {
    //if (count >= kMaxSteps) return false;
    steps[count++] = s;
    return true;
  }
};

// What happened to each step (printed when the protocol ends, for tuning)
struct StepResult {
  bool ran = false;
  bool verified = false;
  bool fellback = false;
  bool aborted = false;
  uint32_t verifyAtMs = 0;
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

// ================================================= STATE =======================================================================

// The drone at compiling should be at Rest and safety armed
FlightState state = FS::Rest;
Armed safety = Armed::SafetyLockEngaged;
Runner runner;

//uint32_t stateenteredMs = 0;      // 
//uint32_t settleSinceMs = 0;       // 
float baseRpm = 0;                //  
float ampDownStartRpm = 0;        // AmpDown Thrust: inbetween landing and 0
uint8_t imuFailStreak = 0;        // Number of failed readings from an IMU
uint32_t lastControlUs = 0;       // 
StateValues att;

// Lean: the step sets a desired value; the current value moves toward it smoothly
float desiredPitchLean = 0;
float desiredRollLean = 0;
float pitchLean = 0;
float rollLean = 0;

// ================================================== HELPERS ===================================================================

const char* stateName(FlightState s) {
  switch (s) {
    case FS::Rest: return "Rest";
    case FS::Launch: return "Launch";
    case FS::AmpUp: return "AmpUp";
    case FS::Ascending: return "Ascending";
    case FS::Descending: return "Descending";
    case FS::Landing: return "Landing";
    case FS::AmpDown: return "AmpDown";
    case FS::Damaged: return "Damaged";
  }
  return "?";
}

bool isUnlocked() { return safety == Armed::SafetyLockedNotEngaged; }

bool inFlight() { // motors may be spinning
  return state == FS::AmpUp || 
          state == FS::Ascending || 
          state == FS::Descending ||
          state == FS::Landing || 
          state == FS::AmpDown;
  }


// ------- Sensor Checks ---------
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

// ---------------- Motors ----------------

void atopAllMotors() {
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

float yawDiffRpm() {
  if (kNumMotors < 2) return 0;
  return constrain(-config::kYawRateGain * att.yawrate,
                   -config::kMaxYawDiffRpm, config::kMaxYawDiffRpm);
}

// ---------------- gimbal ----------------       // DOUBLE CHECK THISSS PLEASEEEEE
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
 

// =============================================== HELPER VALUES ================================================================



// put function declarations here:

void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:

}
