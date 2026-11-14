/*
  ESC + brushless motor test rig with a live IMU readout.
  Type a target RPM into the serial monitor and watch the IMU while the motor runs.
 
  Standalone - does not use any of the drone files.
 
  Wiring
    ESC control lead:
      signal (white/yellow/orange) -> D18
      ground (black/brown)         -> GND        (NOT a GPIO pin)
      red BEC 5V, if present       -> unconnected (never a GPIO pin)
    IMU (MPU-6050 / GY-521):
      VCC -> VIN (board with regulator) or 3V3     GND -> GND
      SDA -> D21     SCL -> D22     AD0 -> GND (address 0x68)
      INT -> D25 (optional, not used here)     XDA / XCL -> unconnected
    Servos: signal A -> D19, signal B -> D23, + -> 5 V (VIN or the ESC's BEC), - -> GND
 
  Without an RPM sensor (kTachPin = -1) the speed is an ESTIMATE:
    throttle = rpm / (KV * battery voltage)
  With a sensor, a PI loop corrects it.
 
  Serial commands (115200 baud, press Enter):
    4000        spin to 4000 RPM
    t 20        raw throttle 20 %
    0 / stop    stop immediately
    imu         show / hide the IMU readout (motor status still prints while running)
    zero        treat the current angle as level
    Servos - angles are GIMBAL degrees from home (0), limited to -15..+15.
    One value per servo, A then B; a single value sets both:
    s 10 -5     A to +10, B to -5 - both move together and arrive at the same time
    s 0         both home (same as 's c')
    s - 8       leave A where it is, B to +8
    s off       both off (limp); 's off 5' turns A off and moves B
    sp 1500 1450  raw pulse widths in microseconds (for calibration)
    sp +10 -    nudge A's pulse by +10 us, leave B (for calibration)
    sweep       both slowly to +15, -15, then home
    szero       the servos' current positions become 0 deg (saved in flash, kept after restarts)
    szero reset go back to homeUs from the table
    At startup both servos move to their saved 0 deg.
    scan        look for I2C devices on several pin pairs, then retry the IMU
    status      print one status line
    help
 
  !! PROPS OFF. Clamp the motor down. Keep a hand on the battery plug.
*/
#include <Arduino.h>
#include <Wire.h>
#include <Preferences.h>   // ESP32 flash storage (keeps the servo zero across restarts)
#include <cmath>
#include <cstring>
#include <cstdlib>
#include <cctype>
#include <initializer_list>
 
#ifndef ESP_ARDUINO_VERSION_MAJOR
#define ESP_ARDUINO_VERSION_MAJOR 1
#endif
 
// ================================ CONFIG ================================
// ESC / motor
constexpr int      kEscPin       = 18;      // ESC signal wire
constexpr int      kTachPin      = -1;      // RPM sensor pin, or -1 for none
constexpr uint8_t  kPulsesPerRev = 7;       // optical: blades; ESC rpm wire: pole pairs
 
constexpr float    kMotorKv     = 1300.0f;  // RPM per volt
constexpr float    kBatteryV    = 11.1f;    // FULL pack voltage: 3S = 11.1, 4S = 14.8
constexpr float    kMaxTestRpm  = 8000.0f;  // refuse anything above this
constexpr float    kMaxThrottle = 0.60f;    // never output more than 60 % in this test
 
constexpr uint16_t kMinPulseUs  = 1000;     // ESC throttle range
constexpr uint16_t kMaxPulseUs  = 2000;
constexpr uint32_t kPwmFreqHz   = 50;       // 50 Hz works with every ESC
constexpr uint8_t  kPwmBits     = 16;
constexpr uint8_t  kLedcChannel = 0;        // used on Arduino core 2.x only
 
constexpr uint32_t kArmMs      = 3000;      // hold zero throttle so the ESC arms
constexpr uint32_t kControlMs  = 5;         // 200 Hz motor update
constexpr float    kSlewPerSec = 0.5f;      // throttle change per second (0 -> 50 % in 1 s)
 
// PI gains for closed-loop mode (only used when kTachPin >= 0)
constexpr float kKp = 0.00005f;             // throttle per RPM of error
constexpr float kKi = 0.00015f;
constexpr float kIntegralLimit = 0.15f / kKi;   // integral can add at most 15 % throttle
 
// IMU
constexpr int      kSdaPin      = 21;
constexpr int      kSclPin      = 22;
constexpr uint32_t kI2cClockHz  = 400000;   // 400 kHz
constexpr uint8_t  kImuAddrLow  = 0x68;     // AD0 to GND
constexpr uint8_t  kImuAddrHigh = 0x69;     // AD0 to 3.3 V (checked as a backup)
constexpr uint32_t kImuPeriodUs = 2000;     // read at 500 Hz
constexpr float    kAlpha       = 0.98f;    // complementary filter: trust in the gyro
constexpr float    kAccelLsbPerG   = 8192.0f;  // +/-4 g range
constexpr float    kGyroLsbPerDps  = 65.5f;    // +/-500 deg/s range
 
// Servos drive the gimbal. Angles are GIMBAL degrees from home: 0 = props straight,
// + and - tilt either way. Both servos are set by one command and move together.
//
// Each servo is calibrated with three pulse widths you measure on the real gimbal:
//   loUs   = pulse that puts the gimbal at loDeg (-15)
//   homeUs = pulse that puts the gimbal at 0 (home)
//   hiUs   = pulse that puts the gimbal at hiDeg (+15)
// Angles in between are interpolated. If a gimbal tilts the wrong way, swap its loUs and hiUs.
//
// Calibrating one servo (gimbal attached, motor stopped), servo A shown, use '- +10' for B:
//   1. 'sp 1500 -', then nudge with 'sp +10 -' / 'sp -10 -' until the gimbal is at home -> homeUs
//   2. keep nudging one way until the gimbal reaches +15 deg           -> hiUs
//      (protractor or IMU; stop as soon as it gets there - don't push into the stop)
//   3. go back to homeUs, then nudge the other way to -15 deg         -> loUs
struct ServoConfig {
  const char* name;
  int         pin;
  uint8_t     channel;    // core 2.x only; 0/1 share the ESC's timer
  uint16_t    loUs;       // pulse at loDeg
  uint16_t    homeUs;     // pulse at 0 deg (home)
  uint16_t    hiUs;       // pulse at hiDeg
  float       loDeg;      // gimbal limits
  float       hiDeg;
};
constexpr int kNumServos = 2;
const ServoConfig kServos[kNumServos] = {
  // TODO: calibrate. These are CONSERVATIVE placeholders (+/-100 us) so nothing hits the
  // gimbal's stops before calibration - until then, "15 deg" will really be less.
  // name  pin  ch  loUs  homeUs  hiUs   loDeg    hiDeg
  {  "A",  19,  2,  1400, 1500,   1600,  -15.0f,  15.0f },
  {  "B",  23,  3,  1400, 1500,   1600,  -15.0f,  15.0f },
};
constexpr uint16_t kServoRawMinUs      = 1000;    // limits for the raw 'sp' command (servo rating)
constexpr uint16_t kServoRawMaxUs      = 2000;
constexpr float    kServoMoveDegPerSec = 60.0f;   // gimbal deg/s for 's' moves; 0 = jump instantly
constexpr float    kSweepDegPerSec     = 20.0f;
constexpr bool     kServoHomeAtBoot    = true;    // drive both servos to their saved 0 at startup
constexpr int16_t  kMaxZeroShiftUs     = 400;     // 'szero' may move home at most this far
constexpr uint32_t kServoUpdateMs      = 10;      // how often a move is stepped
 
constexpr uint32_t kPrintMs = 200;          // live readout rate (5 Hz)
 
// ================================ STATE ================================
enum class Mode { Stopped, Rpm, Throttle };
 
Mode     mode           = Mode::Stopped;
float    targetRpm      = 0;
float    targetThrottle = 0;   // used in Throttle mode
float    throttle       = 0;   // what is actually being sent (0..1)
float    integral       = 0;
float    measuredRpm    = 0;
uint32_t lastControlMs  = 0;
 
// Servos
struct ServoState {
  bool     active;    // false = no signal (servo limp)
  bool     raw;       // last command was a raw pulse ('sp')
  float    deg;       // current commanded angle
  uint16_t us;        // current pulse width
  bool     moving;    // part of the move in progress
  float    fromDeg;   // where the move started
  float    toDeg;     // where it ends
};
ServoState servo[kNumServos] = {};   // all zero / false
 
// Set by 'szero': how far the 0-deg pulse has moved from homeUs in the table.
// Saved in flash, so it survives restarts and uploads (only 'Erase Flash' clears it).
int16_t     zeroShiftUs[kNumServos] = {};
Preferences prefs;
 
bool     moveActive     = false;
uint32_t moveStartMs    = 0;
uint32_t moveDurationMs = 0;
uint8_t  sweepPhase     = 0;   // 0 = not sweeping, 1 = to high limits, 2 = to low, 3 = to center
 
// Tachometer (written by the interrupt)
portMUX_TYPE      tachMux     = portMUX_INITIALIZER_UNLOCKED;
volatile uint32_t lastPulseUs = 0;
volatile uint32_t periodUs    = 0;
 
// IMU
bool     imuOk       = false;
uint8_t  imuAddr     = kImuAddrLow;
bool     imuSeeded   = false;
bool     liveReadout = true;
uint32_t imuErrors   = 0;
uint32_t lastImuUs   = 0;
float    gyroBias[3] = {0, 0, 0};    // raw counts, measured at boot
float    roll = 0, pitch = 0;        // degrees
float    rollZero = 0, pitchZero = 0;
float    rollRate = 0, pitchRate = 0, yawRate = 0;   // deg/s
float    accelG = 1.0f;              // total acceleration in g
// Vibration: spread of the accel reading over each print window
float    vibSum = 0, vibSumSq = 0;
uint32_t vibCount = 0;
float    vibG = 0;
 
// ================================ ESC OUTPUT ================================
// Set up a 50 Hz pulse output (ESC and servo use the same kind of signal)
void pwmAttach(int pin, uint8_t channel) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  (void)channel;
  ledcAttach(pin, kPwmFreqHz, kPwmBits);
#else
  ledcSetup(channel, kPwmFreqHz, kPwmBits);
  ledcAttachPin(pin, channel);
#endif
}
 
// Output a pulse of 'us' microseconds every frame. us = 0 means no pulse at all.
void pwmWriteUs(int pin, uint8_t channel, uint16_t us) {
  const uint32_t frameUs = 1000000UL / kPwmFreqHz;
  const uint32_t maxDuty = (1UL << kPwmBits) - 1;
  const uint32_t duty = (uint32_t)((uint64_t)us * maxDuty / frameUs);
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  (void)channel;
  ledcWrite(pin, duty);
#else
  (void)pin;
  ledcWrite(channel, duty);
#endif
}
 
void writePulseUs(uint16_t us) { pwmWriteUs(kEscPin, kLedcChannel, us); }
 
// ================================ SERVOS ================================
// Home is 0 deg (kept inside the limits in case they don't include 0)
float servoCenterDeg(int i) { return constrain(0.0f, kServos[i].loDeg, kServos[i].hiDeg); }
 
// The pulse that currently means 0 deg: the table's homeUs plus any 'szero' shift.
// The calibrated spans (hiUs - homeUs, loUs - homeUs) move along with it.
uint16_t servoHomeUs(int i) { return (uint16_t)(kServos[i].homeUs + zeroShiftUs[i]); }
 
// Gimbal angle -> pulse: interpolate between home and the calibrated end on that side
uint16_t servoDegToUs(int i, float deg) {
  const ServoConfig& c = kServos[i];
  float us = servoHomeUs(i);
  if (deg > 0.0f && c.hiDeg != 0.0f)      us += deg / c.hiDeg * ((float)c.hiUs - c.homeUs);
  else if (deg < 0.0f && c.loDeg != 0.0f) us += deg / c.loDeg * ((float)c.loUs - c.homeUs);
  return (uint16_t)constrain(us + 0.5f, (float)kServoRawMinUs, (float)kServoRawMaxUs);
}
 
// Pulse -> approximate gimbal angle (for showing raw 'sp' pulses as degrees)
float servoUsToDeg(int i, uint16_t us) {
  const ServoConfig& c = kServos[i];
  const float d      = (float)us - servoHomeUs(i);
  const float hiSpan = (float)c.hiUs - c.homeUs;
  const float loSpan = (float)c.loUs - c.homeUs;
  if (hiSpan != 0.0f && d * hiSpan >= 0.0f) return d / hiSpan * c.hiDeg;
  if (loSpan != 0.0f)                       return d / loSpan * c.loDeg;
  return 0.0f;
}
 
// Send an angle to one servo now
void servoWriteDeg(int i, float deg) {
  deg = constrain(deg, kServos[i].loDeg, kServos[i].hiDeg);
  servo[i].deg    = deg;
  servo[i].us     = servoDegToUs(i, deg);
  servo[i].active = true;
  servo[i].raw    = false;
  pwmWriteUs(kServos[i].pin, kServos[i].channel, servo[i].us);
}
 
// Send a raw pulse width to one servo now
void servoWriteUs(int i, uint16_t us) {
  const ServoConfig& c = kServos[i];
  servo[i].us     = us;
  servo[i].deg    = servoUsToDeg(i, us);
  servo[i].active = true;
  servo[i].raw    = true;
  servo[i].moving = false;
  pwmWriteUs(c.pin, c.channel, us);
}
 
void servoOff(int i) {
  servo[i].active = false;
  servo[i].moving = false;
  pwmWriteUs(kServos[i].pin, kServos[i].channel, 0);   // no pulses: the servo stops holding
}
 
// Move the chosen servos to their targets so they all ARRIVE AT THE SAME MOMENT.
// The servo with the furthest to go sets the time; the others move proportionally slower.
void startMove(const float target[], const bool use[], float degPerSec) {
  float maxDist = 0;
  for (int i = 0; i < kNumServos; ++i) {
    servo[i].moving = false;
    if (!use[i]) continue;
    const float to = constrain(target[i], kServos[i].loDeg, kServos[i].hiDeg);
    if (!servo[i].active || degPerSec <= 0.0f) {
      servoWriteDeg(i, to);      // position unknown (was off) or instant mode: jump
      continue;
    }
    servo[i].fromDeg = servo[i].deg;
    servo[i].toDeg   = to;
    servo[i].moving  = true;
    const float dist = std::fabs(to - servo[i].deg);
    if (dist > maxDist) maxDist = dist;
  }
  moveDurationMs = (degPerSec > 0.0f) ? (uint32_t)(maxDist / degPerSec * 1000.0f) : 0;
  moveStartMs    = millis();
  moveActive     = false;
  for (int i = 0; i < kNumServos; ++i) moveActive |= servo[i].moving;   // only if something moves
}
 
void startSweepPhase() {
  float target[kNumServos];
  bool  use[kNumServos];
  for (int i = 0; i < kNumServos; ++i) {
    use[i]    = true;
    target[i] = (sweepPhase == 1) ? kServos[i].hiDeg
              : (sweepPhase == 2) ? kServos[i].loDeg
                                  : servoCenterDeg(i);
  }
  startMove(target, use, kSweepDegPerSec);
}
 
void startSweep() {
  for (int i = 0; i < kNumServos; ++i) {
    if (!servo[i].active) servoWriteDeg(i, servoCenterDeg(i));   // start from a known spot
  }
  sweepPhase = 1;
  startSweepPhase();
}
 
// Step the move in progress. Non-blocking: called every pass through loop().
void updateServos() {
  if (!moveActive) return;
  static uint32_t lastStepMs = 0;
  const uint32_t now     = millis();
  const uint32_t elapsed = now - moveStartMs;
  const bool done = (elapsed >= moveDurationMs);
  if (!done && now - lastStepMs < kServoUpdateMs) return;
  lastStepMs = now;
 
  const float frac = done ? 1.0f : elapsed / (float)moveDurationMs;
  for (int i = 0; i < kNumServos; ++i) {
    if (!servo[i].moving) continue;
    servoWriteDeg(i, servo[i].fromDeg + (servo[i].toDeg - servo[i].fromDeg) * frac);
    servo[i].moving = !done;
  }
  if (!done) return;
 
  moveActive = false;
  if (sweepPhase == 3) {
    sweepPhase = 0;
    Serial.println("[SERVO] sweep done");
  } else if (sweepPhase > 0) {
    sweepPhase++;
    startSweepPhase();
  }
}
 
// ---- parsing the 's' / 'sp' values ----
// One value per servo. "-" = leave it alone. A single value applies to every servo.
enum class ServoCmd { Keep, Off, Angle };
 
bool parseServoValue(int i, const char* tok, ServoCmd& cmd, float& deg) {
  if (!strcmp(tok, "-"))                          { cmd = ServoCmd::Keep; return true; }
  if (!strcmp(tok, "off"))                        { cmd = ServoCmd::Off;  return true; }
  if (!strcmp(tok, "c") || !strcmp(tok, "center")) {
    cmd = ServoCmd::Angle;
    deg = servoCenterDeg(i);
    return true;
  }
  const bool signedNum = (tok[0] == '-' || tok[0] == '+') &&
                         (isdigit((unsigned char)tok[1]) || tok[1] == '.');
  if (isdigit((unsigned char)tok[0]) || tok[0] == '.' || signedNum) {
    deg = atof(tok);
    if (deg < kServos[i].loDeg || deg > kServos[i].hiDeg) {
      Serial.printf("refused: servo %s limits are %.0f..%.0f deg\n",
                    kServos[i].name, kServos[i].loDeg, kServos[i].hiDeg);
      return false;
    }
    cmd = ServoCmd::Angle;
    return true;
  }
  Serial.printf("servo %s: '%s' is not an angle, 'c', 'off' or '-'\n", kServos[i].name, tok);
  return false;
}
 
void printServoLine() {
  Serial.print("[SERVO]");
  for (int i = 0; i < kNumServos; ++i) {
    if (!servo[i].active) Serial.printf(" %s off", kServos[i].name);
    else Serial.printf(" %s -> %.1f deg", kServos[i].name,
                       servo[i].moving ? servo[i].toDeg : servo[i].deg);
  }
  if (moveActive && moveDurationMs > 0) Serial.printf("  (moving together, %.2f s)", moveDurationMs / 1000.0f);
  Serial.println();
}
 
// ---- servo zero ('szero') ----
void loadServoZero() {
  prefs.begin("servozero", /*readOnly=*/true);
  for (int i = 0; i < kNumServos; ++i) {
    const int16_t v = prefs.getShort(kServos[i].name, 0);
    zeroShiftUs[i] = (v >= -kMaxZeroShiftUs && v <= kMaxZeroShiftUs) ? v : 0;   // ignore junk
  }
  prefs.end();
}
 
void saveServoZero() {
  prefs.begin("servozero", /*readOnly=*/false);
  for (int i = 0; i < kNumServos; ++i) prefs.putShort(kServos[i].name, zeroShiftUs[i]);
  prefs.end();
}
 
void printServoZero() {
  Serial.print("[SERVO] 0 deg =");
  for (int i = 0; i < kNumServos; ++i) {
    Serial.printf(" %s %u us (shift %+d)", kServos[i].name, (unsigned)servoHomeUs(i), zeroShiftUs[i]);
  }
  Serial.println();
}
 
// szero        -> each servo's current position becomes its 0 deg (servos that are on)
// szero reset  -> back to homeUs from the table
void handleServoZero(const char* arg) {
  if (moveActive || sweepPhase) {
    Serial.println("wait for the servos to finish moving first");
    return;
  }
  if (arg && !strcmp(arg, "reset")) {
    for (int i = 0; i < kNumServos; ++i) {
      zeroShiftUs[i] = 0;
      if (servo[i].active) servo[i].deg = servoUsToDeg(i, servo[i].us);
    }
    saveServoZero();
    printServoZero();
    return;
  }
  if (arg) { Serial.println("usage: szero | szero reset"); return; }
 
  bool any = false;
  for (int i = 0; i < kNumServos; ++i) {
    if (!servo[i].active) {
      Serial.printf("servo %s is off - give it a position first (e.g. 's 0' or 'sp 1500')\n", kServos[i].name);
      continue;
    }
    const int shift = (int)servo[i].us - (int)kServos[i].homeUs;
    if (shift < -kMaxZeroShiftUs || shift > kMaxZeroShiftUs) {
      Serial.printf("servo %s: refused, %+d us is more than %d us from homeUs - check the table\n",
                    kServos[i].name, shift, kMaxZeroShiftUs);
      continue;
    }
    zeroShiftUs[i] = (int16_t)shift;
    servo[i].deg   = 0.0f;
    any = true;
    // Warn if the +/- limits would now need pulses outside the servo's rating
    const int hi = (int)servoHomeUs(i) + ((int)kServos[i].hiUs - kServos[i].homeUs);
    const int lo = (int)servoHomeUs(i) + ((int)kServos[i].loUs - kServos[i].homeUs);
    if (hi < kServoRawMinUs || hi > kServoRawMaxUs || lo < kServoRawMinUs || lo > kServoRawMaxUs) {
      Serial.printf("servo %s: note - its full +/- range now runs past %u..%u us and will be cut short\n",
                    kServos[i].name, (unsigned)kServoRawMinUs, (unsigned)kServoRawMaxUs);
    }
  }
  if (!any) return;
  saveServoZero();
  printServoZero();
}
 
// s <A> [B]
void handleServoAngles(char* const toks[], int count) {
  if (count < 1 || count > kNumServos) {
    Serial.println("usage: s <A> <B>   e.g. s 90 45 | s 90 | s - 120 | s c | s off");
    return;
  }
  ServoCmd cmd[kNumServos];
  float    target[kNumServos] = {};
  // Check every value before moving anything, so a typo can't move just one servo
  for (int i = 0; i < kNumServos; ++i) {
    const char* tok = (count == 1) ? toks[0] : toks[i];
    if (!parseServoValue(i, tok, cmd[i], target[i])) return;
  }
  sweepPhase = 0;
  bool use[kNumServos];
  for (int i = 0; i < kNumServos; ++i) {
    if (cmd[i] == ServoCmd::Off) servoOff(i);
    use[i] = (cmd[i] == ServoCmd::Angle);
  }
  startMove(target, use, kServoMoveDegPerSec);
  printServoLine();
}
 
// sp <A us> [B us]
void handleServoRaw(char* const toks[], int count) {
  if (count < 1 || count > kNumServos) {
    Serial.println("usage: sp <A us> <B us>   e.g. sp 1500 1450 | sp 1500 | sp - 1550 | sp +10 -");
    return;
  }
  long us[kNumServos];
  bool use[kNumServos];
  for (int i = 0; i < kNumServos; ++i) {
    const char* tok = (count == 1) ? toks[0] : toks[i];
    use[i] = strcmp(tok, "-") != 0;
    if (!use[i]) continue;
    // "+10" / "-10" nudge from the current pulse (or from home if the servo is off)
    const bool relative = (tok[0] == '+' || tok[0] == '-');
    const long base = servo[i].active ? servo[i].us : servoHomeUs(i);
    us[i] = relative ? base + atol(tok) : atol(tok);
    if (us[i] < kServoRawMinUs || us[i] > kServoRawMaxUs) {
      Serial.printf("refused: servo %s raw pulse must be %u..%u us\n", kServos[i].name,
                    (unsigned)kServoRawMinUs, (unsigned)kServoRawMaxUs);
      return;
    }
  }
  sweepPhase = 0;
  moveActive = false;
  for (int i = 0; i < kNumServos; ++i) {
    if (use[i]) servoWriteUs(i, (uint16_t)us[i]);
  }
  Serial.print("[SERVO] raw");
  for (int i = 0; i < kNumServos; ++i) {
    if (servo[i].active) Serial.printf(" %s %u us (~%.1f deg)", kServos[i].name, (unsigned)servo[i].us, servo[i].deg);
    else                 Serial.printf(" %s off", kServos[i].name);
  }
  Serial.println();
}
 
uint16_t throttleToPulse(float t) {
  return kMinPulseUs + (uint16_t)(t * (kMaxPulseUs - kMinPulseUs));
}
 
// ================================ TACHOMETER ================================
void IRAM_ATTR tachIsr() {
  const uint32_t now = micros();
  portENTER_CRITICAL_ISR(&tachMux);
  const uint32_t p = now - lastPulseUs;
  if (p >= 50) {              // ignore electrical noise
    periodUs    = p;
    lastPulseUs = now;
  }
  portEXIT_CRITICAL_ISR(&tachMux);
}
 
float readRpm() {
  if (kTachPin < 0) return 0;
  portENTER_CRITICAL(&tachMux);
  const uint32_t p    = periodUs;
  const uint32_t last = lastPulseUs;
  portEXIT_CRITICAL(&tachMux);
  if (p == 0 || micros() - last > 200000UL) return 0;    // no pulse for 200 ms = stopped
  const float raw = 60000000.0f / ((float)p * kPulsesPerRev);
  measuredRpm += 0.3f * (raw - measuredRpm);             // light smoothing
  return measuredRpm;
}
 
// ================================ IMU (MPU-6050) ================================
// Register addresses from the MPU-6050 register map
constexpr uint8_t REG_SMPLRT_DIV   = 0x19;
constexpr uint8_t REG_CONFIG       = 0x1A;
constexpr uint8_t REG_GYRO_CONFIG  = 0x1B;
constexpr uint8_t REG_ACCEL_CONFIG = 0x1C;
constexpr uint8_t REG_ACCEL_XOUT_H = 0x3B;   // first of 14 data bytes
constexpr uint8_t REG_PWR_MGMT_1   = 0x6B;
constexpr uint8_t REG_WHO_AM_I     = 0x75;
 
bool imuWrite(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(imuAddr);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}
 
bool imuReadBytes(uint8_t reg, uint8_t* out, uint8_t n) {
  Wire.beginTransmission(imuAddr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;     // false = keep the bus for the read
  if (Wire.requestFrom((int)imuAddr, (int)n) != n) return false;
  for (uint8_t i = 0; i < n; ++i) out[i] = (uint8_t)Wire.read();
  return true;
}
 
const char* imuName(uint8_t id) {
  switch (id) {
    case 0x68: return "MPU-6050";
    case 0x70: return "MPU-6500";
    case 0x71: return "MPU-9250";
    case 0x73: return "MPU-9255";
    default:   return "unknown (possibly a clone)";
  }
}
 
// Raw readings: accel x/y/z, temperature, gyro x/y/z
bool imuReadRaw(int16_t raw[7]) {
  uint8_t b[14];
  if (!imuReadBytes(REG_ACCEL_XOUT_H, b, 14)) return false;
  for (int i = 0; i < 7; ++i) raw[i] = (int16_t)((b[2 * i] << 8) | b[2 * i + 1]);   // high byte first
  return true;
}
 
bool imuBegin() {
  Wire.begin(kSdaPin, kSclPin, kI2cClockHz);
 
  // Find the IMU at 0x68, or 0x69 if AD0 is high
  uint8_t id = 0;
  bool found = false;
  for (uint8_t addr : {kImuAddrLow, kImuAddrHigh}) {
    imuAddr = addr;
    if (imuReadBytes(REG_WHO_AM_I, &id, 1)) { found = true; break; }
  }
  if (!found) {
    Serial.println("[IMU] not found at 0x68 or 0x69 - check SDA=D21, SCL=D22, VCC, GND");
    Serial.println("[IMU] type 'scan' once the motor is stopped to search for it");
    return false;
  }
  Serial.printf("[IMU] found at 0x%02X, WHO_AM_I = 0x%02X (%s)\n", imuAddr, id, imuName(id));
  if (imuAddr == kImuAddrHigh) Serial.println("[IMU] note: AD0 is high - tie it to GND for 0x68");
 
  bool ok = true;
  ok &= imuWrite(REG_PWR_MGMT_1, 0x01);     // wake up, clock from the X gyro (more stable)
  delay(50);
  ok &= imuWrite(REG_CONFIG, 0x02);         // low-pass filter ~94 Hz
  ok &= imuWrite(REG_SMPLRT_DIV, 0x01);     // 1 kHz / (1 + 1) = 500 Hz
  ok &= imuWrite(REG_GYRO_CONFIG, 0x08);    // +/-500 deg/s
  ok &= imuWrite(REG_ACCEL_CONFIG, 0x08);   // +/-4 g
  if (!ok) Serial.println("[IMU] configuration write failed");
  return ok;
}
 
// Average the gyro while still to find its zero offset. Runs during ESC arming.
void imuCalibrate(uint32_t durationMs) {
  float sum[3] = {0, 0, 0};
  uint32_t samples = 0;
  const uint32_t start = millis();
  while (millis() - start < durationMs) {
    int16_t raw[7];
    if (imuReadRaw(raw)) {
      for (int i = 0; i < 3; ++i) sum[i] += raw[4 + i];
      samples++;
    }
    delay(2);
  }
  if (samples == 0) { imuOk = false; return; }
  for (int i = 0; i < 3; ++i) gyroBias[i] = sum[i] / samples;
  Serial.printf("[IMU] gyro calibrated from %lu samples\n", (unsigned long)samples);
}
 
void imuUpdate() {
  int16_t raw[7];
  if (!imuReadRaw(raw)) { imuErrors++; return; }
 
  const uint32_t now = micros();
  const float dt = lastImuUs ? (now - lastImuUs) * 1e-6f : 0.0f;
  lastImuUs = now;
 
  const float ax = raw[0] / kAccelLsbPerG;
  const float ay = raw[1] / kAccelLsbPerG;
  const float az = raw[2] / kAccelLsbPerG;
  rollRate  = (raw[4] - gyroBias[0]) / kGyroLsbPerDps;
  pitchRate = (raw[5] - gyroBias[1]) / kGyroLsbPerDps;
  yawRate   = (raw[6] - gyroBias[2]) / kGyroLsbPerDps;
  accelG    = std::sqrt(ax * ax + ay * ay + az * az);
 
  // Angles from gravity alone (noisy, but never drifts)
  const float accRoll  = std::atan2(ay, az) * RAD_TO_DEG;
  const float accPitch = std::atan2(-ax, std::sqrt(ay * ay + az * az)) * RAD_TO_DEG;
 
  // Complementary filter: gyro for fast changes, gravity to stop drift
  if (!imuSeeded) {
    roll = accRoll;
    pitch = accPitch;
    imuSeeded = true;
  } else {
    roll  = kAlpha * (roll  + rollRate  * dt) + (1.0f - kAlpha) * accRoll;
    pitch = kAlpha * (pitch + pitchRate * dt) + (1.0f - kAlpha) * accPitch;
  }
 
  vibSum   += accelG;
  vibSumSq += accelG * accelG;
  vibCount++;
}
 
// Spread (standard deviation) of the accel reading since the last call
void updateVibration() {
  if (vibCount < 2) return;
  const float mean = vibSum / vibCount;
  const float var  = vibSumSq / vibCount - mean * mean;
  vibG = var > 0 ? std::sqrt(var) : 0.0f;
  vibSum = vibSumSq = 0;
  vibCount = 0;
}
 
// ---- I2C scanner: finds which pins / addresses actually respond ----
int scanPins(int sda, int scl) {
  Wire.end();
  Wire.begin(sda, scl, 100000);   // 100 kHz: slow and forgiving for the scan
  Wire.setTimeOut(10);            // don't wait long on silent addresses
  int found = 0;
  for (uint8_t addr = 1; addr < 127; ++addr) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      Serial.printf("    device at 0x%02X%s\n", addr,
                    (addr == 0x68 || addr == 0x69) ? "  <- IMU" : "");
      found++;
    }
  }
  return found;
}
 
void scanBus() {
  if (mode != Mode::Stopped) { Serial.println("stop the motor first"); return; }
  struct PinPair { int sda, scl; const char* label; };
  const PinPair pairs[] = {
    {kSdaPin, kSclPin, "configured pins"},
    {kSclPin, kSdaPin, "SDA and SCL swapped"},
    {12, 13, "old wiring"},
  };
  Serial.println("[SCAN] a few error lines from Wire during the scan are normal");
  for (const auto& pp : pairs) {
    Serial.printf("[SCAN] SDA=D%d SCL=D%d (%s):\n", pp.sda, pp.scl, pp.label);
    if (scanPins(pp.sda, pp.scl) == 0) Serial.println("    nothing responded");
  }
 
  // Back to the real pins and try the IMU again
  Wire.end();
  Wire.setTimeOut(50);
  imuSeeded = false;
  imuErrors = 0;
  imuOk = imuBegin();
  if (imuOk) {
    Serial.println("[IMU] calibrating gyro - keep still");
    imuCalibrate(1000);
  }
}
 
// ================================ MOTOR CONTROL ================================
void stopMotor() {
  mode = Mode::Stopped;
  targetRpm = targetThrottle = 0;
  throttle = 0;
  integral = 0;
  writePulseUs(kMinPulseUs);    // immediate, no ramp
}
 
void controlStep(float dt) {
  const float rpm = readRpm();
  if (mode == Mode::Stopped) {
    writePulseUs(kMinPulseUs);
    return;
  }
 
  float desired;
  if (mode == Mode::Throttle) {
    desired = targetThrottle;
  } else {
    // Feed-forward: estimated throttle for this RPM on an unloaded motor
    desired = targetRpm / (kMotorKv * kBatteryV);
    if (kTachPin >= 0) {
      // Closed loop: PI correction from the measured RPM
      const float error = targetRpm - rpm;
      integral = constrain(integral + error * dt, -kIntegralLimit, kIntegralLimit);
      desired += kKp * error + kKi * integral;
    }
  }
  desired = constrain(desired, 0.0f, kMaxThrottle);
 
  // Slew limit: change throttle gradually so the motor doesn't jerk
  const float maxStep = kSlewPerSec * dt;
  throttle += constrain(desired - throttle, -maxStep, maxStep);
  writePulseUs(throttleToPulse(throttle));
}
 
// ================================ SERIAL ================================
const char* modeName() {
  switch (mode) {
    case Mode::Stopped:  return "stopped";
    case Mode::Rpm:      return "rpm";
    case Mode::Throttle: return "throttle";
  }
  return "?";
}
 
void printStatus() {
  Serial.printf("[%-8s] thr %5.1f%% %4uus", modeName(), throttle * 100.0f,
                (unsigned)throttleToPulse(throttle));
  if (kTachPin >= 0) Serial.printf(" rpm %5.0f", measuredRpm);
  else               Serial.printf(" est %5.0f", throttle * kMotorKv * kBatteryV);
  if (mode == Mode::Rpm) Serial.printf("/%.0f", targetRpm);
  for (int i = 0; i < kNumServos; ++i) {
    if (servo[i].active) Serial.printf(" | %s %5.1fdeg %4uus%s", kServos[i].name, servo[i].deg,
                                       (unsigned)servo[i].us, servo[i].raw ? " raw" : "");
    else                 Serial.printf(" | %s off", kServos[i].name);
  }
  if (sweepPhase) Serial.print(" (sweep)");
 
  if (!liveReadout) {
    // IMU section hidden ('imu' to show it again)
  } else if (imuOk) {
    Serial.printf(" | roll %6.1f pitch %6.1f | gyro %6.1f %6.1f %6.1f dps | %.2fg vib %.3fg",
                  roll - rollZero, pitch - pitchZero, rollRate, pitchRate, yawRate, accelG, vibG);
    if (imuErrors) Serial.printf(" | i2c err %lu", (unsigned long)imuErrors);
  } else {
    Serial.print(" | IMU offline");
  }
  Serial.println();
}
 
void printHelp() {
  Serial.println("commands:  <rpm>      spin to that RPM, e.g. 4000");
  Serial.println("           t <pct>    raw throttle percent, e.g. t 20");
  Serial.println("           0 | stop   stop immediately");
  Serial.println("           imu        show / hide the IMU readout");
  Serial.println("           zero       treat the current angle as level");
  Serial.println("           scan       look for I2C devices, then retry the IMU");
  Serial.println("servos:    s <A> <B>  gimbal deg from home, e.g. s 10 -5 (both together), s 0, s - 8, s off");
  Serial.println("           sp <A us> <B us> raw pulses, sp +10 - nudges A | sweep");
  Serial.println("           szero      current servo positions become 0 deg (saved) | szero reset");
  Serial.println("           status | help");
  Serial.printf("limits: %.0f rpm, %.0f %% throttle", kMaxTestRpm, kMaxThrottle * 100.0f);
  for (int i = 0; i < kNumServos; ++i) {
    Serial.printf(", servo %s %.0f..%.0f deg", kServos[i].name, kServos[i].loDeg, kServos[i].hiDeg);
  }
  Serial.println();
}
 
void handleCommand(char* line) {
  // Split into words: cmd, then up to 4 arguments
  char* cmd = strtok(line, " ");
  if (!cmd) return;
  char* args[4];
  int   argCount = 0;
  for (char* tok = strtok(nullptr, " "); tok && argCount < 4; tok = strtok(nullptr, " ")) {
    args[argCount++] = tok;
  }
  char* arg = argCount > 0 ? args[0] : nullptr;
 
  if (!strcmp(cmd, "stop")) {
    stopMotor();
    Serial.println("stopped");
  } else if (!strcmp(cmd, "status")) {
    printStatus();
  } else if (!strcmp(cmd, "help")) {
    printHelp();
  } else if (!strcmp(cmd, "imu")) {
    liveReadout = !liveReadout;
    Serial.printf("IMU readout %s\n", liveReadout ? "ON" : "OFF");
  } else if (!strcmp(cmd, "scan")) {
    scanBus();
  } else if (!strcmp(cmd, "szero")) {
    handleServoZero(argCount > 0 ? args[0] : nullptr);
  } else if (!strcmp(cmd, "zero")) {
    if (!imuOk) { Serial.println("IMU offline"); return; }
    rollZero = roll;
    pitchZero = pitch;
    Serial.printf("zeroed at roll %.1f, pitch %.1f\n", rollZero, pitchZero);
  } else if (!strcmp(cmd, "s")) {
    handleServoAngles(args, argCount);
  } else if (!strcmp(cmd, "sp")) {
    handleServoRaw(args, argCount);
  } else if (!strcmp(cmd, "sweep")) {
    startSweep();
    Serial.println("[SERVO] sweeping both: high limits -> low limits -> home");
  } else if (!strcmp(cmd, "t")) {
    if (!arg) { Serial.println("usage: t <percent>"); return; }
    const float pct = atof(arg);
    if (pct <= 0.0f) { stopMotor(); Serial.println("stopped"); return; }
    if (pct > kMaxThrottle * 100.0f) {
      Serial.printf("refused: above the %.0f %% limit\n", kMaxThrottle * 100.0f);
      return;
    }
    mode = Mode::Throttle;
    targetThrottle = pct / 100.0f;
    Serial.printf("throttle -> %.1f %%\n", pct);
  } else if (isdigit((unsigned char)cmd[0]) || cmd[0] == '.') {
    const float rpm = atof(cmd);
    if (rpm <= 0.0f) { stopMotor(); Serial.println("stopped"); return; }
    if (rpm > kMaxTestRpm) {
      Serial.printf("refused: above the %.0f rpm limit\n", kMaxTestRpm);
      return;
    }
    if (mode != Mode::Rpm) integral = 0;
    mode = Mode::Rpm;
    targetRpm = rpm;
    Serial.printf("target -> %.0f rpm (needs about %.1f %% throttle)\n",
                  rpm, 100.0f * rpm / (kMotorKv * kBatteryV));
  } else {
    printHelp();
  }
}
 
void handleSerial() {
  static char    buf[32];
  static uint8_t len = 0;
  while (Serial.available()) {
    const char ch = (char)Serial.read();
    if (ch == '\n' || ch == '\r') {
      if (len == 0) continue;
      buf[len] = '\0';
      len = 0;
      handleCommand(buf);
    } else if (len < sizeof(buf) - 1) {
      buf[len++] = (char)tolower((unsigned char)ch);
    }
  }
}
 
// ================================ SETUP / LOOP ================================
void setup() {
  Serial.begin(115200);
 
  // Start the ESC signal at zero throttle straight away
  pwmAttach(kEscPin, kLedcChannel);
  writePulseUs(kMinPulseUs);
 
  // Servos: load the saved zero, then either go to 0 deg or stay limp until a command
  loadServoZero();
  for (int i = 0; i < kNumServos; ++i) {
    pwmAttach(kServos[i].pin, kServos[i].channel);
    if (kServoHomeAtBoot) servoWriteDeg(i, 0.0f);
    else                  servoOff(i);
  }
 
  if (kTachPin >= 0) {
    pinMode(kTachPin, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(kTachPin), tachIsr, FALLING);
  }
 
  delay(500);
  Serial.println("\n=== ESC test rig + IMU ===");
  Serial.printf("motor %.0f KV, battery %.1f V -> about %.0f rpm at full throttle (unloaded)\n",
                kMotorKv, kBatteryV, kMotorKv * kBatteryV);
 
  imuOk = imuBegin();
 
  // The ESC needs ~3 s of zero throttle to arm; calibrate the gyro during that time
  Serial.println("Arming ESC (zero throttle)... keep everything still");
  if (imuOk) imuCalibrate(kArmMs);
  else       delay(kArmMs);
  while (Serial.available()) Serial.read();   // ignore anything typed during arming
 
  Serial.println("Armed. PROPS OFF.");
  if (kServoHomeAtBoot) Serial.print("Servos at 0 deg. ");
  printServoZero();
  printHelp();
  lastControlMs = millis();
  lastImuUs = 0;
}
 
void loop() {
  handleSerial();
 
  if (imuOk && micros() - lastImuUs >= kImuPeriodUs) imuUpdate();
 
  const uint32_t now = millis();
  if (now - lastControlMs >= kControlMs) {
    const float dt = (now - lastControlMs) / 1000.0f;
    lastControlMs = now;
    controlStep(dt);
  }
 
  updateServos();
 
  static uint32_t lastPrint = 0;
  if (now - lastPrint >= kPrintMs) {
    lastPrint = now;
    updateVibration();
    if (liveReadout || mode != Mode::Stopped) printStatus();
  }
}
 
