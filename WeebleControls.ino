// WeebleControls.ino -- Phase 1 glue: ESP32 (Arduino core) <-> weeble_core.h
//
// STATUS: written and logic-tested on a laptop only. NOT compiled or run on an ESP32, and NOT tested with real
// hardware. First power-up MUST be with propellers OFF.
//
// Everything marked TODO must be checked against the real wiring / parts before anything spins:
//   - GPIO pins, IMU orientation (IMU_AXIS / IMU_SIGN), ESC type & calibration, servo direction and us-per-degree.
//
// Needs: Arduino-ESP32 core, library "ESP32Servo" (Library Manager). Wire is part of the core.
//
// Serial monitor: 115200 baud, line ending "Newline". Type "help".

#include <Arduino.h>
#include <Wire.h>
#include <ESP32Servo.h>
#include "weeble_core.h"

using namespace weeble;

// ============================ hardware configuration (TODO: verify) ============================
static const int PIN_ESC_UPPER = 25;   // TODO which ESC drives the upper (or lower) rotor of the coaxial pair
static const int PIN_ESC_LOWER = 26;
static const int PIN_SERVO_X   = 27;   // tilts the pod toward body +x (forward)   TODO verify
static const int PIN_SERVO_Y   = 33;   // tilts the pod toward body +y (left)      TODO verify
static const int PIN_SDA = 21, PIN_SCL = 22;   // ESP32 DevKit default I2C
static const uint8_t MPU_ADDR = 0x68;          // GY-521 with AD0 low

// Which sensor axis feeds body x / y / z (0=X,1=Y,2=Z) and its sign.  Body: x forward, y left, z up (drone upright).
// TODO: set from how the GY-521 is mounted. Check with "imu" command: tilt the drone forward -> az/ax change as expected.
static const int   IMU_AXIS[3] = {0, 1, 2};
static const float IMU_SIGN[3] = {1, 1, 1};

static const float LOOP_HZ = 200.0f;
static const float MAX_MISSION_S = 25.0f;      // absolute limit for any mission
static const float DEFAULT_CAP = 0.15f;        // throttle cap at boot (15 %) -- raise deliberately with "cap"

// ============================ globals ============================
static Params P;
static Flight flight(P);
static Servo escU, escL, srvX, srvY;

static bool armed = false, calibrated = false;
static float throttleCap = DEFAULT_CAP;
static float releaseHeight = 6.096f;           // 20 ft. Measure the real release height and set with "h"
static uint32_t lastTickUs = 0, missionStartMs = 0;
static bool missionActive = false;
static int imuFail = 0;

// bench helpers
static bool benchServoOn = false; static float benchTx = 0, benchTy = 0;
enum SpinPhase { SP_OFF, SP_BASE, SP_RUN };
static SpinPhase spinPhase = SP_OFF; static float spinThr = 0; static uint32_t spinT0 = 0; static float spinDur = 0;
static double spinSum0 = 0, spinSum1 = 0, spinSq1 = 0; static int spinN0 = 0, spinN1 = 0;

// logging (RAM ring, dumped after the mission)
struct Rec { float t, phase, px, pz, vx, vz, tilt, u, tx, fz, fmag; };
static const int LOG_N = 1200;                 // 12 s at 100 Hz, ~53 kB
static Rec logbuf[LOG_N]; static int logCount = 0;

// ============================ MPU6050 (raw registers) ============================
static bool mpuWrite(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_ADDR); Wire.write(reg); Wire.write(val);
  return Wire.endTransmission() == 0;
}
static bool mpuInit() {
  Wire.begin(PIN_SDA, PIN_SCL); Wire.setClock(400000);
  bool ok = mpuWrite(0x6B, 0x00);   // PWR_MGMT_1: wake up
  delay(100);
  ok &= mpuWrite(0x1A, 0x02);       // CONFIG: DLPF ~94 Hz accel / 98 Hz gyro (1 kHz internal rate)
  ok &= mpuWrite(0x19, 0x00);       // SMPLRT_DIV: 1 kHz
  ok &= mpuWrite(0x1B, 0x10);       // GYRO_CONFIG: +-1000 deg/s   (32.8 LSB per deg/s)
  ok &= mpuWrite(0x1C, 0x10);       // ACCEL_CONFIG: +-8 g         (4096 LSB per g)
  return ok;
}
static uint8_t mpuWhoAmI() {
  Wire.beginTransmission(MPU_ADDR); Wire.write(0x75); Wire.endTransmission(false);
  Wire.requestFrom((int)MPU_ADDR, 1); return Wire.available() ? Wire.read() : 0xFF;
}
static bool mpuRead(Vec3& gyro_rads, Vec3& acc_ms2) {
  Wire.beginTransmission(MPU_ADDR); Wire.write(0x3B);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((int)MPU_ADDR, 14) != 14) return false;
  uint8_t b[14]; for (int i = 0; i < 14; i++) b[i] = Wire.read();
  auto s16 = [&](int i) { return (int16_t)((b[i] << 8) | b[i + 1]); };
  float a[3] = {s16(0) / 4096.0f * kG, s16(2) / 4096.0f * kG, s16(4) / 4096.0f * kG};
  float g[3] = {s16(8) / 32.8f * kDeg2Rad, s16(10) / 32.8f * kDeg2Rad, s16(12) / 32.8f * kDeg2Rad};
  acc_ms2   = Vec3{IMU_SIGN[0] * a[IMU_AXIS[0]], IMU_SIGN[1] * a[IMU_AXIS[1]], IMU_SIGN[2] * a[IMU_AXIS[2]]};
  gyro_rads = Vec3{IMU_SIGN[0] * g[IMU_AXIS[0]], IMU_SIGN[1] * g[IMU_AXIS[1]], IMU_SIGN[2] * g[IMU_AXIS[2]]};
  return true;
}

// ============================ outputs ============================
static void writeOutputs(const Outputs& o) {
  escU.writeMicroseconds(o.esc_upper_us); escL.writeMicroseconds(o.esc_lower_us);
  srvX.writeMicroseconds(o.servo_x_us);   srvY.writeMicroseconds(o.servo_y_us);
}
static void safeOutputs() {
  Command c; writeOutputs(mix(c, P, 0.0f));   // motors at ESC minimum, servos centred
}
static void killAll(const char* why) {
  safeOutputs(); flight.stopAll(); armed = false; missionActive = false; spinPhase = SP_OFF; benchServoOn = false;
  Serial.print("KILL/DISARM: "); Serial.println(why);
}

// ============================ calibration ============================
static void doCalibrate() {
  if (armed) { Serial.println("disarm first"); return; }
  Serial.println("Calibrating: keep the drone STILL and UPRIGHT for 2 s ...");
  const int N = 400; Vec3 gs, as; double gm2 = 0;
  for (int i = 0; i < N; i++) {
    Vec3 g, a; uint32_t t0 = micros();
    if (!mpuRead(g, a)) { Serial.println("IMU read failed"); return; }
    gs = gs + g; as = as + a; gm2 += norm(g);
    while (micros() - t0 < 5000) {}
  }
  Vec3 gmean = gs * (1.0f / N), amean = as * (1.0f / N);
  float amag = norm(amean);
  flight.est.initAtRest(amean);
  flight.est.setGyroBias(gmean);
  calibrated = true;
  Serial.print("gyro bias [rad/s]: "); Serial.print(gmean.x, 4); Serial.print(' '); Serial.print(gmean.y, 4); Serial.print(' '); Serial.println(gmean.z, 4);
  Serial.print("mean accel [m/s^2]: "); Serial.print(amean.x, 3); Serial.print(' '); Serial.print(amean.y, 3); Serial.print(' '); Serial.print(amean.z, 3);
  Serial.print("  |a|="); Serial.print(amag, 3); Serial.print("  tilt="); Serial.print(flight.est.tiltDeg(), 1); Serial.println(" deg");
  if (fabsf(amag - kG) > 1.0f) Serial.println("WARNING: |a| is far from 9.81 -> moving, or accelerometer badly off.");
  if (gm2 / N > 0.05) Serial.println("WARNING: gyro noisy -> drone was not still.");
  if (flight.est.tiltDeg() > 10.0f) Serial.println("WARNING: drone is tilted > 10 deg at calibration.");
}

// ============================ serial commands ============================
static void printHelp() {
  Serial.println(F(
    "Commands:\n"
    "  help | status | imu                 info (imu prints live accel/gyro/attitude once)\n"
    "  cal                                 2 s stillness calibration (upright, still) -- required before arming\n"
    "  arm | disarm | k                    'k' (or 'kill') = instant stop at any time\n"
    "  cap <percent>                       throttle cap, default 15. Raise deliberately.\n"
    "  h <metres>                          release height for descent (measure it!)\n"
    "  preset robust|gentle                descent tuning (see notes)\n"
    "  servo <x_deg> <y_deg>               bench: move pod (no motors). 'servo off' releases\n"
    "  spin <percent> <seconds>            bench: spin both motors (RESTRAIN THE DRONE, props off or guarded) and\n"
    "                                      report the accelerometer shift caused by the motors (vibration offset)\n"
    "  descent                             wait for release (free-fall) then brake & land\n"
    "  hop <feet> <bearing_deg>            ground-to-ground hop; bearing 0=forward, +90=right (clockwise)\n"
    "  log                                 dump the RAM log as CSV"));
}
static void printStatus() {
  Serial.print("armed="); Serial.print(armed); Serial.print(" cal="); Serial.print(calibrated);
  Serial.print(" cap="); Serial.print(throttleCap * 100, 0); Serial.print("% H="); Serial.print(releaseHeight, 2);
  Serial.print("m tilt="); Serial.print(flight.est.tiltDeg(), 1); Serial.print("deg mission="); Serial.print(missionActive);
  Serial.print(" log="); Serial.println(logCount);
}
static void printImu() {
  Vec3 g, a; if (!mpuRead(g, a)) { Serial.println("IMU read failed"); return; }
  Serial.print("who_am_i=0x"); Serial.print(mpuWhoAmI(), HEX);
  Serial.print(" a[m/s2]="); Serial.print(a.x, 2); Serial.print(','); Serial.print(a.y, 2); Serial.print(','); Serial.print(a.z, 2);
  Serial.print(" g[deg/s]="); Serial.print(g.x * kRad2Deg, 1); Serial.print(','); Serial.print(g.y * kRad2Deg, 1); Serial.print(','); Serial.println(g.z * kRad2Deg, 1);
}
static void dumpLog() {
  Serial.println("t,phase,px,pz,vx,vz,tilt_deg,collective,tilt_cmd_x_deg,fz,fmag");
  for (int i = 0; i < logCount; i++) {
    const Rec& r = logbuf[i];
    Serial.print(r.t, 3); Serial.print(','); Serial.print(r.phase, 0); Serial.print(','); Serial.print(r.px, 3); Serial.print(',');
    Serial.print(r.pz, 3); Serial.print(','); Serial.print(r.vx, 3); Serial.print(','); Serial.print(r.vz, 3); Serial.print(',');
    Serial.print(r.tilt, 1); Serial.print(','); Serial.print(r.u, 3); Serial.print(','); Serial.print(r.tx, 1); Serial.print(',');
    Serial.print(r.fz, 2); Serial.print(','); Serial.println(r.fmag, 2);
  }
}
static bool readyToStart() {
  if (!armed) { Serial.println("not armed"); return false; }
  if (!calibrated) { Serial.println("run 'cal' first"); return false; }
  if (missionActive || spinPhase != SP_OFF) { Serial.println("busy"); return false; }
  benchServoOn = false; return true;
}
static void beginMission() { missionActive = true; missionStartMs = millis(); logCount = 0; }

static void handleLine(char* line) {
  char* cmd = strtok(line, " \t");
  if (!cmd) return;
  if (!strcmp(cmd, "help")) printHelp();
  else if (!strcmp(cmd, "status")) printStatus();
  else if (!strcmp(cmd, "imu")) printImu();
  else if (!strcmp(cmd, "cal")) doCalibrate();
  else if (!strcmp(cmd, "arm")) {
    if (!calibrated) Serial.println("run 'cal' first"); else { armed = true; safeOutputs(); Serial.println("ARMED (motors still at minimum)"); }
  }
  else if (!strcmp(cmd, "disarm")) killAll("disarm");
  else if (!strcmp(cmd, "k") || !strcmp(cmd, "kill")) killAll("kill");
  else if (!strcmp(cmd, "cap")) {
    char* a = strtok(NULL, " \t"); if (a) { float v = (float)atof(a) / 100.0f; throttleCap = clampf(v, 0.0f, 1.0f); }
    Serial.print("cap = "); Serial.print(throttleCap * 100, 0); Serial.println(" %");
  }
  else if (!strcmp(cmd, "h")) { char* a = strtok(NULL, " \t"); if (a) releaseHeight = (float)atof(a); Serial.print("release height = "); Serial.println(releaseHeight, 3); }
  else if (!strcmp(cmd, "preset")) {
    char* a = strtok(NULL, " \t");
    if (a && !strcmp(a, "gentle")) { applyDescentPreset(P, true); Serial.println("descent preset GENTLE"); }
    else { applyDescentPreset(P, false); Serial.println("descent preset ROBUST"); }
  }
  else if (!strcmp(cmd, "servo")) {
    char* a = strtok(NULL, " \t"); char* b = strtok(NULL, " \t");
    if (a && !strcmp(a, "off")) { benchServoOn = false; Serial.println("bench servo off"); }
    else if (a && b && armed && !missionActive) { benchServoOn = true; benchTx = (float)atof(a); benchTy = (float)atof(b); Serial.println("bench servo set (clamped to tilt limit)"); }
    else Serial.println("usage: servo <x_deg> <y_deg> (armed, no mission) | servo off");
  }
  else if (!strcmp(cmd, "spin")) {
    char* a = strtok(NULL, " \t"); char* b = strtok(NULL, " \t");
    if (!a || !b) { Serial.println("usage: spin <percent> <seconds>"); return; }
    if (!readyToStart()) return;
    spinThr = clampf((float)atof(a) / 100.0f, 0.0f, throttleCap); spinDur = clampf((float)atof(b), 0.5f, 6.0f);
    spinPhase = SP_BASE; spinT0 = millis(); spinN0 = spinN1 = 0; spinSum0 = spinSum1 = spinSq1 = 0;
    Serial.print("spin test: baseline 1 s, then "); Serial.print(spinThr * 100, 0); Serial.print(" % for "); Serial.print(spinDur, 1); Serial.println(" s");
  }
  else if (!strcmp(cmd, "descent")) {
    if (!readyToStart()) return;
    flight.armDescent(releaseHeight); beginMission();
    Serial.print("DESCENT armed, waiting for free-fall. Assumed release height "); Serial.print(releaseHeight, 2); Serial.println(" m. Type 'k' to abort.");
  }
  else if (!strcmp(cmd, "hop")) {
    char* a = strtok(NULL, " \t"); char* b = strtok(NULL, " \t");
    if (!a) { Serial.println("usage: hop <feet> <bearing_deg>"); return; }
    if (!readyToStart()) return;
    float dist = (float)atof(a) * 0.3048f, brg = b ? (float)atof(b) : 0.0f;
    flight.startHop(dist, brg); beginMission();
    Serial.print("HOP "); Serial.print(dist, 3); Serial.print(" m, bearing "); Serial.print(brg, 0); Serial.println(" deg (cw from forward). Type 'k' to abort.");
  }
  else if (!strcmp(cmd, "log")) dumpLog();
  else Serial.println("unknown command (help)");
}

static void handleSerial() {
  static char buf[64]; static int n = 0;
  while (Serial.available()) {
    char ch = (char)Serial.read();
    if (ch == '\r' || ch == '\n') { if (n > 0) { buf[n] = 0; n = 0; handleLine(buf); } continue; }
    if (n == 0 && (ch == 'k' || ch == 'K')) { killAll("kill key"); continue; }   // single 'k' kills immediately
    if (n < (int)sizeof(buf) - 1) buf[n++] = ch;
  }
}

// ============================ setup / loop ============================
void setup() {
  Serial.begin(115200); delay(300);
  ESP32PWM::allocateTimer(0); ESP32PWM::allocateTimer(1); ESP32PWM::allocateTimer(2); ESP32PWM::allocateTimer(3);
  escU.setPeriodHertz(50); escL.setPeriodHertz(50); srvX.setPeriodHertz(50); srvY.setPeriodHertz(50);   // TODO servo rate
  escU.attach(PIN_ESC_UPPER, P.esc_min_us, P.esc_max_us);
  escL.attach(PIN_ESC_LOWER, P.esc_min_us, P.esc_max_us);
  srvX.attach(PIN_SERVO_X, P.servo_min_us, P.servo_max_us);
  srvY.attach(PIN_SERVO_Y, P.servo_min_us, P.servo_max_us);
  safeOutputs();
  applyDescentPreset(P, false);
  if (!mpuInit() || mpuWhoAmI() != 0x68) Serial.println("WARNING: MPU6050 not found / unexpected WHO_AM_I (check wiring, I2C address, 3.3 V)");
  Serial.println("WeebleControls ready. Motors OFF. PROPS OFF for first tests. Type 'help'.");
  lastTickUs = micros();
}

void loop() {
  handleSerial();
  uint32_t now = micros();
  if ((uint32_t)(now - lastTickUs) < (uint32_t)(1e6f / LOOP_HZ)) return;
  float dt = (now - lastTickUs) * 1e-6f; lastTickUs = now;
  dt = clampf(dt, 0.002f, 0.02f);

  Vec3 g, a;
  if (!mpuRead(g, a)) { if (++imuFail > 10 && (armed || missionActive)) killAll("IMU read failure"); return; }
  imuFail = 0;

  Command cmd = flight.update(g, a, dt);

  // ---- bench overrides (only while no mission) ----
  if (!missionActive) {
    cmd = Command();
    if (benchServoOn) { cmd.tilt_x_deg = benchTx; cmd.tilt_y_deg = benchTy; }
    if (spinPhase != SP_OFF) {
      float el = (millis() - spinT0) * 1e-3f; float fm = norm(flight.est.f_body);
      if (spinPhase == SP_BASE) {
        spinSum0 += fm; spinN0++;
        if (el >= 1.0f) { spinPhase = SP_RUN; spinT0 = millis(); }
      } else {
        cmd.motors_on = true; cmd.collective = (el < 0.5f) ? spinThr * (el / 0.5f) : spinThr;   // 0.5 s ramp
        if (el > 0.7f) { spinSum1 += fm; spinSq1 += (double)fm * fm; spinN1++; }                  // skip the ramp
        if (el >= spinDur) {
          double m0 = spinSum0 / max(spinN0, 1), m1 = spinSum1 / max(spinN1, 1);
          double sd = sqrt(max(0.0, spinSq1 / max(spinN1, 1) - m1 * m1));
          Serial.print("SPIN RESULT: mean |f| baseline "); Serial.print(m0, 3); Serial.print(" -> spinning "); Serial.print(m1, 3);
          Serial.print("  shift = "); Serial.print(m1 - m0, 3); Serial.print(" m/s^2, std "); Serial.print(sd, 3); Serial.println(" m/s^2");
          spinPhase = SP_OFF; cmd = Command();
        }
      }
    }
  }
  if (!armed) cmd = Command();

  // ---- mission supervision ----
  if (missionActive) {
    bool done = false; const char* why = "";
    if (flight.mode == Flight::Mode::DESCENT) {
      DState s = flight.desc.state();
      if (s == DState::LANDED) { done = true; why = "descent LANDED (touchdown detected / height reached)"; }
      else if (s == DState::FAULT) { done = true; why = "descent FAULT (timeout or not enough thrust to brake)"; }
    } else if (flight.mode == Flight::Mode::HOP) {
      HState s = flight.hop.state();
      if (s == HState::DONE) { done = true; why = "hop DONE"; }
      else if (s == HState::FAULT) {
        done = true;
        switch (flight.hop.fault_code) {
          case 1: why = "hop FAULT: not enough thrust to hop at this lean"; break;
          case 2: why = "hop FAULT: drone not at rest"; break;
          case 3: why = "hop FAULT: could not tip to the lean angle"; break;
          default: why = "hop FAULT"; }
      }
    }
    if ((millis() - missionStartMs) * 1e-3f > MAX_MISSION_S) { done = true; why = "mission time limit"; }
    if (done) {
      safeOutputs(); flight.stopAll(); missionActive = false; cmd = Command();
      Serial.print("MISSION END: "); Serial.println(why);
      if (flight.mode == Flight::Mode::HOP) { Serial.print("estimated final position [m]: "); Serial.print(flight.est.pos.x, 2); Serial.print(", "); Serial.println(flight.est.pos.y, 2); }
      Serial.println("Type 'log' to dump data; measure the real landing point with a tape measure.");
    }
  }

  // ---- outputs ----
  if (armed) writeOutputs(mix(cmd, P, throttleCap)); else safeOutputs();

  // ---- logging at 100 Hz ----
  static uint8_t dec = 0;
  if (missionActive && (dec++ & 1) == 0 && logCount < LOG_N) {
    Rec& r = logbuf[logCount++];
    r.t = (millis() - missionStartMs) * 1e-3f;
    r.phase = flight.mode == Flight::Mode::HOP ? (float)(int)flight.hop.state() : (float)(int)flight.desc.state();
    r.px = flight.est.pos.x; r.pz = flight.est.pos.z; r.vx = flight.est.vel.x; r.vz = flight.est.vel.z;
    r.tilt = flight.est.tiltDeg(); r.u = cmd.collective; r.tx = cmd.tilt_x_deg;
    r.fz = flight.est.f_body.z; r.fmag = norm(flight.est.f_body);
  }
}
