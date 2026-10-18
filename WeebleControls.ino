// WeebleControls.ino -- Phase 1 glue: ESP32 (Arduino core) <-> weeble_core.h      (v2)
//
// STATUS: logic tested on a laptop against an ASSUMED physics model. This file was syntax-checked against STUB
// headers only: it has NOT been compiled for an ESP32 and NOT run on hardware. First power-up MUST be PROPS OFF.
//
// Everything marked TODO must be checked against the real wiring / parts before anything spins:
//   GPIO pins, IMU orientation (IMU_AXIS / IMU_SIGN), ESC type & calibration, servo direction and us-per-degree,
//   safety-lock hardware (if any), battery divider (if any).
//
// Needs: Arduino-ESP32 core (Preferences and BluetoothSerial are part of it), library "ESP32Servo".
// Serial monitor: 115200 baud, line ending "Newline". Type "help".
//
// OPTIONAL WIRELESS LINK: set WEEBLE_BT to 1 to accept the same commands over Bluetooth Classic serial (device name
// "Weeble"). Only works on ESP32 chips with Bluetooth Classic (original ESP32 / ESP32-WROOM / -WROVER), NOT on
// ESP32-S2/S3/C3. A drop test cannot use a USB cable, so some wireless link is needed for 'descent' and 'k'.
// UNTESTED: check kill latency and range before relying on it.
#ifndef WEEBLE_BT
#define WEEBLE_BT 0
#endif

#include <Arduino.h>
#include <Wire.h>
#include <ESP32Servo.h>
#include <Preferences.h>
#if WEEBLE_BT
#include <BluetoothSerial.h>
static BluetoothSerial SerialBT;
#endif
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
// TODO: set from how the GY-521 is mounted. Check with "imu": tilt the drone nose-down -> body ax goes positive.
static const int   IMU_AXIS[3] = {0, 1, 2};
static const float IMU_SIGN[3] = {1, 1, 1};

// Safety lock. The CSV requires "safety lock disengaged" before arming but does not say what the lock IS.
// Implemented: a SOFTWARE lock ('unlock' / 'lock'), engaged at boot and re-engaged every time the drone returns to Rest
// or Fault. Optional HARDWARE switch: set PIN_SAFETY to its GPIO (input, internal pull-up) and SAFETY_RELEASED_LEVEL.
static const int PIN_SAFETY = -1;              // -1 = no hardware switch. TODO if you have one.
static const int SAFETY_RELEASED_LEVEL = LOW;  // TODO: level read when the switch is in the "released / fly" position

// Battery monitor (CSV: "battery OK (if monitored)"). Disabled until the divider exists.
static const int   PIN_BATT = -1;              // -1 = not monitored. TODO: ADC1 pin (e.g. 34/35/36/39) if a divider is fitted
static const float BATT_DIVIDER = 0.0f;        // TODO: (R1+R2)/R2 of the divider
static const float BATT_MIN_V = 0.0f;          // TODO: refuse to arm below this pack voltage (depends on cell count)

static const float LOOP_HZ = 200.0f;
static const float DEFAULT_CAP = 0.15f;        // throttle cap at boot (15 %) -- raise deliberately with "cap"

// ============================ output stream (USB serial + optional Bluetooth) ============================
class TeeOut : public Print {
 public:
  size_t write(uint8_t c) override {
    Serial.write(c);
#if WEEBLE_BT
    if (SerialBT.hasClient()) SerialBT.write(c);
#endif
    return 1;
  }
  size_t write(const uint8_t* b, size_t n) override {
    Serial.write(b, n);
#if WEEBLE_BT
    if (SerialBT.hasClient()) SerialBT.write(b, n);
#endif
    return n;
  }
};
static TeeOut out;

// ============================ globals ============================
static Params P;
static Flight flight(P);
static Servo escU, escL, srvX, srvY;
static Preferences prefs;
static AccelCal acal;                          // six-position calibration (sensor frame), loaded from flash

static bool softLocked = true;
static float releaseHeight = 6.096f;           // 20 ft. MEASURE (shell bottom to ground) and set with "h"
static bool presetGentle = false;
static uint32_t lastTickUs = 0;
static int imuFail = 0;
static float imuTempC = 0;
static Macro lastMacro = Macro::REST;

// bench helpers
static bool benchServoOn = false; static float benchTx = 0, benchTy = 0;
enum SpinPhase { SP_OFF, SP_BASE, SP_RUN };
static SpinPhase spinPhase = SP_OFF; static float spinThr = 0; static uint32_t spinT0 = 0; static float spinDur = 0;
static double spinSum0 = 0, spinSq0 = 0, spinSum1 = 0, spinSq1 = 0; static int spinN0 = 0, spinN1 = 0;

// six-position calibration poses
static const int CAL6_MAX = 12;
static Vec3 cal6Pose[CAL6_MAX]; static int cal6N = 0;

// logging: RAM ring buffer at 100 Hz, fixed-point to save RAM (32 bytes/row)
struct Rec {
  uint16_t t10;            // time since command [10 ms]
  uint8_t macro, sub;
  int16_t px, py, pz;      // [mm]
  int16_t vx, vy, vz;      // [mm/s]
  int16_t tilt;            // [0.1 deg]
  uint16_t u;              // throttle [1e-4]
  int16_t podx, pody;      // commanded pod tilt [0.1 deg]
  int16_t fz, fmag;        // specific force [0.01 m/s^2]
  int16_t aux;             // hop: predicted range [mm]; vhop: predicted apex [mm]; descent: v_ref [mm/s]
  int16_t temp;            // IMU temperature [0.01 C]
};
static const int LOG_N = 1800;                 // 18 s at 100 Hz, 57.6 kB. Ring: always keeps the LAST 18 s.
static Rec logbuf[LOG_N]; static int logHead = 0, logCount = 0;
static uint32_t missionStartMs = 0;

static int16_t q16(float v, float scale) { float x = v * scale; if (x > 32767.f) x = 32767.f; if (x < -32768.f) x = -32768.f; return (int16_t)lrintf(x); }

// ============================ MPU6050 (raw registers) ============================
static bool mpuWrite(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_ADDR); Wire.write(reg); Wire.write(val);
  return Wire.endTransmission() == 0;
}
static bool mpuInit() {
  Wire.begin(PIN_SDA, PIN_SCL); Wire.setClock(400000);
  bool ok = mpuWrite(0x6B, 0x00);   // PWR_MGMT_1: wake up
  delay(100);
  ok &= mpuWrite(0x1A, 0x02);       // CONFIG: DLPF ~94 Hz accel / 98 Hz gyro
  ok &= mpuWrite(0x19, 0x00);       // SMPLRT_DIV
  ok &= mpuWrite(0x1B, 0x10);       // GYRO_CONFIG: +-1000 deg/s   (32.8 LSB per deg/s)
  ok &= mpuWrite(0x1C, 0x10);       // ACCEL_CONFIG: +-8 g         (4096 LSB per g)  -- hard impacts may SATURATE at 8 g
  return ok;
}
static uint8_t mpuWhoAmI() {
  Wire.beginTransmission(MPU_ADDR); Wire.write(0x75); Wire.endTransmission(false);
  Wire.requestFrom((int)MPU_ADDR, 1); return Wire.available() ? Wire.read() : 0xFF;
}
// Sensor-frame reading (no calibration, no axis mapping). a [m/s^2], g [rad/s], temperature [C].
static bool mpuReadSensor(float a[3], float g[3], float& tempC) {
  Wire.beginTransmission(MPU_ADDR); Wire.write(0x3B);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((int)MPU_ADDR, 14) != 14) return false;
  uint8_t b[14]; for (int i = 0; i < 14; i++) b[i] = Wire.read();
  auto s16 = [&](int i) { return (int16_t)((b[i] << 8) | b[i + 1]); };
  for (int i = 0; i < 3; i++) { a[i] = s16(2 * i) / 4096.0f * kG; g[i] = s16(8 + 2 * i) / 32.8f * kDeg2Rad; }
  tempC = s16(6) / 340.0f + 36.53f;   // MPU-6050 register map formula
  return true;
}
// Body-frame reading: six-position calibration applied in the sensor frame, then axis mapping.
static bool mpuRead(Vec3& gyro_rads, Vec3& acc_ms2) {
  float a[3], g[3];
  if (!mpuReadSensor(a, g, imuTempC)) return false;
  applyAccelCal(acal, a);
  acc_ms2   = Vec3{IMU_SIGN[0] * a[IMU_AXIS[0]], IMU_SIGN[1] * a[IMU_AXIS[1]], IMU_SIGN[2] * a[IMU_AXIS[2]]};
  gyro_rads = Vec3{IMU_SIGN[0] * g[IMU_AXIS[0]], IMU_SIGN[1] * g[IMU_AXIS[1]], IMU_SIGN[2] * g[IMU_AXIS[2]]};
  return true;
}

// ============================ outputs / safety ============================
static void writeOutputs(const Outputs& o) {
  escU.writeMicroseconds(o.esc_upper_us); escL.writeMicroseconds(o.esc_lower_us);
  srvX.writeMicroseconds(o.servo_x_us);   srvY.writeMicroseconds(o.servo_y_us);
}
static void safeOutputs() {
  Command c; writeOutputs(mix(c, P));   // motors off -> ESC minimum, servos centred
}
static void stopBench() { spinPhase = SP_OFF; benchServoOn = false; }
static void killAll(const char* why) {
  safeOutputs(); stopBench(); flight.kill(why);
  out.print("KILL -> Fault: "); out.println(why);
}
static bool hwLockReleased() { return PIN_SAFETY < 0 || digitalRead(PIN_SAFETY) == SAFETY_RELEASED_LEVEL; }
static float batteryVolts() { return (PIN_BATT < 0 || BATT_DIVIDER <= 0) ? -1.0f : analogReadMilliVolts(PIN_BATT) * 1e-3f * BATT_DIVIDER; }

// ============================ calibration storage ============================
static void loadAccelCal() {
  AccelCal c; prefs.begin("weeble", true);
  size_t n = prefs.getBytes("acal", &c, sizeof(c)); prefs.end();
  if (n == sizeof(c) && c.magic == AccelCal().magic && c.valid) { acal = c; out.println("six-position accel calibration loaded from flash"); }
  else { acal = AccelCal(); acal.valid = false; out.println("no stored six-position accel calibration (run 'cal6' -- recommended)"); }
}
static void saveAccelCal() { prefs.begin("weeble", false); prefs.putBytes("acal", &acal, sizeof(acal)); prefs.end(); }
static void clearAccelCal() { prefs.begin("weeble", false); prefs.remove("acal"); prefs.end(); acal = AccelCal(); acal.valid = false; }
static void printAccelCal() {
  if (!acal.valid) { out.println("accel cal: none"); return; }
  out.print("accel cal (sensor frame): offset [m/s^2] "); for (int i = 0; i < 3; i++) { out.print(acal.off[i], 3); out.print(' '); }
  out.print(" scale "); for (int i = 0; i < 3; i++) { out.print(acal.scl[i], 4); out.print(' '); }
  out.print(" rms "); out.print(acal.rms, 3); out.print(" ("); out.print(acal.n_poses); out.println(" poses)");
}

// Average the sensor for `ms` milliseconds at 200 Hz. Returns false on read failure.
static bool averageSensor(int ms, Vec3& amean, Vec3& astd, Vec3& gmean, Vec3& gstd, bool body) {
  int N = ms / 5; double sa[3] = {0}, qa[3] = {0}, sg[3] = {0}, qg[3] = {0};
  for (int i = 0; i < N; i++) {
    uint32_t t0 = micros(); float a[3], g[3];
    if (body) { Vec3 gv, av; if (!mpuRead(gv, av)) return false; a[0] = av.x; a[1] = av.y; a[2] = av.z; g[0] = gv.x; g[1] = gv.y; g[2] = gv.z; }
    else if (!mpuReadSensor(a, g, imuTempC)) return false;
    for (int k = 0; k < 3; k++) { sa[k] += a[k]; qa[k] += (double)a[k] * a[k]; sg[k] += g[k]; qg[k] += (double)g[k] * g[k]; }
    while (micros() - t0 < 5000) {}
  }
  float* am = &amean.x; float* as = &astd.x; float* gm = &gmean.x; float* gsd = &gstd.x;
  for (int k = 0; k < 3; k++) {
    am[k] = sa[k] / N; as[k] = sqrt(fmax(0.0, qa[k] / N - (sa[k] / N) * (sa[k] / N)));
    gm[k] = sg[k] / N; gsd[k] = sqrt(fmax(0.0, qg[k] / N - (sg[k] / N) * (sg[k] / N)));
  }
  return true;
}

// 'cal': at-rest calibration (CSV Calibrate IMU). Rest only. Sets gravity direction + gyro bias.
static void doCalibrate() {
  if (flight.macro() != Macro::REST) { out.println("cal only in Rest (disarm first)"); return; }
  out.println("Calibrating: keep the drone STILL and UPRIGHT for 2 s ...");
  Vec3 am, as, gm, gsd;
  if (!averageSensor(2000, am, as, gm, gsd, true)) { out.println("IMU read failed"); flight.calibrated = false; return; }
  float amag = norm(am), gstdMax = fmaxf(gsd.x, fmaxf(gsd.y, gsd.z)), astdMax = fmaxf(as.x, fmaxf(as.y, as.z));
  flight.est.initAtRest(am);
  flight.est.setGyroBias(gm);
  float tilt = flight.est.tiltDeg();
  out.print("gyro bias [rad/s]: "); out.print(gm.x, 4); out.print(' '); out.print(gm.y, 4); out.print(' '); out.println(gm.z, 4);
  out.print("mean accel [m/s^2]: "); out.print(am.x, 3); out.print(' '); out.print(am.y, 3); out.print(' '); out.print(am.z, 3);
  out.print("  |a|="); out.print(amag, 3); out.print("  tilt="); out.print(tilt, 1); out.print(" deg  gyro std="); out.print(gstdMax, 4);
  out.print(" accel std="); out.print(astdMax, 3); out.print("  T="); out.print(imuTempC, 1); out.println(" C");
  bool ok = true;
  if (fabsf(amag - kG) > P.cal_max_amag_err) { out.println("FAIL: |a| far from 9.81 -> moving, or accelerometer badly off (try 'cal6')."); ok = false; }
  if (gstdMax > P.cal_max_gyro_std) { out.println("FAIL: gyro noisy -> drone was not still."); ok = false; }
  if (astdMax > P.cal_max_acc_std) { out.println("FAIL: accelerometer noisy -> drone was not still."); ok = false; }
  if (tilt > P.cal_max_tilt_deg) { out.println("FAIL: drone tilted > 10 deg."); ok = false; }
  if (!acal.valid) out.println("note: no six-position accel calibration stored ('cal6'); offsets only corrected along gravity.");
  flight.calibrated = ok;
  out.println(ok ? "CALIBRATION OK" : "CALIBRATION NOT VALID -- arming blocked. Fix and repeat 'cal'.");
}

// ============================ text helpers ============================
static float argF(const char* s, float dflt) { return s ? (float)atof(s) : dflt; }
// "key=value" lookup among the remaining strtok tokens (collected by the caller)
static const char* kv(char** toks, int n, const char* key) {
  size_t L = strlen(key);
  for (int i = 0; i < n; i++) if (!strncmp(toks[i], key, L) && toks[i][L] == '=') return toks[i] + L + 1;
  return nullptr;
}
static void printTwr() {
  out.print("mass "); out.print(P.mass_kg, 3); out.print(" kg (base "); out.print(P.base_mass_kg, 3); out.print(" + payload "); out.print(P.payload_kg, 3);
  out.print(")  TWR at cap "); out.print(twrAvail(P), 2); out.print("  spare braking decel at cap "); out.print(spareDecel(P), 2);
  out.print(" m/s^2  (TWR at 100 %: "); out.print(P.thrust_max_N / (P.mass_kg * kG), 2); out.println(", thrust value is a PLACEHOLDER until the thrust stand)");
}

// ============================ serial commands ============================
static void printHelp() {
  out.println(F(
    "Commands (Rest = disarmed, Armed = motors at minimum):\n"
    "  help | status | imu                 info ('imu' prints one live reading)\n"
    "  cal                                 2 s at-rest calibration (still, upright). Rest only. Required before arming.\n"
    "  cal6 start|pose|solve|show|clear    six-position accel calibration, stored in flash (Rest only). 'cal6 pose' once\n"
    "                                      per orientation; every axis up AND down; 8-12 poses; then 'cal6 solve'.\n"
    "  unlock | lock                       software safety lock (engaged at boot and after every Rest/Fault)\n"
    "  arm | disarm | k                    'k' (or 'kill') = instant stop -> Fault. 'disarm' leaves Fault -> Rest.\n"
    "  cap <percent>                       throttle cap, boot 15. Feasibility checks use thrust AT THE CAP.\n"
    "  payload <kg>                        payload mass (base drone 1.690 kg from CDR). Weigh it.\n"
    "  h <metres>                          release height = shell bottom to ground. MEASURE it.\n"
    "  preset robust|gentle                descent tuning (gentle only after a clean 'spin' test)\n"
    "  servo <x_deg> <y_deg> | servo off   bench (Armed, props off): move pod\n"
    "  spin <percent> <seconds>            bench (Armed, RESTRAINED, props off/guarded): accel shift caused by motors\n"
    "  descent                             Controlled Landing: wait for release (free fall), brake, land\n"
    "  hop <feet> [bearing] [lean=deg] [land=ballistic|brake] [scale=x]\n"
    "                                      horizontal hop; bearing deg clockwise from forward (default 0), lean default 15\n"
    "  vhop <feet> [land=brake|ballistic]  vertical hop to an apex height (landing brake default)\n"
    "  log                                 dump the RAM log (last 18 s) as CSV"));
}
static void printStatus() {
  out.print("state="); out.print(macroName(flight.macro())); out.print(" routine="); out.print(routineName(flight.routine));
  out.print(" cal="); out.print(flight.calibrated ? "ok" : "NO"); out.print(" cal6="); out.print(acal.valid ? "stored" : "none");
  out.print(" lock="); out.print(softLocked ? "LOCKED" : "unlocked"); if (PIN_SAFETY >= 0) { out.print(" hwlock="); out.print(hwLockReleased() ? "released" : "ENGAGED"); }
  out.print(" cap="); out.print(P.throttle_cap * 100, 0); out.print("% H="); out.print(releaseHeight, 2); out.print("m preset="); out.print(presetGentle ? "GENTLE" : "ROBUST");
  out.print(" tilt="); out.print(flight.est.tiltDeg(), 1); out.print("deg T="); out.print(imuTempC, 1); out.print("C");
  float bv = batteryVolts(); out.print(" batt="); if (bv < 0) out.print("not monitored"); else { out.print(bv, 2); out.print("V"); }
  out.print(" log="); out.println(logCount);
  printTwr();
}
static void printImu() {
  float a[3], g[3]; if (!mpuReadSensor(a, g, imuTempC)) { out.println("IMU read failed"); return; }
  Vec3 gb, ab; mpuRead(gb, ab);
  out.print("who_am_i=0x"); out.print(mpuWhoAmI(), HEX);
  out.print(" body a[m/s2]="); out.print(ab.x, 2); out.print(','); out.print(ab.y, 2); out.print(','); out.print(ab.z, 2);
  out.print(" g[deg/s]="); out.print(gb.x * kRad2Deg, 1); out.print(','); out.print(gb.y * kRad2Deg, 1); out.print(','); out.print(gb.z * kRad2Deg, 1);
  out.print(" | sensor raw a="); out.print(a[0], 2); out.print(','); out.print(a[1], 2); out.print(','); out.print(a[2], 2);
  out.print(" T="); out.print(imuTempC, 1); out.println("C");
}
static void dumpLog() {
  out.println("t,macro,sub,px,py,pz,vx,vy,vz,tilt_deg,throttle,pod_x_deg,pod_y_deg,fz,fmag,aux,temp_c");
  out.println("# macro: 0 Rest 1 Armed 2 AwaitRelease 3 Launch 4 AmpUp 5 Ascending 6 Descending 7 Landing 8 AmpDown 9 Fault");
  out.println("# aux: hop = predicted landing distance [m], vhop = predicted apex [m], descent = v_ref [m/s]");
  int start = (logHead - logCount + LOG_N) % LOG_N;
  for (int k = 0; k < logCount; k++) {
    const Rec& r = logbuf[(start + k) % LOG_N];
    out.print(r.t10 * 0.01f, 2); out.print(','); out.print(r.macro); out.print(','); out.print(r.sub); out.print(',');
    out.print(r.px * 1e-3f, 3); out.print(','); out.print(r.py * 1e-3f, 3); out.print(','); out.print(r.pz * 1e-3f, 3); out.print(',');
    out.print(r.vx * 1e-3f, 3); out.print(','); out.print(r.vy * 1e-3f, 3); out.print(','); out.print(r.vz * 1e-3f, 3); out.print(',');
    out.print(r.tilt * 0.1f, 1); out.print(','); out.print(r.u * 1e-4f, 4); out.print(','); out.print(r.podx * 0.1f, 1); out.print(',');
    out.print(r.pody * 0.1f, 1); out.print(','); out.print(r.fz * 0.01f, 2); out.print(','); out.print(r.fmag * 0.01f, 2); out.print(',');
    out.print(r.aux * 1e-3f, 3); out.print(','); out.println(r.temp * 0.01f, 2);
  }
}
static void printSummary() {
  out.println("---- mission summary (IMU estimates: VERIFY with tape measure / video) ----");
  out.print("routine "); out.print(routineName(flight.last_routine)); out.print(", end: "); out.println(flight.end_reason);
  out.print("peak |specific force| "); out.print(flight.peak_f / kG, 2); out.println(" g (accelerometer saturates at 8 g)");
  switch (flight.last_routine) {
    case Routine::DESCENT: {
      const DescentController& d = flight.desc;
      out.print("release height entered "); out.print(flight.H0, 3); out.print(" m; touchdown est. vertical speed "); out.print(-flight.td_vz, 2);
      out.print(" m/s, tilt "); out.print(flight.td_tilt, 1); out.println(" deg");
      if (flight.td_by_height) out.println("WARNING: touchdown declared by the height backstop (no impact sensed) -> motors may have run on the ground");
      if (d.zg_measured) {
        out.print("zero-g window mean (body, after stored cal) [m/s^2]: "); out.print(d.zg_mean.x, 3); out.print(' '); out.print(d.zg_mean.y, 3); out.print(' '); out.print(d.zg_mean.z, 3);
        out.print("  |.|="); out.print(norm(d.zg_mean), 3); out.print("  accepted scale "); out.println(flight.est.accel_scale, 4);
        out.print("at-rest bias along gravity (from 'cal') [m/s^2]: "); out.print(flight.est.rest_bias.x, 3); out.print(' '); out.print(flight.est.rest_bias.y, 3); out.print(' '); out.println(flight.est.rest_bias.z, 3);
      }
      printAccelCal();
      break;
    }
    case Routine::HOP: {
      const HopController& h = flight.hop;
      out.print("target "); out.print(h.A.dist_m / 0.3048f, 2); out.print(" ft at bearing "); out.print(h.A.bearing_deg, 0); out.print(" deg, lean "); out.print(h.A.lean_deg, 0);
      out.print(" deg, scale "); out.print(h.A.range_scale, 3); out.println(h.A.brake ? ", brake landing" : ", ballistic landing");
      out.print("burn "); out.print(h.burn_s, 3); out.print(" s; at cut predicted landing "); out.print(h.cut_pred_along / 0.3048f, 2); out.print(" ft along, ");
      out.print(h.cut_pred_lat / 0.3048f, 2); out.print(" ft left of line"); if (h.cut_by_maxburn) out.print(" [cut by max burn]"); if (h.cut_by_tilt) out.print(" [cut by tilt guard]"); out.println();
      out.print("liftoff "); out.print(h.airborne ? "detected" : "NOT detected"); out.print("; est. apex "); out.print(h.apex_z, 3); out.println(" m");
      if (h.touched) {
        out.print("first touch est. "); out.print(h.touch_along / 0.3048f, 2); out.print(" ft along, "); out.print(h.touch_lat / 0.3048f, 2);
        out.print(" ft left; est. touch speed horiz "); out.print(sqrtf(h.touch_vel.x * h.touch_vel.x + h.touch_vel.y * h.touch_vel.y), 2); out.print(" vert "); out.print(-h.touch_vel.z, 2); out.println(" m/s");
      } else out.println("no touchdown recorded");
      out.print("est. final rest position [m] x "); out.print(flight.final_pos.x, 2); out.print(" y "); out.print(flight.final_pos.y, 2); out.println(flight.final_rested ? "" : " (did not come to rest in time)");
      break;
    }
    case Routine::VHOP: {
      const VHopController& v = flight.vhop;
      out.print("target apex "); out.print(v.A.height_m, 3); out.print(" m ("); out.print(v.A.height_m / 0.3048f, 2); out.print(" ft)"); out.println(v.A.brake ? ", brake landing" : ", ballistic landing");
      out.print("burn "); out.print(v.burn_s, 3); out.print(" s; predicted apex at cut "); out.print(v.cut_pred_apex, 3); out.print(" m; est. apex "); out.print(v.apex_z, 3); out.println(" m");
      if (v.cut_by_maxburn) out.println("cut by burn time limit");
      if (v.cut_by_tilt) out.println("cut by tilt guard");
      out.print("touchdown est. vertical speed "); out.print(-flight.td_vz, 2); out.print(" m/s, tilt "); out.print(flight.td_tilt, 1); out.println(" deg");
      if (flight.td_by_height) out.println("WARNING: touchdown declared by the height backstop (no impact sensed)");
      break;
    }
    default: break;
  }
  out.println("Type 'log' to dump data.");
}

static bool canChangeSetup() {
  if (flight.macro() == Macro::REST || flight.idleArmed()) return true;
  out.println("not while a routine is running"); return false;
}
static bool readyToStart() {
  if (!flight.idleArmed()) { out.print("must be Armed and idle (state "); out.print(macroName(flight.macro())); out.println(")"); return false; }
  if (spinPhase != SP_OFF) { out.println("bench spin running"); return false; }
  stopBench(); return true;
}
static void beginLog() { logHead = 0; logCount = 0; missionStartMs = millis(); }

static void cmdCal6(const char* sub) {
  if (flight.macro() != Macro::REST) { out.println("cal6 only in Rest"); return; }
  if (!sub || !strcmp(sub, "show")) { printAccelCal(); out.print(cal6N); out.println(" poses collected"); return; }
  if (!strcmp(sub, "start")) { cal6N = 0; out.println("cal6: collecting. Hold the drone (or bare IMU) still in an orientation, type 'cal6 pose'. Repeat with every axis up and down (8-12 poses)."); return; }
  if (!strcmp(sub, "clear")) { clearAccelCal(); flight.calibrated = false; out.println("stored accel calibration erased; run 'cal' again"); return; }
  if (!strcmp(sub, "pose")) {
    if (cal6N >= CAL6_MAX) { out.println("max poses reached; 'cal6 solve'"); return; }
    Vec3 am, as, gm, gsd;
    if (!averageSensor(2000, am, as, gm, gsd, false)) { out.println("IMU read failed"); return; }
    if (fmaxf(as.x, fmaxf(as.y, as.z)) > P.cal_max_acc_std || fmaxf(gsd.x, fmaxf(gsd.y, gsd.z)) > P.cal_max_gyro_std) { out.println("REJECTED: moved during the pose, try again"); return; }
    cal6Pose[cal6N++] = am;
    out.print("pose "); out.print(cal6N); out.print(" raw mean "); out.print(am.x, 3); out.print(' '); out.print(am.y, 3); out.print(' '); out.println(am.z, 3);
    return;
  }
  if (!strcmp(sub, "solve")) {
    AccelCal c; const char* why = "";
    if (!solveAccelCal(cal6Pose, cal6N, c, why)) { out.print("cal6 REFUSED: "); out.println(why); return; }
    acal = c; saveAccelCal(); flight.calibrated = false;
    printAccelCal(); out.println("saved to flash. Run 'cal' again (the at-rest calibration depends on it).");
    return;
  }
  out.println("usage: cal6 start|pose|solve|show|clear");
}

static void handleLine(char* line) {
  char* toks[8]; int nt = 0;
  for (char* t = strtok(line, " \t"); t && nt < 8; t = strtok(NULL, " \t")) toks[nt++] = t;
  if (nt == 0) return;
  const char* cmd = toks[0]; char** rest = toks + 1; int nr = nt - 1;
  const char* a1 = nr > 0 ? rest[0] : nullptr; const char* a2 = nr > 1 ? rest[1] : nullptr;

  if (!strcmp(cmd, "help")) printHelp();
  else if (!strcmp(cmd, "status")) printStatus();
  else if (!strcmp(cmd, "imu")) printImu();
  else if (!strcmp(cmd, "cal")) doCalibrate();
  else if (!strcmp(cmd, "cal6")) cmdCal6(a1);
  else if (!strcmp(cmd, "unlock")) { softLocked = false; out.println("software safety lock released"); }
  else if (!strcmp(cmd, "lock")) { softLocked = true; out.println("software safety lock engaged"); if (flight.idleArmed()) { flight.disarm(); out.println("disarmed"); } }
  else if (!strcmp(cmd, "arm")) {
    if (softLocked) { out.println("refused: safety lock engaged ('unlock')"); return; }
    if (!hwLockReleased()) { out.println("refused: hardware safety switch engaged"); return; }
    float bv = batteryVolts(); if (bv >= 0 && bv < BATT_MIN_V) { out.print("refused: battery low "); out.println(bv, 2); return; }
    const char* why = "";
    if (flight.arm(why)) { safeOutputs(); out.println("ARMED (motors at minimum)"); printTwr(); } else { out.print("refused: "); out.println(why); }
  }
  else if (!strcmp(cmd, "disarm")) { safeOutputs(); stopBench(); flight.disarm(); out.println("disarmed -> Rest"); }
  else if (!strcmp(cmd, "k") || !strcmp(cmd, "kill")) killAll("kill command");
  else if (!strcmp(cmd, "cap")) {
    if (a1 && canChangeSetup()) P.throttle_cap = clampf(argF(a1, P.throttle_cap * 100) / 100.0f, 0.0f, 1.0f);
    out.print("cap = "); out.print(P.throttle_cap * 100, 0); out.println(" %"); printTwr();
  }
  else if (!strcmp(cmd, "payload")) {
    if (a1 && canChangeSetup()) { float kg = argF(a1, P.payload_kg); if (kg < 0) out.println("payload must be >= 0"); else setPayload(P, kg); }
    out.print("payload = "); out.print(P.payload_kg, 3); out.println(" kg (CG / inertia shift NOT modelled)"); printTwr();
  }
  else if (!strcmp(cmd, "h")) {
    if (a1 && canChangeSetup()) releaseHeight = argF(a1, releaseHeight);
    out.print("release height = "); out.print(releaseHeight, 3); out.print(" m ("); out.print(releaseHeight / 0.3048f, 2); out.println(" ft), shell bottom to ground");
  }
  else if (!strcmp(cmd, "preset")) {
    if (!canChangeSetup()) return;
    presetGentle = a1 && !strcmp(a1, "gentle"); applyDescentPreset(P, presetGentle);
    out.println(presetGentle ? "descent preset GENTLE (only if the 'spin' shift is near zero)" : "descent preset ROBUST");
  }
  else if (!strcmp(cmd, "servo")) {
    if (a1 && !strcmp(a1, "off")) { benchServoOn = false; out.println("bench servo off"); }
    else if (a1 && a2 && flight.idleArmed()) { benchServoOn = true; benchTx = argF(a1, 0); benchTy = argF(a2, 0); out.println("bench servo set (clamped to tilt limit)"); }
    else out.println("usage: servo <x_deg> <y_deg> (Armed, idle) | servo off");
  }
  else if (!strcmp(cmd, "spin")) {
    if (!a1 || !a2) { out.println("usage: spin <percent> <seconds>"); return; }
    if (!readyToStart()) return;
    spinThr = clampf(argF(a1, 0) / 100.0f, 0.0f, P.throttle_cap); spinDur = clampf(argF(a2, 1), 0.5f, 6.0f);
    spinPhase = SP_BASE; spinT0 = millis(); spinN0 = spinN1 = 0; spinSum0 = spinSq0 = spinSum1 = spinSq1 = 0;
    out.print("spin test: baseline 1 s, then "); out.print(spinThr * 100, 0); out.print(" % for "); out.print(spinDur, 1); out.println(" s");
  }
  else if (!strcmp(cmd, "descent")) {
    if (!readyToStart()) return;
    const char* why = "";
    beginLog();
    if (!flight.startDescent(releaseHeight, why)) { out.print("descent refused: "); out.println(why); return; }
    out.print("DESCENT armed, waiting for free fall. Release height "); out.print(releaseHeight, 2); out.print(" m, preset "); out.print(presetGentle ? "GENTLE" : "ROBUST");
    out.print(", spare decel "); out.print(spareDecel(P), 2); out.println(" m/s^2. Type 'k' to abort.");
    out.print("NOTE: if not released within "); out.print(P.await_release_timeout_s, 0); out.println(" s the drone goes to Fault (disarmed) and a later release would NOT be braked.");
  }
  else if (!strcmp(cmd, "hop")) {
    if (!a1) { out.println("usage: hop <feet> [bearing_deg] [lean=deg] [land=ballistic|brake] [scale=x]"); return; }
    if (!readyToStart()) return;
    HopArgs h; h.dist_m = argF(a1, 5) * 0.3048f; h.lean_deg = P.hop_lean_deg; h.range_scale = P.hop_range_scale; h.brake = P.hop_land_brake;
    if (a2 && !strchr(a2, '=')) h.bearing_deg = argF(a2, 0);
    if (const char* v = kv(rest, nr, "brg")) h.bearing_deg = argF(v, 0);
    if (const char* v = kv(rest, nr, "lean")) h.lean_deg = argF(v, h.lean_deg);
    if (const char* v = kv(rest, nr, "scale")) h.range_scale = argF(v, 1);
    if (const char* v = kv(rest, nr, "land")) h.brake = !strcmp(v, "brake");
    const char* why = "";
    beginLog();
    if (!flight.startHop(h, why)) { out.print("hop refused: "); out.println(why); printTwr(); return; }
    out.print("HOP "); out.print(h.dist_m / 0.3048f, 2); out.print(" ft, bearing "); out.print(h.bearing_deg, 0); out.print(" deg cw, lean "); out.print(h.lean_deg, 0);
    out.print(" deg, "); out.print(h.brake ? "brake" : "ballistic"); out.println(" landing. Type 'k' to abort.");
  }
  else if (!strcmp(cmd, "vhop")) {
    if (!a1) { out.println("usage: vhop <feet> [land=brake|ballistic]"); return; }
    if (!readyToStart()) return;
    VHopArgs v; v.height_m = argF(a1, 1) * 0.3048f; v.brake = true;
    if (const char* s = kv(rest, nr, "land")) v.brake = strcmp(s, "ballistic") != 0;
    const char* why = "";
    beginLog();
    if (!flight.startVHop(v, why)) { out.print("vhop refused: "); out.println(why); out.print("(max height "); out.print(P.vhop_max_height_m / 0.3048f, 2); out.println(" ft)"); printTwr(); return; }
    out.print("VHOP to "); out.print(v.height_m, 2); out.print(" m, "); out.print(v.brake ? "brake" : "ballistic");
    if (!v.brake) { out.print(" (expected impact ~"); out.print(sqrtf(2 * kG * v.height_m), 1); out.print(" m/s)"); }
    out.println(" landing. Type 'k' to abort.");
  }
  else if (!strcmp(cmd, "log")) dumpLog();
  else out.println("unknown command (help)");
}

static void feedChar(char ch, char* buf, int& n, int cap) {
  if (ch == '\r' || ch == '\n') { if (n > 0) { buf[n] = 0; n = 0; handleLine(buf); } return; }
  if (n == 0 && (ch == 'k' || ch == 'K')) { killAll("kill key"); return; }   // a line starting with 'k' kills immediately
  if (n < cap - 1) buf[n++] = ch;
}
static void handleSerial() {
  static char buf[96]; static int n = 0;
  while (Serial.available()) feedChar((char)Serial.read(), buf, n, sizeof(buf));
#if WEEBLE_BT
  static char bbuf[96]; static int bn = 0;
  while (SerialBT.available()) feedChar((char)SerialBT.read(), bbuf, bn, sizeof(bbuf));
#endif
}

// ============================ setup / loop ============================
void setup() {
  Serial.begin(115200); delay(300);
#if WEEBLE_BT
  SerialBT.begin("Weeble");
#endif
  if (PIN_SAFETY >= 0) pinMode(PIN_SAFETY, INPUT_PULLUP);
  ESP32PWM::allocateTimer(0); ESP32PWM::allocateTimer(1); ESP32PWM::allocateTimer(2); ESP32PWM::allocateTimer(3);
  escU.setPeriodHertz(50); escL.setPeriodHertz(50); srvX.setPeriodHertz(50); srvY.setPeriodHertz(50);   // TODO ESC/servo rate
  escU.attach(PIN_ESC_UPPER, P.esc_min_us, P.esc_max_us);
  escL.attach(PIN_ESC_LOWER, P.esc_min_us, P.esc_max_us);
  srvX.attach(PIN_SERVO_X, P.servo_min_us, P.servo_max_us);
  srvY.attach(PIN_SERVO_Y, P.servo_min_us, P.servo_max_us);
  P.throttle_cap = DEFAULT_CAP;
  setPayload(P, P.payload_kg);
  applyDescentPreset(P, false);
  safeOutputs();
  if (!mpuInit() || mpuWhoAmI() != 0x68) out.println("WARNING: MPU6050 not found / unexpected WHO_AM_I (check wiring, I2C address, 3.3 V)");
  loadAccelCal();
  out.println("WeebleControls v2 ready. State Rest, safety lock ENGAGED, cap 15 %. PROPS OFF for first tests. Type 'help'.");
  lastTickUs = micros();
}

void loop() {
  handleSerial();
  uint32_t now = micros();
  if ((uint32_t)(now - lastTickUs) < (uint32_t)(1e6f / LOOP_HZ)) return;
  float dt = (now - lastTickUs) * 1e-6f; lastTickUs = now;
  dt = clampf(dt, 0.002f, 0.02f);

  Vec3 g, a;
  if (!mpuRead(g, a)) {
    if (++imuFail > 10 && flight.macro() != Macro::REST && flight.macro() != Macro::FAULT) killAll("IMU read failure");
    return;
  }
  imuFail = 0;

  Command cmd = flight.update(g, a, dt);

  // ---- bench overrides (Armed, no routine) ----
  if (flight.idleArmed()) {
    cmd = Command();
    if (benchServoOn) { cmd.tilt_x_deg = benchTx; cmd.tilt_y_deg = benchTy; }
    if (spinPhase != SP_OFF) {
      float el = (millis() - spinT0) * 1e-3f; float fm = norm(flight.est.f_body);
      if (spinPhase == SP_BASE) {
        spinSum0 += fm; spinSq0 += (double)fm * fm; spinN0++;
        if (el >= 1.0f) { spinPhase = SP_RUN; spinT0 = millis(); }
      } else {
        cmd.motors_on = true; cmd.collective = (el < 0.5f) ? spinThr * (el / 0.5f) : spinThr;   // 0.5 s ramp
        if (el > 0.7f) { spinSum1 += fm; spinSq1 += (double)fm * fm; spinN1++; }                  // skip the ramp
        if (el >= spinDur) {
          int n0 = spinN0 > 0 ? spinN0 : 1, n1 = spinN1 > 0 ? spinN1 : 1;
          double m0 = spinSum0 / n0, m1 = spinSum1 / n1;
          double s0 = sqrt(fmax(0.0, spinSq0 / n0 - m0 * m0)), s1 = sqrt(fmax(0.0, spinSq1 / n1 - m1 * m1));
          out.print("SPIN RESULT at "); out.print(spinThr * 100, 0); out.print(" %: mean |f| baseline "); out.print(m0, 3); out.print(" (std "); out.print(s0, 3);
          out.print(") -> spinning "); out.print(m1, 3); out.print(" (std "); out.print(s1, 3); out.print(")  shift = "); out.print(m1 - m0, 3); out.println(" m/s^2");
          out.println(fabs(m1 - m0) < 0.05 ? "  shift < 0.05: GENTLE preset may be considered (repeat at several throttles)" : "  shift >= 0.05: stay on ROBUST (or add a rangefinder)");
          spinPhase = SP_OFF; cmd = Command();
        }
      }
    }
  }

  // ---- outputs ----
  if (flight.outputsLive()) writeOutputs(mix(cmd, P)); else { safeOutputs(); stopBench(); }

  // ---- messages / transitions ----
  static bool msgCont = false;   // core posts "mission end: " + reason as two messages -> join them on one line
  while (const char* m = flight.popMsg()) {
    if (!msgCont) { out.print("["); out.print((millis() - missionStartMs) * 1e-3f, 2); out.print(" s] "); }
    size_t L = strlen(m); msgCont = L >= 2 && m[L - 2] == ':' && m[L - 1] == ' ';
    if (msgCont) out.print(m); else out.println(m);
  }
  Macro mc = flight.macro();
  if (mc != lastMacro) {
    if ((mc == Macro::REST || mc == Macro::FAULT) && !softLocked) { softLocked = true; out.println("safety lock re-engaged"); }
    if (mc == Macro::FAULT) { out.print("FAULT: "); out.println(flight.fault_reason); out.println("'disarm' to return to Rest."); }
    lastMacro = mc;
  }
  if (flight.summary_ready) { flight.summary_ready = false; printSummary(); }

  // ---- logging at 100 Hz while a routine runs (including Await Release and Amp Down) ----
  static uint8_t dec = 0;
  if (flight.missionRunning() && (dec++ & 1) == 0) {
    Rec& r = logbuf[logHead]; logHead = (logHead + 1) % LOG_N; if (logCount < LOG_N) logCount++;
    uint32_t t10 = (millis() - missionStartMs) / 10; r.t10 = (uint16_t)(t10 > 65535 ? 65535 : t10);
    r.macro = (uint8_t)mc; r.sub = (uint8_t)flight.subState();
    const Estimator& e = flight.est;
    r.px = q16(e.pos.x, 1000); r.py = q16(e.pos.y, 1000); r.pz = q16(e.pos.z, 1000);
    r.vx = q16(e.vel.x, 1000); r.vy = q16(e.vel.y, 1000); r.vz = q16(e.vel.z, 1000);
    r.tilt = q16(e.tiltDeg(), 10); r.u = (uint16_t)(clampf(fminf(cmd.collective, P.throttle_cap), 0.0f, 1.0f) * 10000.0f);
    r.podx = q16(cmd.tilt_x_deg, 10); r.pody = q16(cmd.tilt_y_deg, 10);
    r.fz = q16(e.f_body.z, 100); r.fmag = q16(norm(e.f_body), 100); r.aux = q16(flight.auxValue(), 1000); r.temp = q16(imuTempC, 100);
  }
}
