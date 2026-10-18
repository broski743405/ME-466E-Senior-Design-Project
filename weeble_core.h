// weeble_core.h -- Phase 1 (ESP32, no ArduCopter) controls core for the UTME Weeble drone.
// Hardware-independent C++ (no Arduino headers) so it compiles on a laptop for testing.
//
// ROUTINES (see Controls_State_Conditions_Specific_Command_Routines.csv):
//   Controlled Landing (descent from mothership release)   -> DescentController + Flight::startDescent
//   Launch Horizontal Distance (hop)                        -> HopController     + Flight::startHop
//   Launch Vertical Distance (vertical hop)                 -> VHopController    + Flight::startVHop
//   Calibrate IMU / six-position accel calibration          -> Estimator::initAtRest, solveAccelCal
//   Arm / Disarm / Kill                                     -> Flight::arm / disarm / kill
//   Bench gimbal + spin tests                               -> in the .ino (need hardware)
//
// MACROSTATES (Flight::macro()): Rest, Armed, Await Release, Launch, Amp Up, Ascending, Descending, Landing,
//   Amp Down, Fault.  Rest = disarmed. Every mission ends in Rest (auto-disarm) or Fault.
//
// Frames / conventions (please check against the real wiring):
//   Body : x = forward, y = left, z = up when the drone is upright (right-handed).
//   World: z up; x/y horizontal, aligned with the body heading when the navigation estimate was last zeroed.
//   Bearing: degrees CLOCKWISE seen from above, 0 = forward, +90 = right.
//   Height: estimated z = height of the CG above its resting height = height of the lowest point of the shell
//           above the ground (so "release height" = shell bottom to ground, measured with a tape).
//   Units: SI (m, s, rad, N, m/s^2) except servo/tilt angles, which are degrees.
//
// Every number marked ASSUMED / TODO / TBD is NOT from your documents and must be confirmed or measured.
#pragma once
#include <math.h>
#include <stdint.h>

namespace weeble {

static constexpr float kG = 9.80665f;
static constexpr float kDeg2Rad = 0.017453292f;
static constexpr float kRad2Deg = 57.29578f;

template <class T> static inline T clampf(T v, T lo, T hi) { return v < lo ? lo : (v > hi ? hi : v); }

// ============================================================================
// Parameters
// ============================================================================
struct Params {
  // ---- Vehicle -------------------------------------------------------------
  float base_mass_kg = 1.690f;       // CDR table: 2.144 kg total incl. 0.454 kg payload -> 1.690 kg. RE-WEIGH the built drone.
  float payload_kg = 0.454f;         // operator input ('payload'), weigh it every time
  float mass_kg = 2.144f;            // DERIVED = base + payload. Always change through setPayload().

  // ---- Runtime safety ---------------------------------------------------------
  float throttle_cap = 0.15f;        // boot default 15 %. ALL thrust budgets and feasibility checks use thrust AT THE CAP.

  // ---- Propulsion (TODO(measure): thrust-stand test, with shell around pod) --
  float thrust_max_N = 27.25f;       // 98 oz static thrust (CDR 4.1). No download loss applied yet -> likely optimistic.
  float thrust_curve_exp = 1.0f;     // thrust = thrust_max * throttle^exp. TODO(measure). 1.0 = linear (ASSUMED)
  float thrust_lag_up_s = 0.25f;     // ASSUMED first-order spin-up time constant. TODO(measure). USED by descent braking.
  float thrust_lag_down_s = 0.25f;   // ASSUMED spin-down time constant. TODO(measure). USED by hop / vertical-hop cut-off.
  float idle_throttle = 0.06f;       // ASSUMED: lowest throttle at which both motors reliably spin. TODO(ESC)

  // ---- Output mapping (TODO(calibrate) during the power-on + tilt tests) ----
  uint16_t esc_min_us = 1000, esc_max_us = 2000;          // ASSUMED standard PWM, ESCs already calibrated
  uint16_t servo_center_us = 1500;                         // ASSUMED
  uint16_t servo_min_us = 1100, servo_max_us = 1900;       // conservative hard limits, widen after bind check
  float servo_us_per_pod_deg = 5.0f;                       // DELIBERATELY SMALL placeholder; from horn/rod kinematics (CDR A.8)
  float servo_sign_x = 1.0f, servo_sign_y = 1.0f;          // flip after the powered tilt test
  float yaw_sign = 1.0f;                                   // flip after spinning the coaxial pair

  // ---- Gimbal ----------------------------------------------------------------
  float tilt_limit_deg = 25.0f;      // CDR 5.2 mechanical design is 25 deg. Requirement is +/-30 -> change after range-of-motion test.

  // ---- Attitude loop (TODO(tune) on the tether; values below are first guesses) ----
  float att_ref_thrust_N = 15.0f;    // gains below are for this thrust; they are scaled by (ref / actual thrust) because pod torque ~ thrust
  float att_kp = 1.0f;               // pod-deg per degree of body lean error   (at att_ref_thrust_N)
  float att_kd = 0.30f;              // pod-deg per (deg/s) of body rate         (at att_ref_thrust_N)
  float att_ki = 0.5f;               // pod-deg per (deg*s) of lean error (removes steady lean offset)
  float att_i_max_deg = 12.0f;       // integrator clamp
  float yaw_kp_rate = 0.002f;        // differential throttle per (deg/s)
  float yaw_kp_ang = 0.004f;         // differential throttle per degree of heading error
  float yaw_limit = 0.08f;           // max differential throttle

  // ---- Estimator --------------------------------------------------------------
  float mahony_kp = 1.0f;            // accel->attitude correction gain [1/s]. Only used while ON THE GROUND / hanging (see Flight).
  float accel_gate_g = 0.15f;        // only trust accel for attitude when | |f| - 1g | < this
  float rest_time_s = 0.30f;         // stillness time before "at rest"

  // ---- Calibration acceptance (CSV: |a| within 1 m/s^2 of 9.81, tilt < 10 deg, gyro quiet) ----
  float cal_max_amag_err = 1.0f;     // m/s^2 (CSV)
  float cal_max_tilt_deg = 10.0f;    // (CSV)
  float cal_max_gyro_std = 0.02f;    // rad/s (~1.1 deg/s) std over the 2 s window. ASSUMED -- "gyro quiet" has no number in the CSV
  float cal_max_acc_std = 0.15f;     // m/s^2 std over the window. ASSUMED

  // ---- Descent ---------------------------------------------------------------
  float freefall_g = 0.35f;          // release = |specific force| below this fraction of 1 g ...
  float freefall_debounce_s = 0.04f; // ... for this long
  float zero_g_cal_s = 0.10f;        // motors OFF this long after release to calibrate the accelerometer at true zero-g (0 = skip)
  bool  use_rest_scale = true;       // derive accel scale from the at-rest 1 g reading + zero-g reading (see Estimator::applyZeroG)
  float land_cut_height_m = -0.50f;  // also declare touchdown when the ESTIMATED height drops to this (soft touchdowns are hard to sense)
  float brake_plan_fraction = 0.75f; // plan braking at this fraction of available decel (keeps margin for lag/sag)
  float lead_extra_s = 0.05f;        // braking-profile dead time = thrust_lag_up_s + this  (set by preset)
  float v_touch_mps = 0.8f;          // target final descent speed
  float h_flare_m = 0.3f;            // below this estimated height, hold v_touch until contact
  float vel_kp = 1.0f;               // [1/s] gain when falling FASTER than the profile (needs more thrust)
  float vel_kp_slow = 3.0f;          // [1/s] gain when falling SLOWER than the profile (cut thrust fast -> true free fall until braking is needed)
  float acc_fb_gain = 0.5f;          // [1/s] measured-vs-commanded specific-force trim
  float impact_g = 3.0f;             // hard touchdown = |specific force| above this ...
  float contact_delta_g = 0.6f;      // ... or specific force this far above what we are commanding (soft touchdown). TODO(tune)
  float max_tilt_thrust_deg = 50.0f; // CSV: tilt > 50 deg -> thrust cut
  float max_powered_s = 5.0f;        // CSV: ROBUST 5 s, GENTLE 12 s (set by preset)
  float min_spare_decel = 0.5f;      // CSV: refuse below this spare braking acceleration [m/s^2], computed AT THE THROTTLE CAP

  // ---- Hop (horizontal) ------------------------------------------------------
  bool  hop_attitude_hold = true;    // true: tip the WHOLE drone to the lean (recommended); false: fixed pod deflection (sim: tumbles)
  float hop_lean_deg = 15.0f;        // default body (thrust-axis) lean during burn. Operator can override per hop.
                                     // CHANGED from 25 (v1/CSV). Sim (TWR 1.5): at 5 ft both work, but at 25 deg the cut often fires on
                                     // the first airborne tick, so other distances scatter (3 ft target -> 2.95..4.52 ft), horizontal
                                     // touchdown speed is ~40 % higher (3.0-4.2 m/s), peak tilt 39 deg, and it is REFUSED at TWR 1.30
                                     // (the CDR thrust). 15 deg: 2-8 ft within ~0.3 ft, 1.9-2.9 m/s horizontal, peak tilt 22 deg.
                                     // Cost of 15 deg: higher arc -> harder VERTICAL touchdown (~2 m/s vs ~1 m/s at 5 ft).
  float hop_twr_margin = 1.15f;      // CSV: TWR(cap) * cos(lean + 10 deg) >= this
  float tip_thrust_N = 5.0f;         // thrust while tipping on the ground. Upright is unstable above ~4.4 N (CDR numbers). Must stay well below weight.
  float tip_ramp_s = 0.8f;           // time to ramp the lean target to the lean angle
  float hop_throttle = 1.0f;         // burn throttle (the runtime throttle cap still applies)
  float hop_settle_s = 0.4f;         // CSV
  float settle_timeout_s = 3.0f;     // CSV: not at rest after 3 s -> back to Armed
  float tip_timeout_s = 1.5f;        // CSV
  float servo_settle_s = 0.3f;       // open-loop mode only
  float hop_max_burn_s = 0.8f;       // CSV: no liftoff within 0.8 s -> cut, Amp Down
  float hop_neutralize_hold_s = 0.5f;// CSV: keep pod tilt this long after cut (thrust tail), then centre
  float hop_range_scale = 1.0f;      // TODO(tune) from tests: measured / predicted range
  float hop_cut_margin_m = 0.0f;
  float cmd_delay_s = 0.01f;         // average wait before a throttle change reaches the rotors (half a 50 Hz ESC frame). Used by both predictors.
  float hop_lat_gain = 0.5f;         // steer away from predicted sideways miss (rad per normalised miss)
  bool  hop_land_brake = false;      // default landing mode for the horizontal hop: ballistic
  float hop_land_flare_m = 0.10f;
  float hop_land_v_touch = 1.0f;
  float hop_touch_g = 2.0f;          // touchdown during ballistic coast = |specific force| above this
  float liftoff_h_m = 0.03f;         // liftoff = estimated height above this AND rising faster than liftoff_vz ...
  float liftoff_vz = 0.05f;
  float liftoff_vz_fast = 0.20f;     // ... or rising faster than this

  // ---- Vertical hop ----------------------------------------------------------
  float vhop_max_height_m = 1.0f;    // TBD by the team (CSV "max TBD"). Refuses above this. Dead-reckoned height trusted ~1 s.
  float vhop_throttle = 1.0f;        // burn throttle (cap still applies)
  float vhop_height_scale = 1.0f;    // TODO(tune) from video: measured / predicted apex
  float vhop_cut_margin_m = 0.0f;
  float vhop_twr_margin = 1.15f;     // TWR(cap) * cos(10 deg) >= this (same rule as the hop with lean 0)

  // ---- Mission supervision ---------------------------------------------------
  float await_release_timeout_s = 25.0f; // descent: give up waiting for release after this (-> Fault, disarmed). See README warning.
  float mission_time_s = 25.0f;          // CSV mission time limit. Descent: counted from RELEASE. Hops: from the command.
  float ampdown_rest_s = 0.3f;           // Amp Down -> Rest once at rest this long ...
  float ampdown_timeout_s = 3.0f;        // ... or after this
};

static inline void setPayload(Params& P, float kg) { P.payload_kg = kg < 0 ? 0 : kg; P.mass_kg = P.base_mass_kg + P.payload_kg; }
static inline float thrustAt(const Params& P, float u) { return P.thrust_max_N * powf(clampf(u, 0.0f, 1.0f), P.thrust_curve_exp); }
static inline float thrustAvail(const Params& P) { return thrustAt(P, P.throttle_cap); }
static inline float twrAvail(const Params& P) { return thrustAvail(P) / (P.mass_kg * kG); }
static inline float spareDecel(const Params& P) { return thrustAvail(P) / P.mass_kg - kG; }
static inline float leadTime(const Params& P) { return fmaxf(P.thrust_lag_up_s + P.lead_extra_s, 0.02f); }
static inline float thrustToThrottle(const Params& P, float thrust_N) {
  float frac = clampf(thrust_N / P.thrust_max_N, 0.0f, 1.0f);
  return powf(frac, 1.0f / P.thrust_curve_exp);
}

// Descent presets. ROBUST (default): short powered time, so hidden accelerometer error has little time to grow; firmer
// touchdown. GENTLE: slow touchdown, but long powered creep -> only safe if the IMU is clean while the motors run.
static inline void applyDescentPreset(Params& P, bool gentle) {
  if (gentle) { P.vel_kp_slow = 1.5f; P.lead_extra_s = 0.35f; P.brake_plan_fraction = 0.60f; P.h_flare_m = 0.30f; P.v_touch_mps = 0.5f; P.max_powered_s = 12.0f; }
  else        { P.vel_kp_slow = 3.0f; P.lead_extra_s = 0.05f; P.brake_plan_fraction = 0.75f; P.h_flare_m = 0.30f; P.v_touch_mps = 0.8f; P.max_powered_s = 5.0f; }
}

// ============================================================================
// Small math
// ============================================================================
struct Vec3 { float x = 0, y = 0, z = 0; };
static inline Vec3 operator+(Vec3 a, Vec3 b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
static inline Vec3 operator-(Vec3 a, Vec3 b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
static inline Vec3 operator*(Vec3 a, float s) { return {a.x * s, a.y * s, a.z * s}; }
static inline float dot(Vec3 a, Vec3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
static inline Vec3 cross(Vec3 a, Vec3 b) { return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x}; }
static inline float norm(Vec3 a) { return sqrtf(dot(a, a)); }

struct Quat { float w = 1, x = 0, y = 0, z = 0; };  // rotates body -> world
static inline Quat qmul(const Quat& a, const Quat& b) {
  return {a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z, a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
          a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x, a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w};
}
static inline Vec3 qrot(const Quat& q, Vec3 v) {
  Vec3 u{q.x, q.y, q.z};
  Vec3 t = cross(u, v) * 2.0f;
  return v + t * q.w + cross(u, t);
}
static inline Vec3 qrotInv(const Quat& q, Vec3 v) { return qrot(Quat{q.w, -q.x, -q.y, -q.z}, v); }
static inline float wrapPi(float a) {
  while (a > 3.14159265f) a -= 6.2831853f;
  while (a < -3.14159265f) a += 6.2831853f;
  return a;
}

// ============================================================================
// Six-position (or more) accelerometer calibration, sensor frame.
//   corrected_i = (raw_i - off_i) * scl_i
// Samples = mean accelerometer readings with the sensor held still in different orientations. Orientations do NOT
// need to be exact: the fit finds offset+scale so that every corrected sample has magnitude 1 g (least squares).
// Needs every axis pointed roughly up AND roughly down at least once. Use 8-12 poses so the residual means something
// (with exactly 6 the fit is exact and the residual is always ~0).
// ============================================================================
struct AccelCal {
  uint32_t magic = 0x57454231u;   // "WEB1" -- changes if the layout changes
  float off[3] = {0, 0, 0};
  float scl[3] = {1, 1, 1};
  float rms = 0;
  uint8_t n_poses = 0;
  bool valid = false;
};
static inline void applyAccelCal(const AccelCal& c, float a[3]) {
  if (!c.valid) return;
  for (int i = 0; i < 3; i++) a[i] = (a[i] - c.off[i]) * c.scl[i];
}
static inline bool solve6(float A[6][6], float b[6], float x[6]) {   // Gaussian elimination, partial pivoting
  for (int c = 0; c < 6; c++) {
    int p = c; for (int r = c + 1; r < 6; r++) if (fabsf(A[r][c]) > fabsf(A[p][c])) p = r;
    if (fabsf(A[p][c]) < 1e-9f) return false;
    if (p != c) { for (int k = 0; k < 6; k++) { float t = A[c][k]; A[c][k] = A[p][k]; A[p][k] = t; } float t = b[c]; b[c] = b[p]; b[p] = t; }
    for (int r = c + 1; r < 6; r++) { float f = A[r][c] / A[c][c]; for (int k = c; k < 6; k++) A[r][k] -= f * A[c][k]; b[r] -= f * b[c]; }
  }
  for (int r = 5; r >= 0; r--) { float s = b[r]; for (int k = r + 1; k < 6; k++) s -= A[r][k] * x[k]; x[r] = s / A[r][r]; }
  return true;
}
// Acceptance limits: MPU-6050 datasheet tolerances are roughly +/-3 % sensitivity and tens of mg zero-g offset
// (check the datasheet); limits below are wider on purpose and only catch a bad fit / wrong poses.
static inline bool solveAccelCal(const Vec3* s, int n, AccelCal& out, const char*& why) {
  why = "";
  if (n < 6) { why = "need at least 6 poses"; return false; }
  float mn[3] = {1e9f, 1e9f, 1e9f}, mx[3] = {-1e9f, -1e9f, -1e9f};
  for (int k = 0; k < n; k++) { const float v[3] = {s[k].x, s[k].y, s[k].z}; for (int i = 0; i < 3; i++) { mn[i] = fminf(mn[i], v[i]); mx[i] = fmaxf(mx[i], v[i]); } }
  for (int i = 0; i < 3; i++) if (mx[i] < 0.5f * kG || mn[i] > -0.5f * kG) { why = "poses do not cover every axis pointing up AND down"; return false; }
  float p[6];
  for (int i = 0; i < 3; i++) { p[i] = 0.5f * (mx[i] + mn[i]); p[3 + i] = 2.0f * kG / (mx[i] - mn[i]); }
  for (int it = 0; it < 30; it++) {
    float JTJ[6][6] = {}, JTr[6] = {};
    for (int k = 0; k < n; k++) {
      const float v[3] = {s[k].x, s[k].y, s[k].z};
      float u[3], nn = 0;
      for (int i = 0; i < 3; i++) { u[i] = (v[i] - p[i]) * p[3 + i]; nn += u[i] * u[i]; }
      nn = sqrtf(nn); if (nn < 1e-3f) continue;
      float r = nn - kG, J[6];
      for (int i = 0; i < 3; i++) { J[i] = -p[3 + i] * u[i] / nn; J[3 + i] = (v[i] - p[i]) * u[i] / nn; }
      for (int a = 0; a < 6; a++) { JTr[a] += J[a] * r; for (int b = 0; b < 6; b++) JTJ[a][b] += J[a] * J[b]; }
    }
    float rhs[6], dx[6];
    for (int a = 0; a < 6; a++) { JTJ[a][a] *= 1.0f + 1e-6f; JTJ[a][a] += 1e-9f; rhs[a] = -JTr[a]; }
    if (!solve6(JTJ, rhs, dx)) { why = "fit is singular (poses too similar)"; return false; }
    float step = 0; for (int a = 0; a < 6; a++) { p[a] += dx[a]; step += fabsf(dx[a]); }
    if (step < 1e-7f) break;
  }
  float ss = 0;
  for (int k = 0; k < n; k++) {
    const float v[3] = {s[k].x, s[k].y, s[k].z}; float nn = 0;
    for (int i = 0; i < 3; i++) { float u = (v[i] - p[i]) * p[3 + i]; nn += u * u; }
    float r = sqrtf(nn) - kG; ss += r * r;
  }
  AccelCal c;
  for (int i = 0; i < 3; i++) { c.off[i] = p[i]; c.scl[i] = p[3 + i]; }
  c.rms = sqrtf(ss / n); c.n_poses = (uint8_t)n; c.valid = true;
  for (int i = 0; i < 3; i++) {
    if (fabsf(c.off[i]) > 1.5f) { why = "offset > 1.5 m/s^2 on an axis (bad fit or damaged sensor)"; return false; }
    if (fabsf(c.scl[i] - 1.0f) > 0.08f) { why = "scale off by > 8 % on an axis (bad fit)"; return false; }
  }
  if (n > 6 && c.rms > 0.10f) { why = "residual > 0.10 m/s^2 (drone moved during a pose?)"; return false; }
  out = c;
  return true;
}

// ============================================================================
// Controller -> mixer command
// ============================================================================
struct Command {
  bool motors_on = false;
  float collective = 0;          // 0..1 throttle fraction (mixer applies the cap)
  bool use_att = false;          // true: attitude loop computes pod tilt toward lean_*_des
  float lean_x_des = 0, lean_y_des = 0;   // desired lean of the body's top toward +x / +y [deg]
  float tilt_x_deg = 0, tilt_y_deg = 0;   // pod tilt toward +x / +y [deg] (overwritten if use_att)
  float yaw = 0;                 // differential throttle (upper minus lower), filled by attitude loop
};

// ============================================================================
// State estimator: attitude (gated Mahony) + dead-reckoned velocity/position
// ============================================================================
class Estimator {
 public:
  explicit Estimator(const Params& p) : P(p) {}

  // Call with the average specific force (m/s^2, body frame) while the drone is held still and upright.
  void initAtRest(Vec3 f_mean) {
    float n = norm(f_mean);
    Vec3 a = f_mean * (1.0f / n);                   // up direction in body frame
    if (a.z < -0.99f) { q = Quat{0, 1, 0, 0}; }
    else {
      Quat r{1.0f + a.z, a.y, -a.x, 0.0f};           // rotates a onto +z
      float m = sqrtf(r.w * r.w + r.x * r.x + r.y * r.y + r.z * r.z);
      q = Quat{r.w / m, r.x / m, r.y / m, r.z / m};
    }
    accel_bias = a * (n - kG);                       // assume residual error lies along gravity
    rest_bias = accel_bias;
    accel_scale = 1.0f;
    f_rest_raw = f_mean;
    pos = Vec3(); vel = Vec3(); still_t = 0;
  }
  // Two-point accelerometer calibration using a true free-fall window (specific force ~ 0):
  //   offset = mean reading in free fall;  scale = 1 g / (at-rest reading - offset), accepted only within +/-6 %.
  void applyZeroG(Vec3 raw_mean_in_freefall, bool use_rest_scale) {
    zero_g_span = norm(f_rest_raw - raw_mean_in_freefall);
    zero_g_mean = raw_mean_in_freefall;
    accel_bias = raw_mean_in_freefall;
    float k = (zero_g_span > 1.0f) ? kG / zero_g_span : 1.0f;
    accel_scale = (use_rest_scale && fabsf(k - 1.0f) < 0.06f) ? k : 1.0f;
    zero_g_applied = true;
  }
  void setGyroBias(Vec3 b) { gyro_bias = b; }
  void resetNav(Vec3 p, Vec3 v) { pos = p; vel = v; }

  void update(Vec3 gyro_raw, Vec3 accel, float dt) {
    gyro = gyro_raw - gyro_bias;
    accel_raw = accel;
    f_body = (accel - accel_bias) * accel_scale;
    float fn = norm(f_body);
    Vec3 w = gyro;
    // FIX (v2): the accelerometer only measures "up" when the drone is NOT accelerating, i.e. on the ground or
    // hanging. In flight it measures the THRUST direction, and a hover-like 1 g reading passes the magnitude gate and
    // drags the attitude estimate toward the thrust axis (sim showed ~3 deg steady error). Flight disables it in air.
    if (accel_correct && fabsf(fn / kG - 1.0f) < P.accel_gate_g && fn > 1.0f) {
      Vec3 fhat = f_body * (1.0f / fn);
      Vec3 vhat = qrotInv(q, Vec3{0, 0, 1});
      w = w + cross(fhat, vhat) * P.mahony_kp;
    }
    Quat dq = qmul(q, Quat{0, w.x, w.y, w.z});
    q.w += 0.5f * dq.w * dt; q.x += 0.5f * dq.x * dt; q.y += 0.5f * dq.y * dt; q.z += 0.5f * dq.z * dt;
    float m = sqrtf(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
    q.w /= m; q.x /= m; q.y /= m; q.z /= m;

    f_world = qrot(q, f_body);
    a_world = f_world - Vec3{0, 0, kG};
    pos = pos + vel * dt + a_world * (0.5f * dt * dt);
    vel = vel + a_world * dt;

    bool still = norm(gyro) < 0.06f && fabsf(fn - kG) < 0.4f;
    still_t = still ? still_t + dt : 0.0f;
    // Zero-velocity update: ONLY allowed on the ground. Steady slow flight looks identical to "at rest" to an IMU.
    if (zupt_enable && still_t > P.rest_time_s) {
      vel = Vec3();
      gyro_bias = gyro_bias + (gyro_raw - gyro_bias) * 0.002f;
    }
  }

  bool atRest() const { return still_t > P.rest_time_s; }
  float stillTime() const { return still_t; }
  Vec3 upInBody() const { return qrotInv(q, Vec3{0, 0, 1}); }
  float tiltDeg() const { return acosf(clampf(upInBody().z, -1.0f, 1.0f)) * kRad2Deg; }
  float yaw() const { return atan2f(2.0f * (q.w * q.z + q.x * q.y), 1.0f - 2.0f * (q.y * q.y + q.z * q.z)); }

  Quat q;
  Vec3 pos, vel;          // world frame; z = height of CG above its resting height
  Vec3 gyro;              // bias-corrected [rad/s]
  Vec3 f_body, f_world, a_world;
  Vec3 accel_bias, gyro_bias, accel_raw, f_rest_raw, rest_bias, zero_g_mean;
  float accel_scale = 1.0f, zero_g_span = 0;
  bool zero_g_applied = false;
  bool zupt_enable = false;
  bool accel_correct = true;

 private:
  const Params& P;
  float still_t = 0;
};

// ============================================================================
// Attitude + yaw loop.  Pod tilt is the only attitude actuator:
//   pod toward +x  -> torque that leans the top of the drone toward +x (thrust pivot sits above the CG).
// ============================================================================
class AttitudeCtl {
 public:
  explicit AttitudeCtl(const Params& p) : P(p) {}
  void setYawRef(float psi) { yaw_ref = psi; }
  void reset() { ix = iy = 0; }
  void update(const Estimator& e, Command& c, float dt) {
    Vec3 up = e.upInBody();
    float lean_x = asinf(clampf(-up.x, -1.0f, 1.0f)) * kRad2Deg;
    float lean_y = asinf(clampf(-up.y, -1.0f, 1.0f)) * kRad2Deg;
    float wx = e.gyro.x * kRad2Deg, wy = e.gyro.y * kRad2Deg, wz = e.gyro.z * kRad2Deg;
    float T_est = thrustAt(P, fminf(c.collective, P.throttle_cap));   // FIX (v2): the cap limits the real thrust
    float gs = clampf(P.att_ref_thrust_N / fmaxf(T_est, 1.0f), 0.6f, 4.0f);   // gain scheduling on thrust
    float ex = c.lean_x_des - lean_x, ey = c.lean_y_des - lean_y;
    if (fabsf(ex) < 15.0f) ix = clampf(ix + gs * P.att_ki * ex * dt, -P.att_i_max_deg, P.att_i_max_deg);
    if (fabsf(ey) < 15.0f) iy = clampf(iy + gs * P.att_ki * ey * dt, -P.att_i_max_deg, P.att_i_max_deg);
    float tx = gs * (P.att_kp * ex - P.att_kd * wy) + ix;
    float ty = gs * (P.att_kp * ey + P.att_kd * wx) + iy;
    c.tilt_x_deg = clampf(tx, -P.tilt_limit_deg, P.tilt_limit_deg);
    c.tilt_y_deg = clampf(ty, -P.tilt_limit_deg, P.tilt_limit_deg);
    float dpsi = wrapPi(e.yaw() - yaw_ref) * kRad2Deg;
    c.yaw = clampf(-(P.yaw_kp_rate * wz + P.yaw_kp_ang * dpsi), -P.yaw_limit, P.yaw_limit);
  }
 private:
  const Params& P;
  float yaw_ref = 0, ix = 0, iy = 0;
};

// Liftoff test shared by both hops (dead-reckoned; ZUPT is off once the settle phase ends).
static inline bool liftoffSeen(const Params& P, const Estimator& e) {
  return (e.vel.z > P.liftoff_vz_fast) || (e.pos.z > P.liftoff_h_m && e.vel.z > P.liftoff_vz);
}

// ============================================================================
// Controlled descent (IMU only).  Also used as the landing brake at the end of a hop ("entry case B").
//   Release is detected from the accelerometer (near-zero specific force).  Height and vertical speed are
//   dead-reckoned from the assumed release height.  The controller tracks a speed-vs-height profile that already
//   includes the thrust lag, then holds v_touch until an impact is sensed.
// ============================================================================
enum class DState : uint8_t { OFF, WAIT_RELEASE, FALLING, LANDED, FAULT };

class DescentController {
 public:
  explicit DescentController(const Params& p) : P(p) {}
  bool feasible() const { return spareDecel(P) >= P.min_spare_decel; }
  void arm(float release_height_m) {
    H0 = release_height_m; st = DState::WAIT_RELEASE; ff_t = 0; t = 0; resetRun(P.h_flare_m, P.v_touch_mps);
    impact_gate_s = 0.15f; zg_done = (P.zero_g_cal_s <= 0.0f);
  }
  void startTracking(float h_flare, float v_touch) {
    st = DState::FALLING; t = 0; resetRun(h_flare, v_touch); impact_gate_s = 0.05f; zg_done = true;
  }
  void stop() { st = DState::OFF; }
  DState state() const { return st; }
  bool braking() const { return braking_; }
  float lastThrottle() const { return u_last; }
  float lastVref() const { return vref_last; }
  int fault_code = 0;            // 1 = not enough thrust (at the cap) to brake, 2 = powered-time limit
  float td_vz = 0, td_tilt = 0, td_h = 0;   // estimated vertical speed, tilt and height at the touchdown decision
  bool td_by_height = false;     // true: touchdown declared by the height backstop, not by an impact
  Vec3 zg_mean; bool zg_measured = false;

  Command update(Estimator& est, float dt) {
    Command c;
    t += dt;
    switch (st) {
      case DState::WAIT_RELEASE: {
        c.motors_on = true; c.collective = 0.0f;
        if (norm(est.f_body) < P.freefall_g * kG) ff_t += dt; else ff_t = 0;
        if (ff_t >= P.freefall_debounce_s) {
          float td = ff_t;
          est.resetNav(Vec3{0, 0, H0 - 0.5f * kG * td * td}, Vec3{0, 0, -kG * td});
          st = DState::FALLING; t = 0; impact_cnt = 0; ucorr = 0;
          t_fall0 = td; zg_n = 0; zg_sum = Vec3();
        }
        break;
      }
      case DState::FALLING: {
        if (!zg_done) {                                   // zero-g calibration window: rotors fully off, no control
          zg_sum = zg_sum + est.accel_raw; zg_n++;
          if (t >= P.zero_g_cal_s) {
            zg_mean = zg_sum * (1.0f / (float)zg_n); zg_measured = true;
            est.applyZeroG(zg_mean, P.use_rest_scale);
            float tt = t_fall0 + t;                        // analytic free-fall state (drag neglected over this short window)
            est.resetNav(Vec3{0, 0, H0 - 0.5f * kG * tt * tt}, Vec3{0, 0, -kG * tt});
            zg_done = true;
          }
          c.motors_on = true; c.collective = 0.0f; c.use_att = false;
          break;
        }
        float fn = norm(est.f_body);
        bool hard = fn > P.impact_g * kG;
        bool soft = (f_cmd_last > 0.0f) && (est.f_body.z > f_cmd_last + P.contact_delta_g * kG);
        if (t > impact_gate_s && (hard || soft)) impact_cnt++; else impact_cnt = 0;
        bool by_h = est.pos.z <= P.land_cut_height_m;
        if (impact_cnt >= 2 || by_h) {
          td_vz = est.vel.z; td_tilt = est.tiltDeg(); td_h = est.pos.z; td_by_height = by_h && impact_cnt < 2;
          st = DState::LANDED; break;
        }
        if (t > P.max_powered_s) { fault_code = 2; st = DState::FAULT; break; }

        float f_full = thrustAvail(P) / P.mass_kg;           // FIX (v2): specific force at the THROTTLE CAP, not at 100 %
        float a_avail = f_full - kG;
        if (a_avail < P.min_spare_decel) { fault_code = 1; st = DState::FAULT; break; }
        float a_plan = P.brake_plan_fraction * a_avail;

        float h = est.pos.z, v = est.vel.z;
        float hh = fmaxf(h - flare, 0.0f);
        // Braking profile with thrust dead-time tau: reaches v_touch exactly at the flare height when decelerating at
        // a_plan after a delay tau. FIX (v2): tau now follows the MEASURED spin-up lag (thrust_lag_up_s + preset extra).
        float tau = leadTime(P);
        float s_ = sqrtf(tau * tau + 2.0f * hh / a_plan);
        float v_ref = -(vt + a_plan * (s_ - tau));
        vref_last = v_ref;
        float a_ff = clampf(-v / s_, 0.0f, a_plan);
        float kp = (v_ref - v < 0.0f) ? P.vel_kp_slow : P.vel_kp;
        float a_up = clampf(a_ff + kp * (v_ref - v), -kG, a_avail);
        float f_cmd = kG + a_up;
        f_cmd_last = f_cmd;

        float ucap = fminf(P.throttle_cap, 1.0f);
        float u_ff = thrustToThrottle(P, f_cmd * P.mass_kg);
        if (u_ff > P.idle_throttle + 0.05f) {
          braking_ = true;
          float du = P.acc_fb_gain * (f_cmd - est.f_body.z) / f_full * dt;
          if (!(du > 0.0f && u_ff + ucorr >= ucap)) ucorr = clampf(ucorr + du, -0.25f, 0.25f);   // anti-windup at the cap
        } else {
          ucorr *= 0.95f;
        }
        float u = clampf(u_ff + ucorr, P.idle_throttle, ucap);
        if (est.tiltDeg() > P.max_tilt_thrust_deg) u = P.idle_throttle;

        c.motors_on = true; c.collective = u; c.use_att = true;
        u_last = u;
        break;
      }
      default: break;   // OFF, LANDED, FAULT -> motors off
    }
    return c;
  }

 private:
  void resetRun(float h_flare, float v_touch) {
    ucorr = 0; flare = h_flare; vt = v_touch; impact_cnt = 0; f_cmd_last = 0; braking_ = false; fault_code = 0;
    td_vz = td_tilt = td_h = 0; td_by_height = false; zg_measured = false;
  }
  const Params& P;
  DState st = DState::OFF;
  float H0 = 6.0f, ff_t = 0, t = 0, ucorr = 0, flare = 0.6f, vt = 0.4f, u_last = 0, vref_last = 0, f_cmd_last = 0, t_fall0 = 0;
  float impact_gate_s = 0.15f;
  Vec3 zg_sum; int zg_n = 0; bool zg_done = true;
  int impact_cnt = 0;
  bool braking_ = false;
};

// ============================================================================
// Hop: move a fixed distance along a bearing, ground to ground.
//   SETTLE -> TIP (lean the whole drone toward the target at sub-weight thrust)
//          -> BURN (full thrust, attitude held; liftoff detected; cut when PREDICTED landing point reaches the target)
//          -> COAST (idle, pod held, then centred; apex detected) -> [optional brake] -> DONE (-> Amp Down)
// ============================================================================
enum class HState : uint8_t { OFF, SETTLE, TIP, BURN, COAST, LAND, DONE, REFUSED };
enum HopEnd : uint8_t { HOP_OK = 0, HOP_INFEASIBLE, HOP_NOT_AT_REST, HOP_TIP_TIMEOUT, HOP_NO_LIFTOFF, HOP_LAND_FAULT, HOP_COAST_TIMEOUT, HOP_BAD_ARGS, HOP_TILT };
static inline const char* hopEndText(uint8_t e) {
  switch (e) {
    case HOP_OK: return "touchdown";
    case HOP_INFEASIBLE: return "not feasible: TWR at the throttle cap x cos(lean+10) too low";
    case HOP_NOT_AT_REST: return "drone not at rest within the settle timeout";
    case HOP_TIP_TIMEOUT: return "could not tip to the lean angle in time";
    case HOP_NO_LIFTOFF: return "no liftoff within the max burn time";
    case HOP_LAND_FAULT: return "landing brake hit its powered-time limit";
    case HOP_COAST_TIMEOUT: return "no touchdown detected (coast timeout)";
    case HOP_BAD_ARGS: return "bad arguments (lean above gimbal limit, height above max, ...)";
    case HOP_TILT: return "tilt above limit before liftoff -> thrust cut";
    default: return "?";
  }
}

struct HopArgs {
  float dist_m = 1.524f;      // 5 ft
  float bearing_deg = 0;      // clockwise from body forward at start
  float lean_deg = 15.0f;     // default = Params::hop_lean_deg
  float range_scale = 1.0f;
  bool brake = false;
};

class HopController {
 public:
  explicit HopController(const Params& p) : P(p), land(p) {}

  // Returns false (and REFUSED) if the hop is not feasible; reason in end_code.
  bool start(const Estimator& est, const HopArgs& a) {
    A = a;
    th_b = -A.bearing_deg * kDeg2Rad;                  // CCW angle of target direction, from body-forward at start
    psi0 = est.yaw();
    dx = cosf(psi0 + th_b); dy = sinf(psi0 + th_b);
    st = HState::SETTLE; t = 0; t_in = 0; ok_t = 0; last_tx = last_ty = last_lx = last_ly = 0;
    end_code = HOP_OK; cut_by_maxburn = cut_by_tilt = false; airborne = past_apex = touched = false;
    pred_along = pred_lat = cut_pred_along = cut_pred_lat = burn_s = 0; apex_z = 0; t_liftoff = -1;
    if (A.lean_deg < 0 || A.lean_deg > P.tilt_limit_deg || A.dist_m <= 0 || A.range_scale <= 0.1f) { end_code = HOP_BAD_ARGS; st = HState::REFUSED; return false; }
    if (P.hop_attitude_hold && twrAvail(P) * cosf((A.lean_deg + 10.0f) * kDeg2Rad) < P.hop_twr_margin) { end_code = HOP_INFEASIBLE; st = HState::REFUSED; return false; }
    if (A.brake && !land.feasible()) { end_code = HOP_INFEASIBLE; st = HState::REFUSED; return false; }
    return true;
  }
  HState state() const { return st; }
  bool isAirborne() const { return airborne; }
  bool pastApex() const { return past_apex; }
  bool landBraking() const { return st == HState::LAND && land.braking(); }
  float predictedRange() const { return pred_along; }
  float predictedMiss() const { return pred_lat; }
  float heading0() const { return psi0; }
  bool settleDone() const { return st != HState::SETTLE && st != HState::OFF && st != HState::REFUSED; }

  Command update(Estimator& est, float dt) {
    Command c;
    t += dt; t_in += dt;
    float dpsi = wrapPi(est.yaw() - psi0);
    float dir_b = th_b - dpsi;                           // target direction in CURRENT body frame (CCW from body x)
    if (airborne && !past_apex && est.vel.z < 0.0f) past_apex = true;
    if (airborne) apex_z = fmaxf(apex_z, est.pos.z);
    switch (st) {
      case HState::SETTLE:
        c.motors_on = true;
        if (t_in > P.hop_settle_s && est.atRest()) {
          est.resetNav(Vec3(), Vec3());
          psi0 = est.yaw(); dx = cosf(psi0 + th_b); dy = sinf(psi0 + th_b);   // CSV: store heading at end of settle
          go(HState::TIP);
        } else if (t_in > P.settle_timeout_s) { end_code = HOP_NOT_AT_REST; go(HState::REFUSED); }
        break;

      case HState::TIP:
        c.motors_on = true;
        if (P.hop_attitude_hold) {
          c.collective = thrustToThrottle(P, P.tip_thrust_N); c.use_att = true;
          float ramp = clampf(t_in / P.tip_ramp_s, 0.0f, 1.0f) * A.lean_deg;
          c.lean_x_des = ramp * cosf(dir_b); c.lean_y_des = ramp * sinf(dir_b);
          Vec3 up = est.upInBody();
          float ex = c.lean_x_des - asinf(clampf(-up.x, -1.f, 1.f)) * kRad2Deg;
          float ey = c.lean_y_des - asinf(clampf(-up.y, -1.f, 1.f)) * kRad2Deg;
          ok_t = (ramp >= A.lean_deg - 0.01f && sqrtf(ex * ex + ey * ey) < 3.0f) ? ok_t + dt : 0.0f;
          if (ok_t > 0.15f) go(HState::BURN);
          else if (t_in > P.tip_timeout_s) { end_code = HOP_TIP_TIMEOUT; go(HState::DONE); }   // CSV: -> Amp Down
        } else {
          c.collective = 0.0f;
          float a = clampf(A.lean_deg, 0.0f, P.tilt_limit_deg);
          c.tilt_x_deg = a * cosf(dir_b); c.tilt_y_deg = a * sinf(dir_b);
          if (t_in > P.servo_settle_s) go(HState::BURN);
        }
        break;

      case HState::BURN: {
        c.motors_on = true; c.collective = P.hop_throttle;
        if (!airborne && liftoffSeen(P, est)) { airborne = true; t_liftoff = t; }
        predict(est);
        float D = A.dist_m / A.range_scale;
        float dir_cmd = dir_b - clampf(P.hop_lat_gain * pred_lat / fmaxf(A.dist_m, 0.5f), -0.35f, 0.35f);  // steer against predicted miss
        if (P.hop_attitude_hold) {
          c.use_att = true;
          c.lean_x_des = A.lean_deg * cosf(dir_cmd); c.lean_y_des = A.lean_deg * sinf(dir_cmd);
        } else {
          float a = clampf(A.lean_deg, 0.0f, P.tilt_limit_deg);
          c.tilt_x_deg = a * cosf(dir_cmd); c.tilt_y_deg = a * sinf(dir_cmd);
        }
        last_tx = c.tilt_x_deg; last_ty = c.tilt_y_deg; last_lx = c.lean_x_des; last_ly = c.lean_y_des;
        bool tilt_bad = est.tiltDeg() > P.max_tilt_thrust_deg;
        bool reached = airborne && pred_along >= D - P.hop_cut_margin_m;
        bool timeout = t_in > P.hop_max_burn_s;
        if (reached || tilt_bad || timeout) {
          burn_s = t_in; cut_pred_along = pred_along; cut_pred_lat = pred_lat;
          cut_by_tilt = tilt_bad && !reached; cut_by_maxburn = timeout && !reached && !tilt_bad;
          if (!airborne) { end_code = cut_by_tilt ? HOP_TILT : HOP_NO_LIFTOFF; go(HState::DONE); }      // CSV: cut, Amp Down
          else go(HState::COAST);
        }
        break;
      }

      case HState::COAST: {
        c.motors_on = true; c.collective = P.idle_throttle;
        if (!airborne && liftoffSeen(P, est)) { airborne = true; t_liftoff = t; }   // thrust tail can still lift it
        if (t_in < P.hop_neutralize_hold_s) {
          c.use_att = false;
          c.tilt_x_deg = last_tx; c.tilt_y_deg = last_ty;
          if (P.hop_attitude_hold) { c.use_att = true; c.lean_x_des = last_lx; c.lean_y_des = last_ly; }
        }
        float fn = norm(est.f_body);
        if (t_in > 0.1f && fn > P.hop_touch_g * kG) { recordTouch(est); end_code = HOP_OK; go(HState::DONE); }
        else if (t_in > P.hop_neutralize_hold_s && A.brake) { land.startTracking(P.hop_land_flare_m, P.hop_land_v_touch); go(HState::LAND); }
        else if (t_in > 4.0f) { end_code = HOP_COAST_TIMEOUT; go(HState::DONE); }
        break;
      }

      case HState::LAND: {
        c = land.update(est, dt);
        if (land.state() == DState::LANDED) { recordTouch(est); end_code = HOP_OK; go(HState::DONE); }
        else if (land.state() != DState::FALLING) { end_code = HOP_LAND_FAULT; go(HState::DONE); }
        break;
      }
      default: break;
    }
    return c;
  }

  // ---- results (for the summary / log) ----
  HopArgs A;
  uint8_t end_code = HOP_OK;
  bool cut_by_maxburn = false, cut_by_tilt = false, airborne = false, past_apex = false, touched = false;
  float cut_pred_along = 0, cut_pred_lat = 0, burn_s = 0, apex_z = 0, t_liftoff = -1;
  Vec3 touch_pos, touch_vel; float touch_tilt = 0;

 private:
  void go(HState s) { st = s; t_in = 0; ok_t = 0; }
  void recordTouch(const Estimator& est) {
    touched = true; touch_pos = est.pos; touch_vel = est.vel; touch_tilt = est.tiltDeg();
    Vec3 d = touch_pos; touch_along = d.x * dx + d.y * dy; touch_lat = -d.x * dy + d.y * dx;
  }

  // Forward-simulate from the current estimate with the thrust decaying (lag_down) toward IDLE thrust along the same
  // direction; report the landing point along / across the commanded direction.
  // FIX (v2): previously decayed to zero thrust; idle thrust was ignored.
  void predict(const Estimator& est) {
    Vec3 p = est.pos, v = est.vel;
    if (airborne) {
      Vec3 f = est.f_world;
      float fm = norm(f);
      Vec3 f_idle = (fm > 0.1f) ? f * (thrustAt(P, P.idle_throttle) / P.mass_kg / fm) : Vec3();
      const float h = 0.01f;
      float k = 1.0f, decay = expf(-h / fmaxf(P.thrust_lag_down_s, 0.02f));
      int hold = (int)(P.cmd_delay_s / h + 0.5f);
      for (int i = 0; i < 300; i++) {
        Vec3 a = f_idle + (f - f_idle) * k - Vec3{0, 0, kG};
        v = v + a * h; p = p + v * h;
        if (i >= hold) k *= decay;
        if (p.z <= 0.0f && v.z < 0.0f) break;
      }
    }
    pred_along = p.x * dx + p.y * dy;
    pred_lat = -p.x * dy + p.y * dx;                    // + = landing is to the LEFT of the commanded line
  }

  const Params& P;
  DescentController land;
  HState st = HState::OFF;
  float th_b = 0, psi0 = 0, dx = 1, dy = 0;
  float t = 0, t_in = 0, ok_t = 0;
  float pred_along = 0, pred_lat = 0;
  float last_tx = 0, last_ty = 0, last_lx = 0, last_ly = 0;
 public:
  float touch_along = 0, touch_lat = 0;
};

// ============================================================================
// Vertical hop ("Launch Vertical Distance"): straight up to a target height, land with the descent brake.
//   SETTLE -> SPINUP (Amp Up: burn throttle, upright held from the first moment) -> liftoff -> BURN (Ascending)
//   -> cut when PREDICTED apex >= target (thrust tail included) -> COAST (Ascending) -> apex -> Descending
//   -> LAND (descent brake from the current height/speed estimate, "entry case B") -> DONE (-> Amp Down)
// ============================================================================
enum class VState : uint8_t { OFF, SETTLE, SPINUP, BURN, COAST, LAND, DONE, REFUSED };
struct VHopArgs { float height_m = 0.5f; bool brake = true; };

class VHopController {
 public:
  explicit VHopController(const Params& p) : P(p), land(p) {}

  bool start(const Estimator& est, const VHopArgs& a) {
    A = a; st = VState::SETTLE; t = t_in = 0; airborne = past_apex = touched = false;
    pred_apex = cut_pred_apex = apex_z = burn_s = 0; t_liftoff = -1; end_code = HOP_OK; cut_by_tilt = cut_by_maxburn = false;
    psi0 = est.yaw();
    if (A.height_m <= 0.0f || A.height_m > P.vhop_max_height_m) { end_code = HOP_BAD_ARGS; st = VState::REFUSED; return false; }
    if (twrAvail(P) * cosf(10.0f * kDeg2Rad) < P.vhop_twr_margin) { end_code = HOP_INFEASIBLE; st = VState::REFUSED; return false; }
    if (A.brake && !land.feasible()) { end_code = HOP_INFEASIBLE; st = VState::REFUSED; return false; }
    return true;
  }
  VState state() const { return st; }
  bool isAirborne() const { return airborne; }
  bool pastApex() const { return past_apex; }
  bool landBraking() const { return st == VState::LAND && land.braking(); }
  float predictedApex() const { return pred_apex; }
  float heading0() const { return psi0; }

  Command update(Estimator& est, float dt) {
    Command c;
    t += dt; t_in += dt;
    if (airborne && !past_apex && est.vel.z < 0.0f) past_apex = true;
    if (airborne) apex_z = fmaxf(apex_z, est.pos.z);
    switch (st) {
      case VState::SETTLE:
        c.motors_on = true;
        if (t_in > P.hop_settle_s && est.atRest()) { est.resetNav(Vec3(), Vec3()); psi0 = est.yaw(); t_spin0 = t; go(VState::SPINUP); }
        else if (t_in > P.settle_timeout_s) { end_code = HOP_NOT_AT_REST; go(VState::REFUSED); }
        break;

      case VState::SPINUP:     // on the ground, thrust rising; upright is unstable above ~4.4 N -> attitude loop from t=0
      case VState::BURN: {
        c.motors_on = true; c.collective = P.vhop_throttle; c.use_att = true; c.lean_x_des = c.lean_y_des = 0;
        if (!airborne && liftoffSeen(P, est)) { airborne = true; t_liftoff = t; go(VState::BURN); }
        pred_apex = predictApex(est);
        float target = A.height_m / P.vhop_height_scale;
        bool reached = airborne && pred_apex >= target - P.vhop_cut_margin_m;
        bool tilt_bad = est.tiltDeg() > P.max_tilt_thrust_deg;
        float t_burn = t - t_spin0;
        bool timeout = t_burn > P.hop_max_burn_s && !airborne;                  // CSV: no liftoff within 0.8 s
        bool too_long = airborne && t_burn > P.hop_max_burn_s + 1.0f;           // backstop on the burn itself
        if (reached || tilt_bad || timeout || too_long) {
          burn_s = t_burn; cut_pred_apex = pred_apex; cut_by_tilt = tilt_bad && !reached; cut_by_maxburn = (timeout || too_long) && !reached;
          if (!airborne) { end_code = cut_by_tilt ? HOP_TILT : HOP_NO_LIFTOFF; go(VState::DONE); }
          else go(VState::COAST);
        }
        break;
      }

      case VState::COAST: {
        c.motors_on = true; c.collective = P.idle_throttle; c.use_att = true; c.lean_x_des = c.lean_y_des = 0;
        if (past_apex && A.brake) { land.startTracking(P.h_flare_m, P.v_touch_mps); go(VState::LAND); break; }
        if (t_in > 0.1f && norm(est.f_body) > P.hop_touch_g * kG) { recordTouch(est); end_code = HOP_OK; go(VState::DONE); }
        else if (t_in > 4.0f) { end_code = HOP_COAST_TIMEOUT; go(VState::DONE); }
        break;
      }

      case VState::LAND: {
        c = land.update(est, dt);
        if (land.state() == DState::LANDED) { recordTouch(est); end_code = HOP_OK; go(VState::DONE); }
        else if (land.state() != DState::FALLING) { end_code = HOP_LAND_FAULT; go(VState::DONE); }
        break;
      }
      default: break;
    }
    return c;
  }

  VHopArgs A;
  uint8_t end_code = HOP_OK;
  bool airborne = false, past_apex = false, touched = false, cut_by_tilt = false, cut_by_maxburn = false;
  float pred_apex = 0, cut_pred_apex = 0, apex_z = 0, burn_s = 0, t_liftoff = -1;
  float touch_vz = 0, touch_tilt = 0, touch_h = 0; bool touch_by_height = false;

 private:
  void go(VState s) { st = s; t_in = 0; }
  void recordTouch(const Estimator& est) {
    touched = true; touch_tilt = est.tiltDeg();
    if (st == VState::LAND) { touch_vz = land.td_vz; touch_h = land.td_h; touch_by_height = land.td_by_height; }
    else { touch_vz = est.vel.z; touch_h = est.pos.z; touch_by_height = false; }
  }
  // Vertical forward simulation: thrust decays from the measured specific force toward idle thrust (lag_down).
  float predictApex(const Estimator& est) const {
    float z = est.pos.z, v = est.vel.z;
    if (!airborne) return z;
    float f0 = est.f_world.z, fi = thrustAt(P, P.idle_throttle) / P.mass_kg;
    const float h = 0.005f; float k = 1.0f, decay = expf(-h / fmaxf(P.thrust_lag_down_s, 0.02f)), zmax = z;
    int hold = (int)(P.cmd_delay_s / h + 0.5f);
    for (int i = 0; i < 600 && v > 0.0f; i++) {
      float a = fi + (f0 - fi) * k - kG;
      v += a * h; z += v * h; if (i >= hold) k *= decay; zmax = fmaxf(zmax, z);
    }
    return zmax;
  }
  const Params& P;
  DescentController land;
  VState st = VState::OFF;
  float t = 0, t_in = 0, t_spin0 = 0, psi0 = 0;
};

// ============================================================================
// Macrostates and the top-level supervisor. Call update() at a fixed rate (e.g. 200 Hz).
// ============================================================================
enum class Macro : uint8_t { REST, ARMED, AWAIT_RELEASE, LAUNCH, AMP_UP, ASCENDING, DESCENDING, LANDING, AMP_DOWN, FAULT };
static inline const char* macroName(Macro m) {
  switch (m) {
    case Macro::REST: return "Rest"; case Macro::ARMED: return "Armed"; case Macro::AWAIT_RELEASE: return "Await Release";
    case Macro::LAUNCH: return "Launch"; case Macro::AMP_UP: return "Amp Up"; case Macro::ASCENDING: return "Ascending";
    case Macro::DESCENDING: return "Descending"; case Macro::LANDING: return "Landing"; case Macro::AMP_DOWN: return "Amp Down";
    case Macro::FAULT: return "Fault";
  }
  return "?";
}
enum class Routine : uint8_t { NONE, DESCENT, HOP, VHOP };
static inline const char* routineName(Routine r) {
  switch (r) { case Routine::NONE: return "none"; case Routine::DESCENT: return "descent"; case Routine::HOP: return "hop"; case Routine::VHOP: return "vhop"; }
  return "?";
}

class Flight {
 public:
  explicit Flight(const Params& p) : P(p), est(P), att(P), desc(P), hop(P), vhop(P) {}

  // ---- supervisor commands (each returns false + reason if refused) ----
  bool arm(const char*& why) {
    if (macro_ != Macro::REST) { why = "can only arm from Rest (use 'disarm' to leave Fault)"; return false; }
    if (!calibrated) { why = "calibration not valid -- run 'cal'"; return false; }
    setMacro(Macro::ARMED); routine = Routine::NONE; why = ""; return true;
  }
  void disarm() { routine = Routine::NONE; desc.stop(); setMacro(Macro::REST); }
  void kill(const char* why) { routine = Routine::NONE; desc.stop(); fault_reason = why; setMacro(Macro::FAULT); }
  bool idleArmed() const { return macro_ == Macro::ARMED && routine == Routine::NONE; }

  bool startDescent(float release_height_m, const char*& why) {
    if (!idleArmed()) { why = "must be Armed with no routine running"; return false; }
    if (release_height_m < 0.5f) { why = "release height < 0.5 m: not a descent"; return false; }
    if (!desc.feasible()) { why = "not enough thrust at the throttle cap to brake (raise 'cap' or reduce payload)"; kill("descent not feasible: not enough thrust to brake"); return false; }   // CSV: -> Fault (does not try)
    beginMission(Routine::DESCENT); desc.arm(release_height_m); H0 = release_height_m;
    setMacro(Macro::AWAIT_RELEASE); why = ""; return true;
  }
  bool startHop(const HopArgs& a, const char*& why) {
    if (!idleArmed()) { why = "must be Armed with no routine running"; return false; }
    if (!hop.start(est, a)) { why = hopEndText(hop.end_code); return false; }        // CSV: not feasible -> stays Armed
    beginMission(Routine::HOP); setMacro(Macro::LAUNCH); why = ""; return true;
  }
  bool startVHop(const VHopArgs& a, const char*& why) {
    if (!idleArmed()) { why = "must be Armed with no routine running"; return false; }
    if (!vhop.start(est, a)) { why = hopEndText(vhop.end_code); return false; }
    beginMission(Routine::VHOP); setMacro(Macro::LAUNCH); why = ""; return true;
  }

  Command update(Vec3 gyro_rads, Vec3 accel_ms2, float dt) {
    Macro m = macro_;
    bool ground = (m == Macro::REST || m == Macro::ARMED || m == Macro::AMP_DOWN || m == Macro::FAULT ||
                   (m == Macro::LAUNCH));
    est.zupt_enable = (m == Macro::REST || m == Macro::ARMED || m == Macro::AMP_DOWN || m == Macro::FAULT ||
                       (routine == Routine::HOP && hop.state() == HState::SETTLE) ||
                       (routine == Routine::VHOP && vhop.state() == VState::SETTLE));
    est.accel_correct = ground || m == Macro::AWAIT_RELEASE;
    est.update(gyro_rads, accel_ms2, dt);
    t_mis += dt;

    Command c;
    switch (routine) {
      case Routine::DESCENT: {
        DState before = desc.state();
        c = desc.update(est, dt);
        DState s = desc.state();
        if (before == DState::WAIT_RELEASE && s == DState::FALLING) { att.setYawRef(est.yaw()); t_flight = 0; post("release detected"); }
        if (s == DState::WAIT_RELEASE) {
          setMacro(Macro::AWAIT_RELEASE);
          if (t_mis > P.await_release_timeout_s) { kill("no release within the await-release timeout"); return Command(); }
        } else if (s == DState::FALLING) {
          setMacro(desc.braking() ? Macro::LANDING : Macro::DESCENDING);
        } else if (s == DState::LANDED) {
          td_vz = desc.td_vz; td_tilt = desc.td_tilt; td_by_height = desc.td_by_height;
          endMission(desc.td_by_height ? "touchdown (declared by estimated height, no impact sensed)" : "touchdown");
          c = Command();
        } else if (s == DState::FAULT) {
          endMission(desc.fault_code == 2 ? "powered-time limit -> motors cut" : "not enough thrust to brake -> motors cut");
          c = Command();
        }
        break;
      }
      case Routine::HOP: {
        c = hop.update(est, dt);
        switch (hop.state()) {
          case HState::SETTLE: case HState::TIP: setMacro(Macro::LAUNCH); break;
          case HState::BURN: setMacro(hop.isAirborne() ? Macro::ASCENDING : Macro::AMP_UP); break;
          case HState::COAST: setMacro(hop.pastApex() ? Macro::DESCENDING : Macro::ASCENDING); break;
          case HState::LAND: setMacro(hop.landBraking() ? Macro::LANDING : Macro::DESCENDING); break;
          case HState::DONE:
            if (hop.touched) { td_vz = hop.touch_vel.z; td_tilt = hop.touch_tilt; }
            endMission(hopEndText(hop.end_code)); c = Command(); break;
          case HState::REFUSED: routine = Routine::NONE; setMacro(Macro::ARMED); post2("hop aborted, back to Armed: ", hopEndText(hop.end_code)); c = Command(); break;
          default: break;
        }
        break;
      }
      case Routine::VHOP: {
        c = vhop.update(est, dt);
        switch (vhop.state()) {
          case VState::SETTLE: setMacro(Macro::LAUNCH); break;
          case VState::SPINUP: setMacro(Macro::AMP_UP); break;
          case VState::BURN: setMacro(Macro::ASCENDING); break;
          case VState::COAST: setMacro(vhop.pastApex() ? Macro::DESCENDING : Macro::ASCENDING); break;
          case VState::LAND: setMacro(vhop.landBraking() ? Macro::LANDING : Macro::DESCENDING); break;
          case VState::DONE:
            if (vhop.touched) { td_vz = vhop.touch_vz; td_tilt = vhop.touch_tilt; td_by_height = vhop.touch_by_height; }
            endMission(hopEndText(vhop.end_code)); c = Command(); break;
          case VState::REFUSED: routine = Routine::NONE; setMacro(Macro::ARMED); post2("vhop aborted, back to Armed: ", hopEndText(vhop.end_code)); c = Command(); break;
          default: break;
        }
        break;
      }
      default: break;
    }

    // ---- flight-time limit (CSV 25 s). Descent: counted from release (a kill mid-air would disable the brake). ----
    if (inFlightMacro()) {
      t_flight += dt;
      peak_f = fmaxf(peak_f, norm(est.f_body));
      bool over = (routine == Routine::DESCENT) ? (t_flight > P.mission_time_s) : (t_mis > P.mission_time_s);
      if (over) { kill("mission time limit"); return Command(); }
    }

    // ---- Amp Down: motors at minimum, gimbal centred, wait for rest, then Rest (auto-disarm) ----
    if (macro_ == Macro::AMP_DOWN) {
      c = Command();
      t_amp += dt;
      peak_f = fmaxf(peak_f, norm(est.f_body));
      amp_still = (est.stillTime() > 0.0f) ? amp_still + dt : 0.0f;   // stillness counted only AFTER entering Amp Down
      bool rested = amp_still > P.ampdown_rest_s;
      if (rested || t_amp > P.ampdown_timeout_s) {
        final_pos = est.pos; final_rested = rested;
        post(rested ? "at rest -> Rest (disarmed)" : "amp-down timeout -> Rest (disarmed)");
        summary_ready = true;
        setMacro(Macro::REST);
      }
    }
    if (macro_ == Macro::REST || macro_ == Macro::FAULT) c = Command();

    int phase = (int)routine * 16 + subState();
    if (phase != last_phase) { att.reset(); last_phase = phase; }
    if (c.use_att) att.update(est, c, dt);
    return c;
  }

  // ---- queries ----
  Macro macro() const { return macro_; }
  bool outputsLive() const { return macro_ != Macro::REST && macro_ != Macro::FAULT; }
  bool missionRunning() const { return routine != Routine::NONE || macro_ == Macro::AMP_DOWN; }
  int subState() const {
    switch (routine) { case Routine::DESCENT: return (int)desc.state(); case Routine::HOP: return (int)hop.state(); case Routine::VHOP: return (int)vhop.state(); default: return 0; }
  }
  float auxValue() const {   // logged "aux": hop = predicted range [m], vhop = predicted apex [m], descent = v_ref [m/s]
    switch (last_routine) { case Routine::HOP: return hop.predictedRange(); case Routine::VHOP: return vhop.predictedApex(); case Routine::DESCENT: return desc.lastVref(); default: return 0; }
  }
  const char* popMsg() { if (mq_r == mq_w) return nullptr; const char* s = mq[mq_r]; mq_r = (mq_r + 1) % kMQ; return s; }

  // ---- members ----
  const Params& P;
  Estimator est;
  AttitudeCtl att;
  DescentController desc;
  HopController hop;
  VHopController vhop;
  Routine routine = Routine::NONE, last_routine = Routine::NONE;
  bool calibrated = false;
  const char* fault_reason = "";
  const char* end_reason = "";
  bool summary_ready = false, final_rested = false, td_by_height = false;
  float H0 = 0, peak_f = 0, td_vz = 0, td_tilt = 0, t_mis = 0, t_flight = 0;
  Vec3 final_pos;

 private:
  bool inFlightMacro() const {
    return macro_ == Macro::AMP_UP || macro_ == Macro::ASCENDING || macro_ == Macro::DESCENDING || macro_ == Macro::LANDING ||
           (macro_ == Macro::LAUNCH && routine != Routine::NONE);
  }
  void beginMission(Routine r) {
    routine = r; last_routine = r; t_mis = 0; t_flight = 0; t_amp = 0; peak_f = 0; td_vz = td_tilt = 0; td_by_height = false;
    end_reason = ""; summary_ready = false; final_pos = Vec3(); final_rested = false;
    att.setYawRef(est.yaw()); att.reset();
  }
  void endMission(const char* why) { end_reason = why; routine = Routine::NONE; t_amp = 0; amp_still = 0; setMacro(Macro::AMP_DOWN); post2("mission end: ", why); }
  void setMacro(Macro m) { if (m != macro_) { macro_ = m; post(macroName(m)); } }
  void post(const char* s) { int nw = (mq_w + 1) % kMQ; if (nw == mq_r) mq_r = (mq_r + 1) % kMQ; mq[mq_w] = s; mq_w = nw; }
  void post2(const char* a, const char* b) { post(a); post(b); }
  static const int kMQ = 16;
  const char* mq[kMQ] = {}; int mq_r = 0, mq_w = 0;
  Macro macro_ = Macro::REST;
  float t_amp = 0, amp_still = 0;
  int last_phase = -1;
};

// ============================================================================
// Mixer: Command -> ESC / servo pulse widths (microseconds).  P.throttle_cap is the runtime safety limit.
// ============================================================================
struct Outputs { uint16_t esc_upper_us, esc_lower_us, servo_x_us, servo_y_us; };

static inline Outputs mix(const Command& c, const Params& P) {
  float cap = clampf(P.throttle_cap, 0.0f, 1.0f);
  float u = c.motors_on ? clampf(c.collective, 0.0f, 1.0f) : 0.0f;
  float du = c.motors_on ? clampf(c.yaw, -P.yaw_limit, P.yaw_limit) * P.yaw_sign : 0.0f;
  float up = clampf(u + du, 0.0f, cap), lo = clampf(u - du, 0.0f, cap);
  if (!c.motors_on) { up = 0; lo = 0; }
  Outputs o;
  float span = (float)(P.esc_max_us - P.esc_min_us);
  o.esc_upper_us = (uint16_t)(P.esc_min_us + up * span);
  o.esc_lower_us = (uint16_t)(P.esc_min_us + lo * span);
  float tx = clampf(c.tilt_x_deg, -P.tilt_limit_deg, P.tilt_limit_deg);
  float ty = clampf(c.tilt_y_deg, -P.tilt_limit_deg, P.tilt_limit_deg);
  float sx = P.servo_center_us + P.servo_sign_x * tx * P.servo_us_per_pod_deg;
  float sy = P.servo_center_us + P.servo_sign_y * ty * P.servo_us_per_pod_deg;
  o.servo_x_us = (uint16_t)clampf(sx, (float)P.servo_min_us, (float)P.servo_max_us);
  o.servo_y_us = (uint16_t)clampf(sy, (float)P.servo_min_us, (float)P.servo_max_us);
  return o;
}

}  // namespace weeble
