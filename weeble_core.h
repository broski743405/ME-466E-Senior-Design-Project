// weeble_core.h -- flight logic for the UTME Weeble drone. Runs unchanged on the ESP32 and in the simulator.
//
// Layout (top to bottom):
//   1. math helpers            6. Estimator          (attitude + dead-reckoned position/velocity)
//   2. Params                  7. AttitudeCtl        (keeps the body at a commanded lean using the gimbal)
//   3. IMU axis mapping        8. DescentController  (20 ft drop -> braked landing)
//   4. accelerometer cal       9. PoweredFlight      (hop and ascent: take off, fly a path, land softly)
//   5. Command                10. Flight             (supervisor: which routine runs, mission states, safety)
//                             11. mix()              (Command -> motor RPMs + servo angles through the 4-bar)
//
// Every physical number comes from weeble_config.h, which build.py generates from drone_config.py.
// Tuning gains live in Params below.
#pragma once
#include <math.h>
#include <stdint.h>
#include "weeble_config.h"

namespace weeble {

// =====================================================================================================
// 1. Math helpers
// =====================================================================================================
static constexpr float kG = 9.80665f;
static constexpr float kPi = 3.14159265358979323846f;
static constexpr float kDeg2Rad = 0.017453292f;
static constexpr float kRad2Deg = 57.29578f;

template <class T> static inline T clampf(T v, T lo, T hi) { return v < lo ? lo : (v > hi ? hi : v); }

struct Vec3 { float x = 0, y = 0, z = 0; };
static inline Vec3 operator+(Vec3 a, Vec3 b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
static inline Vec3 operator-(Vec3 a, Vec3 b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
static inline Vec3 operator*(Vec3 a, float s) { return {a.x * s, a.y * s, a.z * s}; }
static inline float dot(Vec3 a, Vec3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
static inline Vec3 cross(Vec3 a, Vec3 b) { return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x}; }
static inline float norm(Vec3 a) { return sqrtf(dot(a, a)); }

struct Quat { float w = 1, x = 0, y = 0, z = 0; };
static inline Quat qmul(const Quat& a, const Quat& b) {
  return {a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z, a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
          a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x, a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w};
}
static inline Vec3 qrot(const Quat& q, Vec3 v) {  // rotate body-frame vector into the world frame
  Vec3 u{q.x, q.y, q.z}; Vec3 t = cross(u, v) * 2.0f; return v + t * q.w + cross(u, t);
}
static inline Vec3 qrotInv(const Quat& q, Vec3 v) { return qrot(Quat{q.w, -q.x, -q.y, -q.z}, v); }
static inline float wrapPi(float a) { while (a > kPi) a -= 2.0f * kPi; while (a < -kPi) a += 2.0f * kPi; return a; }

// =====================================================================================================
// 2. Params -- physical values from the config, plus every tuning gain in one place
// =====================================================================================================
struct Params {
  // ---- physical (generated config; mass and thrust can be changed at run time) ----
  float mass_kg = cfg::MASS_KG;
  float inertia[3] = {cfg::INERTIA_XX, cfg::INERTIA_YY, cfg::INERTIA_ZZ};
  float pivot_arm_m = cfg::PIVOT_ABOVE_COM_M;
  float thrust_max_N = cfg::THRUST_MAX_N, thrust_curve_exp = 1.0f;  // thrust = thrust_max * throttle^exp
  float thrust_lag_up_s = cfg::MOTOR_LAG_UP_S, thrust_lag_down_s = cfg::MOTOR_LAG_DOWN_S;
  float idle_throttle = cfg::IDLE_THROTTLE;
  float throttle_cap = 0.15f;                       // boot-time safety cap; firmware 'cap 100' to fly
  float tilt_limit_deg = cfg::GIMBAL_LIMIT_DEG;     // gimbal never commanded past this

  // ---- mixer / output hardware ----
  uint16_t esc_min_us = 1000, esc_max_us = 2000;
  float servo_sign_x = 1.0f, servo_sign_y = 1.0f, yaw_sign = 1.0f;
  bool use_four_bar_linkage = true;
  float linkage_lengths[4] = {2.5f, 3.0f, 4.1f, 2.0f};
  float gimbal_home_deg = 45.0f, th2_initial_deg = 25.0f;
  float servo_center_rad = 1.5707963f, servo_travel_rad = 3.14159265f;
  bool use_closed_loop_rpm = true;
  float max_rpm = cfg::MAX_RPM;

  // ---- attitude loop (model-based: gains are in "how fast", the code converts using inertia/arm/thrust) ----
  float att_wn = 12.0f;            // natural frequency [rad/s] of the lean response
  float att_zeta = 0.8f;           // damping ratio
  float att_ki_ratio = 0.5f;       // integral gain = ratio * wn^3 (removes trim from CG offsets); stable below 1.6
  float att_i_max = 25.0f;         // max integral contribution [rad/s^2]
  float att_thrust_floor = 0.25f;  // below this fraction of weight, assume this much thrust (limits gain)
  float att_wn_frac = 0.15f;       // keep wn below this fraction of the gimbal kick-back frequency (see AttitudeCtl)
  float att_rate_lpf_hz = 20.0f;   // low-pass on the rate used for damping
  float yaw_kp_rate = 0.002f, yaw_kp_ang = 0.004f, yaw_limit = 0.08f;  // throttle difference per deg/s, per deg

  // ---- estimator ----
  float mahony_kp = 1.0f, accel_gate_g = 0.15f, rest_time_s = 0.30f;

  // ---- descent (drop from the mothership) ----
  float freefall_g = 0.35f, freefall_debounce_s = 0.04f, zero_g_cal_s = 0.10f;
  bool use_rest_scale = true;
  float land_cut_height_m = -0.50f, brake_plan_fraction = 0.45f, lead_extra_s = 0.05f;   // plan with 45 % of spare decel (margin for battery sag)
  float v_touch_mps = 0.8f, h_flare_m = 0.3f, vel_kp = 1.0f, vel_kp_slow = 3.0f, acc_fb_gain = 0.5f;
  float impact_g = 3.0f, contact_delta_g = 0.6f, max_tilt_thrust_deg = 50.0f, max_powered_s = 6.0f, min_spare_decel = 0.5f;

  // ---- powered flight (hop + ascent) ----
  float hop_cruise_m = cfg::HOP_CRUISE_HEIGHT_M, hop_max_lean_deg = cfg::HOP_MAX_LEAN_DEG;
  float hop_max_accel = cfg::HOP_MAX_ACCEL_MPS2, climb_max_accel = cfg::CLIMB_MAX_ACCEL_MPS2;
  float ascent_max_m = cfg::ASCENT_MAX_HEIGHT_M, ascent_hover_s = cfg::ASCENT_HOVER_S;
  float hop_hover_s = 0.25f;                    // pause over the target before landing
  float land_speed = cfg::LAND_SPEED_MPS, touch_speed = cfg::TOUCH_SPEED_MPS, touch_height_m = 0.15f;
  float land_accel = 1.5f;                      // how quickly the sink rate changes
  float alt_kp = 3.0f, alt_kd = 3.5f, alt_ki = 2.0f, alt_i_max = 1.5f;      // height loop [1/s^2, 1/s, 1/s^3, m*s]
  float pos_kp = 3.0f, pos_kd = 3.2f;                                     // horizontal loop [1/s^2, 1/s]
  float hop_ff_lead_s = 0.20f;     // use the path acceleration this far ahead (lean takes time to build up)
  float settle_s = 0.4f, settle_timeout_s = 3.0f;
  float spinup_frac = 0.7f, spinup_s = 0.6f, level_tol_deg = 4.0f, spinup_timeout_s = 3.5f;
  float level_rate_dps = 12.0f;    // how fast spin-up walks the lean target from the resting tilt to level
  float twr_margin = 1.15f, flight_max_s = 15.0f;
  float liftoff_boost_rate = 3.0f, liftoff_boost_max = 4.0f, liftoff_timeout_s = 3.0f;   // m/s^3, m/s^2, s
  float boost_fade_s = 0.25f;      // after liftoff the take-off boost fades with this time constant
  float ampdown_hold_s = 0.6f;     // keep the gimbal holding level at least this long after motor cut
  float touch_impact_g = 1.0f, touch_support_g = 0.3f;   // touchdown: extra specific force beyond thrust
  float stall_vz = 0.12f, stall_s = 0.15f;               // touchdown: sinking this much slower than commanded

  // ---- supervisor ----
  float thrust_learn_s = 0.6f;     // time constant for learning real thrust / modelled thrust in flight
  float await_release_timeout_s = 25.0f, mission_time_s = 25.0f, ampdown_rest_s = 0.3f, ampdown_timeout_s = 3.0f;
};

static inline void setMass(Params& P, float kg) { P.mass_kg = kg; }
static inline float thrustAt(const Params& P, float u) { return P.thrust_max_N * powf(clampf(u, 0.0f, 1.0f), P.thrust_curve_exp); }
static inline float thrustAvail(const Params& P) { return thrustAt(P, P.throttle_cap); }
static inline float twrAvail(const Params& P) { return thrustAvail(P) / (P.mass_kg * kG); }
static inline float spareDecel(const Params& P) { return thrustAvail(P) / P.mass_kg - kG; }
static inline float leadTime(const Params& P) { return fmaxf(P.thrust_lag_up_s + P.lead_extra_s, 0.02f); }
static inline float thrustToThrottle(const Params& P, float thrust_N) {
  float frac = clampf(thrust_N / P.thrust_max_N, 0.0f, 1.0f);
  return powf(frac, 1.0f / P.thrust_curve_exp);
}

// =====================================================================================================
// 3. IMU axis mapping: chip axes -> body axes (set by IMU_AXES in drone_config.py)
// =====================================================================================================
static inline Vec3 imuToBody(Vec3 v) {
  const float a[3] = {v.x, v.y, v.z};
  return Vec3{cfg::IMU_AXIS_SIGN[0] * a[cfg::IMU_AXIS_SRC[0]], cfg::IMU_AXIS_SIGN[1] * a[cfg::IMU_AXIS_SRC[1]],
              cfg::IMU_AXIS_SIGN[2] * a[cfg::IMU_AXIS_SRC[2]]};
}

// =====================================================================================================
// 4. Six-position accelerometer calibration (offset + scale per axis, chip frame)
// =====================================================================================================
struct AccelCal {
  uint32_t magic = 0x57454231u; float off[3] = {0, 0, 0}; float scl[3] = {1, 1, 1};
  float rms = 0; uint8_t n_poses = 0; bool valid = false;
};
static inline void applyAccelCal(const AccelCal& c, float a[3]) {
  if (!c.valid) return;
  for (int i = 0; i < 3; i++) a[i] = (a[i] - c.off[i]) * c.scl[i];
}
static inline bool solve6(float A[6][6], float b[6], float x[6]) {
  for (int c = 0; c < 6; c++) {
    int p = c; for (int r = c + 1; r < 6; r++) if (fabsf(A[r][c]) > fabsf(A[p][c])) p = r;
    if (fabsf(A[p][c]) < 1e-9f) return false;
    if (p != c) { for (int k = 0; k < 6; k++) { float t = A[c][k]; A[c][k] = A[p][k]; A[p][k] = t; } float t = b[c]; b[c] = b[p]; b[p] = t; }
    for (int r = c + 1; r < 6; r++) { float f = A[r][c] / A[c][c]; for (int k = c; k < 6; k++) A[r][k] -= f * A[c][k]; b[r] -= f * b[c]; }
  }
  for (int r = 5; r >= 0; r--) { float s = b[r]; for (int k = r + 1; k < 6; k++) s -= A[r][k] * x[k]; x[r] = s / A[r][r]; }
  return true;
}
// Gauss-Newton fit so that |(raw - off) * scl| = g for every pose.
static inline bool solveAccelCal(const Vec3* s, int n, AccelCal& out, const char*& why) {
  why = ""; if (n < 6) { why = "need at least 6 poses"; return false; }
  float mn[3] = {1e9f, 1e9f, 1e9f}, mx[3] = {-1e9f, -1e9f, -1e9f};
  for (int k = 0; k < n; k++) { const float v[3] = {s[k].x, s[k].y, s[k].z}; for (int i = 0; i < 3; i++) { mn[i] = fminf(mn[i], v[i]); mx[i] = fmaxf(mx[i], v[i]); } }
  for (int i = 0; i < 3; i++) if (mx[i] < 0.5f * kG || mn[i] > -0.5f * kG) { why = "poses do not cover every axis pointing up AND down"; return false; }
  float p[6];
  for (int i = 0; i < 3; i++) { p[i] = 0.5f * (mx[i] + mn[i]); p[3 + i] = 2.0f * kG / (mx[i] - mn[i]); }
  for (int it = 0; it < 30; it++) {
    float JTJ[6][6] = {}, JTr[6] = {};
    for (int k = 0; k < n; k++) {
      const float v[3] = {s[k].x, s[k].y, s[k].z}; float u[3], nn = 0;
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
    ss += (sqrtf(nn) - kG) * (sqrtf(nn) - kG);
  }
  AccelCal c;
  for (int i = 0; i < 3; i++) { c.off[i] = p[i]; c.scl[i] = p[3 + i]; }
  c.rms = sqrtf(ss / n); c.n_poses = (uint8_t)n; c.valid = true;
  for (int i = 0; i < 3; i++) {
    if (fabsf(c.off[i]) > 1.5f) { why = "offset > 1.5 m/s^2"; return false; }
    if (fabsf(c.scl[i] - 1.0f) > 0.08f) { why = "scale off > 8 %"; return false; }
  }
  if (n > 6 && c.rms > 0.10f) { why = "residual > 0.10 m/s^2 (drone moved?)"; return false; }
  out = c; return true;
}

// =====================================================================================================
// 5. Command -- what a routine asks for each tick
// =====================================================================================================
struct Command {
  bool motors_on = false;
  float collective = 0;                  // throttle 0..1 (thrust = thrust_max * collective)
  bool use_att = false;                  // true: AttitudeCtl fills tilt_x/y from lean_x/y_des
  float lean_x_des = 0, lean_y_des = 0;  // desired BODY lean [deg]; +x = top of drone toward body +x
  float tilt_x_deg = 0, tilt_y_deg = 0;  // gimbal (motor stack) tilt relative to the body [deg]
  float yaw = 0;                         // upper/lower throttle difference
  bool on_ground = false;                // sitting on the base: AttitudeCtl adds ground tipping compensation
};

// =====================================================================================================
// 6. Estimator -- orientation (Mahony filter), dead-reckoned position/velocity, rest detection
// =====================================================================================================
class Estimator {
 public:
  explicit Estimator(const Params& p) : P(p) {}

  // Call once at rest with the average accelerometer reading (body frame).
  void initAtRest(Vec3 f_mean) {
    float n = norm(f_mean); Vec3 a = f_mean * (1.0f / n);
    if (a.z < -0.99f) { q = Quat{0, 1, 0, 0}; }
    else { Quat r{1.0f + a.z, a.y, -a.x, 0.0f}; float m = sqrtf(r.w * r.w + r.x * r.x + r.y * r.y + r.z * r.z); q = Quat{r.w / m, r.x / m, r.y / m, r.z / m}; }
    accel_bias = a * (n - kG); accel_scale = 1.0f; f_rest_raw = f_mean;
    pos = Vec3(); vel = Vec3(); still_t = 0; calm_t = 0; gyro_lp = Vec3(); f_meas_lp = kG;
  }
  // Mid-air "true zero g" reading taken during free fall (descent only).
  void applyZeroG(Vec3 raw_mean_in_freefall, bool use_rest_scale) {
    float span = norm(f_rest_raw - raw_mean_in_freefall); accel_bias = raw_mean_in_freefall;
    float k = (span > 1.0f) ? kG / span : 1.0f;
    accel_scale = (use_rest_scale && fabsf(k - 1.0f) < 0.06f) ? k : 1.0f;
  }
  void setGyroBias(Vec3 b) { gyro_bias = b; }
  void resetNav(Vec3 p, Vec3 v) { pos = p; vel = v; }

  // gyro [rad/s] and accel [m/s^2] already in BODY axes (use imuToBody first).
  void update(Vec3 gyro_raw, Vec3 accel, float dt) {
    gyro = gyro_raw - gyro_bias; accel_raw = accel;
    f_body = (accel - accel_bias) * accel_scale;

    // IMU not at the centre of mass: remove the "spinning" accelerations it feels that the CG does not.
    // f_cg = f_imu - w x (w x r) - alpha x r     (r = IMU position relative to the CG, from the config)
    const Vec3 r{cfg::IMU_POS_M[0], cfg::IMU_POS_M[1], cfg::IMU_POS_M[2]};
    if (dot(r, r) > 1e-8f) {
      Vec3 alpha = (gyro - gyro_prev) * (1.0f / fmaxf(dt, 1e-4f));
      alpha_lp = alpha_lp + (alpha - alpha_lp) * fminf(dt / 0.02f, 1.0f);
      f_body = f_body - cross(gyro, cross(gyro, r)) - cross(alpha_lp, r);
    }
    gyro_prev = gyro;

    // Mahony: nudge the attitude toward "accelerometer = up", only when the drone is not accelerating.
    float fn = norm(f_body); Vec3 w = gyro;
    // (and not rotating: a rocking/rolling body's accelerometer feels its own motion, not just gravity)
    if (accel_correct && fabsf(fn / kG - 1.0f) < P.accel_gate_g && fn > 1.0f && norm(gyro) < 0.12f) {
      Vec3 fhat = f_body * (1.0f / fn); Vec3 vhat = qrotInv(q, Vec3{0, 0, 1}); w = w + cross(fhat, vhat) * P.mahony_kp;
    }
    Quat dq = qmul(q, Quat{0, w.x, w.y, w.z});
    q.w += 0.5f * dq.w * dt; q.x += 0.5f * dq.x * dt; q.y += 0.5f * dq.y * dt; q.z += 0.5f * dq.z * dt;
    float m = sqrtf(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
    q.w /= m; q.x /= m; q.y /= m; q.z /= m;

    // Dead reckoning: specific force -> world acceleration -> velocity -> position
    f_world = qrot(q, f_body); a_world = f_world - Vec3{0, 0, kG};
    f_meas_lp += (f_body.z - f_meas_lp) * fminf(dt / 0.03f, 1.0f);
    pos = pos + vel * dt + a_world * (0.5f * dt * dt); vel = vel + a_world * dt;

    // Rest detection + zero-velocity update (only when the supervisor allows it, i.e. on the ground)
    bool still = norm(gyro) < 0.06f && fabsf(fn - kG) < 0.4f;
    still_t = still ? still_t + dt : 0.0f;
    if (zupt_enable && still_t > P.rest_time_s) vel = Vec3();

    // "Calm" is much stricter than "still": a weeble slowly rocking (1-2 deg/s) counts as still but not calm.
    // Gyro bias is only re-learned when calm for a second, and slowly, so rocking is never mistaken for bias.
    gyro_lp = gyro_lp + (gyro - gyro_lp) * fminf(dt / 0.05f, 1.0f);
    bool calm = norm(gyro_lp) < 0.012f && fabsf(fn - kG) < 0.2f;
    calm_t = calm ? calm_t + dt : 0.0f;
    if (zupt_enable && calm_t > 1.0f) gyro_bias = gyro_bias + (gyro_raw - gyro_bias) * (dt / 8.0f);
  }

  bool atRest() const { return still_t > P.rest_time_s; }
  bool calm(float secs = 0.3f) const { return calm_t > secs; }
  float stillTime() const { return still_t; }
  Vec3 upInBody() const { return qrotInv(q, Vec3{0, 0, 1}); }
  float tiltDeg() const { return acosf(clampf(upInBody().z, -1.0f, 1.0f)) * kRad2Deg; }
  float leanXDeg() const { return asinf(clampf(-upInBody().x, -1.0f, 1.0f)) * kRad2Deg; }  // + = top toward body +x
  float leanYDeg() const { return asinf(clampf(-upInBody().y, -1.0f, 1.0f)) * kRad2Deg; }  // + = top toward body +y
  float yaw() const { return atan2f(2.0f * (q.w * q.z + q.x * q.y), 1.0f - 2.0f * (q.y * q.y + q.z * q.z)); }

  Quat q; Vec3 pos, vel, gyro, f_body, f_world, a_world, accel_bias, gyro_bias, accel_raw, f_rest_raw;
  float accel_scale = 1.0f;
  float thrust_model_N = 0.0f;   // thrust the motors make right now (first-order lag of the command; Flight keeps it)
  float thrust_k = 1.0f;         // learned in flight: real thrust / modelled thrust (battery sag, wrong THRUST_MAX_N)
  float thrustNow() const { return thrust_model_N * thrust_k; }   // best guess of the real thrust right now
  float f_meas_lp = kG;          // ~30 ms low-pass of measured body-z specific force [m/s^2]
  bool zupt_enable = false, accel_correct = true;

 private:
  const Params& P; float still_t = 0, calm_t = 0; Vec3 gyro_prev, alpha_lp, gyro_lp;
};

// =====================================================================================================
// 7. AttitudeCtl -- turn "desired body lean" into gimbal tilt
//    1) pick a desired angular acceleration like a spring-damper:  alpha = wn^2*err - 2*zeta*wn*rate + ki*integral
//    2) torque needed = inertia * alpha
//    3) the gimbal makes torque = thrust * arm * sin(tilt)  ->  tilt = asin(torque / (thrust * arm))
//    Because inertia, arm and thrust come from the config/thrust model, the response stays the same
//    when the hardware changes. (+tilt_x pushes the top toward +x, +tilt_y toward +y.)
//    Limit: swinging the heavy motor stack kicks the body the OTHER way. Above w_c = sqrt(T*arm/I_couple)
//    that kick is bigger than the thrust torque, so wn is capped at att_wn_frac * w_c (lower at low thrust).
// =====================================================================================================
class AttitudeCtl {
 public:
  explicit AttitudeCtl(const Params& p) : P(p) {}
  void setYawRef(float psi) { yaw_ref = psi; }
  void reset() { ix = iy = 0; rx_f = ry_f = 0; }

  void update(const Estimator& e, Command& c, float dt) {
    float lean_x = e.leanXDeg() * kDeg2Rad, lean_y = e.leanYDeg() * kDeg2Rad;
    float k = fminf(dt * 2.0f * kPi * P.att_rate_lpf_hz, 1.0f);
    rx_f += (e.gyro.y - rx_f) * k; ry_f += (-e.gyro.x - ry_f) * k;
    float rate_x = rx_f, rate_y = ry_f;                         // how fast each lean angle is changing (filtered)
    float ex = c.lean_x_des * kDeg2Rad - lean_x, ey = c.lean_y_des * kDeg2Rad - lean_y;

    float T = fmaxf(e.thrustNow(), P.att_thrust_floor * P.mass_kg * kG);
    float arm = (fabsf(P.pivot_arm_m) > 0.01f) ? P.pivot_arm_m : 0.01f;
    float w_c = sqrtf(T * fabsf(arm) / cfg::GIMBAL_COUPLING_INERTIA);
    float wn = fminf(P.att_wn, P.att_wn_frac * w_c), zt = P.att_zeta;
    wn_used = wn;
    float smax = sinf(P.tilt_limit_deg * kDeg2Rad);

    float ki = P.att_ki_ratio * wn * wn * wn;                    // integral scaled with the bandwidth
    float i_lim = P.att_i_max / fmaxf(ki, 1e-3f);
    float ax = wn * wn * ex - 2.0f * zt * wn * rate_x + ki * ix;
    float ay = wn * wn * ey - 2.0f * zt * wn * rate_y + ki * iy;
    float sx = P.inertia[1] * ax / (T * arm), sy = P.inertia[0] * ay / (T * arm);

    // integrate only while not saturated in that direction (anti-windup)
    if (fabsf(ex) < 0.3f && !(sx > smax && ex > 0) && !(sx < -smax && ex < 0)) ix = clampf(ix + ex * dt, -i_lim, i_lim);
    if (fabsf(ey) < 0.3f && !(sy > smax && ey > 0) && !(sy < -smax && ey < 0)) iy = clampf(iy + ey * dt, -i_lim, i_lim);

    // On the ground the thrust line passes through the base's centre of curvature, so thrust T tips the drone with
    // moment (T*R - m*g*margin)*sin(lean) about the contact point. The gimbal acts there with arm Lp (pivot height
    // above the contact). Cancel that tipping moment directly; the spring-damper above then only has to do the rest.
    float T_real = e.thrustNow();
    if (c.on_ground && T_real > 0.5f) {
      float Lp = cfg::PIVOT_ABOVE_COM_M + cfg::COM_ABOVE_BOTTOM_M;
      float tip = (T_real * cfg::BASE_RADIUS_M - P.mass_kg * kG * cfg::BASE_CENTER_ABOVE_COM_M) / (T_real * Lp);
      if (tip > 0.0f) { sx -= tip * sinf(lean_x); sy -= tip * sinf(lean_y); }
    }
    c.tilt_x_deg = asinf(clampf(sx, -smax, smax)) * kRad2Deg;
    c.tilt_y_deg = asinf(clampf(sy, -smax, smax)) * kRad2Deg;

    float wz = e.gyro.z * kRad2Deg, dpsi = wrapPi(e.yaw() - yaw_ref) * kRad2Deg;
    c.yaw = clampf(-(P.yaw_kp_rate * wz + P.yaw_kp_ang * dpsi), -P.yaw_limit, P.yaw_limit);
  }

  float wn_used = 0;   // bandwidth actually used last tick (for telemetry)

 private:
  const Params& P; float yaw_ref = 0, ix = 0, iy = 0, rx_f = 0, ry_f = 0;
};

static inline bool liftoffSeen(const Estimator& e) {
  return (e.vel.z > 0.20f) || (e.pos.z > 0.03f && e.vel.z > 0.05f);
}

// =====================================================================================================
// 8. DescentController -- released from the mothership, detect free fall, brake, land
// =====================================================================================================
enum class DState : uint8_t { OFF, WAIT_RELEASE, FREE_FALL, LANDED, FAULT };

class DescentController {
 public:
  explicit DescentController(const Params& p) : P(p) {}
  bool feasible() const { return spareDecel(P) >= P.min_spare_decel; }
  void arm(float release_height_m) {
    H0 = release_height_m; st = DState::WAIT_RELEASE; ff_t = 0; t = 0;
    ucorr = 0; impact_cnt = 0; f_cmd_last = 0; braking_ = false; fault_code = 0; f_cmd_lag = -1.0f;
    v_pre_impact = v_last_step = 0; td_vz = td_tilt = td_h = 0; td_by_height = false;
    impact_gate_s = 0.15f; zg_done = (P.zero_g_cal_s <= 0.0f);
  }
  void stop() { st = DState::OFF; }
  DState state() const { return st; }
  bool braking() const { return braking_; }
  float lastVref() const { return vref_last; }
  int fault_code = 0;
  float td_vz = 0, td_tilt = 0, td_h = 0;
  bool td_by_height = false;

  Command update(Estimator& est, float dt) {
    Command c; t += dt;
    switch (st) {
      case DState::WAIT_RELEASE: {                          // hanging: wait for < 0.35 g for 40 ms
        c.motors_on = true; c.collective = 0.0f;
        if (norm(est.f_body) < P.freefall_g * kG) ff_t += dt; else ff_t = 0;
        if (ff_t >= P.freefall_debounce_s) {
          float td = ff_t; est.resetNav(Vec3{0, 0, H0 - 0.5f * kG * td * td}, Vec3{0, 0, -kG * td});
          st = DState::FREE_FALL; t = 0; impact_cnt = 0; ucorr = 0; t_fall0 = td; zg_n = 0; zg_sum = Vec3();
        }
        break;
      }
      case DState::FREE_FALL: {
        if (!zg_done) {                                     // 0.1 s motors-off: measure the true zero-g reading
          zg_sum = zg_sum + est.accel_raw; zg_n++;
          if (t >= P.zero_g_cal_s) {
            est.applyZeroG(zg_sum * (1.0f / (float)zg_n), P.use_rest_scale);
            float tt = t_fall0 + t; est.resetNav(Vec3{0, 0, H0 - 0.5f * kG * tt * tt}, Vec3{0, 0, -kG * tt}); zg_done = true;
          }
          c.motors_on = true; c.collective = 0.0f; c.use_att = false;
          break;
        }
        // touchdown detection (speed is taken from the step BEFORE the impact spike)
        float fn = norm(est.f_body);
        bool hard = fn > P.impact_g * kG;
        bool soft = (f_cmd_last > 0.0f) && (est.f_body.z > f_cmd_last + P.contact_delta_g * kG);
        if (t > impact_gate_s && (hard || soft)) { if (impact_cnt == 0) v_pre_impact = v_last_step; impact_cnt++; } else impact_cnt = 0;
        bool by_h = est.pos.z <= P.land_cut_height_m;
        if (impact_cnt >= 2 || by_h) {
          td_vz = (impact_cnt >= 2) ? v_pre_impact : est.vel.z; td_tilt = est.tiltDeg(); td_h = est.pos.z;
          td_by_height = by_h && impact_cnt < 2; st = DState::LANDED; break;
        }
        if (t > P.max_powered_s) { fault_code = 2; st = DState::FAULT; break; }

        float f_full = thrustAvail(P) * est.thrust_k / P.mass_kg;   // what the motors can REALLY give (learned)
        float a_avail = f_full - kG;
        if (a_avail < P.min_spare_decel) { fault_code = 1; st = DState::FAULT; break; }
        float a_plan = P.brake_plan_fraction * a_avail;

        // Lag-aware braking curve: the fastest we may be falling at this height. While the motors spin up the
        // drone keeps accelerating at (g - f_now); f_now = thrust/mass right now (model vs accelerometer, smaller wins).
        float h = est.pos.z, v = est.vel.z, hh = fmaxf(h - P.h_flare_m, 0.0f), vt = P.v_touch_mps;
        float tau = leadTime(P);
        float f_now = clampf(fminf(est.thrustNow() / P.mass_kg, est.f_meas_lp), 0.0f, f_full);
        float g_eff = kG - f_now;
        float disc = a_plan * a_plan * tau * tau + vt * vt + 2.0f * a_plan * hh + a_plan * g_eff * tau * tau;
        float V1 = -a_plan * tau + sqrtf(fmaxf(disc, 0.0f));
        float v_ref = -fmaxf(V1 - g_eff * tau, vt); vref_last = v_ref;
        float a_ff = clampf((v * v - vt * vt) / (2.0f * fmaxf(hh, 0.05f)), 0.0f, a_plan);
        float kp = (v_ref - v < 0.0f) ? P.vel_kp_slow : P.vel_kp;
        float a_up = clampf(a_ff + kp * (v_ref - v), -kG, a_avail);
        float f_cmd = kG + a_up; f_cmd_last = f_cmd;

        // accel feedback against the lag-delayed command: corrects thrust-magnitude error (battery sag)
        if (f_cmd_lag < 0.0f) f_cmd_lag = est.thrustNow() / P.mass_kg;
        float lag = (f_cmd > f_cmd_lag) ? P.thrust_lag_up_s : P.thrust_lag_down_s;
        f_cmd_lag += (f_cmd - f_cmd_lag) * fminf(dt / fmaxf(lag, 0.01f), 1.0f);
        float ucap = fminf(P.throttle_cap, 1.0f);
        float u_ff = thrustToThrottle(P, f_cmd * P.mass_kg / est.thrust_k);
        if (u_ff > P.idle_throttle + 0.05f) {
          braking_ = true; float du = P.acc_fb_gain * (f_cmd_lag - est.f_meas_lp) / f_full * dt;
          if (!(du > 0.0f && u_ff + ucorr >= ucap)) ucorr = clampf(ucorr + du, -0.25f, 0.25f);
        } else { ucorr *= 0.95f; }
        float u = clampf(u_ff + ucorr, P.idle_throttle, ucap);
        if (est.tiltDeg() > P.max_tilt_thrust_deg) u = P.idle_throttle;

        c.motors_on = true; c.collective = u; c.use_att = true; v_last_step = est.vel.z;
        break;
      }
      default: break;
    }
    return c;
  }

 private:
  const Params& P; DState st = DState::OFF;
  float H0 = 6.0f, ff_t = 0, t = 0, ucorr = 0, vref_last = 0, f_cmd_last = 0, t_fall0 = 0, impact_gate_s = 0.15f;
  float f_cmd_lag = -1.0f, v_pre_impact = 0, v_last_step = 0;
  Vec3 zg_sum; int zg_n = 0; bool zg_done = true; int impact_cnt = 0; bool braking_ = false;
};

// =====================================================================================================
// 9. PoweredFlight -- hop and ascent
//    SETTLE   wait until still, zero the position (this spot = origin)
//    SPINUP   spin to half hover thrust while holding the body level (the round base is unstable with thrust on)
//    CLIMB    rise to cruise height along a smooth path
//    TRANSLATE (hop only) fly to the target: lean forward to speed up, lean back to slow down
//    HOVER    hold over the target (ascent: hold at the top)
//    LAND     sink slowly while holding position; cut the motors on touchdown
//    The path is "minimum-jerk": position, speed AND acceleration all start and end at zero, so the
//    controller can follow it smoothly. Tracking it gives near-zero speed at touchdown.
// =====================================================================================================
struct MinJerk {
  float p0 = 0, p1 = 0, T = 1;
  void eval(float t, float& p, float& v, float& a) const {
    float s = clampf(t / T, 0.0f, 1.0f), D = p1 - p0;
    p = p0 + D * (s * s * s * (10.0f - 15.0f * s + 6.0f * s * s));
    v = D / T * (30.0f * s * s - 60.0f * s * s * s + 30.0f * s * s * s * s);
    a = D / (T * T) * (60.0f * s - 180.0f * s * s + 120.0f * s * s * s);
    if (t >= T) { v = 0; a = 0; }
  }
  // shortest duration that keeps the peak acceleration below amax (peak = 5.774 * D / T^2)
  static float durationFor(float dist, float amax) { return sqrtf(5.774f * fabsf(dist) / fmaxf(amax, 0.05f)); }
  // time (in the first half of the path) at which the path speed equals v -- used to join a path already moving
  float timeAtSpeed(float v) const {
    float lo = 0, hi = 0.5f * T, p, vv, a;
    for (int i = 0; i < 20; i++) { float m = 0.5f * (lo + hi); eval(m, p, vv, a); if (vv * (p1 - p0 >= 0 ? 1 : -1) < fabsf(v)) lo = m; else hi = m; }
    return lo;
  }
};

enum class PState : uint8_t { OFF, SETTLE, SPINUP, CLIMB, TRANSLATE, HOVER, LAND, DONE, REFUSED };
enum FlightEnd : uint8_t { END_OK = 0, END_INFEASIBLE, END_NOT_AT_REST, END_NOT_LEVEL, END_TILT, END_TIMEOUT, END_BAD_ARGS, END_NO_LIFTOFF };
static inline const char* flightEndText(uint8_t e) {
  switch (e) {
    case END_OK: return "touchdown";
    case END_INFEASIBLE: return "not feasible: thrust-to-weight at cap too low";
    case END_NOT_AT_REST: return "drone not at rest";
    case END_NOT_LEVEL: return "could not level the body during spin-up";
    case END_TILT: return "tilt guard (body tilted past limit)";
    case END_TIMEOUT: return "flight time limit";
    case END_BAD_ARGS: return "bad arguments";
    case END_NO_LIFTOFF: return "no liftoff (not enough thrust)";
    default: return "?";
  }
}

class PoweredFlight {
 public:
  struct Plan { float dist_m = 0, bearing_deg = 0, height_m = 0.3f, hover_s = 0.4f; };

  explicit PoweredFlight(const Params& p) : P(p) {}

  bool start(const Estimator& est, const Plan& plan) {
    A = plan; st = PState::SETTLE; t_in = t_all = 0; end_code = END_OK; airborne = touched = false;
    alt_i = 0; boost = 0; apex_z = 0; stall_t = support_t = level_t = lift_t = lift_v = 0; climb_t0 = 0; want_liftoff = false; k_seen = 1.0f;
    psi0 = est.yaw(); reset_refs();
    if (A.height_m <= 0.05f || A.dist_m < 0 || A.dist_m > 5.0f) { end_code = END_BAD_ARGS; st = PState::REFUSED; return false; }
    if (twrAvail(P) * cosf((P.hop_max_lean_deg + 5.0f) * kDeg2Rad) < P.twr_margin) { end_code = END_INFEASIBLE; st = PState::REFUSED; return false; }
    return true;
  }

  PState state() const { return st; }
  bool isAirborne() const { return airborne; }
  float refX() const { return xr; }
  float refY() const { return yr; }
  float refZ() const { return zr; }
  float apex() const { return apex_z; }
  uint8_t end_code = END_OK;
  bool touched = false;
  Vec3 touch_pos, touch_vel;

  Command update(Estimator& est, float dt) {
    Command c; t_in += dt; t_all += dt;
    if (airborne) apex_z = fmaxf(apex_z, est.pos.z);
    bool flying = (st == PState::SPINUP || st == PState::CLIMB || st == PState::TRANSLATE || st == PState::HOVER || st == PState::LAND);
    if (flying && est.tiltDeg() > P.max_tilt_thrust_deg) { end_code = END_TILT; go(PState::DONE); return Command(); }
    if (flying && st != PState::LAND && t_all > P.flight_max_s) { end_code = END_TIMEOUT; startLand(est); }

    switch (st) {
      case PState::SETTLE: {
        c.motors_on = true;
        if (t_in > P.settle_s && est.atRest() && est.calm()) {
          est.resetNav(Vec3(), Vec3()); psi0 = est.yaw();
          float th = psi0 - A.bearing_deg * kDeg2Rad;            // bearing: clockwise from body +x, seen from above
          dirx = cosf(th); diry = sinf(th); reset_refs(); go(PState::SPINUP);
        } else if (t_in > P.settle_timeout_s) { end_code = END_NOT_AT_REST; go(PState::REFUSED); }
        break;
      }
      case PState::SPINUP: {                                    // ramp up thrust, bring the body level gently
        float ramp = clampf(t_in / P.spinup_s, 0.0f, 1.0f);
        float T = ramp * P.spinup_frac * P.mass_kg * kG;
        if (t_in <= dt * 1.5f) { lean_cmd_x = est.leanXDeg(); lean_cmd_y = est.leanYDeg(); }   // start from where it rests
        float step = P.level_rate_dps * dt;                     // walk the lean target to zero (no swing-through)
        lean_cmd_x -= clampf(lean_cmd_x, -step, step); lean_cmd_y -= clampf(lean_cmd_y, -step, step);
        c.motors_on = true; c.use_att = true; c.collective = fmaxf(thrustToThrottle(P, T), P.idle_throttle);
        c.lean_x_des = lean_cmd_x; c.lean_y_des = lean_cmd_y; c.on_ground = true;
        bool calm = norm(est.gyro) < 0.09f;                     // < ~5 deg/s
        level_t = (est.tiltDeg() < P.level_tol_deg && calm && fabsf(lean_cmd_x) + fabsf(lean_cmd_y) < 0.01f) ? level_t + dt : 0.0f;
        if (ramp >= 1.0f && level_t > 0.15f) {
          climb.p0 = 0; climb.p1 = A.height_m; climb.T = fmaxf(MinJerk::durationFor(A.height_m, P.climb_max_accel), 0.6f);
          go(PState::CLIMB);
        } else if (t_in > P.spinup_timeout_s) { end_code = END_NOT_LEVEL; go(PState::DONE); }
        break;
      }
      case PState::CLIMB: {
        // Before liftoff: hover thrust plus a slowly growing boost (inside flyTo) until the accelerometer says we
        // are airborne. The climb path's clock only starts at liftoff, so the drone is never "behind" the path.
        if (!airborne) {
          zr = vzr = azr = 0; want_liftoff = true;
          c = flyTo(est, dt);
          climb_t0 = t_in;
          if (t_in > P.liftoff_timeout_s) { end_code = END_NO_LIFTOFF; go(PState::DONE); return Command(); }
          break;
        }
        if (want_liftoff) climb_t0 = t_in - climb.timeAtSpeed(fmaxf(est.vel.z, 0.0f));   // join the path at our speed
        want_liftoff = false;
        float tc = t_in - climb_t0, p_, v_;
        climb.eval(tc, zr, vzr, azr);
        climb.eval(tc + P.thrust_lag_up_s, p_, v_, azr);    // feed-forward a little ahead: motors lag
        c = flyTo(est, dt);
        bool at_height = fabsf(A.height_m - est.pos.z) < 0.08f && fabsf(est.vel.z) < 0.3f;
        if (tc >= climb.T && (at_height || tc > climb.T + 2.0f)) {   // move on only once really up there
          if (A.dist_m > 0.01f) {
            path.p0 = 0; path.p1 = A.dist_m; path.T = fmaxf(MinJerk::durationFor(A.dist_m, P.hop_max_accel), 0.8f);
            go(PState::TRANSLATE);
          } else go(PState::HOVER);
        }
        break;
      }
      case PState::TRANSLATE: {
        float s, v, a, s_, v_; path.eval(t_in, s, v, a);
        path.eval(t_in + P.hop_ff_lead_s, s_, v_, a);         // feed-forward ahead: the lean takes time to build
        xr = dirx * s; yr = diry * s; vxr = dirx * v; vyr = diry * v; axr = dirx * a; ayr = diry * a;
        c = flyTo(est, dt);
        if (t_in >= path.T) go(PState::HOVER);
        break;
      }
      case PState::HOVER: {
        c = flyTo(est, dt);
        float hold = A.hover_s;
        float perr = hypotf(xr - est.pos.x, yr - est.pos.y), spd = hypotf(est.vel.x, est.vel.y);
        bool steady = perr < 0.15f && spd < 0.15f && fabsf(zr - est.pos.z) < 0.08f && fabsf(est.vel.z) < 0.15f;
        if (t_in >= hold && (steady || t_in > hold + 2.0f)) startLand(est);
        break;
      }
      case PState::LAND: {
        // sink: land_speed until close to the ground, then touch_speed (sink rate changes smoothly)
        float target_vz = (zr > P.touch_height_m) ? -P.land_speed : -P.touch_speed;
        float dv = clampf(target_vz - vzr, -P.land_accel * dt, P.land_accel * dt);
        vzr += dv; azr = dv / dt; zr = fmaxf(zr + vzr * dt, -0.6f);
        c = flyTo(est, dt);
        // touchdown = (a) impact spike, (b) the ground carrying a good part of the weight, or
        // (c) sinking much slower than commanded AND the ground carrying some weight. (c) needs the support
        // evidence: slow motors also make the drone lag its sink command while still in the air.
        float f_thr = est.thrustNow() / P.mass_kg, extra = est.f_meas_lp - f_thr;
        bool impact = est.f_body.z > f_thr + P.touch_impact_g * kG;
        support_t = (extra > P.touch_support_g * kG) ? support_t + dt : 0.0f;
        stall_t = (t_in > 0.6f && vzr < -0.15f && est.vel.z - vzr > P.stall_vz && extra > 0.1f * kG) ? stall_t + dt : 0.0f;
        if (impact || support_t > 0.08f || stall_t > P.stall_s || zr <= -0.55f) {
          touched = true; touch_pos = est.pos; touch_vel = vel_prev; go(PState::DONE); c = Command();
        }
        break;
      }
      default: break;
    }
    vel_prev = est.vel;
    return c;
  }

 private:
  void go(PState s) { st = s; t_in = 0; }
  void reset_refs() { xr = yr = zr = vxr = vyr = vzr = axr = ayr = azr = 0; }
  void startLand(const Estimator& est) {
    (void)est; vxr = vyr = axr = ayr = 0; azr = 0; vzr = 0; stall_t = support_t = 0; go(PState::LAND);
  }

  // Follow the reference (xr, yr, zr and their speeds/accelerations). Returns throttle + desired lean.
  Command flyTo(Estimator& est, float dt) {
    Command c; c.motors_on = true; c.use_att = true;
    // Liftoff, read straight from the accelerometer: on the ground it reads 1 g (the ground carries whatever the
    // thrust does not); once thrust beats weight the drone accelerates up and it reads more than 1 g.
    bool rising = est.f_meas_lp > 1.012f * kG;
    lift_t = rising ? lift_t + dt : 0.0f;
    lift_v = rising ? lift_v + (est.f_body.z - kG) * dt : 0.0f;      // upward speed gained while confirming
    if (!airborne && (lift_t > 0.025f || liftoffSeen(est))) {
      airborne = true;   // the take-off boost now fades out (below); the height loop picks up whatever is missing
      if (est.vel.z < lift_v) { est.vel.z = lift_v; est.pos.z = 0.5f * lift_v * lift_t; }   // start nav from the real motion
      // Real thrust shortfall = what the motor model says we should be accelerating at, minus what the accelerometer
      // actually reads. Keep that part of the boost (as integral); the rest was only motor lag and fades out.
      float predicted = est.thrustNow() / P.mass_kg - kG, observed = est.f_meas_lp - kG;
      float keep = clampf(predicted - observed, 0.0f, boost);
      alt_i = clampf(alt_i + keep / fmaxf(P.alt_ki, 0.1f), -P.alt_i_max, P.alt_i_max);
      boost -= keep;
    }
    if (airborne) boost *= expf(-dt / P.boost_fade_s);
    // Waiting to lift off: once the motors have actually reached the commanded thrust and we are STILL on the
    // ground, thrust is short (battery sag / thrust error): ramp a boost until we lift off.
    bool caught_up = est.thrust_model_N > 0.95f * last_T_cmd;
    if (!airborne && want_liftoff && caught_up) boost = fminf(boost + P.liftoff_boost_rate * dt, P.liftoff_boost_max);

    // When the thrust learner changes k, re-express the integral so total thrust is unchanged (the learner takes
    // over from the integral instead of both adding thrust).
    if (est.thrust_k != k_seen && k_seen > 0.0f) {
      float ki = fmaxf(P.alt_ki, 0.1f);
      alt_i = clampf(((kG + ki * alt_i) * (est.thrust_k / k_seen) - kG) / ki, -P.alt_i_max, P.alt_i_max);
    }
    k_seen = est.thrust_k;

    // vertical: PD + integral (integral only once flying, so it does not wind up on the ground)
    float ez = zr - est.pos.z, evz = vzr - est.vel.z;
    // integral: may always shrink; may only grow while the target height is holding still and the error is small
    // (while climbing the drone always trails the moving target a little; integrating that causes overshoot)
    bool grow_ok = fabsf(ez) < 0.10f && fabsf(vzr) < 0.05f;
    if (airborne && (grow_ok || ez * alt_i < 0.0f)) alt_i = clampf(alt_i + ez * dt, -P.alt_i_max, P.alt_i_max);
    float az = azr + P.alt_kp * ez + P.alt_kd * evz + P.alt_ki * alt_i + boost;

    // horizontal (world frame): PD + path acceleration, limited to what the max lean can give
    float ax = axr + P.pos_kp * (xr - est.pos.x) + P.pos_kd * (vxr - est.vel.x);
    float ay = ayr + P.pos_kp * (yr - est.pos.y) + P.pos_kd * (vyr - est.vel.y);
    if (!airborne) { ax = 0; ay = 0; }                         // on the ground: just stay level
    float up = fmaxf(kG + az, 0.3f * kG);
    float ah_max = up * tanf(P.hop_max_lean_deg * kDeg2Rad), ah = hypotf(ax, ay);
    if (ah > ah_max) { ax *= ah_max / ah; ay *= ah_max / ah; }

    // rotate into the body's heading and convert acceleration -> lean
    float psi = est.yaw(), cs = cosf(psi), sn = sinf(psi);
    float abx = cs * ax + sn * ay, aby = -sn * ax + cs * ay;
    c.lean_x_des = atan2f(abx, up) * kRad2Deg;
    c.lean_y_des = atan2f(aby, up) * kRad2Deg;
    c.on_ground = !airborne;

    // thrust: enough vertical force for the CURRENT body tilt
    float cos_tilt = fmaxf(est.upInBody().z, 0.6f);
    float T = P.mass_kg * up / cos_tilt / est.thrust_k;        // divide by learned thrust ratio
    T = clampf(T, thrustAt(P, P.idle_throttle), thrustAvail(P));
    c.collective = thrustToThrottle(P, T);
    last_T_cmd = T;
    return c;
  }

  const Params& P; Plan A; PState st = PState::OFF;
  float t_in = 0, t_all = 0, psi0 = 0, dirx = 1, diry = 0;
  float xr = 0, yr = 0, zr = 0, vxr = 0, vyr = 0, vzr = 0, axr = 0, ayr = 0, azr = 0;
  float climb_t0 = 0, last_T_cmd = 0, k_seen = 1.0f; bool want_liftoff = false;
  float alt_i = 0, boost = 0, apex_z = 0, stall_t = 0, support_t = 0, level_t = 0, lean_cmd_x = 0, lean_cmd_y = 0, lift_t = 0, lift_v = 0;
  bool airborne = false; MinJerk climb, path; Vec3 vel_prev;
};

// =====================================================================================================
// 10. Flight -- supervisor: arming, routine selection, mission states, safety limits
// =====================================================================================================
enum class Macro : uint8_t { REST, ARMED, AWAIT_RELEASE, LAUNCH, AMP_UP, ASCENDING, DESCENDING, LANDING, AMP_DOWN, FAULT,
                             TRANSLATING, HOVERING };
static inline const char* macroName(Macro m) {
  switch (m) {
    case Macro::REST: return "Rest"; case Macro::ARMED: return "Armed"; case Macro::AWAIT_RELEASE: return "Await Release";
    case Macro::LAUNCH: return "Launch"; case Macro::AMP_UP: return "Amp Up"; case Macro::ASCENDING: return "Ascending";
    case Macro::DESCENDING: return "Descending"; case Macro::LANDING: return "Landing"; case Macro::AMP_DOWN: return "Amp Down";
    case Macro::FAULT: return "Fault"; case Macro::TRANSLATING: return "Translating"; case Macro::HOVERING: return "Hovering";
  }
  return "?";
}
enum class Routine : uint8_t { NONE, DESCENT, HOP, ASCENT };
static inline const char* routineName(Routine r) {
  switch (r) { case Routine::NONE: return "none"; case Routine::DESCENT: return "descent"; case Routine::HOP: return "hop"; case Routine::ASCENT: return "ascent"; }
  return "?";
}

class Flight {
 public:
  explicit Flight(const Params& p) : P(p), est(P), att(P), desc(P), pf(P) {}

  bool arm(const char*& why) {
    if (macro_ != Macro::REST) { why = "can only arm from Rest"; return false; }
    if (!calibrated) { why = "not calibrated -- run 'cal' with the drone still and upright"; return false; }
    setMacro(Macro::ARMED); routine = Routine::NONE; why = ""; return true;
  }
  void disarm() { routine = Routine::NONE; desc.stop(); setMacro(Macro::REST); }
  void kill(const char* why) { routine = Routine::NONE; desc.stop(); fault_reason = why; setMacro(Macro::FAULT); }
  bool idleArmed() const { return macro_ == Macro::ARMED && routine == Routine::NONE; }

  bool startDescent(float release_height_m, const char*& why) {
    if (!idleArmed()) { why = "must be Armed with no routine"; return false; }
    if (release_height_m < 0.5f) { why = "release height < 0.5 m"; return false; }
    if (!desc.feasible()) { why = "descent not feasible: not enough spare thrust at the current cap"; return false; }
    beginMission(Routine::DESCENT); desc.arm(release_height_m);
    setMacro(Macro::AWAIT_RELEASE); why = ""; return true;
  }
  bool startHop(float dist_m, float bearing_deg, const char*& why) {
    PoweredFlight::Plan p; p.dist_m = dist_m; p.bearing_deg = bearing_deg; p.height_m = P.hop_cruise_m; p.hover_s = P.hop_hover_s;
    return startPowered(Routine::HOP, p, why);
  }
  bool startAscent(float height_m, const char*& why) {
    if (height_m > P.ascent_max_m) { why = "ascent height above ASCENT_MAX_HEIGHT_M"; return false; }
    PoweredFlight::Plan p; p.dist_m = 0; p.height_m = height_m; p.hover_s = P.ascent_hover_s;
    return startPowered(Routine::ASCENT, p, why);
  }

  // One control tick. gyro [rad/s], accel [m/s^2] in BODY axes.
  Command update(Vec3 gyro, Vec3 accel, float dt) {
    Macro m = macro_;
    bool ground = (m == Macro::REST || m == Macro::ARMED || m == Macro::AMP_DOWN || m == Macro::FAULT || m == Macro::LAUNCH);
    // Hop/ascent: until liftoff the drone is sitting on the ground, so keep correcting tilt from gravity and hold
    // the horizontal position/velocity at zero. Dead-reckoning drift then only starts at liftoff.
    bool on_ground_powered = (routine == Routine::HOP || routine == Routine::ASCENT) && pf.state() != PState::SETTLE &&
                             !pf.isAirborne() && (pf.state() == PState::SPINUP || pf.state() == PState::CLIMB);
    est.zupt_enable = ground;
    est.accel_correct = ground || m == Macro::AWAIT_RELEASE || on_ground_powered;
    est.update(gyro, accel, dt);
    if (on_ground_powered) { est.pos = Vec3(); est.vel = Vec3(); }   // sitting still on the ground: nothing moves
    t_mis += dt;

    Command c;
    if (routine == Routine::DESCENT) {
      DState before = desc.state(); c = desc.update(est, dt); DState s = desc.state();
      if (before == DState::WAIT_RELEASE && s == DState::FREE_FALL) { att.setYawRef(est.yaw()); post("release detected"); }
      if (s == DState::WAIT_RELEASE) { setMacro(Macro::AWAIT_RELEASE); if (t_mis > P.await_release_timeout_s) { kill("no release timeout"); return Command(); } }
      else if (s == DState::FREE_FALL) setMacro(desc.braking() ? Macro::LANDING : Macro::DESCENDING);
      else if (s == DState::LANDED) { td_vz = desc.td_vz; td_tilt = desc.td_tilt; endMission("touchdown"); c = Command(); }
      else if (s == DState::FAULT) { endMission("descent fault"); c = Command(); }
    } else if (routine == Routine::HOP || routine == Routine::ASCENT) {
      c = pf.update(est, dt);
      switch (pf.state()) {
        case PState::SETTLE: setMacro(Macro::LAUNCH); break;
        case PState::SPINUP: setMacro(Macro::AMP_UP); break;
        case PState::CLIMB: setMacro(Macro::ASCENDING); break;
        case PState::TRANSLATE: setMacro(Macro::TRANSLATING); break;
        case PState::HOVER: setMacro(Macro::HOVERING); break;
        case PState::LAND: setMacro(est.pos.z < 0.15f ? Macro::LANDING : Macro::DESCENDING); break;
        case PState::DONE: if (pf.touched) { td_vz = pf.touch_vel.z; td_tilt = est.tiltDeg(); } endMission(flightEndText(pf.end_code)); c = Command(); break;
        case PState::REFUSED: routine = Routine::NONE; setMacro(Macro::ARMED); post2("routine refused: ", flightEndText(pf.end_code)); c = Command(); break;
        default: break;
      }
    }

    if (inFlightMacro()) {
      peak_f = fmaxf(peak_f, norm(est.f_body));
      if (t_mis > P.mission_time_s) { kill("mission time limit"); return Command(); }
    }
    if (macro_ == Macro::AMP_DOWN) {                         // motors off, wait for the drone to come to rest
      c = Command(); t_amp += dt; peak_f = fmaxf(peak_f, norm(est.f_body));
      // Props are still spinning down. On the ground any thrust tips the round base, so keep the gimbal
      // holding the body level until the thrust has died away.
      if (est.thrustNow() > 0.1f * P.mass_kg * kG || t_amp < P.ampdown_hold_s) { c.use_att = true; c.lean_x_des = c.lean_y_des = 0; c.on_ground = true; }
      amp_still = (est.stillTime() > 0.0f) ? amp_still + dt : 0.0f;
      bool rested = amp_still > P.ampdown_rest_s;
      if (rested || t_amp > P.ampdown_timeout_s) { final_pos = est.pos; post(rested ? "rested" : "amp-down timeout"); summary_ready = true; setMacro(Macro::REST); }
    }
    if (macro_ == Macro::REST || macro_ == Macro::FAULT) c = Command();

    // learn real/modelled thrust while flying (accelerometer reads thrust/mass in free flight)
    bool learn = (macro_ == Macro::ASCENDING || macro_ == Macro::TRANSLATING || macro_ == Macro::HOVERING ||
                  (macro_ == Macro::LANDING && routine == Routine::DESCENT) || (macro_ == Macro::DESCENDING && routine != Routine::DESCENT));
    if (learn && routine == Routine::ASCENT) learn = pf.isAirborne() && est.pos.z > 0.1f;
    if (learn && routine == Routine::HOP) learn = pf.isAirborne() && est.pos.z > 0.08f;
    // Only while thrust has been steady for a while: during changes a lag mismatch would look like weak motors.
    float T_cmd_now = (outputsLive() && c.motors_on) ? thrustAt(P, fminf(c.collective, P.throttle_cap)) : 0.0f;
    float weight = P.mass_kg * kG;
    thrust_steady_t = (fabsf(T_cmd_now - est.thrust_model_N) < 0.08f * weight) ? thrust_steady_t + dt : 0.0f;
    // descent: braking sits at full throttle (steady) and lasts ~2 s, so learn faster there
    bool in_descent = routine == Routine::DESCENT;
    float steady_need = in_descent ? 0.1f : 0.25f, learn_s = in_descent ? 0.5f * P.thrust_learn_s : P.thrust_learn_s;
    if (learn && est.thrust_model_N > 0.3f * weight && thrust_steady_t > steady_need) {
      float ratio = clampf(est.f_meas_lp / (est.thrust_model_N / P.mass_kg), 0.7f, 1.3f);
      est.thrust_k += (ratio - est.thrust_k) * fminf(dt / learn_s, 1.0f);
    }

    if (c.use_att && !att_was_on) att.reset();
    att_was_on = c.use_att;
    if (c.use_att) att.update(est, c, dt);

    // keep the motor model up to date (what thrust is really being produced, given the lag)
    float T_cmd = (outputsLive() && c.motors_on) ? thrustAt(P, fminf(c.collective, P.throttle_cap)) : 0.0f;
    float lag = (T_cmd > est.thrust_model_N) ? P.thrust_lag_up_s : P.thrust_lag_down_s;
    est.thrust_model_N += (T_cmd - est.thrust_model_N) * fminf(dt / fmaxf(lag, 0.01f), 1.0f);
    return c;
  }

  Macro macro() const { return macro_; }
  bool outputsLive() const { return macro_ != Macro::REST && macro_ != Macro::FAULT; }
  int subState() const {
    switch (routine) { case Routine::DESCENT: return (int)desc.state(); case Routine::HOP: case Routine::ASCENT: return (int)pf.state(); default: return 0; }
  }
  const char* subStateName() const {
    if (routine == Routine::DESCENT) {
      static const char* n[] = {"off", "wait_release", "falling", "landed", "fault"}; return n[(int)desc.state()];
    }
    if (routine == Routine::HOP || routine == Routine::ASCENT) {
      static const char* n[] = {"off", "settle", "spinup", "climb", "translate", "hover", "land", "done", "refused"}; return n[(int)pf.state()];
    }
    return "-";
  }
  const char* popMsg() { if (mq_r == mq_w) return nullptr; const char* s = mq[mq_r]; mq_r = (mq_r + 1) % kMQ; return s; }

  const Params& P; Estimator est; AttitudeCtl att; DescentController desc; PoweredFlight pf;
  Routine routine = Routine::NONE, last_routine = Routine::NONE;
  bool calibrated = false, summary_ready = false;
  const char* fault_reason = ""; const char* end_reason = "";
  float peak_f = 0, td_vz = 0, td_tilt = 0, t_mis = 0; Vec3 final_pos;

 private:
  bool startPowered(Routine r, const PoweredFlight::Plan& p, const char*& why) {
    if (!idleArmed()) { why = "must be Armed with no routine"; return false; }
    if (!pf.start(est, p)) { why = flightEndText(pf.end_code); return false; }
    beginMission(r); setMacro(Macro::LAUNCH); why = ""; return true;
  }
  bool inFlightMacro() const {
    return macro_ == Macro::AMP_UP || macro_ == Macro::ASCENDING || macro_ == Macro::DESCENDING || macro_ == Macro::LANDING ||
           macro_ == Macro::TRANSLATING || macro_ == Macro::HOVERING || (macro_ == Macro::LAUNCH && routine != Routine::NONE);
  }
  void beginMission(Routine r) {
    routine = r; last_routine = r; t_mis = 0; t_amp = 0; peak_f = 0; td_vz = td_tilt = 0; end_reason = ""; summary_ready = false;
    final_pos = Vec3(); att.setYawRef(est.yaw()); att.reset(); est.thrust_k = 1.0f;
  }
  void endMission(const char* why) { end_reason = why; routine = Routine::NONE; t_amp = 0; amp_still = 0; setMacro(Macro::AMP_DOWN); post2("mission end: ", why); }
  void setMacro(Macro m) { if (m != macro_) { macro_ = m; post(macroName(m)); } }
  void post(const char* s) { int nw = (mq_w + 1) % kMQ; if (nw == mq_r) mq_r = (mq_r + 1) % kMQ; mq[mq_w] = s; mq_w = nw; }
  void post2(const char* a, const char* b) { post(a); post(b); }
  static const int kMQ = 16; const char* mq[kMQ] = {}; int mq_r = 0, mq_w = 0;
  Macro macro_ = Macro::REST; float t_amp = 0, amp_still = 0, thrust_steady_t = 0; bool att_was_on = false;
};

// =====================================================================================================
// 11. Mixer: Command -> RPM targets / ESC pulses and servo shaft angles (through the 4-bar linkage)
// =====================================================================================================
// Freudenstein solution: servo crank angle th2 that gives gimbal angle lambda. Picks the branch nearest th2_prev.
static inline bool solveFourBar(const float links[4], float th2_prev_deg, float des_lambda_deg, float& out_th2_deg) {
  const float L1 = links[0], L2 = links[1], L3 = links[2], Gd = links[3];
  if (L2 <= 1e-6f || Gd <= 1e-6f) return false;
  const float lam = des_lambda_deg * kDeg2Rad;
  const float A = -(L1 / Gd + cosf(lam)), B = -sinf(lam);
  const float C = (L1 / L2) * cosf(lam) + (L2 * L2 - L3 * L3 + Gd * Gd + L1 * L1) / (2.0f * L2 * Gd);
  const float disc = A * A + B * B - C * C;
  if (disc < 0.0f) return false;
  const float s = sqrtf(disc);
  const float r1 = 2.0f * atan2f(-B + s, C - A), r2 = 2.0f * atan2f(-B - s, C - A);
  const float prev = th2_prev_deg * kDeg2Rad;
  out_th2_deg = wrapPi((fabsf(wrapPi(r1 - prev)) <= fabsf(wrapPi(r2 - prev))) ? r1 : r2) * kRad2Deg;
  return true;
}

struct Outputs {
  uint16_t esc_upper_us = 1000, esc_lower_us = 1000;
  float rpm_upper = 0.0f, rpm_lower = 0.0f;
  float servo_x_rad = 1.5707963f, servo_y_rad = 1.5707963f;
};

static inline Outputs mix(const Command& c, const Params& P) {
  float cap = clampf(P.throttle_cap, 0.0f, 1.0f);
  float u = c.motors_on ? clampf(c.collective, 0.0f, 1.0f) : 0.0f;
  float du = c.motors_on ? clampf(c.yaw, -P.yaw_limit, P.yaw_limit) * P.yaw_sign : 0.0f;
  float up = clampf(u + du, 0.0f, cap), lo = clampf(u - du, 0.0f, cap);
  if (!c.motors_on) { up = 0.0f; lo = 0.0f; }

  Outputs o;
  o.rpm_upper = (up > 0.0f) ? P.max_rpm * sqrtf(up) : 0.0f;    // thrust ~ rpm^2, thrust ~ throttle
  o.rpm_lower = (lo > 0.0f) ? P.max_rpm * sqrtf(lo) : 0.0f;
  float span = (float)(P.esc_max_us - P.esc_min_us);
  o.esc_upper_us = (uint16_t)(P.esc_min_us + up * span);
  o.esc_lower_us = (uint16_t)(P.esc_min_us + lo * span);

  float tx = clampf(c.tilt_x_deg, -P.tilt_limit_deg, P.tilt_limit_deg);
  float ty = clampf(c.tilt_y_deg, -P.tilt_limit_deg, P.tilt_limit_deg);
  float th2_home = 0.0f, th2_x = 0.0f, th2_y = 0.0f;
  if (P.use_four_bar_linkage && solveFourBar(P.linkage_lengths, P.th2_initial_deg, P.gimbal_home_deg, th2_home) &&
      solveFourBar(P.linkage_lengths, th2_home, P.gimbal_home_deg + tx, th2_x) &&
      solveFourBar(P.linkage_lengths, th2_home, P.gimbal_home_deg + ty, th2_y)) {
    o.servo_x_rad = clampf(P.servo_center_rad + P.servo_sign_x * wrapPi((th2_x - th2_home) * kDeg2Rad), 0.0f, P.servo_travel_rad);
    o.servo_y_rad = clampf(P.servo_center_rad + P.servo_sign_y * wrapPi((th2_y - th2_home) * kDeg2Rad), 0.0f, P.servo_travel_rad);
  } else {
    o.servo_x_rad = clampf(P.servo_center_rad + P.servo_sign_x * tx * kDeg2Rad, 0.0f, P.servo_travel_rad);
    o.servo_y_rad = clampf(P.servo_center_rad + P.servo_sign_y * ty * kDeg2Rad, 0.0f, P.servo_travel_rad);
  }
  return o;
}

}  // namespace weeble
