// weeble_core.h -- Phase 1 (ESP32, no ArduCopter) controls core for the UTME Weeble drone.
// Hardware-independent C++ (no Arduino headers) so it compiles on a laptop for testing.
//
// SCOPE: controlled descent (IMU-only) and the ground-to-ground "hop".
//
// Frames / conventions (please check against the real wiring):
//   Body : x = forward, y = left, z = up when the drone is upright (right-handed).
//   World: z up; x/y horizontal, aligned with the body heading at the moment initAtRest() ran.
//   Bearing: degrees CLOCKWISE seen from above, 0 = forward, +90 = right.
//   Units: SI (m, s, rad, N, m/s^2) except servo/tilt angles, which are degrees.
//
// Every number marked ASSUMED or TODO is NOT from your documents and must be confirmed/measured.
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
  float mass_kg = 2.144f;            // CDR Appendix A.7 (all-up weight). Re-weigh the built drone.

  // ---- Propulsion (TODO(measure): thrust-stand test, with shell around pod) --
  float thrust_max_N = 27.25f;       // 98 oz static thrust (CDR 4.1). No download loss applied yet -> likely optimistic.
  float thrust_curve_exp = 1.0f;     // thrust = thrust_max * throttle^exp. TODO(measure). 1.0 = linear (ASSUMED)
  float thrust_lag_up_s = 0.25f;     // ASSUMED first-order spin-up time constant. TODO(measure)
  float thrust_lag_down_s = 0.25f;   // ASSUMED spin-down time constant. TODO(measure)
  float idle_throttle = 0.06f;       // ASSUMED: lowest throttle at which both motors reliably spin. TODO(ESC)

  // ---- Output mapping (TODO(calibrate) during the 4/7 Oct power-on + tilt tests) ----
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
  float mahony_kp = 1.0f;            // accel->attitude correction gain [1/s]
  float accel_gate_g = 0.15f;        // only trust accel for attitude when | |f| - 1g | < this
  float rest_time_s = 0.30f;         // stillness time before "at rest"

  // ---- Descent ---------------------------------------------------------------
  float freefall_g = 0.35f;          // release = |specific force| below this fraction of 1 g ...
  float freefall_debounce_s = 0.04f; // ... for this long
  float zero_g_cal_s = 0.10f;        // motors OFF this long after release to calibrate the accelerometer at true zero-g (0 = skip)
  bool  use_rest_scale = true;       // derive accel scale from the at-rest 1 g reading + zero-g reading (see Estimator::applyZeroG)
  float land_cut_height_m = -0.50f;  // also declare touchdown when the ESTIMATED height drops to this (soft touchdowns are hard to sense)
  float brake_plan_fraction = 0.75f; // plan braking at this fraction of available decel (keeps margin for lag/sag)
  float lead_time_s = 0.30f;         // effective thrust dead-time used to shape the braking profile (~ real thrust lag). TODO(measure)
  float v_touch_mps = 0.8f;          // target final descent speed
  float h_flare_m = 0.3f;            // below this estimated height, hold v_touch until contact
  float vel_kp = 1.0f;               // [1/s] gain when falling FASTER than the profile (needs more thrust)
  float vel_kp_slow = 3.0f;          // [1/s] gain when falling SLOWER than the profile (cut thrust fast -> true free fall until braking is needed)
  float acc_fb_gain = 0.5f;          // [1/s] measured-vs-commanded specific-force trim
  float impact_g = 3.0f;             // hard touchdown = |specific force| above this ...
  float contact_delta_g = 0.6f;      // ... or specific force this far above what we are commanding (soft touchdown). TODO(tune)
  float max_tilt_thrust_deg = 50.0f; // do not push thrust if body is tilted more than this
  float max_powered_s = 5.0f;        // hard stop on powered time

  // ---- Hop -------------------------------------------------------------------
  bool  hop_attitude_hold = true;    // true: tip the WHOLE drone to hop_lean_deg (recommended, see notes); false: fixed pod deflection
  float hop_lean_deg = 25.0f;        // body (thrust-axis) lean during burn. Also the fixed pod angle if !hop_attitude_hold
  float tip_thrust_N = 5.0f;         // thrust while tipping on the ground. Upright is unstable above ~ m*g*GM/R = 21.0*0.040/0.190 = 4.4 N (CDR numbers);
                                     // just above that the base is nearly neutral, so little pod torque is needed. Must stay well below weight.
  float tip_ramp_s = 0.8f;           // time to ramp the lean target to hop_lean_deg
  float hop_throttle = 1.0f;         // burn throttle (the runtime throttle cap still applies)
  float hop_settle_s = 0.4f;
  float tip_timeout_s = 1.5f;
  float servo_settle_s = 0.3f;       // open-loop mode only
  float hop_max_burn_s = 0.8f;
  float hop_neutralize_hold_s = 0.5f;// keep pod tilt this long after cut (thrust tail), then centre
  float hop_range_scale = 1.0f;      // TODO(tune) from tests: measured/predicted range
  float hop_cut_margin_m = 0.0f;
  float hop_lat_gain = 0.5f;         // steer away from predicted sideways miss (rad per normalised miss)
  bool  hop_land_brake = false;      // brake just before touchdown (extends air time a little); default: ballistic landing
  float hop_land_flare_m = 0.10f;
  float hop_land_v_touch = 1.0f;
};

// Descent presets. ROBUST (default): short powered time, so hidden accelerometer error has little time to grow; firmer
// touchdown. GENTLE: near-zero touchdown speed, but long powered creep -> only safe if the IMU is clean while the
// motors run (measure that on the bench first; see notes).
static inline void applyDescentPreset(Params& P, bool gentle) {
  if (gentle) { P.vel_kp_slow = 1.5f; P.lead_time_s = 0.60f; P.brake_plan_fraction = 0.60f; P.h_flare_m = 0.30f; P.v_touch_mps = 0.5f; P.max_powered_s = 12.0f; }
  else        { P.vel_kp_slow = 3.0f; P.lead_time_s = 0.30f; P.brake_plan_fraction = 0.75f; P.h_flare_m = 0.30f; P.v_touch_mps = 0.8f; P.max_powered_s = 5.0f; }
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
// Controller -> mixer command
// ============================================================================
struct Command {
  bool motors_on = false;
  float collective = 0;          // 0..1 throttle fraction
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
    accel_scale = 1.0f;
    f_rest_raw = f_mean;
    pos = Vec3(); vel = Vec3(); still_t = 0;
  }
  // Two-point accelerometer calibration using a true free-fall window (specific force ~ 0):
  //   offset = mean raw reading in free fall;  scale = 1 g / (at-rest reading - offset).
  // The scale is only accepted if it is within +/-6% of 1 (otherwise the at-rest reading is not trustworthy).
  // A scale error is constant, so this is the right model; it would be wrong if the OFFSET changed between the
  // rest reading and the release (then pass use_rest_scale=false).
  void applyZeroG(Vec3 raw_mean_in_freefall, bool use_rest_scale) {
    zero_g_span = norm(f_rest_raw - raw_mean_in_freefall);
    accel_bias = raw_mean_in_freefall;
    float k = (zero_g_span > 1.0f) ? kG / zero_g_span : 1.0f;
    accel_scale = (use_rest_scale && fabsf(k - 1.0f) < 0.06f) ? k : 1.0f;
  }
  void setGyroBias(Vec3 b) { gyro_bias = b; }
  void resetNav(Vec3 p, Vec3 v) { pos = p; vel = v; }

  void update(Vec3 gyro_raw, Vec3 accel, float dt) {
    gyro = gyro_raw - gyro_bias;
    accel_raw = accel;
    f_body = (accel - accel_bias) * accel_scale;
    float fn = norm(f_body);
    Vec3 w = gyro;
    if (fabsf(fn / kG - 1.0f) < P.accel_gate_g && fn > 1.0f) {
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
  Vec3 upInBody() const { return qrotInv(q, Vec3{0, 0, 1}); }
  float tiltDeg() const { return acosf(clampf(upInBody().z, -1.0f, 1.0f)) * kRad2Deg; }
  float yaw() const { return atan2f(2.0f * (q.w * q.z + q.x * q.y), 1.0f - 2.0f * (q.y * q.y + q.z * q.z)); }

  Quat q;
  Vec3 pos, vel;          // world frame; z = height of CG above its resting height
  Vec3 gyro;              // bias-corrected [rad/s]
  Vec3 f_body, f_world, a_world;
  Vec3 accel_bias, gyro_bias, accel_raw, f_rest_raw;
  float accel_scale = 1.0f, zero_g_span = 0;
  bool zupt_enable = false;

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
    float T_est = P.thrust_max_N * powf(clampf(c.collective, 0.0f, 1.0f), P.thrust_curve_exp);
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

static inline float thrustToThrottle(const Params& P, float thrust_N) {
  float frac = clampf(thrust_N / P.thrust_max_N, 0.0f, 1.0f);
  return powf(frac, 1.0f / P.thrust_curve_exp);
}

// ============================================================================
// Controlled descent (IMU only).
//   Release is detected from the accelerometer (near-zero specific force).  Height and vertical speed are
//   dead-reckoned from the assumed release height.  The controller tracks a speed-vs-height profile
//   v_ref(h) = -(v_touch + sqrt(2*a_plan*(h-h_flare)))  ("late, hard, but planned" braking), then holds v_touch
//   until an impact is sensed.  Braking early is harmless (it just hovers/creeps), braking late is not.
// ============================================================================
enum class DState : uint8_t { OFF, WAIT_RELEASE, FALLING, LANDED, FAULT };

class DescentController {
 public:
  explicit DescentController(const Params& p) : P(p) {}
  void arm(float release_height_m) { H0 = release_height_m; st = DState::WAIT_RELEASE; ff_t = 0; t = 0; ucorr = 0; flare = P.h_flare_m; vt = P.v_touch_mps; }
  void startTracking(float h_flare, float v_touch) { st = DState::FALLING; t = 0; ucorr = 0; flare = h_flare; vt = v_touch; impact_cnt = 0; }
  void stop() { st = DState::OFF; }
  DState state() const { return st; }
  float lastThrottle() const { return u_last; }
  float lastVref() const { return vref_last; }

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
          t_fall0 = td; zg_n = 0; zg_sum = Vec3(); zg_done = (P.zero_g_cal_s <= 0.0f);
        }
        break;
      }
      case DState::FALLING: {
        if (!zg_done) {                                   // zero-g calibration window: rotors fully off, no control
          zg_sum = zg_sum + est.accel_raw; zg_n++;
          if (t >= P.zero_g_cal_s) {
            est.applyZeroG(zg_sum * (1.0f / (float)zg_n), P.use_rest_scale);
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
        if (t > 0.15f && (hard || soft)) impact_cnt++; else impact_cnt = 0;
        if (impact_cnt >= 2 || est.pos.z <= P.land_cut_height_m) { st = DState::LANDED; break; }
        if (t > P.max_powered_s) { st = DState::FAULT; break; }

        float f_full = P.thrust_max_N / P.mass_kg;           // specific force at full throttle
        float a_avail = f_full - kG;                          // upward accel at full throttle
        if (a_avail < 0.5f) { st = DState::FAULT; break; }     // cannot brake meaningfully
        float a_plan = P.brake_plan_fraction * a_avail;

        float h = est.pos.z, v = est.vel.z;
        float hh = fmaxf(h - flare, 0.0f);
        // Braking profile that already accounts for thrust lag tau (dead-time): reaches v_touch exactly at the flare
        // height when decelerating at a_plan after a delay tau. Slope stays bounded (<= 1/tau), so no hunting.
        float tau = fmaxf(P.lead_time_s, 0.02f);
        float s_ = sqrtf(tau * tau + 2.0f * hh / a_plan);
        float v_ref = -(vt + a_plan * (s_ - tau));
        vref_last = v_ref;
        float a_ff = clampf(-v / s_, 0.0f, a_plan);             // d(v_ref)/dt along the profile (feed-forward)
        float kp = (v_ref - v < 0.0f) ? P.vel_kp_slow : P.vel_kp;
        float a_up = clampf(a_ff + kp * (v_ref - v), -kG, a_avail);
        float f_cmd = kG + a_up;                               // commanded specific force along the thrust axis
        f_cmd_last = f_cmd;

        float u_ff = thrustToThrottle(P, f_cmd * P.mass_kg);
        if (u_ff > P.idle_throttle + 0.05f)
          ucorr = clampf(ucorr + P.acc_fb_gain * (f_cmd - est.f_body.z) / f_full * dt, -0.25f, 0.25f);
        else
          ucorr *= 0.95f;
        float u = clampf(u_ff + ucorr, P.idle_throttle, 1.0f);
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
  const Params& P;
  DState st = DState::OFF;
  float H0 = 6.0f, ff_t = 0, t = 0, ucorr = 0, flare = 0.6f, vt = 0.4f, u_last = 0, vref_last = 0, f_cmd_last = 0, t_fall0 = 0;
  Vec3 zg_sum; int zg_n = 0; bool zg_done = true;
  int impact_cnt = 0;
};

// ============================================================================
// Hop: move a fixed distance along a bearing, ground to ground.
//   SETTLE -> TIP (lean the whole drone toward the target at sub-weight thrust)
//          -> BURN (full thrust, attitude held, cut when PREDICTED landing point reaches the target)
//          -> COAST (idle, centre the pod) -> [optional brake] -> DONE
//   Cut-off uses the IMU-integrated velocity + a forward ballistic prediction that includes the thrust tail,
//   so it adapts to whatever thrust-to-weight the drone really has.
// ============================================================================
enum class HState : uint8_t { OFF, SETTLE, TIP, BURN, COAST, LAND, DONE, FAULT };

class HopController {
 public:
  explicit HopController(const Params& p) : P(p), land(p) {}

  void start(const Estimator& est, float dist_m, float bearing_deg_cw) {
    dist = dist_m;
    th_b = -bearing_deg_cw * kDeg2Rad;                  // CCW angle of target direction, from body-forward at start
    psi0 = est.yaw();
    dx = cosf(psi0 + th_b); dy = sinf(psi0 + th_b);
    st = HState::SETTLE; t = 0; t_in = 0; ok_t = 0; last_tx = last_ty = 0; fault_code = 0; cut_reason_maxburn = false;
    // Refuse if the drone cannot sustain lift while leaning (vertical thrust must clearly exceed weight even ~10 deg beyond the lean).
    float twr = P.thrust_max_N / (P.mass_kg * kG);
    if (P.hop_attitude_hold && twr * cosf((P.hop_lean_deg + 10.0f) * kDeg2Rad) < 1.15f) { st = HState::FAULT; fault_code = 1; }
  }
  HState state() const { return st; }
  float predictedRange() const { return pred_along; }
  float predictedMiss() const { return pred_lat; }
  float heading0() const { return psi0; }

  Command update(Estimator& est, float dt) {
    Command c;
    t += dt; t_in += dt;
    float dpsi = wrapPi(est.yaw() - psi0);
    float dir_b = th_b - dpsi;                           // target direction in CURRENT body frame (CCW from body x)
    switch (st) {
      case HState::SETTLE:
        c.motors_on = true;
        if (t_in > P.hop_settle_s && est.atRest()) { est.resetNav(Vec3(), Vec3()); go(HState::TIP); }
        else if (t_in > 3.0f) { fault_code = 2; go(HState::FAULT); }
        break;

      case HState::TIP:
        c.motors_on = true;
        if (P.hop_attitude_hold) {
          c.collective = thrustToThrottle(P, P.tip_thrust_N); c.use_att = true;
          float ramp = clampf(t_in / P.tip_ramp_s, 0.0f, 1.0f) * P.hop_lean_deg;
          c.lean_x_des = ramp * cosf(dir_b); c.lean_y_des = ramp * sinf(dir_b);
          Vec3 up = est.upInBody();
          float ex = c.lean_x_des - asinf(clampf(-up.x, -1.f, 1.f)) * kRad2Deg;
          float ey = c.lean_y_des - asinf(clampf(-up.y, -1.f, 1.f)) * kRad2Deg;
          ok_t = (ramp >= P.hop_lean_deg - 0.01f && sqrtf(ex * ex + ey * ey) < 3.0f) ? ok_t + dt : 0.0f;
          if (ok_t > 0.15f) go(HState::BURN);
          else if (t_in > P.tip_timeout_s) { fault_code = 3; go(HState::FAULT); }
        } else {
          c.collective = 0.0f;
          float a = clampf(P.hop_lean_deg, 0.0f, P.tilt_limit_deg);
          c.tilt_x_deg = a * cosf(dir_b); c.tilt_y_deg = a * sinf(dir_b);
          if (t_in > P.servo_settle_s) go(HState::BURN);
        }
        break;

      case HState::BURN: {
        c.motors_on = true; c.collective = P.hop_throttle;
        predict(est);
        float D = dist / P.hop_range_scale;
        float dir_cmd = dir_b - clampf(P.hop_lat_gain * pred_lat / fmaxf(dist, 0.5f), -0.35f, 0.35f);  // steer against predicted miss
        if (P.hop_attitude_hold) {
          c.use_att = true;
          c.lean_x_des = P.hop_lean_deg * cosf(dir_cmd); c.lean_y_des = P.hop_lean_deg * sinf(dir_cmd);
        } else {
          float a = clampf(P.hop_lean_deg, 0.0f, P.tilt_limit_deg);
          c.tilt_x_deg = a * cosf(dir_cmd); c.tilt_y_deg = a * sinf(dir_cmd);
        }
        last_tx = c.tilt_x_deg; last_ty = c.tilt_y_deg; last_lx = c.lean_x_des; last_ly = c.lean_y_des;
        if (pred_along >= D - P.hop_cut_margin_m || t_in > P.hop_max_burn_s) { cut_reason_maxburn = (t_in > P.hop_max_burn_s); go(HState::COAST); }
        break;
      }

      case HState::COAST: {
        c.motors_on = true; c.collective = P.idle_throttle;
        if (t_in < P.hop_neutralize_hold_s) {
          // keep the last pod direction while the thrust tail decays (prediction assumed it)
          c.use_att = false;
          c.tilt_x_deg = last_tx; c.tilt_y_deg = last_ty;
          if (P.hop_attitude_hold) { c.use_att = true; c.lean_x_des = last_lx; c.lean_y_des = last_ly; }
        }
        float fn = norm(est.f_body);
        if (t_in > 0.1f && fn > 2.0f * kG) { go(HState::DONE); }
        else if (t_in > P.hop_neutralize_hold_s && P.hop_land_brake) { land.startTracking(P.hop_land_flare_m, P.hop_land_v_touch); go(HState::LAND); }
        else if (t_in > 4.0f) go(HState::DONE);
        break;
      }

      case HState::LAND: {
        c = land.update(est, dt);
        if (land.state() != DState::FALLING) go(HState::DONE);
        break;
      }
      default: break;
    }
    return c;
  }

 private:
  void go(HState s) { st = s; t_in = 0; ok_t = 0; }

  // Forward-simulate from the current estimate with the thrust decaying from now on; report landing point
  // along / across the commanded direction.
  void predict(const Estimator& est) {
    Vec3 p = est.pos, v = est.vel;
    bool airborne = (v.z > 0.2f) || (p.z > 0.03f && v.z > 0.05f);
    if (airborne) {
      Vec3 f = est.f_world;
      const float h = 0.01f;
      float k = 1.0f, decay = expf(-h / P.thrust_lag_down_s);
      for (int i = 0; i < 250; i++) {
        Vec3 a = f * k - Vec3{0, 0, kG};
        v = v + a * h; p = p + v * h;
        k *= decay;
        if (p.z <= 0.0f && v.z < 0.0f) break;
      }
    }
    pred_along = p.x * dx + p.y * dy;
    pred_lat = -p.x * dy + p.y * dx;                    // + = landing is to the LEFT of the commanded line
  }

  const Params& P;
  DescentController land;
  HState st = HState::OFF;
  float dist = 1.524f, th_b = 0, psi0 = 0, dx = 1, dy = 0;
  float t = 0, t_in = 0, ok_t = 0;
  float pred_along = 0, pred_lat = 0;
  float last_tx = 0, last_ty = 0, last_lx = 0, last_ly = 0;
 public:
  bool cut_reason_maxburn = false;
  int fault_code = 0;   // 1 = not enough thrust to hop at this lean, 2 = not at rest, 3 = could not tip to the lean angle
};

// ============================================================================
// Top-level: one object, call update() at a fixed rate (e.g. 200 Hz).
// ============================================================================
class Flight {
 public:
  explicit Flight(const Params& p) : P(p), est(P), att(P), desc(P), hop(P) {}
  enum class Mode : uint8_t { IDLE, DESCENT, HOP };

  void armDescent(float release_height_m) { desc.arm(release_height_m); mode = Mode::DESCENT; }
  void startHop(float dist_m, float bearing_deg_cw) { att.setYawRef(est.yaw()); hop.start(est, dist_m, bearing_deg_cw); mode = Mode::HOP; }
  void stopAll() { mode = Mode::IDLE; desc.stop(); }

  Command update(Vec3 gyro_rads, Vec3 accel_ms2, float dt) {
    est.zupt_enable = (mode == Mode::IDLE) || (mode == Mode::HOP && hop.state() == HState::SETTLE);
    est.update(gyro_rads, accel_ms2, dt);
    Command c;
    if (mode == Mode::DESCENT) c = desc.update(est, dt);
    else if (mode == Mode::HOP) c = hop.update(est, dt);
    int phase = (int)mode * 16 + (mode == Mode::HOP ? (int)hop.state() : (int)desc.state());
    if (phase != last_phase) { att.reset(); last_phase = phase; }
    if (c.use_att) att.update(est, c, dt);
    return c;
  }

  const Params& P;
  Estimator est;
  AttitudeCtl att;
  DescentController desc;
  HopController hop;
  Mode mode = Mode::IDLE;
 private:
  int last_phase = -1;
};

// ============================================================================
// Mixer: Command -> ESC / servo pulse widths (microseconds).  throttle_cap is a runtime safety limit.
// ============================================================================
struct Outputs { uint16_t esc_upper_us, esc_lower_us, servo_x_us, servo_y_us; };

static inline Outputs mix(const Command& c, const Params& P, float throttle_cap) {
  float u = c.motors_on ? clampf(c.collective, 0.0f, 1.0f) : 0.0f;
  float du = c.motors_on ? clampf(c.yaw, -P.yaw_limit, P.yaw_limit) * P.yaw_sign : 0.0f;
  float up = clampf(u + du, 0.0f, throttle_cap), lo = clampf(u - du, 0.0f, throttle_cap);
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
