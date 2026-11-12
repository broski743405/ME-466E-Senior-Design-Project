// weeble_bridge.cpp -- C functions so Python (weeble_sim.py) can drive weeble_core.h.
// Built by build.py into sim/weeble.dll (Windows) or sim/libweeble.so (Linux/Mac).
#include <string.h>
#include "weeble_core.h"

using namespace weeble;

#if defined(_WIN32)
#define API extern "C" __declspec(dllexport)
#else
#define API extern "C"
#endif

static Params gP;
static Flight* gF = nullptr;
static Command gC;

API uint32_t sim_config_hash() { return WEEBLE_CONFIG_HASH; }

// Fresh controller with default Params (from the config). Throttle cap opened for simulation.
API void sim_reset() {
  gP = Params();
  gP.throttle_cap = 1.0f;
  delete gF;
  gF = new Flight(gP);
  gC = Command();
}

// Override one parameter (used by tests, e.g. "what if the controller believes the wrong thrust").
// Returns 1 if the name is known.
API int sim_set(const char* name, float v) {
#define S(f) if (!strcmp(name, #f)) { gP.f = v; return 1; }
  S(mass_kg) S(thrust_max_N) S(thrust_lag_up_s) S(thrust_lag_down_s) S(throttle_cap) S(pivot_arm_m)
  S(att_wn) S(att_wn_frac) S(att_rate_lpf_hz) S(att_zeta) S(att_ki_ratio) S(att_i_max) S(att_thrust_floor) S(alt_kp) S(alt_kd) S(alt_ki) S(pos_kp) S(pos_kd)
  S(hop_cruise_m) S(hop_max_lean_deg) S(hop_max_accel) S(climb_max_accel) S(hop_hover_s) S(ascent_hover_s)
  S(land_speed) S(touch_speed) S(touch_height_m) S(thrust_learn_s) S(spinup_frac) S(level_tol_deg) S(boost_fade_s) S(liftoff_boost_rate) S(hop_ff_lead_s) S(brake_plan_fraction) S(v_touch_mps) S(h_flare_m) S(mahony_kp)
#undef S
  if (!strcmp(name, "inertia_x")) { gP.inertia[0] = v; return 1; }
  if (!strcmp(name, "inertia_y")) { gP.inertia[1] = v; return 1; }
  return 0;
}

// At-rest calibration, like the firmware 'cal' command: averaged IMU reading (CHIP axes, rad/s and m/s^2).
API void sim_calibrate(float gx, float gy, float gz, float ax, float ay, float az) {
  Vec3 g = imuToBody(Vec3{gx, gy, gz}), a = imuToBody(Vec3{ax, ay, az});
  gF->est.initAtRest(a);
  gF->est.setGyroBias(g);
  gF->calibrated = true;
}

// routine: 0 = descent (a1 = release height m), 1 = hop (a1 = distance m, a2 = bearing deg), 2 = ascent (a1 = height m)
API int sim_start(int routine, float a1, float a2) {
  const char* why = "";
  if (!gF->arm(why)) return 0;
  bool ok = false;
  if (routine == 0) ok = gF->startDescent(a1, why);
  else if (routine == 1) ok = gF->startHop(a1, a2, why);
  else if (routine == 2) ok = gF->startAscent(a1, why);
  return ok ? 1 : 0;
}

// One 200 Hz tick. IMU values in CHIP axes. out[8]:
//   0 thrust command [N]   1 gimbal x [deg]   2 gimbal y [deg]   3 yaw throttle difference
//   4 macro state          5 upper RPM target 6 servo x shaft [deg]  7 servo y shaft [deg]
API void sim_step(float gx, float gy, float gz, float ax, float ay, float az, float dt, float* out) {
  Command c = gF->update(imuToBody(Vec3{gx, gy, gz}), imuToBody(Vec3{ax, ay, az}), dt);
  gC = c;
  Outputs o = mix(c, gP);
  bool live = gF->outputsLive() && c.motors_on;          // motors
  bool servos = gF->outputsLive();                       // servos stay active while outputs are live (as in main.cpp)
  out[0] = live ? thrustAt(gP, fminf(c.collective, gP.throttle_cap)) : 0.0f;
  out[1] = servos ? clampf(c.tilt_x_deg, -gP.tilt_limit_deg, gP.tilt_limit_deg) : 0.0f;
  out[2] = servos ? clampf(c.tilt_y_deg, -gP.tilt_limit_deg, gP.tilt_limit_deg) : 0.0f;
  out[3] = live ? clampf(c.yaw, -gP.yaw_limit, gP.yaw_limit) : 0.0f;
  out[4] = (float)gF->macro();
  out[5] = o.rpm_upper;
  out[6] = o.servo_x_rad * kRad2Deg;
  out[7] = o.servo_y_rad * kRad2Deg;
}

// Controller internals for logging. t[24]
API void sim_telemetry(float* t) {
  Flight& F = *gF; Estimator& e = F.est;
  t[0] = (float)F.macro(); t[1] = (float)F.routine; t[2] = (float)F.subState();
  t[3] = e.pos.x; t[4] = e.pos.y; t[5] = e.pos.z; t[6] = e.vel.x; t[7] = e.vel.y; t[8] = e.vel.z;
  t[9] = e.tiltDeg(); t[10] = e.leanXDeg(); t[11] = e.leanYDeg(); t[12] = e.yaw() * kRad2Deg;
  t[13] = e.thrust_model_N; t[14] = gC.lean_x_des; t[15] = gC.lean_y_des;
  t[16] = F.pf.refX(); t[17] = F.pf.refY(); t[18] = F.pf.refZ();
  t[19] = F.desc.lastVref(); t[20] = F.td_vz; t[21] = (float)F.pf.end_code; t[22] = e.thrust_k; t[23] = F.pf.isAirborne() ? 1.f : 0.f;
}

API const char* sim_substate() { return gF->subStateName(); }
API const char* sim_routine() { return routineName(gF->routine); }
// Next queued status message, or "" when none.
API const char* sim_message() { const char* s = gF->popMsg(); return s ? s : ""; }
