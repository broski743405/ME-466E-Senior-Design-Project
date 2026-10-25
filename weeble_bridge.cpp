// weeble_bridge.cpp -- C wrapper so Python MuJoCo can call weeble_core.h
#include "weeble_core.h"

using namespace weeble;

static Params gP;
static Flight* gFlight = nullptr; 

extern "C" {

void sim_init(int mode, float arg_val, float twr) {
  gP = Params();
  gP.throttle_cap = 1.0f;
  gP.thrust_max_N = twr * gP.mass_kg * kG;
  
  // Delete the old flight brain and create a fresh one in memory
  if (gFlight) { delete gFlight; }
  gFlight = new Flight(gP);

  gFlight->est.initAtRest(Vec3{0, 0, kG});
  gFlight->calibrated = true;
  const char* why = "";
  gFlight->arm(why);

  if (mode == 0) {         // 0 = Controlled Descent from arg_val [m]
    gFlight->startDescent(arg_val, why);
  } else if (mode == 1) {  // 1 = Horizontal Hop to arg_val [ft]
    HopArgs h; 
    h.dist_m = arg_val * 0.3048f; 
    h.lean_deg = 15.0f; 
    h.brake = false;
    gFlight->startHop(h, why);
  } else if (mode == 2) {  // 2 = Vertical Hop to arg_val [m]
    VHopArgs v; 
    v.height_m = arg_val; 
    v.brake = true;
    gFlight->startVHop(v, why);
  }
}

void sim_step(float gx, float gy, float gz, float ax, float ay, float az, float dt, float* out_buf) {
  if (!gFlight) return;
  
  Command c = gFlight->update(Vec3{gx, gy, gz}, Vec3{ax, ay, az}, dt);
  Outputs o = mix(c, gP);
  bool live = gFlight->outputsLive() && c.motors_on;
  float u = live ? fminf(c.collective, gP.throttle_cap) : 0.0f;
  
  out_buf[0] = live ? thrustAt(gP, u) : 0.0f;         // Desired thrust [N]
  out_buf[1] = live ? c.tilt_x_deg : 0.0f;            // Pod pitch tilt [deg]
  out_buf[2] = live ? c.tilt_y_deg : 0.0f;            // Pod roll tilt [deg]
  out_buf[3] = live ? c.yaw * 2.0f : 0.0f;            // Yaw torque [N*m]
  out_buf[4] = (float)gFlight->macro();               // Macrostate index
  out_buf[5] = o.rpm_upper;                           // Target Upper RPM
  out_buf[6] = o.servo_x_rad * 57.2958f;              // Output actual servo angle in degrees
}

}  // extern "C"