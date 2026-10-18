// sim_test.cpp -- host-side test of weeble_core.h against an ASSUMED planar plant.
//   g++ -O2 -std=c++17 -o sim_test sim_test.cpp && ./sim_test            (all tests)
//   ./sim_test sweep        descent preset tuning sweep
//   ./sim_test trace        print one run of each routine tick by tick (coarse)
//
// What this checks: the control LOGIC of every routine (release detection, braking profile, tip/burn/cut hop,
// vertical hop apex prediction + landing hand-off), the macrostate sequence and aborts, the throttle-cap
// feasibility checks, the six-position accel calibration solver, and sensitivity to the big unknowns.
// What it does NOT check: yaw / lateral axis (planar), real servo & ESC behaviour, motor vibration spectrum,
// aerodynamics (no drag, no ground effect), real ground friction / bounce, 3-D effects, I2C/serial timing.
// Plant numbers marked ASSUMED are placeholders until measured. A good result here is necessary, not sufficient.
#include "../WeebleControls/weeble_core.h"
#include <cstdio>
#include <cmath>
#include <random>
#include <algorithm>
#include <functional>
#include <string>
using namespace weeble;

struct Plant {
  // ---- geometry / mass (CDR CG diagram where available) ----
  double m = 2.144, g = 9.80665;
  double R = 0.19;       // centre of curvature height (CDR A.7: 190 mm)
  double b = 0.04;       // CG below centre of curvature (GM ~ 40 mm)
  double d = 0.13;       // gimbal pivot above CG (28.0 cm ring - 15.0 cm CG; CDR A.7)   TODO(CAD)
  double I = 0.04;       // ASSUMED pitch inertia about CG [kg m^2]   TODO(CAD)
  // ---- propulsion ----
  double Tmax = 27.25, tau_up = 0.25, tau_dn = 0.25;     // tau ASSUMED
  double servo_rate = 11.6, servo_tau = 0.03;            // ~667 deg/s ASSUMED
  double c_roll = 0.03;                                  // rolling damping, ASSUMED
  double pod_bias = 0.0;                                 // pod misalignment [rad] (unknown real value; disturbance test)
  double contact_s = 0.015;                              // ASSUMED impact duration seen by the accelerometer (shell compliance).
                                                         // v1 sim had none: velocity jumped in one step, so the IMU never saw an impact.
  double sp_x = 0, sp_z = 0, sp_t = 0;                   // active impact spike [m/s^2] and time left
  // ---- state ----
  bool ground = true; double x0 = 0, th = 0, thd = 0;    // ground (rolling) state
  double x = 0, z = 0, vx = 0, vz = 0;                   // CG state when airborne
  double T = 0, alpha = 0, cmd_u = 0, cmd_alpha = 0;
  double ax = 0, az = 0, t = 0;
  bool has_flown = false, touched = false;
  double touch_vz = 0, touch_xc = 0, touch_vx = 0, touch_t = 0, touch_th = 0, max_th = 0, min_th = 0, max_z = 0;

  double Mth() const { return m * (R * R - 2 * R * b * cos(th) + b * b) + I; }

  void step(double dt) {
    double Tcmd = Tmax * cmd_u;
    T += (Tcmd - T) * dt / (Tcmd > T ? tau_up : tau_dn);
    double rate = std::clamp((cmd_alpha - alpha) / servo_tau, -servo_rate, servo_rate);
    alpha += rate * dt;
    double a_eff = alpha + pod_bias;
    double phi = th + a_eff, Fx = T * sin(phi), Fz = T * cos(phi);
    if (ground) {
      double Q = Fx * (R + (d - b) * cos(th)) - Fz * (d - b) * sin(th);
      double thdd = (Q - m * R * b * sin(th) * thd * thd - m * g * b * sin(th) - c_roll * thd) / Mth();
      double zdd = b * cos(th) * thd * thd + b * sin(th) * thdd;
      double N = m * zdd - Fz + m * g;
      if (N < 0) {   // lift off
        ground = false; has_flown = true;
        x = x0 + R * th - b * sin(th); z = R - b * cos(th);
        vx = thd * (R - b * cos(th)); vz = thd * b * sin(th);
      } else {
        thd += thdd * dt; th += thd * dt;
        ax = (R - b * cos(th)) * thdd + b * sin(th) * thd * thd; az = zdd;
        if (sp_t > 0) { ax += sp_x; az += sp_z; sp_t -= dt; }
        track(); t += dt; return;
      }
    }
    ax = Fx / m; az = Fz / m - g;
    double thdd = T * d * sin(a_eff) / I;
    vx += ax * dt; vz += az * dt; x += vx * dt; z += vz * dt; thd += thdd * dt; th += thd * dt;
    double zc = z + b * cos(th), vzc = vz - b * sin(th) * thd;
    if (!touched) max_z = std::max(max_z, z - (R - b));
    if (zc <= R && vzc < 0 && has_flown) {     // touchdown of the spherical base (plastic, no slip)
      double xc = x + b * sin(th), vcx = vx + b * cos(th) * thd;
      if (!touched) { touched = true; touch_vz = vzc; touch_t = t; touch_th = th; touch_xc = xc; touch_vx = vcx; }
      double rx = -b * sin(th), rz = z;
      double L = I * thd + m * (rz * vx - rx * vz);
      x0 = xc - R * th; thd = L / Mth(); ground = true;
      double vxa = thd * (R - b * cos(th)), vza = thd * b * sin(th);      // CG velocity after the (plastic) impact
      sp_x = (vxa - vx) / contact_s; sp_z = (vza - vz) / contact_s; sp_t = contact_s;
    }
    track(); t += dt;
  }
  void track() { if (!touched) { max_th = std::max(max_th, th); min_th = std::min(min_th, th); } }
};

struct Noise {
  std::mt19937 rng{1};
  std::normal_distribution<double> n{0.0, 1.0};
  double acc_sigma = 0.05, gyr_sigma = 0.002;      // ASSUMED MPU6050-class noise after filtering
  Vec3 acc_bias{0.0f, 0.0f, 0.1f};                 // ASSUMED residual bias [m/s^2] (x,y,z body)
  float gyro_bias_y = 0.01f;
  double gyro_bias_err = 0.0;  // bias error left after calibration [rad/s] (drift)
  double scale_err = 0.0;      // accelerometer scale error (fraction)
  double vib_bias = 0.0;       // offset that appears only while rotors spin, proportional to thrust fraction [m/s^2]
  void seed(int s) { rng.seed(s); }
};

static void sense(const Plant& p, Noise& nz, bool hanging, Vec3& gyro, Vec3& acc) {
  double fx = hanging ? 0.0 : p.ax, fz = hanging ? p.g : p.az + p.g;
  double fbx = fx * cos(p.th) - fz * sin(p.th), fbz = fx * sin(p.th) + fz * cos(p.th);
  fbx *= (1.0 + nz.scale_err); fbz *= (1.0 + nz.scale_err);
  double vib = nz.vib_bias * (p.T / p.Tmax);
  acc = Vec3{(float)(fbx + nz.acc_bias.x + nz.acc_sigma * nz.n(nz.rng)), (float)(nz.acc_bias.y + nz.acc_sigma * nz.n(nz.rng)),
             (float)(fbz + nz.acc_bias.z + vib + nz.acc_sigma * nz.n(nz.rng))};
  double w = hanging ? 0.0 : p.thd;
  gyro = Vec3{(float)(nz.gyr_sigma * nz.n(nz.rng)), (float)(w + nz.gyro_bias_y + nz.gyro_bias_err + nz.gyr_sigma * nz.n(nz.rng)), (float)(nz.gyr_sigma * nz.n(nz.rng))};
}

struct Cfg {
  double twr = 1.50, tau = 0.25, I = 0.04, bias_z = 0.1, scale = 0.0, vib = 0.0, drift_z = 0.0, H_true = 6.096, H_assumed = 6.096;
  double ctrl_twr_err = 1.0, th0_deg = 5, ctrl_tau = -1;   // ctrl_tau < 0: controller knows the true lag (it was measured)
  double payload_err = 0.0;   // real mass - entered mass [kg]
  int seed = 1; std::function<void(Params&)> tweak; bool trace = false; bool brake = false; double lean = 15; double dist = 1.524;
  double throttle_cap = 1.0; double pod_bias_deg = 0; double gyro_bias_err = 0; double vh = 0.5;
};

static void applyCmd(const Command& c, Plant& pl, const Params& P) {
  pl.cmd_u = c.motors_on ? std::min<double>(c.collective, P.throttle_cap) : 0.0;
  pl.cmd_alpha = std::clamp<double>(c.tilt_x_deg, -P.tilt_limit_deg, P.tilt_limit_deg) * kDeg2Rad;
}

static void setupParams(const Cfg& c, Plant& pl, Params& P) {
  pl.I = c.I; pl.tau_up = pl.tau_dn = c.tau; pl.Tmax = c.twr * pl.m * pl.g; pl.pod_bias = c.pod_bias_deg * kDeg2Rad;
  pl.m += c.payload_err;
  P.thrust_max_N = (float)(pl.Tmax * c.ctrl_twr_err);
  double lag = c.ctrl_tau < 0 ? c.tau : c.ctrl_tau;
  P.thrust_lag_up_s = P.thrust_lag_down_s = (float)lag;
  P.throttle_cap = (float)c.throttle_cap;
  if (c.tweak) c.tweak(P);
}

// at-rest calibration + arm, like 'cal' then 'arm' on the real drone
static bool calAndArm(Flight& fl, Plant& pl, Noise& nz, bool hanging) {
  Vec3 g_, a_; double sx = 0, sy = 0, sz = 0; const int N = 400;
  for (int i = 0; i < N; i++) { sense(pl, nz, hanging, g_, a_); sx += a_.x; sy += a_.y; sz += a_.z; }
  fl.est.initAtRest(Vec3{(float)(sx / N), (float)(sy / N), (float)(sz / N)});
  fl.est.setGyroBias(Vec3{0, nz.gyro_bias_y, 0});
  nz.gyro_bias_err = 0;  // set by caller after calibration if wanted
  fl.calibrated = true;
  const char* w; return fl.arm(w);
}

static std::string macroSeq;
static void trackMacro(Macro& last, Macro now) {
  if (now != last) { if (!macroSeq.empty()) macroSeq += " > "; macroSeq += macroName(now); last = now; }
}

// ------------------------------------------------------------------------------------------------
struct DRes { double vz, tilt, t; Macro end; bool started; std::string why; double est_vz; double peak_g; };
static DRes runDescent(const Cfg& c) {
  Plant pl; Params P; setupParams(c, pl, P);
  pl.ground = false; pl.has_flown = true; pl.z = (pl.R - pl.b) + c.H_true; pl.th = c.th0_deg * kDeg2Rad;
  Noise nz; nz.seed(c.seed); nz.acc_bias.z = (float)c.bias_z; nz.scale_err = c.scale; nz.vib_bias = c.vib;
  Flight fl(P);
  calAndArm(fl, pl, nz, true);
  nz.acc_bias.z += (float)c.drift_z;   // error that appears AFTER the at-rest calibration
  nz.gyro_bias_err = c.gyro_bias_err;
  const char* why = "";
  DRes r{}; r.started = fl.startDescent((float)c.H_assumed, why); r.why = why;
  macroSeq.clear(); Macro last = Macro::ARMED; trackMacro(last, fl.macro());
  Command latched; const double dtp = 0.0005; int tick = 0; bool released = false;
  for (int k = 0; k < 60000; k++) {
    double tt = k * dtp;
    if (!released && tt > 0.3) { released = true; }
    if (k % 10 == 0) {
      Vec3 g_, a_; sense(pl, nz, !released, g_, a_);
      Command cmd = fl.update(g_, a_, 0.005f);
      trackMacro(last, fl.macro());
      if (tick++ % 4 == 0) latched = cmd;       // 50 Hz pulse frame latch
      applyCmd(fl.outputsLive() ? latched : Command(), pl, P);
      if (c.trace && released && (k % 200 == 0)) printf("   t=%.2f %-13s true h=%.2f vz=%.2f | est h=%.2f vz=%.2f | vref=%.2f u=%.2f T=%.1f\n", tt - 0.3, macroName(fl.macro()), pl.z - (pl.R - pl.b), pl.vz, fl.est.pos.z, fl.est.vel.z, fl.desc.lastVref(), cmd.collective, pl.T);
    }
    if (released) pl.step(dtp);
    if (pl.touched && (fl.macro() == Macro::REST || fl.macro() == Macro::FAULT)) break;
  }
  r.vz = -pl.touch_vz; r.tilt = pl.touch_th * kRad2Deg; r.t = pl.touch_t; r.end = fl.macro(); r.est_vz = -fl.td_vz; r.peak_g = fl.peak_f / kG;
  return r;
}

// ------------------------------------------------------------------------------------------------
struct HRes { double first_touch, touch_vx, touch_vz, burn_s, pred_at_cut, peak_th, est_touch; int end_code; bool flew; Macro end; bool started; std::string why; };
static HRes runHop(const Cfg& c, bool attitude_hold = true) {
  Plant pl; Params P; setupParams(c, pl, P); P.hop_attitude_hold = attitude_hold;
  Noise nz; nz.seed(c.seed); nz.acc_bias.z = (float)c.bias_z; nz.scale_err = c.scale;
  Flight fl(P);
  calAndArm(fl, pl, nz, false);
  nz.gyro_bias_err = c.gyro_bias_err;
  HopArgs a; a.dist_m = (float)c.dist; a.lean_deg = (float)c.lean; a.brake = c.brake; a.range_scale = P.hop_range_scale;
  macroSeq.clear(); Macro last = Macro::ARMED;
  HRes r{}; Command latched; const double dtp = 0.0005; int tick = 0; bool started = false;
  for (int k = 0; k < 24000; k++) {
    double tt = k * dtp;
    if (k % 10 == 0) {
      Vec3 g_, a_; sense(pl, nz, false, g_, a_);
      if (!started && tt > 1.0) { const char* w = ""; r.started = fl.startHop(a, w); r.why = w; started = true; }
      Command cmd = fl.update(g_, a_, 0.005f);
      if (started) trackMacro(last, fl.macro());
      if (tick++ % 4 == 0) latched = cmd;
      applyCmd(fl.outputsLive() ? latched : Command(), pl, P);
      if (c.trace && started && (k % 100 == 0)) printf("   t=%.2f %-11s th=%6.1f pod=%5.1f u=%.2f T=%5.1f est_x=%.2f est_z=%.3f vz=%.2f pred=%.2f\n", tt, macroName(fl.macro()), pl.th * kRad2Deg, pl.alpha * kRad2Deg, cmd.collective, pl.T, fl.est.pos.x, fl.est.pos.z, fl.est.vel.z, fl.hop.predictedRange());
    }
    pl.step(dtp);
    if (started && (fl.macro() == Macro::REST || fl.macro() == Macro::FAULT || (fl.macro() == Macro::ARMED && !r.started) ||
                    (fl.macro() == Macro::ARMED && tt > 2.0))) break;
  }
  r.first_touch = pl.touch_xc / 0.3048; r.touch_vx = pl.touch_vx; r.touch_vz = -pl.touch_vz; r.flew = pl.has_flown;
  r.burn_s = fl.hop.burn_s; r.pred_at_cut = fl.hop.cut_pred_along / 0.3048; r.end_code = fl.hop.end_code; r.end = fl.macro();
  r.peak_th = std::max(fabs(pl.max_th), fabs(pl.min_th)) * kRad2Deg; r.est_touch = fl.hop.touch_along / 0.3048;
  return r;
}

// ------------------------------------------------------------------------------------------------
struct VRes { double apex, touch_vz, pred_at_cut, est_apex, burn_s, peak_th; int end_code; Macro end; bool started; std::string why; bool flew; };
static VRes runVHop(const Cfg& c) {
  Plant pl; Params P; setupParams(c, pl, P);
  Noise nz; nz.seed(c.seed); nz.acc_bias.z = (float)c.bias_z; nz.scale_err = c.scale; nz.vib_bias = c.vib;
  Flight fl(P);
  calAndArm(fl, pl, nz, false);
  nz.gyro_bias_err = c.gyro_bias_err;
  VHopArgs a; a.height_m = (float)c.vh; a.brake = c.brake;
  macroSeq.clear(); Macro last = Macro::ARMED;
  VRes r{}; Command latched; const double dtp = 0.0005; int tick = 0; bool started = false;
  for (int k = 0; k < 24000; k++) {
    double tt = k * dtp;
    if (k % 10 == 0) {
      Vec3 g_, a_; sense(pl, nz, false, g_, a_);
      if (!started && tt > 1.0) { const char* w = ""; r.started = fl.startVHop(a, w); r.why = w; started = true; }
      Command cmd = fl.update(g_, a_, 0.005f);
      if (started) trackMacro(last, fl.macro());
      if (tick++ % 4 == 0) latched = cmd;
      applyCmd(fl.outputsLive() ? latched : Command(), pl, P);
      if (c.trace && started && (k % 100 == 0)) printf("   t=%.2f %-11s true z=%.3f vz=%5.2f th=%5.1f | est z=%.3f vz=%5.2f apex_pred=%.2f u=%.2f T=%5.1f\n", tt, macroName(fl.macro()), pl.has_flown ? pl.z - (pl.R - pl.b) : 0.0, pl.vz, pl.th * kRad2Deg, fl.est.pos.z, fl.est.vel.z, fl.vhop.predictedApex(), cmd.collective, pl.T);
    }
    pl.step(dtp);
    if (started && (fl.macro() == Macro::REST || fl.macro() == Macro::FAULT || (fl.macro() == Macro::ARMED && tt > 1.01 && !r.started))) break;
  }
  r.apex = pl.max_z; r.touch_vz = -pl.touch_vz; r.pred_at_cut = fl.vhop.cut_pred_apex; r.est_apex = fl.vhop.apex_z; r.burn_s = fl.vhop.burn_s;
  r.end_code = fl.vhop.end_code; r.end = fl.macro(); r.flew = pl.has_flown; r.peak_th = std::max(fabs(pl.max_th), fabs(pl.min_th)) * kRad2Deg;
  return r;
}

// ------------------------------------------------------------------------------------------------
static void sweep() {
  printf("Descent preset sweep. kp_slow extra plan flare vt | IDEAL (TWR{1.5,1.85} x lag{.15,.25,.40}, lag known) worst/mean | ERRORS scale+-3%% x vib{-.3,+.3} x height{-.5,0,+.5} worst/mean, time\n");
  for (double ks : {1.5, 3.0}) for (double extra : {0.05, 0.15, 0.25, 0.35}) for (double pf : {0.6, 0.75}) for (double vt : {0.5, 0.8}) {
    auto tw = [&](Params& P) { P.vel_kp_slow = (float)ks; P.lead_extra_s = (float)extra; P.brake_plan_fraction = (float)pf; P.v_touch_mps = (float)vt; };
    double w1 = 0, s1 = 0; int n1 = 0;
    for (double twr : {1.5, 1.85}) for (double tau : {0.15, 0.25, 0.40}) { Cfg c; c.twr = twr; c.tau = tau; c.tweak = tw; DRes r = runDescent(c); w1 = std::max(w1, r.vz); s1 += r.vz; n1++; }
    double w2 = 0, s2 = 0, ts = 0; int n2 = 0;
    for (double sc : {-0.03, 0.03}) for (double vib : {-0.3, 0.3}) for (double dh : {-0.5, 0.0, 0.5}) for (double twr : {1.5, 1.85}) {
      Cfg c; c.twr = twr; c.scale = sc; c.vib = vib; c.H_assumed = c.H_true + dh; c.tweak = tw; DRes r = runDescent(c); w2 = std::max(w2, r.vz); s2 += r.vz; ts += r.t; n2++; }
    printf("%.1f %.2f %.2f %.1f | %5.2f %5.2f | %5.2f %5.2f  %.1fs\n", ks, extra, pf, vt, w1, s1 / n1, w2, s2 / n2, ts / n2);
  }
}

static void attitudeHang(bool old_behaviour, double gyro_err) {
  Plant pl; pl.ground = false; pl.has_flown = false; pl.z = 10; pl.th = 15 * kDeg2Rad; pl.Tmax = 1.3 * pl.m * pl.g;
  Params P; P.thrust_max_N = (float)pl.Tmax; P.throttle_cap = 1.0f; Noise nz; Flight fl(P);
  Vec3 g_, a_; double sx = 0, sz = 0; for (int i = 0; i < 200; i++) { sense(pl, nz, true, g_, a_); sx += a_.x; sz += a_.z; }
  fl.est.initAtRest(Vec3{(float)(sx / 200), 0, (float)(sz / 200)}); fl.est.setGyroBias(Vec3{0, nz.gyro_bias_y, 0});
  nz.gyro_bias_err = gyro_err;
  fl.est.accel_correct = old_behaviour;      // v1 left accel correction on in the air; v2 turns it off
  Command latched; int tick = 0; pl.cmd_u = 0.77;
  printf("  %s, gyro bias error %.1f deg/s: ", old_behaviour ? "v1 (accel correction ON in air)" : "v2 (gyro only in air)     ", gyro_err * kRad2Deg);
  for (int k = 0; k <= 8000; k++) {
    if (k % 10 == 0) {
      sense(pl, nz, false, g_, a_);
      Command c; c.motors_on = true; c.collective = 0.77f; c.use_att = true; fl.est.update(g_, a_, 0.005f); fl.att.update(fl.est, c, 0.005f);
      if (tick++ % 4 == 0) latched = c;
      pl.cmd_alpha = latched.tilt_x_deg * kDeg2Rad; }
    pl.step(0.0005);
    if (k % 1600 == 0) printf("t=%.1fs %5.1f  ", k * 0.0005, pl.th * kRad2Deg);
  }
  printf(" (true body tilt, deg)\n");
}

static void calTest() {
  printf("\n=== 6. Six-position accelerometer calibration solver (synthetic, sensor frame) ===\n");
  std::mt19937 rng(3); std::normal_distribution<double> n(0, 1);
  const double off[3] = {0.35, -0.22, 0.61}, scl[3] = {1.021, 0.984, 1.012};   // invented 'true' errors for the test
  auto meas = [&](double ux, double uy, double uz, double tilt_err_deg) {
    double nn = sqrt(ux * ux + uy * uy + uz * uz); ux /= nn; uy /= nn; uz /= nn;
    double e = tilt_err_deg * kDeg2Rad; double vx = ux + e * n(rng) * 0.7, vy = uy + e * n(rng) * 0.7, vz = uz + e * n(rng) * 0.7;
    nn = sqrt(vx * vx + vy * vy + vz * vz); double u[3] = {vx / nn * kG, vy / nn * kG, vz / nn * kG};
    Vec3 r; float* p = &r.x;
    for (int i = 0; i < 3; i++) p[i] = (float)(u[i] / scl[i] + off[i] + 0.004 * n(rng));   // 2 s average noise
    return r;
  };
  for (int variant = 0; variant < 3; variant++) {
    std::vector<Vec3> s;
    double err = variant == 0 ? 2.0 : 8.0;
    const double faces[6][3] = {{1, 0, 0}, {-1, 0, 0}, {0, 1, 0}, {0, -1, 0}, {0, 0, 1}, {0, 0, -1}};
    for (auto& f : faces) s.push_back(meas(f[0], f[1], f[2], err));
    if (variant >= 1) { s.push_back(meas(1, 1, 0, err)); s.push_back(meas(0, 1, 1, err)); s.push_back(meas(1, 0, -1, err)); s.push_back(meas(-1, -1, 1, err)); }
    if (variant == 2) { s.pop_back(); s.erase(s.begin() + 1); }   // remove the "-x down" pose -> must refuse... (x never < -0.5 g?)
    AccelCal c; const char* why = "";
    bool ok = solveAccelCal(s.data(), (int)s.size(), c, why);
    printf("  %zu poses, pose error ~%.0f deg: %s %s", s.size(), err, ok ? "ACCEPTED" : "REFUSED:", ok ? "" : why);
    if (ok) printf(" off %.3f %.3f %.3f (true .350 -.220 .610)  scale %.4f %.4f %.4f (true 1.0210 .9840 1.0120)  rms %.3f", c.off[0], c.off[1], c.off[2], c.scl[0], c.scl[1], c.scl[2], c.rms);
    printf("\n");
  }
}

static void supervisorTests() {
  printf("\n=== 7. Supervisor / macrostate checks ===\n");
  { Params P; Flight fl(P); const char* w = ""; bool ok = fl.arm(w); printf("  arm without calibration: %s (%s)\n", ok ? "ALLOWED (BUG)" : "refused", w); }
  { Params P; P.throttle_cap = 1.0f; Flight fl(P); fl.calibrated = true; const char* w; fl.arm(w);
    HopArgs a; a.lean_deg = 40; bool ok = fl.startHop(a, w); printf("  hop with lean 40 deg (> gimbal limit 25): %s (%s), state %s\n", ok ? "STARTED (BUG)" : "refused", w, macroName(fl.macro())); }
  { Params P; P.throttle_cap = 1.0f; Flight fl(P); fl.calibrated = true; const char* w; fl.arm(w);
    VHopArgs a; a.height_m = 3.0f; bool ok = fl.startVHop(a, w); printf("  vhop 3 m (> max %.1f m): %s (%s)\n", P.vhop_max_height_m, ok ? "STARTED (BUG)" : "refused", w); }
  { // hop when the drone keeps moving -> not at rest -> back to Armed
    Params P; P.throttle_cap = 1.0f; P.thrust_max_N = 31.5f; Flight fl(P); fl.calibrated = true; const char* w; fl.arm(w); HopArgs a; fl.startHop(a, w);
    macroSeq.clear(); Macro last = Macro::ARMED;
    for (int i = 0; i < 1000; i++) { Vec3 g{0.3f, 0, 0}, acc{0, 0, kG}; fl.update(g, acc, 0.005f); trackMacro(last, fl.macro()); }
    printf("  hop while drone is rotating (never at rest): %s\n", macroSeq.c_str()); }
  { Params P; P.throttle_cap = 1.0f; Flight fl(P); fl.calibrated = true; const char* w; fl.arm(w); fl.startDescent(6.0f, w);
    fl.kill("operator kill"); const char* m1 = macroName(fl.macro()); bool armAgain = fl.arm(w); fl.disarm(); const char* m2 = macroName(fl.macro()); bool arm2 = fl.arm(w);
    printf("  kill during Await Release -> %s; arm from Fault: %s; after disarm: %s -> arm: %s\n", m1, armAgain ? "ALLOWED (BUG)" : "refused", m2, arm2 ? "ok" : "refused"); }
  { Params P; P.throttle_cap = 1.0f; P.thrust_max_N = 31.5f; Flight fl(P); fl.calibrated = true; const char* w; fl.arm(w); bool ok = fl.startDescent(6.0f, w);
    bool ok2 = fl.startHop(HopArgs(), w); printf("  second routine while descent armed: %s (%s)\n", (ok && !ok2) ? "refused" : "ALLOWED (BUG)", w); }
  { Params P; P.throttle_cap = 1.0f; P.await_release_timeout_s = 2.0f; Flight fl(P); fl.calibrated = true; const char* w; fl.arm(w); fl.startDescent(6.0f, w);
    for (int i = 0; i < 600; i++) { Vec3 g, acc{0, 0, kG}; fl.update(g, acc, 0.005f); }
    printf("  descent, no release within await timeout (2 s here): %s (%s)\n", macroName(fl.macro()), fl.fault_reason); }
}

int main(int argc, char** argv) {
  if (argc > 1 && std::string(argv[1]) == "sweep") { sweep(); return 0; }
  if (argc > 1 && std::string(argv[1]) == "trace") {
    printf("descent trace (TWR 1.5, ROBUST)\n"); { Cfg c; c.trace = true; DRes r = runDescent(c); printf("  %s\n", macroSeq.c_str()); (void)r; }
    printf("hop trace (TWR 1.5, 5 ft)\n"); { Cfg c; c.trace = true; runHop(c); printf("  %s\n", macroSeq.c_str()); }
    printf("vhop trace (TWR 1.5, 0.5 m, brake)\n"); { Cfg c; c.trace = true; c.brake = true; runVHop(c); printf("  %s\n", macroSeq.c_str()); }
    return 0;
  }
  printf("Weeble Phase 1 sim (v2). Planar, assumed plant. Every number below is a MODEL result, not a measurement.\n");
  printf("\n=== 0. Mixer sanity ===\n");
  { Params P; P.throttle_cap = 0.3f; Command c; c.motors_on = true; c.collective = 0.5f; c.tilt_x_deg = 10; c.tilt_y_deg = -40; c.yaw = 0.02f;
    Outputs o = mix(c, P);
    printf("  collective 0.5 capped at 0.3 -> ESC us %u/%u (expect 1300/1300), servo x %u y %u (y clamped at tilt limit -> 1375)\n", o.esc_upper_us, o.esc_lower_us, o.servo_x_us, o.servo_y_us); }

  printf("\n=== 1. Attitude loop in the air (start tilted 15 deg, thrust ~ weight) -- estimator fix ===\n");
  attitudeHang(true, 0.0); attitudeHang(false, 0.0); attitudeHang(false, 0.5 * kDeg2Rad);

  printf("\n=== 2. Controlled Landing from a 6.1 m release (IMU only). Touchdown speed [m/s], free fall would be ~10.9 ===\n");
  { Cfg c; c.throttle_cap = 0.15; DRes r = runDescent(c); printf("  throttle cap 15 %% (boot default): started=%d -> %s, state %s   [v1: started, hit at 9.9 m/s]\n", (int)r.started, r.why.c_str(), macroName(r.end)); }
  { Cfg c; c.throttle_cap = 0.70; DRes r = runDescent(c); printf("  throttle cap 70 %% at TWR 1.5 (spare decel %.2f m/s^2 < 0.5): started=%d\n", 0.70 * 1.5 * kG - kG, (int)r.started); }
  { Cfg c; c.throttle_cap = 0.80; DRes r = runDescent(c); printf("  throttle cap 80 %% at TWR 1.5 (spare decel %.2f m/s^2): started=%d, touchdown %.2f m/s (plans with the capped thrust)\n", 0.80 * 1.5 * kG - kG, (int)r.started, r.vz); }
  { Cfg c; c.twr = 1.05; DRes r = runDescent(c); printf("  TWR 1.05: started=%d (%s)\n", (int)r.started, r.why.c_str()); }
  for (int gentle = 0; gentle < 2; gentle++) {
    auto tw = [gentle](Params& P) { applyDescentPreset(P, gentle != 0); };
    printf("  -- preset %s --\n", gentle ? "GENTLE" : "ROBUST (default)");
    { Cfg c; c.tweak = tw; DRes r = runDescent(c); printf("    nominal (TWR 1.5, lag 0.25 known): %.2f m/s, %.1f s, tilt %.1f deg, sequence: %s\n", r.vz, r.t, r.tilt, macroSeq.c_str()); }
    printf("    thrust lag MEASURED and entered (controller knows it):\n");
    for (double twr : {1.30, 1.50, 1.85}) { printf("      TWR %.2f: ", twr); for (double tau : {0.15, 0.25, 0.40}) { Cfg c; c.twr = twr; c.tau = tau; c.tweak = tw; DRes r = runDescent(c); printf("lag %.2f -> %.2f (%.1fs)  ", tau, r.vz, r.t); } printf("\n"); }
    printf("    thrust lag left at the 0.25 s placeholder (true lag differs):\n      TWR 1.50: ");
    for (double tau : {0.15, 0.40}) { Cfg c; c.tau = tau; c.ctrl_tau = 0.25; c.tweak = tw; DRes r = runDescent(c); printf("true lag %.2f -> %.2f   ", tau, r.vz); } printf("\n");
    printf("    release height error (assumed - true): ");
    for (double dh : {-1.0, -0.5, 0.0, 0.5, 1.0}) { Cfg c; c.H_assumed = c.H_true + dh; c.tweak = tw; DRes r = runDescent(c); printf("%+.1f m -> %.2f  ", dh, r.vz); } printf("\n");
    printf("    thrust belief (x real): ");
    for (double e : {0.85, 1.15}) { Cfg c; c.ctrl_twr_err = e; c.tweak = tw; DRes r = runDescent(c); printf("x%.2f -> %.2f  ", e, r.vz); }
    printf("  payload entered wrong (real - entered): ");
    for (double e : {-0.2, 0.2}) { Cfg c; c.payload_err = e; c.tweak = tw; DRes r = runDescent(c); printf("%+.1f kg -> %.2f  ", e, r.vz); } printf("\n");
    printf("    accel scale / offset after the rest reading: ");
    for (double sc : {-0.03, 0.03}) { Cfg c; c.scale = sc; c.tweak = tw; DRes r = runDescent(c); printf("scale %+.0f%% -> %.2f  ", sc * 100, r.vz); }
    for (double dr : {-0.3, 0.3}) { Cfg c; c.drift_z = dr; c.tweak = tw; DRes r = runDescent(c); printf("offset %+.1f -> %.2f  ", dr, r.vz); } printf("\n");
    printf("    vibration offset only while rotors run (m/s^2, - = under-reads): ");
    for (double vb : {-0.3, -0.1, -0.05, 0.0, 0.05, 0.3}) { Cfg c; c.vib = vb; c.tweak = tw; DRes r = runDescent(c); printf("%+.2f -> %.2f  ", vb, r.vz); } printf("\n");
  }

  printf("\n=== 3. Launch Horizontal Distance (hop). 'first touch' = where the base first hits (what the controller aims at) ===\n");
  printf("  (roll-out after touchdown depends on ground friction / shell: NOT modelled credibly, not shown)\n");
  { Cfg c; c.throttle_cap = 0.15; HRes h = runHop(c); printf("  throttle cap 15 %%: started=%d (%s)   [v1: started, never lifted off, no reason given]\n", (int)h.started, h.why.c_str()); }
  { Cfg c; HRes h = runHop(c); printf("  nominal 5 ft, TWR 1.5: first touch %.2f ft (controller estimate %.2f ft), predicted at cut %.2f ft, burn %.2f s, horiz speed %.1f m/s\n    sequence: %s\n", h.first_touch, h.est_touch, h.pred_at_cut, h.burn_s, h.touch_vx, macroSeq.c_str()); }
  printf("  target distance -> first touch [ft] (TWR 1.50 / 1.85, default lean 15):\n    ");
  for (double ft : {2.0, 3.0, 4.0, 5.0, 6.0, 8.0}) { Cfg c; c.dist = ft * 0.3048; HRes a = runHop(c); Cfg d = c; d.twr = 1.85; HRes b = runHop(d); printf("%.0f -> %.2f / %.2f   ", ft, a.first_touch, b.first_touch); } printf("\n");
  printf("  TWR 1.30 lean 25 (old default): "); { Cfg c; c.twr = 1.3; c.lean = 25; HRes h = runHop(c); printf("started=%d (%s)\n", (int)h.started, h.why.c_str()); }
  printf("  TWR 1.30 lean 15: "); { Cfg c; c.twr = 1.3; c.lean = 15; HRes h = runHop(c); printf("first touch %.2f ft, %s\n", h.first_touch, hopEndText(h.end_code)); }
  printf("  old default lean 25, 3 ft target, TWR 1.5, 6 seeds: "); for (int sd = 1; sd <= 6; sd++) { Cfg c; c.lean = 25; c.dist = 3 * 0.3048; c.seed = sd; HRes a = runHop(c); printf("%.2f ", a.first_touch); } printf(" (cut often fires on the first airborne tick)\n");
  printf("  lean 15, 3 ft target, same seeds: "); for (int sd = 1; sd <= 6; sd++) { Cfg c; c.dist = 3 * 0.3048; c.seed = sd; HRes a = runHop(c); printf("%.2f ", a.first_touch); } printf("\n");
  printf("  sensitivity at 5 ft, TWR 1.5, lean 15 (first touch, ft):\n    ");
  for (double tau : {0.15, 0.40}) { Cfg c; c.tau = tau; HRes a = runHop(c); printf("lag %.2f: %.2f (%s)   ", tau, a.first_touch, hopEndText(a.end_code)); } printf("\n    ");
  for (double I : {0.03, 0.06}) { Cfg c; c.I = I; HRes a = runHop(c); printf("inertia %.2f: %.2f   ", I, a.first_touch); }
  for (double e : {0.85, 1.15}) { Cfg c; c.ctrl_twr_err = e; HRes a = runHop(c); printf("thrust belief x%.2f: %.2f%s   ", e, a.first_touch, a.started ? "" : " (refused)"); } printf("\n    ");
  for (double pb : {-2.0, 2.0}) { Cfg c; c.pod_bias_deg = pb; HRes a = runHop(c); printf("pod misalignment %+.0f deg: %.2f   ", pb, a.first_touch); }
  for (double pe : {-0.2, 0.2}) { Cfg c; c.payload_err = pe; HRes a = runHop(c); printf("payload err %+.1f kg: %.2f   ", pe, a.first_touch); }
  for (double gb : {0.5}) { Cfg c; c.gyro_bias_err = gb * kDeg2Rad; HRes a = runHop(c); printf("gyro bias err %.1f deg/s: %.2f   ", gb, a.first_touch); }
  for (double ln : {10.0, 20.0}) { Cfg c; c.lean = ln; HRes a = runHop(c); printf("lean %.0f: %.2f   ", ln, a.first_touch); } printf("\n    seeds: ");
  for (int s = 1; s <= 6; s++) { Cfg c; c.seed = s; HRes a = runHop(c); printf("%.2f ", a.first_touch); } printf("\n");
  { Cfg c; c.brake = true; HRes a = runHop(c); printf("  landing brake ON: first touch %.2f ft, touchdown vz %.2f m/s, sequence: %s\n", a.first_touch, a.touch_vz, macroSeq.c_str()); }
  { Cfg c; HRes o = runHop(c, false); printf("  open-loop pod mode (not recommended): first touch %.2f ft, peak body pitch %.0f deg\n", o.first_touch, o.peak_th); }

  printf("\n=== 4. Launch Vertical Distance (vertical hop). true apex of the base [m] / touchdown speed [m/s] ===\n");
  { Cfg c; c.brake = true; c.vh = 0.5; VRes v = runVHop(c); printf("  nominal 0.5 m, brake, TWR 1.5: apex %.2f m (est %.2f, predicted at cut %.2f), touchdown %.2f m/s, burn %.2f s, peak tilt %.1f deg\n    sequence: %s\n", v.apex, v.est_apex, v.pred_at_cut, v.touch_vz, v.burn_s, v.peak_th, macroSeq.c_str()); }
  { Cfg c; c.throttle_cap = 0.15; c.brake = true; VRes v = runVHop(c); printf("  throttle cap 15 %%: started=%d (%s)\n", (int)v.started, v.why.c_str()); }
  printf("  target -> apex / touchdown, landing brake ON:\n");
  for (double twr : {1.30, 1.50, 1.85}) { printf("    TWR %.2f lag .25: ", twr); for (double h : {0.15, 0.30, 0.50, 0.75, 1.00}) { Cfg c; c.twr = twr; c.brake = true; c.vh = h; VRes v = runVHop(c); printf("%.2f -> %.2f/%.2f  ", h, v.apex, v.touch_vz); } printf("\n"); }
  for (double tau : {0.15, 0.40}) { printf("    TWR 1.50 lag %.2f: ", tau); for (double h : {0.15, 0.30, 0.50, 0.75, 1.00}) { Cfg c; c.tau = tau; c.brake = true; c.vh = h; VRes v = runVHop(c); printf("%.2f -> %.2f/%.2f  ", h, v.apex, v.touch_vz); } printf("\n"); }
  printf("  ballistic (no brake), TWR 1.5: ");
  for (double h : {0.15, 0.30, 0.50}) { Cfg c; c.brake = false; c.vh = h; VRes v = runVHop(c); printf("%.2f -> %.2f/%.2f  ", h, v.apex, v.touch_vz); } printf("\n");
  printf("  sensitivity at 0.5 m, brake, TWR 1.5 (apex / touchdown):\n    ");
  for (double e : {0.85, 1.15}) { Cfg c; c.brake = true; c.ctrl_twr_err = e; VRes v = runVHop(c); printf("thrust belief x%.2f: %.2f/%.2f   ", e, v.apex, v.touch_vz); }
  for (double tl : {0.15, 0.40}) { Cfg c; c.brake = true; c.ctrl_tau = tl; VRes v = runVHop(c); printf("lag believed %.2f (true .25): %.2f/%.2f   ", tl, v.apex, v.touch_vz); } printf("\n    ");
  for (double pb : {-2.0, 2.0}) { Cfg c; c.brake = true; c.pod_bias_deg = pb; VRes v = runVHop(c); printf("pod misalign %+.0f: %.2f/%.2f tilt %.0f   ", pb, v.apex, v.touch_vz, v.peak_th); }
  for (double vb : {-0.3, 0.3}) { Cfg c; c.brake = true; c.vib = vb; VRes v = runVHop(c); printf("vib %+.1f: %.2f/%.2f   ", vb, v.apex, v.touch_vz); }
  for (double sc : {-0.03, 0.03}) { Cfg c; c.brake = true; c.scale = sc; VRes v = runVHop(c); printf("scale %+.0f%%: %.2f/%.2f   ", sc * 100, v.apex, v.touch_vz); } printf("\n    seeds: ");
  for (int s = 1; s <= 6; s++) { Cfg c; c.brake = true; c.seed = s; VRes v = runVHop(c); printf("%.2f/%.2f ", v.apex, v.touch_vz); } printf("\n");

  printf("\n=== 5. Abort paths (CSV 'Abort / Fallback' column) ===\n");
  { Cfg c; c.tweak = [](Params& P) { P.max_powered_s = 1.2f; }; DRes r = runDescent(c); printf("  descent, powered-time limit forced to 1.2 s: motors cut, touchdown %.2f m/s, %s\n", r.vz, macroSeq.c_str()); }
  { Cfg c; c.twr = 0.95; c.ctrl_twr_err = 1.6; HRes h = runHop(c); printf("  hop, real TWR 0.95 but controller believes 1.52: '%s', %s\n", hopEndText(h.end_code), macroSeq.c_str()); }
  { Cfg c; c.twr = 0.95; c.ctrl_twr_err = 1.6; c.brake = true; VRes v = runVHop(c); printf("  vhop, same: '%s', %s\n", hopEndText(v.end_code), macroSeq.c_str()); }
  { Cfg c; c.pod_bias_deg = 30; c.brake = true; VRes v = runVHop(c); printf("  vhop, 30 deg pod misalignment (beyond gimbal authority): '%s', %s\n", hopEndText(v.end_code), macroSeq.c_str()); }
  { Cfg c; c.pod_bias_deg = 20; c.brake = true; VRes v = runVHop(c); printf("  vhop, 20 deg pod misalignment: apex %.2f m, peak tilt %.0f deg, touchdown %.2f m/s, '%s'\n", v.apex, v.peak_th, v.touch_vz, hopEndText(v.end_code)); }

  calTest();
  supervisorTests();
  return 0;
}
