// sim_test.cpp -- host-side test of weeble_core.h against an ASSUMED planar plant.
//   g++ -O2 -std=c++17 -o sim_test sim_test.cpp && ./sim_test
//
// What this checks: the control LOGIC (release detection, dead-reckoned braking profile, tip/burn/cut hop),
// the sign conventions of the attitude loop, and sensitivity to the big unknowns (TWR, thrust lag, inertia,
// release-height error, accel bias).  What it does NOT check: yaw / lateral axis, real servo & ESC behaviour,
// motor vibration in the IMU, aerodynamics, real ground friction, 3-D effects.  Plant numbers marked ASSUMED
// are placeholders until measured.
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
  double I = 0.04;       // ASSUMED pitch inertia about CG [kg m^2]; axial point-mass lower bound was 0.028   TODO(CAD)
  // ---- propulsion ----
  double Tmax = 27.25, tau_up = 0.25, tau_dn = 0.25;     // tau ASSUMED
  double servo_rate = 11.6, servo_tau = 0.03;            // ~667 deg/s (URL says 0.09 s/60deg; unverified) ASSUMED
  double c_roll = 0.03;                                  // rolling damping, ASSUMED (just lets it settle)
  // ---- state ----
  bool ground = true; double x0 = 0, th = 0, thd = 0;    // ground (rolling) state
  double x = 0, z = 0, vx = 0, vz = 0;                   // CG state when airborne
  double T = 0, alpha = 0, cmd_u = 0, cmd_alpha = 0;
  double ax = 0, az = 0, t = 0;
  bool has_flown = false, touched = false;
  double touch_vz = 0, touch_slip = 0, touch_xc = 0, touch_vx = 0, touch_rest_x = 0, touch_t = 0, touch_th = 0, max_th = 0, min_th = 0;

  double Mth() const { return m * (R * R - 2 * R * b * cos(th) + b * b) + I; }

  void step(double dt) {
    double Tcmd = Tmax * cmd_u;
    T += (Tcmd - T) * dt / (Tcmd > T ? tau_up : tau_dn);
    double rate = std::clamp((cmd_alpha - alpha) / servo_tau, -servo_rate, servo_rate);
    alpha += rate * dt;
    double phi = th + alpha, Fx = T * sin(phi), Fz = T * cos(phi);
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
        track(); t += dt; return;
      }
    }
    ax = Fx / m; az = Fz / m - g;
    double thdd = T * d * sin(alpha) / I;
    vx += ax * dt; vz += az * dt; x += vx * dt; z += vz * dt; thd += thdd * dt; th += thd * dt;
    double zc = z + b * cos(th), vzc = vz - b * sin(th) * thd;
    if (zc <= R && vzc < 0 && has_flown) {     // touchdown of the spherical base (plastic, no slip)
      double xc = x + b * sin(th), vcx = vx + b * cos(th) * thd;
      if (!touched) {
        touched = true; touch_vz = vzc; touch_slip = vcx - R * thd; touch_t = t; touch_th = th;
        touch_rest_x = xc - R * th; touch_xc = xc; touch_vx = vcx;
      }
      double rx = -b * sin(th), rz = z;
      double L = I * thd + m * (rz * vx - rx * vz);
      x0 = xc - R * th; thd = L / Mth(); ground = true;
    }
    track(); t += dt;
  }
  void track() { if (!touched) { max_th = std::max(max_th, th); min_th = std::min(min_th, th); } }
  double restX() const { return ground ? x0 + R * th : x; }
};

struct Noise {
  std::mt19937 rng{1};
  std::normal_distribution<double> n{0.0, 1.0};
  double acc_sigma = 0.05, gyr_sigma = 0.002;      // ASSUMED MPU6050-class noise after filtering
  Vec3 acc_bias{0.0f, 0.0f, 0.1f};                 // ASSUMED uncalibrated bias [m/s^2] (x,y,z body)
  float gyro_bias_y = 0.01f;
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
  gyro = Vec3{(float)(nz.gyr_sigma * nz.n(nz.rng)), (float)(w + nz.gyro_bias_y + nz.gyr_sigma * nz.n(nz.rng)), (float)(nz.gyr_sigma * nz.n(nz.rng))};
}

struct Cfg { double twr = 1.30, tau = 0.25, I = 0.04, bias_z = 0.1, scale = 0.0, vib = 0.0, drift_z = 0.0, H_true = 6.096, H_assumed = 6.096, ctrl_twr_err = 1.0, th0_deg = 5;
             int seed = 1; std::function<void(Params&)> tweak; bool trace = false; bool brake = false; double lean = 25; double dist = 1.524; double throttle_cap = 1.0; };

static void applyCmd(const Command& c, Plant& pl, double cap) {
  pl.cmd_u = c.motors_on ? std::min<double>(c.collective, cap) : 0.0;
  pl.cmd_alpha = std::clamp<double>(c.tilt_x_deg, -25.0, 25.0) * kDeg2Rad;
}

// ------------------------------------------------------------------------------------------------
struct DRes { double vz, slip, tilt, t; int st; };
static DRes runDescent(const Cfg& c) {
  Plant pl; pl.I = c.I; pl.tau_up = pl.tau_dn = c.tau; pl.Tmax = c.twr * pl.m * pl.g;
  pl.ground = false; pl.has_flown = true; pl.z = 0.15 + c.H_true; pl.th = c.th0_deg * kDeg2Rad;
  Params P; P.thrust_max_N = (float)(pl.Tmax * c.ctrl_twr_err); P.mass_kg = (float)pl.m; if (c.tweak) c.tweak(P);
  Noise nz; nz.seed(c.seed); nz.acc_bias.z = (float)c.bias_z; nz.scale_err = c.scale; nz.vib_bias = c.vib;
  Flight fl(P);
  Vec3 g_, a_; double sx = 0, sy = 0, sz = 0; const int N = 200;
  for (int i = 0; i < N; i++) { sense(pl, nz, true, g_, a_); sx += a_.x; sy += a_.y; sz += a_.z; }
  fl.est.initAtRest(Vec3{(float)(sx / N), (float)(sy / N), (float)(sz / N)});
  fl.est.setGyroBias(Vec3{0, nz.gyro_bias_y, 0});
  nz.acc_bias.z += (float)c.drift_z;   // error that appears AFTER the at-rest calibration (temperature, scale error, vibration)
  fl.armDescent((float)c.H_assumed);
  Command latched; const double dtp = 0.0005; int tick = 0; bool released = false; double t_rel = 0;
  for (int k = 0; k < 40000; k++) {
    double tt = k * dtp;
    if (!released && tt > 0.3) { released = true; pl.ground = false; }
    if (k % 10 == 0) {
      sense(pl, nz, !released, g_, a_);
      Command cmd = fl.update(g_, a_, 0.005f);
      if (tick++ % 4 == 0) latched = cmd;       // 50 Hz pulse frame latch
      applyCmd(latched, pl, c.throttle_cap);
      if (c.trace && released && (k % 200 == 0)) printf("   t=%.2f dstate=%d true z=%.2f vz=%.2f | est z=%.2f vz=%.2f | vref=%.2f u=%.2f T=%.1f fz_meas=%.1f\n", tt-0.3, (int)fl.desc.state(), pl.z-0.15+pl.b*(1-cos(pl.th))*0, pl.vz, fl.est.pos.z, fl.est.vel.z, fl.desc.lastVref(), cmd.collective, pl.T, fl.est.f_body.z);
    }
    if (released) { if (t_rel == 0) t_rel = tt; pl.step(dtp); }
    if (pl.touched && tt - (t_rel + pl.touch_t) > 0.2) break;
  }
  return {pl.touch_vz, pl.touch_slip, pl.touch_th * kRad2Deg, pl.touch_t, (int)fl.desc.state()};
}

// ------------------------------------------------------------------------------------------------
struct HRes { double first_touch, touch_vx, range, apex_lean_max, t_air, touch_vz, peak_th; int st; bool flew; double burn_s; bool maxburn; double pred_at_cut; };
static HRes runHop(const Cfg& c, bool attitude_hold) {
  Plant pl; pl.I = c.I; pl.tau_up = pl.tau_dn = c.tau; pl.Tmax = c.twr * pl.m * pl.g;
  Params P; P.thrust_max_N = (float)(pl.Tmax * c.ctrl_twr_err); P.mass_kg = (float)pl.m;
  P.hop_attitude_hold = attitude_hold; P.hop_land_brake = c.brake; P.hop_lean_deg = (float)c.lean;
  P.thrust_lag_down_s = (float)c.tau; P.thrust_lag_up_s = (float)c.tau; P.lead_time_s = (float)c.tau; if (c.tweak) c.tweak(P);
  Noise nz; nz.seed(c.seed); nz.acc_bias.z = (float)c.bias_z;
  Flight fl(P);
  Vec3 g_, a_; double sx = 0, sy = 0, sz = 0; const int N = 200;
  for (int i = 0; i < N; i++) { sense(pl, nz, true, g_, a_); sx += a_.x; sy += a_.y; sz += a_.z; }
  fl.est.initAtRest(Vec3{(float)(sx / N), (float)(sy / N), (float)(sz / N)});
  fl.est.setGyroBias(Vec3{0, nz.gyro_bias_y, 0});
  Command latched; const double dtp = 0.0005; int tick = 0; bool started = false; double t_launch = -1, t_burn0 = -1, t_burn1 = -1;
  double pred_cut = 0; bool prevBurn = false;
  for (int k = 0; k < 20000; k++) {           // 10 s
    double tt = k * dtp;
    if (k % 10 == 0) {
      sense(pl, nz, false, g_, a_);
      if (!started && tt > 1.0) { fl.startHop((float)c.dist, 0.0f); started = true; }
      Command cmd = fl.update(g_, a_, 0.005f);
      if (started) {
        bool inBurn = fl.hop.state() == HState::BURN;
        if (inBurn && t_burn0 < 0) t_burn0 = tt;
        if (prevBurn && !inBurn) { t_burn1 = tt; pred_cut = fl.hop.predictedRange(); }
        prevBurn = inBurn;
      }
      if (tick++ % 4 == 0) latched = cmd;
      applyCmd(latched, pl, c.throttle_cap);
      if (c.trace && started && (k % 100 == 0)) printf("   t=%.2f st=%d th=%6.1f thd=%7.1f alpha=%5.1f cmdtilt=%5.1f u=%.2f T=%.1f est_tilt=%.1f z=%.3f vz=%.2f vx=%.2f\n", tt, (int)fl.hop.state(), pl.th*kRad2Deg, pl.thd*kRad2Deg, pl.alpha*kRad2Deg, cmd.tilt_x_deg, cmd.collective, pl.T, asinf(-fl.est.upInBody().x)*kRad2Deg, fl.est.pos.z, fl.est.vel.z, fl.est.vel.x);
    }
    pl.step(dtp);
    if (t_launch < 0 && pl.has_flown) t_launch = tt;
    if (started && (fl.hop.state() == HState::DONE || fl.hop.state() == HState::FAULT) && tt > 9.0) break;
  }
  HRes r;
  r.first_touch = pl.touch_xc / 0.3048; r.touch_vx = pl.touch_vx; r.range = pl.restX() / 0.3048; r.touch_vz = pl.touch_vz; r.st = (int)fl.hop.state(); r.flew = pl.has_flown;
  r.t_air = pl.touched ? pl.touch_t - t_launch : -1; r.peak_th = std::max(fabs(pl.max_th), fabs(pl.min_th)) * kRad2Deg;
  r.burn_s = (t_burn1 > 0) ? t_burn1 - t_burn0 : -1; r.maxburn = fl.hop.cut_reason_maxburn; r.pred_at_cut = pred_cut / 0.3048; r.apex_lean_max = 0;
  return r;
}

static void sweep7() {
  printf("kp_slow lead plan flare vt | IDEAL sensing (TWR{1.5,1.85} x lag{.15,.25,.40}) worst/mean | ERRORS scale+-3%% x vib{-.3,+.3} x height{-.5,0,+.5} worst/mean, time\n");
  for (double ks : {1.5, 2.0, 3.0}) for (double lead : {0.3, 0.45, 0.6}) for (double pf : {0.6, 0.75}) for (double flare : {0.3, 0.5}) for (double vt : {0.5, 0.8}) {
    auto tw = [&](Params& P) { P.vel_kp_slow = (float)ks; P.lead_time_s = (float)lead; P.brake_plan_fraction = (float)pf; P.h_flare_m = (float)flare; P.v_touch_mps = (float)vt; };
    double w1 = 0, s1 = 0; int n1 = 0;
    for (double twr : {1.5, 1.85}) for (double tau : {0.15, 0.25, 0.40}) { Cfg c; c.twr = twr; c.tau = tau; c.tweak = tw; DRes r = runDescent(c); double v = -r.vz; w1 = std::max(w1, v); s1 += v; n1++; }
    double w2 = 0, s2 = 0, ts = 0; int n2 = 0;
    for (double sc : {-0.03, 0.03}) for (double vib : {-0.3, 0.3}) for (double dh : {-0.5, 0.0, 0.5}) for (double twr : {1.5, 1.85}) {
      Cfg c; c.twr = twr; c.tau = 0.25; c.scale = sc; c.vib = vib; c.H_assumed = c.H_true + dh; c.tweak = tw; DRes r = runDescent(c); double v = -r.vz; w2 = std::max(w2, v); s2 += v; ts += r.t; n2++; }
    printf("%.1f  %.2f %.2f %.1f %.1f | %5.2f %5.2f | %5.2f %5.2f  %.1fs\n", ks, lead, pf, flare, vt, w1, s1 / n1, w2, s2 / n2, ts / n2);
  }
}
static void sweep6() {
  printf("Short-burn (robust) tuning. Errors: scale{-3%%,+3%%} x vib{-0.3,0,+0.3} x TWR{1.5,1.85} x lag{.15,.25} x height err{-.5,0,+.5} = 72 cases\n");
  for (double lead : {0.2, 0.3}) for (double flare : {0.1, 0.2, 0.3}) for (double vt : {0.8, 1.0, 1.3}) for (double pf : {0.85}) {
    double worst = 0, sum = 0, ts = 0; int n = 0, hard = 0;
    for (double sc : {-0.03, 0.03}) for (double vib : {-0.3, 0.0, 0.3}) for (double twr : {1.5, 1.85}) for (double tau : {0.15, 0.25}) for (double dh : {-0.5, 0.0, 0.5}) {
      Cfg c; c.twr = twr; c.tau = tau; c.scale = sc; c.vib = vib; c.H_assumed = c.H_true + dh;
      c.tweak = [&](Params& P) { P.lead_time_s = (float)lead; P.h_flare_m = (float)flare; P.v_touch_mps = (float)vt; P.brake_plan_fraction = (float)pf; P.max_powered_s = 4.0f; };
      DRes r = runDescent(c); double v = -r.vz; n++; sum += v; ts += r.t; worst = std::max(worst, v); if (v > 4.0) hard++;
    }
    printf("lead %.2f flare %.1f vt %.1f plan %.2f | worst %5.2f mean %.2f time %.1f s  >4m/s: %d/%d\n", lead, flare, vt, pf, worst, sum / n, ts / n, hard, n);
  }
}
static void sweep5() {
  printf("Vibration-offset tolerance (offset that exists only while rotors spin, prop. to thrust; negative = IMU under-reads). TWR{1.5,1.85} x lag{.15,.25} x height err{-.5,+.5}\n");
  struct V { double lead, flare, vt, pf, mp; };
  for (V v : {V{0.50, 0.6, 0.4, 0.70, 15}, V{0.30, 0.4, 0.5, 0.70, 7}, V{0.25, 0.3, 0.6, 0.85, 6}, V{0.25, 0.3, 0.8, 0.85, 5}}) {
    printf("lead %.2f flare %.1f vt %.1f plan %.2f maxT %.0f:\n", v.lead, v.flare, v.vt, v.pf, v.mp);
    for (double vib : {-0.3, -0.2, -0.1, -0.05, 0.0, 0.1, 0.2, 0.3}) {
      double worst = 0, sum = 0, ts = 0; int n = 0;
      for (double twr : {1.5, 1.85}) for (double tau : {0.15, 0.25}) for (double dh : {-0.5, 0.5}) {
        Cfg c; c.twr = twr; c.tau = tau; c.vib = vib; c.H_assumed = c.H_true + dh;
        c.tweak = [&](Params& P) { P.lead_time_s = (float)v.lead; P.h_flare_m = (float)v.flare; P.v_touch_mps = (float)v.vt; P.brake_plan_fraction = (float)v.pf; P.max_powered_s = (float)v.mp; };
        DRes r = runDescent(c); double s = -r.vz; n++; sum += s; ts += r.t; worst = std::max(worst, s);
      }
      printf("   vib %+.2f m/s^2 | worst %5.2f  mean %.2f m/s  time %.1f s\n", vib, worst, sum / n, ts / n);
    }
  }
}
static void sweep4() {
  printf("Robust-tuning sweep. Errors: accel scale {-3%%,+3%%} x vibration offset {-0.3,0,+0.3} ; TWR{1.5,1.85} x lag{.15,.25} x height err{-.5,+.5} (48 cases)\n");
  for (double lead : {0.2, 0.3, 0.4}) for (double flare : {0.2, 0.4}) for (double vt : {0.5, 0.8}) for (double pf : {0.7, 0.85}) for (double mp : {5.0, 7.0}) {
    double worst = 0, sum = 0, ts = 0; int n = 0, hard = 0;
    for (double sc : {-0.03, 0.03}) for (double vib : {-0.3, 0.0, 0.3}) for (double twr : {1.5, 1.85}) for (double tau : {0.15, 0.25}) for (double dh : {-0.5, 0.5}) {
      Cfg c; c.twr = twr; c.tau = tau; c.scale = sc; c.vib = vib; c.H_assumed = c.H_true + dh;
      c.tweak = [&](Params& P) { P.lead_time_s = (float)lead; P.h_flare_m = (float)flare; P.v_touch_mps = (float)vt; P.brake_plan_fraction = (float)pf; P.max_powered_s = (float)mp; };
      DRes r = runDescent(c); double v = -r.vz; n++; sum += v; ts += r.t; worst = std::max(worst, v); if (v > 4.0) hard++;
    }
    printf("lead %.1f flare %.1f vt %.1f plan %.2f maxT %.0f | worst %5.2f mean %.2f time %.1f  >4m/s: %d/%d\n", lead, flare, vt, pf, mp, worst, sum / n, ts / n, hard, n);
  }
}
static void sweep3() {
  printf("Residual error study (default controller). accel scale error and vibration offset (only while rotors spin, prop. to thrust)\n");
  for (double sc : {-0.03, 0.0, 0.03}) for (double vib : {0.0, 0.3, 0.6, -0.3, -0.6}) {
    double worst = 0, sum = 0; int n = 0;
    for (double twr : {1.5, 1.85}) for (double tau : {0.15, 0.25, 0.40}) for (double dh : {-0.5, 0.5}) {
      Cfg c; c.twr = twr; c.tau = tau; c.scale = sc; c.vib = vib; c.H_assumed = c.H_true + dh;
      DRes r = runDescent(c); double v = -r.vz; n++; sum += v; worst = std::max(worst, v);
    }
    printf("scale %+.2f vib %+.1f m/s^2 (TWR>=1.5) | worst %.2f mean %.2f m/s\n", sc, vib, worst, sum / n);
  }
}
static void sweep2() {
  printf("Uncorrectable accel drift study. worst/mean touchdown speed [m/s] and mean time [s] over TWR{1.3,1.85} x lag{.15,.40} x drift{-0.6,-0.3,0,+0.3,+0.6} x height err{-0.5,+0.5}\n");
  for (double lead : {0.25, 0.35, 0.50}) for (double flare : {0.3, 0.6}) for (double vt : {0.3, 0.6}) {
    double worst = 0, sum = 0, ts = 0; int n = 0, hard = 0;
    for (double twr : {1.3, 1.85}) for (double tau : {0.15, 0.40}) for (double dr : {-0.6, -0.3, 0.0, 0.3, 0.6}) for (double dh : {-0.5, 0.5}) {
      Cfg c; c.twr = twr; c.tau = tau; c.drift_z = dr; c.H_assumed = c.H_true + dh;
      c.tweak = [&](Params& P) { P.lead_time_s = (float)lead; P.h_flare_m = (float)flare; P.v_touch_mps = (float)vt; };
      DRes r = runDescent(c); double v = -r.vz; n++; sum += v; ts += r.t; worst = std::max(worst, v); if (v > 2.0) hard++;
    }
    printf("lead %.2f flare %.1f vtouch %.1f | worst %.2f  mean %.2f  mean time %.1f s  cases>2 m/s: %d/%d\n", lead, flare, vt, worst, sum / n, ts / n, hard, n);
  }
}
static void sweep() {
  printf("kp  fb  lead flare vtouch | worst touchdown speed over TWR{1.3,1.5,1.85} x lag{.15,.25,.40} x height err{-.5,0,+.5} ; #timeouts/faults\n");
  for (double kp : {0.6, 1.0, 1.5}) for (double fb : {0.3, 0.5, 1.0}) for (double lead : {0.30, 0.40, 0.50}) {
    double worst = 0; int bad = 0, n = 0; double sum = 0;
    for (double twr : {1.3, 1.5, 1.85}) for (double tau : {0.15, 0.25, 0.40}) for (double dh : {-0.5, 0.0, 0.5}) {
      Cfg c; c.twr = twr; c.tau = tau; c.H_assumed = c.H_true + dh;
      c.tweak = [&](Params& P) { P.vel_kp = (float)kp; P.acc_fb_gain = (float)fb; P.lead_time_s = (float)lead; };
      DRes r = runDescent(c); double v = -r.vz; n++;
      if (r.t < 0.01 || v < 0) { bad++; }
      worst = std::max(worst, v); sum += v;
    }
    printf("%.1f %.1f %.2f | worst %.2f  mean %.2f  bad %d/%d\n", kp, fb, lead, worst, sum / n, bad, n);
  }
}

int main(int argc, char** argv) {
  if (argc > 1 && std::string(argv[1]) == "sweep") { sweep(); return 0; }
  if (argc > 1 && std::string(argv[1]) == "sweep7") { sweep7(); return 0; }
  if (argc > 1 && std::string(argv[1]) == "sweep6") { sweep6(); return 0; }
  if (argc > 1 && std::string(argv[1]) == "sweep5") { sweep5(); return 0; }
  if (argc > 1 && std::string(argv[1]) == "sweep4") { sweep4(); return 0; }
  if (argc > 1 && std::string(argv[1]) == "sweep3") { sweep3(); return 0; }
  if (argc > 1 && std::string(argv[1]) == "sweep2") { sweep2(); return 0; }
  printf("=== 0. Mixer sanity ===\n");
  { Params P; Command c; c.motors_on = true; c.collective = 0.5f; c.tilt_x_deg = 10; c.tilt_y_deg = -40; c.yaw = 0.02f;
    Outputs o = mix(c, P, 0.3f);
    printf("collective 0.5 capped at 0.3 -> ESC us %u/%u (expect 1300/1300 +-yaw), servo x %u y %u (y clamped at tilt limit)\n", o.esc_upper_us, o.esc_lower_us, o.servo_x_us, o.servo_y_us); }

  printf("\n=== 1. Attitude loop sign check (hang upright under power, start tilted 15 deg, planar) ===\n");
  { Plant pl; pl.ground = false; pl.has_flown = false; pl.z = 10; pl.th = 15 * kDeg2Rad; pl.Tmax = 1.3 * pl.m * pl.g;
    Params P; P.thrust_max_N = (float)pl.Tmax; Noise nz; Flight fl(P);
    Vec3 g_, a_; double sx = 0, sz = 0; for (int i = 0; i < 200; i++) { sense(pl, nz, true, g_, a_); sx += a_.x; sz += a_.z; }
    fl.est.initAtRest(Vec3{(float)(sx / 200), 0, (float)(sz / 200)}); fl.est.setGyroBias(Vec3{0, nz.gyro_bias_y, 0});
    Command latched; int tick = 0; pl.cmd_u = 0.77;
    for (int k = 0; k < 8000; k++) {
      if (k % 10 == 0) { // feed specific force of a body being held up at ~1 g: emulate with plant accel (falls slowly) fine for attitude test
        sense(pl, nz, false, g_, a_);
        Command c; c.motors_on = true; c.collective = 0.77f; c.use_att = true; fl.est.update(g_, a_, 0.005f); fl.att.update(fl.est, c, 0.005f);
        if (tick++ % 4 == 0) latched = c;
        pl.cmd_alpha = latched.tilt_x_deg * kDeg2Rad; }
      pl.step(0.0005);
      if (k % 1600 == 0) printf("  t=%.1fs  body tilt %.1f deg  pod %.1f deg\n", k * 0.0005, pl.th * kRad2Deg, pl.alpha * kRad2Deg);
    } }

  for (int gentle = 0; gentle < 2; gentle++) {
    auto tw = [gentle](Params& P) { applyDescentPreset(P, gentle != 0); };
    printf("\n=== 2%c. Controlled descent from a 6.1 m release, IMU only, preset %s ===\n", gentle ? 'b' : 'a', gentle ? "GENTLE" : "ROBUST (default)");
    printf("touchdown vertical speed [m/s]   (uncontrolled free-fall impact would be ~10.9 m/s; time = release to touchdown)\n");
    printf("  ideal sensing, exact release height:\n");
    for (double twr : {1.30, 1.50, 1.85}) { printf("    TWR %.2f: ", twr); for (double tau : {0.15, 0.25, 0.40}) { Cfg c; c.twr = twr; c.tau = tau; c.tweak = tw; DRes r = runDescent(c); printf("lag %.2f -> %.2f (%.1fs)   ", tau, -r.vz, r.t); } printf("\n"); }
    printf("  TWR 1.5, lag 0.25, release height error (assumed - true):\n    ");
    for (double dh : {-1.0, -0.5, 0.0, 0.5, 1.0, 1.5}) { Cfg c; c.twr = 1.5; c.H_assumed = c.H_true + dh; c.tweak = tw; DRes r = runDescent(c); printf("%+.1f m -> %.2f   ", dh, -r.vz); } printf("\n");
    printf("  TWR 1.5, lag 0.25, controller's belief about thrust (x real):\n    ");
    for (double e : {0.85, 1.0, 1.15}) { Cfg c; c.twr = 1.5; c.ctrl_twr_err = e; c.tweak = tw; DRes r = runDescent(c); printf("x%.2f -> %.2f   ", e, -r.vz); } printf("\n");
    printf("  TWR 1.5, lag 0.25, accelerometer scale error / offset that appeared after the rest reading:\n    ");
    for (double sc : {-0.03, 0.03}) { Cfg c; c.twr = 1.5; c.scale = sc; c.tweak = tw; DRes r = runDescent(c); printf("scale %+.0f%% -> %.2f   ", sc * 100, -r.vz); }
    for (double dr : {-0.3, 0.3}) { Cfg c; c.twr = 1.5; c.drift_z = dr; c.tweak = tw; DRes r = runDescent(c); printf("offset %+.1f -> %.2f   ", dr, -r.vz); } printf("\n");
    printf("  TWR 1.5, lag 0.25, vibration offset that exists only while rotors run (m/s^2, negative = IMU under-reads):\n    ");
    for (double vb : {-0.3, -0.1, -0.05, -0.02, 0.0, 0.05, 0.3}) { Cfg c; c.twr = 1.5; c.vib = vb; c.tweak = tw; DRes r = runDescent(c); printf("%+.2f -> %.2f   ", vb, -r.vz); } printf("\n");
  }
  { Cfg c; c.twr = 1.05; DRes r = runDescent(c); printf("\n  TWR 1.05 (cannot brake): %.2f m/s, controller state %d (4 = FAULT, refuses to try)\n", -r.vz, r.st); }

  if (argc > 1 && std::string(argv[1]) == "hoptrace") { Cfg c; c.twr = 1.5; c.trace = true; printf("hop trace\n"); runHop(c, true); return 0; }
  printf("\n=== 3. Hop, target 5 ft (1.524 m) forward, planar sim; 'first touch' = where the base first hits the ground (what the controller aims at) ===\n");
  printf("  (roll-out after touchdown depends on ground friction / shell, which is NOT modelled credibly; 'rest' shown for information only)\n");
  for (double twr : {1.30, 1.50, 1.85}) {
    Cfg c; c.twr = twr;
    HRes a = runHop(c, true), o = runHop(c, false);
    printf("  TWR %.2f attitude-hold: first touch %.2f ft, horiz speed at touch %.1f m/s, burn %.2f s, airborne %.2f s, peak pitch %.0f deg, flew %d, final state %d | rest %.1f ft\n",
           twr, a.first_touch, a.touch_vx, a.burn_s, a.t_air, a.peak_th, (int)a.flew, a.st, a.range);
    printf("           open-loop pod 25 deg : first touch %.2f ft, peak body pitch %.0f deg, flew %d, final state %d\n", o.first_touch, o.peak_th, (int)o.flew, o.st);
  }
  { Cfg c; c.twr = 1.5; HRes a = runHop(c, true); printf("  TWR 1.50: predicted landing at the moment of cut-off = %.2f ft, actual first touch = %.2f ft (bias %.2f ft)\n", a.pred_at_cut, a.first_touch, a.first_touch - a.pred_at_cut); }
  printf("  -- effect of hop_range_scale (TWR 1.50 / 1.85), first-touch distance in ft --\n   ");
  for (double rs : {1.0, 1.05, 1.10, 1.15}) { Cfg c; c.twr = 1.5; c.tweak = [rs](Params& P) { P.hop_range_scale = (float)rs; }; HRes a = runHop(c, true); Cfg d = c; d.twr = 1.85; HRes b = runHop(d, true); printf("scale %.2f: %.2f / %.2f   ", rs, a.first_touch, b.first_touch); } printf("\n");
  printf("  -- attitude-hold hop sensitivity (TWR 1.50), first-touch distance in ft --\n   ");
  for (double tau : {0.15, 0.25, 0.40}) { Cfg c; c.twr = 1.5; c.tau = tau; HRes a = runHop(c, true); printf("lag %.2f: %.2f   ", tau, a.first_touch); } printf("\n   ");
  for (double I : {0.03, 0.06}) { Cfg c; c.twr = 1.5; c.I = I; HRes a = runHop(c, true); printf("inertia %.2f: %.2f   ", I, a.first_touch); } printf("\n   ");
  for (double e : {0.85, 1.15}) { Cfg c; c.twr = 1.5; c.ctrl_twr_err = e; HRes a = runHop(c, true); printf("thrust belief x%.2f: %.2f   ", e, a.first_touch); } printf("\n   ");
  for (double sc : {-0.03, 0.03}) { Cfg c; c.twr = 1.5; c.scale = sc; HRes a = runHop(c, true); printf("accel scale %+.0f%%: %.2f   ", sc * 100, a.first_touch); } printf("\n   ");
  for (double ln : {20.0, 30.0}) { Cfg c; c.twr = 1.5; c.lean = ln; HRes a = runHop(c, true); printf("lean %.0f deg: %.2f (state %d)   ", ln, a.first_touch, a.st); } printf("\n   ");
  for (int s = 1; s <= 6; s++) { Cfg c; c.twr = 1.5; c.seed = s; HRes a = runHop(c, true); printf("seed %d: %.2f   ", s, a.first_touch); } printf("\n   ");
  { Cfg c; c.twr = 1.5; c.brake = true; HRes a = runHop(c, true); printf("landing brake ON: first touch %.2f ft, touchdown vz %.2f m/s, state %d\n", a.first_touch, -a.touch_vz, a.st); }
  return 0;
}
