"""
weeble_sim.py  --  run the real controller (include/weeble_core.h) against the MuJoCo model.

    py weeble_sim.py hop                 5 ft hop (distance from drone_config.py)
    py weeble_sim.py hop --dist 4 --bearing 90   4 ft hop toward the drone's right
    py weeble_sim.py ascent --height 0.8
    py weeble_sim.py descent
    add --cg-dz -20   to try the routine with the CG 20 mm lower (the CAD CG is not self-righting yet)
    add --no-viewer   to run without a window (prints the log + a report)
    add --record 10   to save the first 10 s to weeble_<routine>.mp4 (no window)

What is simulated besides the rigid-body physics:
    * motor thrust lag (first order), ESC pulses only updated at ESC_PWM_HZ
    * servo speed limit + lag, servo pulses only updated at SERVO_PWM_HZ
    * IMU: the chip's low-pass filter, noise, residual gyro bias, range clipping and 16-bit steps
    * the real base shape: an exact R100 sphere until the drone tilts off the spherical face, then the side band
Everything comes from drone_config.py. If you change the config, run  py build.py  first.
"""
import argparse, ctypes, json, math, os, sys, time

import numpy as np
import mujoco

HERE = os.path.dirname(os.path.abspath(__file__))
SIM_DIR = os.path.join(HERE, "sim")
sys.path.insert(0, HERE)
import build  # noqa: E402  (config loader + hash)

cfg = build.load_config()
MACRO = ["Rest", "Armed", "Await Release", "Launch", "Amp Up", "Ascending", "Descending", "Landing", "Amp Down",
         "Fault", "Translating", "Hovering"]
ROUTINE = {"descent": 0, "hop": 1, "ascent": 2}
G = 9.80665


def load_lib():
    info_path = os.path.join(SIM_DIR, "model_info.json")
    name = "weeble.dll" if os.name == "nt" else "libweeble.so"
    path = os.path.join(SIM_DIR, name)
    if not (os.path.exists(info_path) and os.path.exists(path)):
        raise SystemExit("Simulator files missing. Run:  py build.py")
    try:
        lib = ctypes.CDLL(path)
    except OSError as e:
        raise SystemExit(f"Could not load {name}: {e}\n(64-bit Python needs a 64-bit compiler; rerun build.py after fixing.)")
    f, i, p, s = ctypes.c_float, ctypes.c_int, ctypes.POINTER(ctypes.c_float), ctypes.c_char_p
    lib.sim_config_hash.restype = ctypes.c_uint32
    lib.sim_set.argtypes = [s, f]; lib.sim_set.restype = i
    lib.sim_calibrate.argtypes = [f] * 6
    lib.sim_start.argtypes = [i, f, f]; lib.sim_start.restype = i
    lib.sim_step.argtypes = [f] * 7 + [p]
    lib.sim_telemetry.argtypes = [p]
    for fn in (lib.sim_substate, lib.sim_routine, lib.sim_message):
        fn.restype = s
    info = json.load(open(info_path))
    h_now = build.config_hash(cfg)
    if lib.sim_config_hash() != h_now or info["config_hash"] != h_now:
        raise SystemExit("drone_config.py changed since the last build. Run:  py build.py")
    return lib, info


LIB, INFO = load_lib()


class WeebleSim:
    """One simulated flight. plant = differences between the REAL drone and what the controller believes
    (used by run_tests.py); params = controller parameter overrides."""

    PLANT_DEFAULTS = dict(
        thrust_scale=1.0,                         # real thrust / believed thrust (battery sag, wrong THRUST_MAX_N)
        lag_up=cfg.MOTOR_LAG_UP_S, lag_down=cfg.MOTOR_LAG_DOWN_S,
        servo_lag=cfg.SERVO_LAG_S, servo_speed=cfg.SERVO_SPEED_DPS,
        esc_hz=cfg.ESC_PWM_HZ, servo_hz=cfg.SERVO_PWM_HZ,
        imu_noise=1.0,                            # multiplier on the config noise levels
        gyro_bias=cfg.IMU_GYRO_BIAS_LEFT,         # rad/s left after calibration
        accel_bias=0.0,                           # m/s^2 offset that appears after calibration
        cg_dx_mm=0.0,                             # real CG shifted sideways from the CAD value
        cg_dz_mm=0.0,                             # real CG shifted up (+) / down (-) from the CAD value
        hold_s=0.6,                               # descent: how long it hangs before release
    )

    def __init__(self, routine, value=None, bearing=0.0, plant=None, params=None, seed=0, quiet=False):
        self.routine = routine
        self.pl = dict(self.PLANT_DEFAULTS, **(plant or {}))
        self.quiet = quiet
        self.rng = np.random.default_rng(seed)
        m = self.model = mujoco.MjModel.from_xml_path(os.path.join(SIM_DIR, "WeebleDrone.xml"))
        d = self.data = mujoco.MjData(m)
        self.dt = m.opt.timestep
        self.bid = m.body("weeble").id
        self.pid = m.body("pod").id
        self.base_gid = m.geom("base").id
        self.theta_cap = math.radians(INFO["theta_cap_deg"])
        self.z_rest = INFO["z_rest"]
        # optional CG shift of the shell (tests / "what if" studies). The model is rebuilt so rest height stays right.
        k = m.body_subtreemass[self.bid] / m.body_mass[self.bid]
        m.body_ipos[self.bid][0] += self.pl["cg_dx_mm"] * 1e-3 * k
        m.body_ipos[self.bid][2] += self.pl["cg_dz_mm"] * 1e-3 * k

        # mission arguments
        if routine == "descent":
            self.value = cfg.DESCENT_RELEASE_HEIGHT_M if value is None else value
            d.qpos[2] = self.z_rest + self.value
        elif routine == "hop":
            self.value = cfg.HOP_DISTANCE_M if value is None else value
            d.qpos[2] = self.z_rest
        else:
            self.value = cfg.ASCENT_HEIGHT_M if value is None else value
            d.qpos[2] = self.z_rest
        self.bearing = bearing
        th = -math.radians(bearing)
        self.target = np.array([math.cos(th), math.sin(th)]) * (self.value if routine == "hop" else 0.0)
        for name in ("target", "target_tol"):
            sid = m.site(name).id
            m.site_pos[sid][0:2] = self.target
            if routine != "hop":
                m.site_rgba[sid][3] = 0.0
        mujoco.mj_forward(m, d)

        # controller
        LIB.sim_reset()
        for k, v in (params or {}).items():
            if not LIB.sim_set(k.encode(), float(v)):
                raise SystemExit(f"unknown controller parameter '{k}'")
        self.out = (ctypes.c_float * 8)()
        self.tel = (ctypes.c_float * 24)()

        # sensor model state
        self.R_bi = np.array(INFO["R_bi"])
        gb = self.rng.normal(size=3); self.gyro_bias = gb / np.linalg.norm(gb) * self.pl["gyro_bias"]
        ab = self.rng.normal(size=3); self.accel_bias_after = ab / np.linalg.norm(ab) * self.pl["accel_bias"]
        self.k_imu = 1.0 - math.exp(-2 * math.pi * cfg.IMU_DLPF_HZ * self.dt)
        self.g_f = np.zeros(3)
        self.a_f = self.R_bi.T @ np.array([0.0, 0.0, G])
        self.a_lsb = G / (16384.0 / 2 ** build.AFS[cfg.IMU_ACCEL_RANGE_G])
        self.g_lsb = math.radians(1.0 / (131.0 / 2 ** build.GFS[cfg.IMU_GYRO_RANGE_DPS]))
        self.a_max = cfg.IMU_ACCEL_RANGE_G * G
        self.g_max = math.radians(cfg.IMU_GYRO_RANGE_DPS)

        # actuator state
        self.T = 0.0; self.T_latched = 0.0
        self.srv_cmd = np.zeros(2); self.srv = np.zeros(2)
        self.esc_div = max(1, int(round(1.0 / (self.pl["esc_hz"] * self.dt))))
        self.srv_div = max(1, int(round(1.0 / (self.pl["servo_hz"] * self.dt))))
        self.n = 0

        # bookkeeping
        self.events = []; self.touchdowns = []; self.air_t = 0.0 if routine == "descent" else None
        self.prev_v = np.zeros(3); self.apex = 0.0; self.max_tilt_air = 0.0
        self.t_release = None; self.started = False; self.done_t = None
        self.log = []

        self._calibrate_and_start()

    # --------------------------------------------------------------------------------- sensors
    def _imu_filter(self):
        """Runs every physics step: the chip's low-pass filter on the true signals (chip axes)."""
        d = self.data
        if self.holding():
            g_true, a_true = np.zeros(3), self.R_bi.T @ self._held_accel_body()
        else:
            g_true, a_true = d.sensor("imu_gyro").data, d.sensor("imu_accel").data
        self.g_f += (g_true - self.g_f) * self.k_imu
        self.a_f += (a_true - self.a_f) * self.k_imu

    def _held_accel_body(self):
        R = self.data.xmat[self.bid].reshape(3, 3)
        return R.T @ np.array([0.0, 0.0, G])

    def _imu_sample(self, after_cal=True):
        """What the firmware would read at a control tick: filtered + noise + bias, clipped, quantised."""
        s = self.pl["imu_noise"]
        g = self.g_f + self.gyro_bias + self.rng.normal(0, cfg.IMU_GYRO_NOISE * s, 3)
        a = self.a_f + self.rng.normal(0, cfg.IMU_ACCEL_NOISE * s, 3)
        if after_cal:
            a = a + self.accel_bias_after
        g = np.round(np.clip(g, -self.g_max, self.g_max) / self.g_lsb) * self.g_lsb
        a = np.round(np.clip(a, -self.a_max, self.a_max) / self.a_lsb) * self.a_lsb
        return g, a

    # --------------------------------------------------------------------------------- setup
    def holding(self):
        return self.routine == "descent" and self.t_release is None

    def _support(self, on):
        """Descent: hold the drone like a rope at its CG (cancels gravity on each body)."""
        d, m = self.data, self.model
        for b in (self.bid, self.pid):
            d.xfrc_applied[b][:] = 0.0
            if on:
                d.xfrc_applied[b][2] = m.body_mass[b] * G

    def _calibrate_and_start(self):
        """Like typing 'cal' then the routine on the real drone: average 0.3 s of still IMU data first."""
        # Set it down and wait until it is completely still (a "hand" damps the rocking, it does not hold it,
        # so the drone ends up at its own resting tilt). MuJoCo's soft rolling friction alone barely damps rocking,
        # and calibrating a rocking drone bakes the rocking into the gyro bias.
        self.steady = True
        still = 0
        for i in range(8000):
            self._physics_step(controller=False)
            still = still + 1 if (np.linalg.norm(self.data.qvel[3:6]) < 0.002 and np.linalg.norm(self.data.qvel[0:3]) < 0.001) else 0
            if i > 300 and still > 300:
                break
        self.steady = False
        for i in range(200):
            self._physics_step(controller=False)
        acc_g, acc_a, k = np.zeros(3), np.zeros(3), 0
        for i in range(300):
            self._physics_step(controller=False)
            if i % 5 == 0:
                g, a = self._imu_sample(after_cal=False)
                acc_g += g; acc_a += a; k += 1
        LIB.sim_calibrate(*(acc_g / k), *(acc_a / k))
        args = {"descent": (self.value, 0.0), "hop": (self.value, self.bearing), "ascent": (self.value, 0.0)}[self.routine]
        ok = LIB.sim_start(ROUTINE[self.routine], *args)
        self.t0 = self.data.time
        self._drain_messages()
        if not ok:
            raise SystemExit(f"controller refused to start {self.routine}: {self.events[-1][1] if self.events else '?'}")
        self.started = True

    # --------------------------------------------------------------------------------- stepping
    def _physics_step(self, controller=True):
        m, d, o, pl = self.model, self.data, self.out, self.pl
        if self.routine == "descent":
            if self.t_release is None and self.started and d.time - self.t0 >= pl["hold_s"]:
                self.t_release = d.time
            self._support(self.t_release is None)

        if getattr(self, "steady", False) and self.routine != "descent":
            R0 = d.xmat[self.bid].reshape(3, 3)
            d.xfrc_applied[self.bid][3:6] = -0.05 * (R0 @ d.qvel[3:6])
            d.xfrc_applied[self.bid][0:3] = -2.0 * d.qvel[0:3]
        elif self.routine != "descent":
            d.xfrc_applied[self.bid][:] = 0.0

        # base sphere contact only while the drone sits on its spherical face
        R = d.xmat[self.bid]
        tilt = math.acos(max(-1.0, min(1.0, R[8])))
        on = 1 if tilt < self.theta_cap else 0
        m.geom_contype[self.base_gid] = on; m.geom_conaffinity[self.base_gid] = on

        self._imu_filter()
        if controller and self.n % 5 == 0:
            g, a = self._imu_sample()
            LIB.sim_step(*g, *a, 5 * self.dt, o)
            self._drain_messages()
        if not controller:
            for k in range(8):
                o[k] = 0.0

        if self.n % self.esc_div == 0:
            self.T_latched = o[0] * pl["thrust_scale"]
        if self.n % self.srv_div == 0:
            self.srv_cmd = np.array([o[1], o[2]])
        lag = pl["lag_up"] if self.T_latched > self.T else pl["lag_down"]
        self.T += (self.T_latched - self.T) * min(1.0, self.dt / max(lag, 1e-3))
        # servo: heads toward the command with time constant servo_lag, never faster than servo_speed
        rate = (self.srv_cmd - self.srv) / max(pl["servo_lag"], self.dt)
        self.srv += np.clip(rate, -pl["servo_speed"], pl["servo_speed"]) * self.dt
        d.ctrl[0] = math.radians(self.srv[0]); d.ctrl[1] = math.radians(self.srv[1])
        d.ctrl[2] = self.T
        d.ctrl[3] = o[3] * cfg.YAW_TORQUE_PER_DIFF if o[0] > 0 else 0.0

        self.prev_v = d.qvel[0:3].copy()
        mujoco.mj_step(m, d)
        self.n += 1
        if controller:
            self._track()

    def _drain_messages(self):
        while True:
            s = LIB.sim_message().decode()
            if not s:
                break
            self.events.append((self.data.time - getattr(self, "t0", self.data.time), s))
            if not self.quiet:
                print(f"   [{self.events[-1][0]:5.2f}s] {s}")

    def _track(self):
        d = self.data
        if self.holding():
            return
        tilt = self.tilt()
        contact = d.ncon > 0
        if not contact:
            self.air_t = (self.air_t or 0.0) + self.dt
            if self.air_t > 0.05:
                self.apex = max(self.apex, d.qpos[2] - self.z_rest)
                self.max_tilt_air = max(self.max_tilt_air, tilt)
        else:
            if self.air_t is not None and self.air_t > 0.08:
                td = dict(t=d.time - self.t0, x=d.qpos[0], y=d.qpos[1], vz=self.prev_v[2],
                          vh=float(np.hypot(*self.prev_v[0:2])), tilt=tilt)
                self.touchdowns.append(td)
                if not self.quiet:
                    print(f"   >>> TOUCHDOWN #{len(self.touchdowns)} t={td['t']:.2f}s at ({td['x']:+.2f}, {td['y']:+.2f}) m  "
                          f"vz={td['vz']:+.2f} m/s  sideways={td['vh']:.2f} m/s  tilt={td['tilt']:.1f} deg  (true)")
            self.air_t = 0.0
        if self.macro() not in ("Rest", "Armed"):
            self.mission_seen = True
        if self.done_t is None and getattr(self, "mission_seen", False) and self.macro() in ("Rest", "Fault"):
            self.done_t = d.time - self.t0

    def step(self):
        self._physics_step(controller=True)

    # --------------------------------------------------------------------------------- queries
    def tilt(self):
        return math.degrees(math.acos(max(-1.0, min(1.0, self.data.xmat[self.bid][8]))))

    def macro(self):
        return MACRO[int(self.out[4])]

    def telemetry(self):
        LIB.sim_telemetry(self.tel)
        return list(self.tel)

    def log_line(self):
        d, o = self.data, self.out
        return (f"t={d.time - self.t0:5.2f}s | {self.macro():<13s} {LIB.sim_substate().decode():<12s} | "
                f"pos=({d.qpos[0]:+.2f},{d.qpos[1]:+.2f}) h={d.qpos[2] - self.z_rest:+.2f}m | tilt={self.tilt():5.1f} | "
                f"T={self.T:5.2f}N | gimbal=({o[1]:+5.1f},{o[2]:+5.1f})")

    def run(self, seconds=12.0, settle_after_done=1.5, print_every=0.2):
        k = max(1, int(round(print_every / self.dt)))
        while self.data.time - self.t0 < seconds:
            self.step()
            if not self.quiet and self.n % k == 0:
                print(self.log_line())
            if self.done_t is not None and self.data.time - self.t0 > self.done_t + settle_after_done:
                break
        return self.report()

    def report(self):
        d = self.data
        r = dict(routine=self.routine, value=self.value, touchdowns=self.touchdowns, apex=self.apex,
                 max_tilt_air=self.max_tilt_air, final_xy=(float(d.qpos[0]), float(d.qpos[1])), final_tilt=self.tilt(),
                 events=self.events, ended=self.done_t is not None,
                 end_reason=next((e[1] for e in reversed(self.events) if e[1] not in MACRO and not e[1].startswith("mission")), ""))
        r["rest_error"] = float(np.hypot(*(np.array(r["final_xy"]) - self.target)))
        r["first"] = self.touchdowns[0] if self.touchdowns else None
        return r


def print_report(r):
    print("\n==================== RESULT (true values from MuJoCo) ====================")
    f = r["first"]
    if f:
        print(f"first touchdown      t={f['t']:.2f}s  vz={f['vz']:+.2f} m/s  sideways={f['vh']:.2f} m/s  tilt={f['tilt']:.1f} deg")
    else:
        print("no touchdown detected")
    if r["routine"] == "hop":
        print(f"came to rest at      ({r['final_xy'][0]:+.2f}, {r['final_xy'][1]:+.2f}) m  ->  {r['rest_error'] * 100:.0f} cm from target "
              f"({'within' if r['rest_error'] <= 0.5 else 'OUTSIDE'} +-0.5 m)")
    if r["routine"] == "ascent":
        print(f"apex (CG rise)       {r['apex']:.3f} m   (target {r['value']:.2f} m)")
    print(f"max tilt in the air  {r['max_tilt_air']:.1f} deg")
    print(f"final tilt           {r['final_tilt']:.1f} deg  {'(upright)' if r['final_tilt'] < 10 else '(NOT upright)'}")
    print(f"controller           {'finished' if r['ended'] else 'still running at end of sim'}")
    print("==========================================================================")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("routine", nargs="?", default="hop", choices=["hop", "ascent", "descent"])
    ap.add_argument("--dist", type=float, help="hop distance in FEET")
    ap.add_argument("--bearing", type=float, default=0.0, help="hop direction, degrees clockwise from body +x")
    ap.add_argument("--height", type=float, help="ascent height or descent release height, metres")
    ap.add_argument("--no-viewer", action="store_true")
    ap.add_argument("--record", type=float, default=0.0, help="record N seconds to MP4 (no window)")
    ap.add_argument("--seconds", type=float, default=12.0)
    ap.add_argument("--cg-dz", type=float, default=0.0,
                    help="what-if: move the real CG up (+) / down (-) by this many mm (e.g. -20 to try a self-righting CG)")
    a = ap.parse_args()
    value = a.dist * 0.3048 if (a.routine == "hop" and a.dist) else a.height
    print(f"=== Weeble {a.routine} | mass {INFO['mass'] * 1000:.0f} g, T/W {INFO['twr']:.2f}, "
          f"self-righting margin {INFO['margin'] * 1000:+.1f} mm ===")
    if INFO["margin"] <= 0 and a.cg_dz == 0:
        print("NOTE: this CG is not self-righting, so the drone may roll over before it can start.\n"
              "      Try  --cg-dz -20  to see the routine with a CG 20 mm lower.")
    sim = WeebleSim(a.routine, value=value, bearing=a.bearing, quiet=bool(a.record), plant=dict(cg_dz_mm=a.cg_dz))

    if a.record > 0:
        import imageio
        r = mujoco.Renderer(sim.model, 480, 640)
        cam = mujoco.MjvCamera(); cam.distance = 2.2; cam.elevation = -12; cam.azimuth = 90
        frames, every = [], int(round(0.02 / sim.dt))
        while sim.data.time - sim.t0 < a.record:
            sim.step()
            if sim.n % every == 0:
                cam.lookat[:] = sim.data.qpos[0:3]; r.update_scene(sim.data, cam); frames.append(r.render())
        fn = f"weeble_{a.routine}.mp4"; imageio.mimsave(fn, frames, fps=50); print("saved", fn)
        print_report(sim.report()); return

    if a.no_viewer:
        print_report(sim.run(a.seconds)); return

    from mujoco import viewer as mj_viewer   # (aliased: a plain "import mujoco.viewer" here would shadow mujoco)
    with mj_viewer.launch_passive(sim.model, sim.data) as v:
        v.cam.distance = 2.2; v.cam.elevation = -12
        k = int(round(0.2 / sim.dt)); reported = False
        while v.is_running():
            t_wall = time.time()
            sim.step()
            v.cam.lookat[:] = sim.data.qpos[0:3]
            if sim.n % 20 == 0:
                v.sync()
            if sim.n % k == 0:
                print(sim.log_line())
            if not reported and sim.done_t is not None and sim.data.time - sim.t0 > sim.done_t + 1.5:
                print_report(sim.report()); reported = True
            lag = sim.dt - (time.time() - t_wall)
            if lag > 0:
                time.sleep(lag)


if __name__ == "__main__":
    main()
