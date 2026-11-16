"""
run_tests.py  --  fly every routine under realistic "the real drone is not exactly the model" conditions.

    py run_tests.py            all routines, all conditions (takes a minute or two)
    py run_tests.py hop        one routine

Each row changes ONE thing about the simulated drone that the controller does not know about.
Pass criteria (edit below): descent touchdown <= 1.0 m/s, hop rests within 0.5 m of the target,
ascent apex within 0.10 m of the target, soft touchdowns, and the drone ends upright.
"""
import sys, time
from weeble_sim import WeebleSim, INFO

# The CAD as measured is NOT self-righting (CG 0.7 mm above the base's centre of curvature), so the
# conditions below are flown with the CG moved down by DESIGN_CG_DZ_MM to stand in for the hardware fix.
# Set it to 0 once drone_config.py has a self-righting CG.
DESIGN_CG_DZ_MM = -20.0 if INFO["margin"] <= 0.005 else 0.0

CONDITIONS = [
    ("nominal", {}),
    ("battery sag: thrust -15%", dict(thrust_scale=0.85)),
    ("motors faster (lag 0.04 s)", dict(lag_up=0.04, lag_down=0.04)),
    ("motors slower (lag 0.25 s)", dict(lag_up=0.25, lag_down=0.25)),
    ("IMU worse (2x noise, drift)", dict(imu_noise=2.0, accel_bias=0.05, gyro_bias=0.005)),
    ("CG 3 mm off-axis", dict(cg_dx_mm=3.0)),
    ("servo half speed", dict(servo_speed=300.0, servo_lag=0.06)),
]
LIMITS = dict(descent_vz=1.0, hop_rest=0.5, ascent_apex=0.10, soft_vz=0.5, upright_deg=15.0)


def judge(routine, r):
    f = r["first"]
    if f is None:
        return False, "no touchdown"
    up = r["final_tilt"] < LIMITS["upright_deg"]
    if routine == "descent":
        ok = abs(f["vz"]) <= LIMITS["descent_vz"] and up
        return ok, f"touchdown {f['vz']:+.2f} m/s, final tilt {r['final_tilt']:4.1f}"
    if routine == "hop":
        ok = r["rest_error"] <= LIMITS["hop_rest"] and abs(f["vz"]) <= LIMITS["soft_vz"] and up
        return ok, (f"rest {r['rest_error'] * 100:3.0f} cm from target, touchdown {f['vz']:+.2f}/{f['vh']:.2f} m/s, "
                    f"final tilt {r['final_tilt']:4.1f}")
    ok = abs(r["apex"] - r["value"]) <= LIMITS["ascent_apex"] and abs(f["vz"]) <= LIMITS["soft_vz"] and up
    return ok, f"apex {r['apex']:.2f} m (target {r['value']:.2f}), touchdown {f['vz']:+.2f} m/s, final tilt {r['final_tilt']:4.1f}"


def main():
    routines = sys.argv[1:] or ["descent", "hop", "ascent"]
    print(f"mass {INFO['mass'] * 1000:.0f} g | T/W {INFO['twr']:.2f} | self-righting margin {INFO['margin'] * 1000:+.1f} mm")
    if DESIGN_CG_DZ_MM:
        print(f"NOTE: config CG is not self-righting; tests fly with the CG {abs(DESIGN_CG_DZ_MM):.0f} mm lower (design fix stand-in).")
    t0 = time.time(); n_pass = n = 0
    for routine in routines:
        print(f"\n== {routine}")
        rows = [("CAD as-is (no CG fix)", dict())] + [(name, dict(cond, cg_dz_mm=DESIGN_CG_DZ_MM)) for name, cond in CONDITIONS]
        for name, plant in rows:
            try:
                r = WeebleSim(routine, plant=plant, quiet=True).run(16)
                ok, txt = judge(routine, r)
                if r["end_reason"] and not ok:
                    txt += f"  [{r['end_reason']}]"
            except SystemExit as e:
                ok, txt = False, f"did not start: {e}"
            if name.startswith("CAD") and DESIGN_CG_DZ_MM:
                print(f"  {'info':4s}  {name:30s} {txt}")
                continue
            n += 1; n_pass += ok
            print(f"  {'PASS' if ok else 'FAIL':4s}  {name:30s} {txt}")
    print(f"\n{n_pass}/{n} passed in {time.time() - t0:.0f} s")


if __name__ == "__main__":
    main()
