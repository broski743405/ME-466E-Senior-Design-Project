"""
build.py  --  turn drone_config.py into everything the simulator and firmware need.

    py build.py            regenerate model + header, compile the simulator library, print summary
    py build.py --no-compile   (skip the C++ compile step)

Outputs:
    sim/WeebleDrone.xml       MuJoCo model (body + tilting motor stack), built from the config
    sim/model_info.json       derived numbers the simulator script reads (rest height, etc.)
    include/weeble_config.h   the same physical numbers as C++ constants (used by the ESP32 firmware too)
    sim/weeble.dll            the controller compiled for the simulator (libweeble.so on Linux/Mac)

Every step of the mass-property math is written out below so it can be checked by hand.
"""
import hashlib, importlib, json, math, os, platform, shutil, struct, subprocess, sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
G = 9.80665


# --------------------------------------------------------------------------------------- config
def load_config():
    sys.path.insert(0, HERE)
    import drone_config
    importlib.reload(drone_config)
    return drone_config


def config_hash(cfg):
    """32-bit fingerprint of every UPPERCASE setting. The compiled library carries the same number,
    so the simulator can refuse to run with a library built from an older config."""
    items = {k: getattr(cfg, k) for k in sorted(dir(cfg)) if k.isupper()}
    return int(hashlib.sha256(repr(items).encode()).hexdigest()[:8], 16)


# --------------------------------------------------------------------------------------- helpers
def shift_tensor(m, r):
    """Parallel-axis term: inertia of point mass m at offset r about the reference point."""
    r = np.asarray(r, float)
    return m * (np.dot(r, r) * np.eye(3) - np.outer(r, r))


def cylinder_inertia(m, radius, length):
    ixx = m * (3 * radius ** 2 + length ** 2) / 12.0
    return np.diag([ixx, ixx, 0.5 * m * radius ** 2])


def parse_axes(axes):
    """("+y","-x","+z") -> rotation matrix R with v_body = R @ v_imu."""
    R = np.zeros((3, 3))
    for i, s in enumerate(axes):
        s = s.strip().lower()
        sign = -1.0 if s.startswith("-") else 1.0
        j = "xyz".index(s[-1])
        R[i, j] = sign
    if abs(np.linalg.det(R) - 1.0) > 1e-6:
        raise SystemExit(f"IMU_AXES {axes} is not a proper rotation (it mirrors the axes). Check the signs.")
    return R


def mat_to_quat(R):
    w = math.sqrt(max(0.0, 1 + R[0, 0] + R[1, 1] + R[2, 2])) / 2
    x = math.sqrt(max(0.0, 1 + R[0, 0] - R[1, 1] - R[2, 2])) / 2
    y = math.sqrt(max(0.0, 1 - R[0, 0] + R[1, 1] - R[2, 2])) / 2
    z = math.sqrt(max(0.0, 1 - R[0, 0] - R[1, 1] + R[2, 2])) / 2
    x = math.copysign(x, R[2, 1] - R[1, 2]); y = math.copysign(y, R[0, 2] - R[2, 0]); z = math.copysign(z, R[1, 0] - R[0, 1])
    n = math.sqrt(w * w + x * x + y * y + z * z)
    return [w / n, x / n, y / n, z / n]


def read_stl(path):
    """Minimal STL reader (binary or ASCII). Returns an (N,3) array of unique vertices in file units."""
    with open(path, "rb") as f:
        data = f.read()
    verts = []
    if data[:5].lower() == b"solid" and b"facet" in data[:400]:
        for line in data.decode(errors="ignore").splitlines():
            t = line.split()
            if len(t) == 4 and t[0] == "vertex":
                verts.append([float(t[1]), float(t[2]), float(t[3])])
    else:
        n = struct.unpack("<I", data[80:84])[0]
        for k in range(n):
            o = 84 + 50 * k
            v = struct.unpack("<12f", data[o:o + 48])
            verts += [v[3:6], v[6:9], v[9:12]]
    v = np.unique(np.round(np.asarray(verts, float), 2), axis=0)
    return v


# --------------------------------------------------------------------------------------- physics
def derive(cfg):
    """All the mass/geometry math. Frame 'B': origin at the BOTTOM point, z up, metres."""
    mm, gram = 1e-3, 1e-3
    O = cfg.CAD_ORIGIN_ABOVE_BOTTOM_MM

    def cad_to_B(p):
        return np.array([p[0], p[1], p[2] + O]) * mm

    # (a) the CAD assembly as measured
    m0 = cfg.CAD_MASS_G * gram
    c0 = cad_to_B(cfg.CAD_COM_MM)
    I = cfg.CAD_INERTIA_G_MM2
    I0 = np.array([[I["xx"], I["xy"], I["xz"]],
                   [I["xy"], I["yy"], I["yz"]],
                   [I["xz"], I["yz"], I["zz"]]]) * 1e-9          # g*mm^2 -> kg*m^2

    # (b) extra masses not yet in CAD
    extras = [(e["mass_g"] * gram, np.array(e["pos_mm"], float) * mm, e["on_pod"], e["name"])
              for e in cfg.EXTRA_MASSES if e.get("enabled")]

    # (c) whole drone = CAD + extras
    m = m0 + sum(e[0] for e in extras)
    c = (m0 * c0 + sum(e[0] * e[1] for e in extras)) / m
    I_tot = I0 + shift_tensor(m0, c0 - c) + sum((shift_tensor(e[0], e[1] - c) for e in extras), np.zeros((3, 3)))

    # (d) the tilting part (motor stack) + extras that ride on it
    pivot = np.array([0.0, 0.0, cfg.GIMBAL_PIVOT_ABOVE_BOTTOM_MM * mm])
    mp = cfg.POD_MASS_G * gram
    cp = pivot + np.array([0, 0, cfg.POD_COM_ABOVE_PIVOT_MM * mm])
    Ip = cylinder_inertia(mp, cfg.POD_RADIUS_MM * mm, cfg.POD_LENGTH_MM * mm)
    pod_parts = [(mp, cp, Ip)] + [(e[0], e[1], np.zeros((3, 3))) for e in extras if e[2]]
    mP = sum(p[0] for p in pod_parts)
    cP = sum(p[0] * p[1] for p in pod_parts) / mP
    IP = sum((p[2] + shift_tensor(p[0], p[1] - cP) for p in pod_parts), np.zeros((3, 3)))

    # (e) the shell/body = whole drone minus the tilting part
    mB = m - mP
    cB = (m * c - mP * cP) / mB
    IB = I_tot - (IP + shift_tensor(mP, cP - c)) - shift_tensor(mB, cB - c)
    if np.min(np.linalg.eigvalsh(IB)) <= 0:
        raise SystemExit("Pod placeholder is inconsistent with the CAD inertia (shell inertia came out negative). "
                         "Lower POD_MASS_G or POD_COM_ABOVE_PIVOT_MM.")

    # (f) base sphere and stability
    R = cfg.BASE_RADIUS_MM * mm
    sc = cad_to_B(cfg.BASE_CENTER_CAD_MM)
    margin = sc[2] - c[2]                                  # + means CG BELOW the centre of curvature (self-rights)
    lateral = float(np.hypot(c[0] - sc[0], c[1] - sc[1]))
    bottom_z = sc[2] - R
    theta_cap = math.asin(min(cfg.BASE_CAP_EDGE_RADIUS_MM / cfg.BASE_RADIUS_MM, 1.0))

    # (g) IMU
    R_bi = parse_axes(cfg.IMU_AXES)
    imu_B = c.copy() if cfg.IMU_POSITION == "com" else np.array(cfg.IMU_POSITION, float) * mm

    # (h) shell collision vertices (body frame = relative to the CG)
    if cfg.SHELL_STL:
        p = os.path.join(HERE, cfg.SHELL_STL)
        raw = read_stl(p)
        shell_B = np.column_stack([raw[:, 0], raw[:, 1], raw[:, 2] + O]) * mm
        if len(shell_B) > 20000:
            shell_B = shell_B[np.random.default_rng(0).choice(len(shell_B), 20000, replace=False)]
    else:
        # start exactly where the spherical face ends, so sphere and side band meet without a step
        r_edge = cfg.BASE_CAP_EDGE_RADIUS_MM
        z_edge = sc[2] / mm - math.sqrt(cfg.BASE_RADIUS_MM ** 2 - r_edge ** 2)
        profile = [(z_edge, r_edge)] + [(z, r) for z, r in cfg.SHELL_PROFILE_MM if z > z_edge + 0.5]
        rings = []
        for z, r in profile:
            for k in range(48):
                a = 2 * math.pi * k / 48
                rings.append([r * math.cos(a), r * math.sin(a), z])
        shell_B = np.array(rings) * mm

    # (i) gimbal "coupling inertia": how hard the body is kicked back when the servo swings the motor stack.
    #     Above w_c = sqrt(thrust * arm / I_couple) this kick beats the thrust torque, so the attitude loop must stay slower.
    e = float(cP[2] - pivot[2])                     # motor-stack CG above the pivot
    I_couple = IP[1, 1] + mP * e * (e + (pivot[2] - c[2])) * (1 - mP / m)
    I_couple = max(I_couple, 1e-6)

    # (j) visual-only outline: spherical cap + the collision profile (or the STL), drawn as one opaque shell
    cap = []
    for k in range(1, 7):
        th = theta_cap * k / 6.0
        for j in range(48):
            a = 2 * math.pi * j / 48
            cap.append([sc[0] + R * math.sin(th) * math.cos(a), sc[1] + R * math.sin(th) * math.sin(a), sc[2] - R * math.cos(th)])
    cap.append([sc[0], sc[1], sc[2] - R])
    visual_B = np.vstack([np.array(cap), shell_B])

    weight = m * G
    T = cfg.THRUST_MAX_N
    d = dict(
        mass=m, com_B=c, I_tot=I_tot, mass_body=mB, com_body_B=cB, I_body=IB, mass_pod=mP, com_pod_B=cP, I_pod=IP,
        pivot_B=pivot, sphere_center_B=sc, radius=R, margin=margin, lateral=lateral, bottom_z=bottom_z,
        theta_cap=theta_cap, R_bi=R_bi, imu_B=imu_B, shell_B=shell_B, visual_B=visual_B, extras=[e[3] for e in extras],
        z_rest=c[2] - bottom_z, arm=pivot[2] - c[2], weight=weight, twr=T / weight, I_couple=I_couple,
        thrust_unstable_N=(weight * margin / R) if margin > 0 else 0.0,
    )
    return d


# --------------------------------------------------------------------------------------- writers
def f(v, n=6):
    return " ".join(f"{x:.{n}g}" for x in np.atleast_1d(v))


def full_inertia(Im):
    return f([Im[0, 0], Im[1, 1], Im[2, 2], Im[0, 1], Im[0, 2], Im[1, 2]], 8)


def write_xml(cfg, d, path):
    c = d["com_B"]
    rel = lambda p: np.asarray(p) - c                                 # B frame -> body frame (origin at CG)
    lim = math.radians(cfg.GIMBAL_LIMIT_DEG)
    piv = d["pivot_B"]
    # gimbal servo stiffness: fast (~30 Hz) so the servo model in weeble_sim.py sets the real response
    I_piv = d["I_pod"][1, 1] + d["mass_pod"] * float(np.sum((d["com_pod_B"] - piv) ** 2))
    kp = I_piv * (2 * math.pi * 30) ** 2
    kd = 2 * 0.7 * math.sqrt(kp * I_piv)
    verts = " ".join(f"{x:.5f}" for x in rel(d["shell_B"]).ravel())
    vverts = " ".join(f"{x:.5f}" for x in rel(d["visual_B"]).ravel())
    q_imu = mat_to_quat(d["R_bi"])
    pods = [f'<geom type="cylinder" pos="0 0 {h * 1e-3:.4f}" size="{cfg.PROP_RADIUS_MM * 1e-3:.4f} 0.0015" '
            f'rgba="0.8 0.8 0.85 0.25" contype="0" conaffinity="0" mass="0"/>' for h in cfg.PROP_DISC_ABOVE_PIVOT_MM]
    D = cfg.HOP_DISTANCE_M
    xml = f"""<!-- AUTO-GENERATED by build.py from drone_config.py. Do not edit: change drone_config.py and rerun build.py. -->
<mujoco model="weeble">
  <compiler angle="radian" autolimits="true"/>
  <option timestep="{cfg.SIM_TIMESTEP_S}" gravity="0 0 -{G}" integrator="RK4" noslip_iterations="4"/>
  <visual><global offwidth="1280" offheight="960"/></visual>
  <asset>
    <texture name="grid" type="2d" builtin="checker" rgb1="0.15 0.22 0.30" rgb2="0.22 0.30 0.40" width="512" height="512"/>
    <material name="grid" texture="grid" texrepeat="20 20" reflectance="0.1"/>
    <material name="shell" rgba="0.55 0.75 0.95 1"/>
    <material name="hidden" rgba="0 0 0 0"/>
    <material name="pod" rgba="0.85 0.25 0.20 1"/>
    <mesh name="shell" vertex="{verts}"/>
    <mesh name="shell_visual" vertex="{vverts}"/>
  </asset>
  <worldbody>
    <light pos="0 0 8" dir="0 0 -1" diffuse="0.9 0.9 0.9"/>
    <geom name="ground" type="plane" size="20 20 0.1" material="grid" condim="6"
          friction="{cfg.GROUND_FRICTION} 0.005 {cfg.GROUND_ROLLING_FRICTION_M}"/>
    <site name="target" pos="{D} 0 0.002" type="cylinder" size="0.06 0.002" rgba="0.2 0.9 0.3 0.9"/>
    <site name="target_tol" pos="{D} 0 0.001" type="cylinder" size="0.5 0.001" rgba="0.2 0.9 0.3 0.15"/>
    <body name="weeble" pos="0 0 {d['z_rest']:.5f}">
      <freejoint name="root"/>
      <inertial pos="{f(rel(d['com_body_B']))}" mass="{d['mass_body']:.6f}" fullinertia="{full_inertia(d['I_body'])}"/>
      <!-- bottom face: exact R{cfg.BASE_RADIUS_MM:g} sphere (weeble_sim.py turns its contact off past {math.degrees(d['theta_cap']):.1f} deg of tilt,
           where the real drone rolls off the spherical face onto the side band = the mesh) -->
      <geom name="base" type="sphere" pos="{f(rel(d['sphere_center_B']))}" size="{d['radius']:.5f}" material="hidden"
            condim="6" friction="0.1 0.0001 0.00001" mass="0" group="3"/>
      <geom name="shell" type="mesh" mesh="shell" material="hidden" condim="6" friction="0.1 0.0001 0.00001" mass="0" group="3"/>
      <geom name="shell_visual" type="mesh" mesh="shell_visual" material="shell" contype="0" conaffinity="0" mass="0"/>
      <site name="imu" pos="{f(rel(d['imu_B']))}" quat="{f(q_imu)}" size="0.008" rgba="1 1 0 1"/>
      <body name="pod" pos="{f(rel(piv))}">
        <joint name="gimbal_x" type="hinge" axis="0 1 0" range="{-lim:.5f} {lim:.5f}" damping="{kd:.6g}"/>
        <joint name="gimbal_y" type="hinge" axis="-1 0 0" range="{-lim:.5f} {lim:.5f}" damping="{kd:.6g}"/>
        <inertial pos="{f(np.asarray(d['com_pod_B']) - piv)}" mass="{d['mass_pod']:.6f}" fullinertia="{full_inertia(d['I_pod'])}"/>
        <geom type="cylinder" pos="0 0 {cfg.POD_COM_ABOVE_PIVOT_MM * 1e-3:.4f}" size="{cfg.POD_RADIUS_MM * 1e-3:.4f} {cfg.POD_LENGTH_MM * 0.5e-3:.4f}"
              material="pod" contype="0" conaffinity="0" mass="0"/>
        {pods[0]}
        {pods[1] if len(pods) > 1 else ''}
        <site name="thrust" pos="0 0 0" size="0.006"/>
      </body>
    </body>
  </worldbody>
  <actuator>
    <!-- gimbal targets are in RADIANS (compiler angle="radian") -->
    <position name="gimbal_x" joint="gimbal_x" kp="{kp:.6g}" ctrlrange="{-lim:.5f} {lim:.5f}"/>
    <position name="gimbal_y" joint="gimbal_y" kp="{kp:.6g}" ctrlrange="{-lim:.5f} {lim:.5f}"/>
    <general name="thrust" site="thrust" gear="0 0 1 0 0 0" ctrlrange="0 {cfg.THRUST_MAX_N * 2:.3f}"/>
    <general name="yaw" site="thrust" gear="0 0 0 0 0 1" ctrlrange="-1 1"/>
  </actuator>
  <sensor>
    <gyro name="imu_gyro" site="imu"/>
    <accelerometer name="imu_accel" site="imu"/>
  </sensor>
</mujoco>
"""
    with open(path, "w", newline="\n") as fh:
        fh.write(xml)


DLPF = {260: 0, 184: 1, 94: 2, 44: 3, 21: 4, 10: 5, 5: 6}
AFS = {2: 0, 4: 1, 8: 2, 16: 3}
GFS = {250: 0, 500: 1, 1000: 2, 2000: 3}


def write_header(cfg, d, h, path):
    for name, table, val in (("IMU_DLPF_HZ", DLPF, cfg.IMU_DLPF_HZ), ("IMU_ACCEL_RANGE_G", AFS, cfg.IMU_ACCEL_RANGE_G),
                             ("IMU_GYRO_RANGE_DPS", GFS, cfg.IMU_GYRO_RANGE_DPS)):
        if val not in table:
            raise SystemExit(f"{name} = {val} is not allowed; pick one of {sorted(table)}")
    It = d["I_tot"]
    imu_rel = d["imu_B"] - d["com_B"]
    src = [int(np.argmax(np.abs(d["R_bi"][i]))) for i in range(3)]
    sgn = [int(np.sign(d["R_bi"][i, src[i]])) for i in range(3)]
    L = []
    w = L.append
    w("// AUTO-GENERATED by build.py from drone_config.py -- do not edit by hand.")
    w("// Change drone_config.py and rerun build.py. Used by both the ESP32 firmware and the simulator.")
    w("#pragma once")
    w("#include <stdint.h>")
    w(f"#define WEEBLE_CONFIG_HASH 0x{h:08X}u")
    w("namespace weeble { namespace cfg {")
    w("// ---- mass properties of the whole drone (about its centre of mass, body axes) ----")
    w(f"constexpr float MASS_KG = {d['mass']:.6f}f;")
    w(f"constexpr float INERTIA_XX = {It[0, 0]:.6e}f, INERTIA_YY = {It[1, 1]:.6e}f, INERTIA_ZZ = {It[2, 2]:.6e}f;  // kg m^2")
    w(f"constexpr float COM_ABOVE_BOTTOM_M = {d['z_rest']:.5f}f;")
    w(f"constexpr float PIVOT_ABOVE_COM_M = {d['arm']:.5f}f;        // gimbal lever arm (negative = pivot below CG)")
    w(f"constexpr float BASE_RADIUS_M = {d['radius']:.5f}f;")
    w(f"constexpr float BASE_CENTER_ABOVE_COM_M = {d['margin']:.5f}f;  // self-righting margin (>0 = CG below centre)")
    w(f"constexpr float GIMBAL_LIMIT_DEG = {cfg.GIMBAL_SOFT_LIMIT_DEG:.3f}f;")
    w(f"constexpr float GIMBAL_COUPLING_INERTIA = {d['I_couple']:.4e}f;  // kg m^2, motor-stack swing -> body kick")
    w("// ---- IMU ----")
    w(f"constexpr float IMU_POS_M[3] = {{{imu_rel[0]:.5f}f, {imu_rel[1]:.5f}f, {imu_rel[2]:.5f}f}};  // relative to CG, body axes")
    w(f"constexpr int IMU_AXIS_SRC[3] = {{{src[0]}, {src[1]}, {src[2]}}};    // body axis i = sign[i] * imu axis src[i]")
    w(f"constexpr int IMU_AXIS_SIGN[3] = {{{sgn[0]}, {sgn[1]}, {sgn[2]}}};")
    w(f"constexpr uint8_t IMU_DLPF_CFG = {DLPF[cfg.IMU_DLPF_HZ]};       // MPU6050 register 0x1A")
    w(f"constexpr uint8_t IMU_GYRO_FS_SEL = {GFS[cfg.IMU_GYRO_RANGE_DPS]};    // register 0x1B bits 4:3  (+-{cfg.IMU_GYRO_RANGE_DPS} dps)")
    w(f"constexpr uint8_t IMU_ACCEL_FS_SEL = {AFS[cfg.IMU_ACCEL_RANGE_G]};   // register 0x1C bits 4:3  (+-{cfg.IMU_ACCEL_RANGE_G} g)")
    w(f"constexpr float IMU_ACCEL_LSB_PER_G = {16384.0 / 2 ** AFS[cfg.IMU_ACCEL_RANGE_G]:.3f}f;")
    w(f"constexpr float IMU_GYRO_LSB_PER_DPS = {131.0 / 2 ** GFS[cfg.IMU_GYRO_RANGE_DPS]:.4f}f;")
    w("// ---- propulsion ----")
    w(f"constexpr float THRUST_MAX_N = {cfg.THRUST_MAX_N:.4f}f;")
    w(f"constexpr float MOTOR_LAG_UP_S = {cfg.MOTOR_LAG_UP_S:.4f}f, MOTOR_LAG_DOWN_S = {cfg.MOTOR_LAG_DOWN_S:.4f}f;")
    w(f"constexpr float IDLE_THROTTLE = {cfg.IDLE_THROTTLE:.4f}f;")
    w(f"constexpr float MOTOR_KV = {cfg.MOTOR_KV:.1f}f, BATTERY_V = {cfg.BATTERY_V:.2f}f, MAX_RPM = {cfg.MAX_RPM:.1f}f;")
    w(f"constexpr int ESC_PWM_HZ = {int(cfg.ESC_PWM_HZ)}, SERVO_PWM_HZ = {int(cfg.SERVO_PWM_HZ)};")
    w("// ---- mission defaults ----")
    w(f"constexpr float DESCENT_RELEASE_HEIGHT_M = {cfg.DESCENT_RELEASE_HEIGHT_M:.4f}f;")
    w(f"constexpr float HOP_DISTANCE_M = {cfg.HOP_DISTANCE_M:.4f}f, HOP_CRUISE_HEIGHT_M = {cfg.HOP_CRUISE_HEIGHT_M:.4f}f;")
    w(f"constexpr float HOP_MAX_LEAN_DEG = {cfg.HOP_MAX_LEAN_DEG:.3f}f, HOP_MAX_ACCEL_MPS2 = {cfg.HOP_MAX_ACCEL_MPS2:.3f}f;")
    w(f"constexpr float ASCENT_HEIGHT_M = {cfg.ASCENT_HEIGHT_M:.4f}f, ASCENT_MAX_HEIGHT_M = {cfg.ASCENT_MAX_HEIGHT_M:.4f}f, ASCENT_HOVER_S = {cfg.ASCENT_HOVER_S:.3f}f;")
    w(f"constexpr float CLIMB_MAX_ACCEL_MPS2 = {cfg.CLIMB_MAX_ACCEL_MPS2:.3f}f;")
    w(f"constexpr float LAND_SPEED_MPS = {cfg.LAND_SPEED_MPS:.3f}f, TOUCH_SPEED_MPS = {cfg.TOUCH_SPEED_MPS:.3f}f;")
    w(f"constexpr float CONTROL_HZ = {float(cfg.CONTROL_HZ):.1f}f;")
    w("}}  // namespace weeble::cfg")
    with open(path, "w", newline="\n") as fh:
        fh.write("\n".join(L) + "\n")


def write_info(cfg, d, h, path):
    info = dict(config_hash=h, z_rest=d["z_rest"], theta_cap_deg=math.degrees(d["theta_cap"]), mass=d["mass"],
                weight=d["weight"], twr=d["twr"], margin=d["margin"], arm=d["arm"], radius=d["radius"],
                com_above_bottom=d["z_rest"], R_bi=d["R_bi"].tolist(), extras=d["extras"])
    with open(path, "w") as fh:
        json.dump(info, fh, indent=2)


# --------------------------------------------------------------------------------------- compile
def compile_lib(sim_dir):
    src = os.path.join(sim_dir, "weeble_bridge.cpp")
    inc = os.path.join(HERE, "include")
    win = platform.system() == "Windows"
    out = os.path.join(sim_dir, "weeble.dll" if win else "libweeble.so")
    cxx = next((c for c in ("g++", "clang++") if shutil.which(c)), None)
    if cxx is None:
        print("\n!! No C++ compiler (g++) found on PATH, so the simulator library was NOT rebuilt.")
        print("   Install MinGW-w64 (64-bit, to match 64-bit Python) or MSYS2 and make sure 'g++' works in a terminal,")
        print("   then run build.py again.")
        return False
    cmd = [cxx, "-O2", "-std=c++17", "-shared", "-I", inc, "-o", out, src]
    cmd += ["-static", "-static-libgcc", "-static-libstdc++"] if win else ["-fPIC"]
    print("compiling:", " ".join(cmd))
    r = subprocess.run(cmd, capture_output=True, text=True)
    if r.returncode != 0:
        print(r.stdout, r.stderr)
        raise SystemExit("C++ compile failed (see above).")
    print("  ->", os.path.relpath(out, HERE))
    return True


# --------------------------------------------------------------------------------------- summary
def summary(cfg, d):
    c = d["com_B"]
    print("\n================ WEEBLE BUILD SUMMARY ================")
    print(f"total mass            {d['mass'] * 1000:8.1f} g   (extras included: {', '.join(d['extras']) or 'none'})")
    print(f"CG above bottom       {c[2] * 1000:8.1f} mm  (off-axis {np.hypot(c[0], c[1]) * 1000:.2f} mm)")
    print(f"base centre of curv.  {d['sphere_center_B'][2] * 1000:8.1f} mm above bottom, R = {d['radius'] * 1000:.0f} mm")
    print(f"self-righting margin  {d['margin'] * 1000:+8.1f} mm  (CG below centre of curvature; must be > 0)")
    print(f"gimbal arm            {d['arm'] * 1000:8.1f} mm  (pivot above CG)")
    print(f"inertia about CG      {d['I_tot'][0, 0]:.5f} / {d['I_tot'][1, 1]:.5f} / {d['I_tot'][2, 2]:.5f} kg m^2 (x/y/z)")
    print(f"thrust / weight       {d['twr']:8.2f}    ({cfg.THRUST_MAX_N:.1f} N vs {d['weight']:.1f} N weight)")
    wc = math.sqrt(d['weight'] * abs(d['arm']) / d['I_couple'])
    print(f"gimbal kick-back      attitude loop must stay well below {wc:.0f} rad/s ({wc / 6.283:.1f} Hz) at hover thrust")
    warn = []
    if d["margin"] <= 0:
        warn.append(f"NOT SELF-RIGHTING: the CG is {-d['margin'] * 1000:.1f} mm ABOVE the base's centre of curvature. "
                    f"The drone will not return upright by itself and tips over after landing.")
    elif d["margin"] < 0.01:
        warn.append(f"Weak self-righting: CG only {d['margin'] * 1000:.1f} mm below the centre of curvature (aim for 15+ mm).")
    else:
        print(f"ground instability    thrust above {d['thrust_unstable_N']:.1f} N makes the base unstable on the ground")
    if d["lateral"] > 0.002:
        warn.append(f"CG is {d['lateral'] * 1000:.1f} mm off the base axis: drone will rest tilted.")
    if d["twr"] < 1.3:
        warn.append(f"Thrust-to-weight {d['twr']:.2f} is low (< 1.3). Hop/ascent margins and the descent brake suffer.")
    for wmsg in warn:
        print("WARNING:", wmsg)
    print("=======================================================\n")


def main():
    cfg = load_config()
    d = derive(cfg)
    h = config_hash(cfg)
    sim_dir = os.path.join(HERE, "sim")
    os.makedirs(sim_dir, exist_ok=True)
    os.makedirs(os.path.join(HERE, "include"), exist_ok=True)
    write_xml(cfg, d, os.path.join(sim_dir, "WeebleDrone.xml"))
    write_header(cfg, d, h, os.path.join(HERE, "include", "weeble_config.h"))
    write_info(cfg, d, h, os.path.join(sim_dir, "model_info.json"))
    print("wrote sim/WeebleDrone.xml, sim/model_info.json, include/weeble_config.h")
    if "--no-compile" not in sys.argv:
        compile_lib(sim_dir)
    summary(cfg, d)


if __name__ == "__main__":
    main()
