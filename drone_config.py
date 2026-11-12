"""
drone_config.py  --  THE ONE PLACE FOR EVERY PHYSICAL NUMBER.

When the hardware team changes something:
    1. edit the numbers below
    2. run:   py build.py
That regenerates:
    sim/WeebleDrone.xml        (the MuJoCo model)
    include/weeble_config.h    (the same numbers for the C++ controller / ESP32 firmware)
    sim/weeble.dll             (the controller compiled for the simulator; libweeble.so on Linux/Mac)
and prints a summary (total mass, CG, thrust-to-weight, self-righting margin) with warnings.

Conventions
    * "BOTTOM" = the lowest point of the base when the drone stands upright.
    * "CAD" coordinates = exactly what Onshape shows (relative to the CAD origin), in mm.
    * Body axes: z up through the drone's centre line, x = the axis the X servo tilts the motor
      stack toward, y = the axis the Y servo tilts toward (same as the CAD X / Y axes).
    * Anything marked PLACEHOLDER is a guess. Replace it with a measurement when you have one.
"""

# =====================================================================================
# 1. MASS PROPERTIES  --  copy straight from Onshape "Mass and section properties"
#    (Assembly measured WITH the CRM2413 motor stack and battery, WITHOUT props or payload.)
# =====================================================================================
CAD_ORIGIN_ABOVE_BOTTOM_MM = 234.0          # Onshape Measure: CAD origin -> lowest point of base (Z)
CAD_MASS_G = 1023.569
CAD_COM_MM = (-0.033, -0.774, -133.296)     # Center of mass X, Y, Z (relative to the CAD origin)
# Onshape reports these about the center of mass, in g*mm^2.
CAD_INERTIA_G_MM2 = dict(xx=8.467e6, yy=8.409e6, zz=1.703e6,
                         xy=2349.957, xz=-1362.714, yz=7649.244)

# =====================================================================================
# 2. SHELL / BASE GEOMETRY
# =====================================================================================
BASE_RADIUS_MM = 100.0                      # radius of curvature of the bottom face
BASE_CENTER_CAD_MM = (0.0, -0.231, -134.0)  # centre of that sphere (Onshape Measure "center dist" on the bottom face)
BASE_CAP_EDGE_RADIUS_MM = 45.6              # how far out the spherical face reaches (Measure "min dist" Y).
                                            # Beyond ~27 deg of tilt the drone rolls onto the curved side band.

# Outline of the shell used for collisions when there is no STL: (height above BOTTOM, radius), both mm.
# Read off the CAD side view. Must start where the spherical cap ends and go up to the top rim.
SHELL_PROFILE_MM = [
    (11.4, 45.6),    # cap edge (end of the R100 sphere)
    (17.0, 56.0),
    (25.0, 64.5),
    (35.0, 70.5),
    (45.0, 73.5),
    (54.0, 74.3),    # widest point
    (175.0, 52.0),   # light-blue / dark-blue seam
    (244.0, 33.6),   # top rim of the cone
    (282.0, 20.0),   # motor housing top (approx.)
]
# Optional: path to an STL of the outer shell exported in the SAME CAD coordinates (mm).
# If set, it replaces SHELL_PROFILE_MM for collisions and visuals (MuJoCo uses its convex hull).
SHELL_STL = None                            # e.g. "cad/shell.stl"

# =====================================================================================
# 3. GIMBAL (fisheye bearing) AND MOTOR STACK
# =====================================================================================
GIMBAL_PIVOT_ABOVE_BOTTOM_MM = 245.0        # centre of the fisheye bearing
GIMBAL_LIMIT_DEG = 15.0                     # mechanical stop, +-
GIMBAL_SOFT_LIMIT_DEG = 15.0                # the controller never commands more than this.
                                            # (A degree of margin below the stop is kinder to the servos.)

# The tilting part (CRM2413 motor stack + mount). It is INSIDE the CAD numbers above; these values
# only decide how the total is split between the shell and the tilting part in the simulator.
POD_MASS_G = 190.0                          # PLACEHOLDER: CRM2413A is listed at 170 g, + ~20 g mount
POD_COM_ABOVE_PIVOT_MM = 25.0               # PLACEHOLDER: where the motor stack's CG sits above the bearing
POD_RADIUS_MM = 28.0                        # for its inertia estimate (treated as a solid cylinder)
POD_LENGTH_MM = 60.0
PROP_DISC_ABOVE_PIVOT_MM = (45.0, 75.0)     # visual only: lower / upper propeller heights above the pivot
PROP_RADIUS_MM = 114.0                      # 9-inch props

# =====================================================================================
# 4. MASSES NOT YET IN THE CAD  (set enabled=True to include them)
#    pos = (x, y, height above BOTTOM) in mm.  on_pod=True if it tilts with the motor stack.
# =====================================================================================
EXTRA_MASSES = [
    dict(name="props",   mass_g=18.0,  pos_mm=(0.0, 0.0, 305.0), on_pod=True,  enabled=False),  # 2x 9050 ~9 g each
    dict(name="payload", mass_g=454.0, pos_mm=(0.0, 0.0, 60.0),  on_pod=False, enabled=False),  # PLACEHOLDER position
]

# =====================================================================================
# 5. IMU (MPU6050)
# =====================================================================================
IMU_POSITION = "com"            # "com" = exactly at the centre of mass, or (x, y, height above BOTTOM) in mm
# How the chip is mounted: which IMU axis (with sign) points along body +x, +y, +z.
# ("+x", "+y", "+z") = chip axes line up with the body axes. Example rotated 90 deg: ("+y", "-x", "+z").
IMU_AXES = ("+x", "+y", "+z")
IMU_ACCEL_RANGE_G = 8           # 2, 4, 8 or 16   (firmware writes this into the chip)
IMU_GYRO_RANGE_DPS = 1000       # 250, 500, 1000 or 2000
IMU_DLPF_HZ = 44                # chip low-pass filter: 260, 184, 94, 44, 21, 10 or 5
# Simulator-only sensor imperfections (typical MPU6050 after calibration)
IMU_ACCEL_NOISE = 0.04          # m/s^2 per sample (1 sigma, after the chip filter)
IMU_GYRO_NOISE = 0.002          # rad/s per sample
IMU_GYRO_BIAS_LEFT = 0.002      # rad/s residual bias after calibration (~0.1 deg/s)

# =====================================================================================
# 6. PROPULSION
# =====================================================================================
THRUST_MAX_N = 16.6             # PLACEHOLDER: vendor figure, 1696 g at 12.6 V with 2x GWS 9050.
                                # Expect less at 11.1 V (roughly x0.8). Measure on a scale.
MOTOR_LAG_UP_S = 0.10           # PLACEHOLDER: time for thrust to reach 63 % of a step. Measure!
MOTOR_LAG_DOWN_S = 0.10         # PLACEHOLDER
IDLE_THROTTLE = 0.06            # lowest throttle while a routine is running
MOTOR_KV = 1300.0               # vendor listing for CRM2413A (main.cpp used to say 920)
BATTERY_V = 11.1                # 3S nominal
MAX_RPM = 9000.0                # RPM at full throttle under load. PLACEHOLDER (read it off the tachs)
ESC_PWM_HZ = 50                 # how often the ESCs get a new pulse. Many multirotor ESCs accept 400.
SERVO_PWM_HZ = 50               # standard hobby servos: 50
SERVO_SPEED_DPS = 600.0         # PLACEHOLDER: max motor-stack tilt speed (deg/s) through the linkage
SERVO_LAG_S = 0.03              # PLACEHOLDER: servo response time constant
YAW_TORQUE_PER_DIFF = 0.25      # simulator only: N*m of yaw torque per 1.0 of upper/lower throttle difference

# =====================================================================================
# 7. MISSION DEFAULTS (can also be given on the command line)
# =====================================================================================
DESCENT_RELEASE_HEIGHT_M = 6.096    # 20 ft drop, measured at the base bottom
HOP_DISTANCE_M = 1.524              # 5 ft
HOP_CRUISE_HEIGHT_M = 0.20          # how high the hop flies (centre of mass rise)
HOP_MAX_LEAN_DEG = 12.0             # body lean used to speed up / slow down
HOP_MAX_ACCEL_MPS2 = 2.0            # horizontal acceleration limit used to plan the hop
ASCENT_HEIGHT_M = 0.60
ASCENT_MAX_HEIGHT_M = 1.0
ASCENT_HOVER_S = 0.5                # time held at the top before coming back down
CLIMB_MAX_ACCEL_MPS2 = 2.5          # vertical acceleration limit when climbing
LAND_SPEED_MPS = 0.40               # sink rate for the final landing of hop and ascent
TOUCH_SPEED_MPS = 0.20              # sink rate for the last 8 cm

# =====================================================================================
# 8. SIMULATOR ENVIRONMENT
# =====================================================================================
GROUND_FRICTION = 0.8
GROUND_ROLLING_FRICTION_M = 0.002   # rolling resistance of a hard flat floor (0.01+ = carpet / grass)
SIM_TIMESTEP_S = 0.001
CONTROL_HZ = 200
