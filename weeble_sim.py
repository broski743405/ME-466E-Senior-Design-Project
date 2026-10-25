# mujoco_weeble_sim.py -- Visual 3D MuJoCo simulator driving weeble_core.h
import sys, time, ctypes, os
import numpy as np
import mujoco
import mujoco.viewer
import imageio

# 1. Load compiled C++ weeble_core.h library
lib_name = "./weeble.dll" if os.name == "nt" else "./libweeble.so"
lib = ctypes.CDLL(lib_name)
lib.sim_init.argtypes = [ctypes.c_int, ctypes.c_float, ctypes.c_float]
lib.sim_step.argtypes = [ctypes.c_float]*7 + [ctypes.POINTER(ctypes.c_float)]

mode_str = sys.argv[1] if len(sys.argv) > 1 else "hop"
model = mujoco.MjModel.from_xml_path("WeebleDrone.xml")
data = mujoco.MjData(model)

# 2. Configure mission mode
if mode_str == "descent":
    H0 = 6.096  # 20 ft drop
    lib.sim_init(0, H0, 1.50)
    data.qpos[2] = 0.15 + H0  # Start 6.096 m above ground
    print("=== 3D MuJoCo: 20 ft (6.1 m) Controlled Descent ===")
elif mode_str == "vhop":
    lib.sim_init(2, 0.60, 1.50)
    print("=== 3D MuJoCo: 0.6 m Vertical Hop with Landing Brake ===")
else:
    lib.sim_init(1, 5.0, 1.50)
    print("=== 3D MuJoCo: 5 ft (1.524 m) Parabolic Hop (Green Ring = Target) ===")

mujoco.mj_forward(model, data)

# First-order motor thrust lag state (matches sim_test.cpp tau = 0.25 s)
actual_thrust = 0.0
tau_up, tau_dn = 0.25, 0.25
out_buf = (ctypes.c_float * 7)()
step_count = 0

with mujoco.viewer.launch_passive(model, data) as viewer:
    viewer.cam.distance = 4.5
    viewer.cam.elevation = -18
    
    while viewer.is_running():
        t0 = time.time()

        # Keep the camera locked onto the drone's current position!
        viewer.cam.lookat[:] = [data.qpos[0], data.qpos[1], data.qpos[2]]

        if mode_str == "descent" and data.time < 0.5:
            data.qpos[2] = 0.15 + 6.096
            data.qvel[:] = 0.0

        # Run C++ Flight::update() at 200 Hz (every 5 ms = 5 physics steps)
        if step_count % 5 == 0:
            gx, gy, gz = data.sensor("imu_gyro").data
            ax, ay, az = data.sensor("imu_accel").data
            if mode_str == "descent" and data.time < 0.5:
                gx, gy, gz, ax, ay, az = 0.0, 0.0, 0.0, 0.0, 0.0, 9.80665
            lib.sim_step(gx, gy, gz, ax, ay, az, 0.005, out_buf)

        # Apply motor spin-up/down lag and drive MuJoCo actuators
        T_cmd = out_buf[0]
        dt = model.opt.timestep
        tau = tau_up if T_cmd > actual_thrust else tau_dn
        actual_thrust += (T_cmd - actual_thrust) * (dt / tau)

        data.ctrl[0] = out_buf[1]      # Gimbal Pitch [deg]
        data.ctrl[1] = out_buf[2]      # Gimbal Roll [deg]
        data.ctrl[2] = actual_thrust   # Coaxial Thrust [N]
        data.ctrl[3] = out_buf[3]      # Differential Yaw Torque [N*m]

        mujoco.mj_step(model, data)
        step_count += 1

        if step_count % 20 == 0:
            viewer.sync()
        if step_count % 200 == 0:
            print(f"t={data.time:4.2f}s | pos=({data.qpos[0]:.2f}, {data.qpos[2]-0.15:.2f})m | "
                  f"T={actual_thrust:4.1f}N | RPM={out_buf[5]:4.0f} | 4Bar_th2={out_buf[6]:5.1f}deg")

        elapsed = time.time() - t0
        if elapsed < dt:
            time.sleep(dt - elapsed)


width, height = 640, 480
renderer = mujoco.Renderer(model, height=height, width=width)

fps = 50                 # 50 frames per second for smooth playback
target_duration = 10.0   # Record the first 10 seconds
total_frames = int(fps * target_duration)
sim_dt = model.opt.timestep
steps_per_frame = int(1.0 / (fps * sim_dt))

frames = []
print("Recording the first 10 seconds of simulation...")

# Run simulation and capture frames for the first 10 seconds
for frame_idx in range(total_frames):
    # Step the physics forward for the duration of one frame
    for _ in range(steps_per_frame):
        # Your control loop / step logic here:
        if step_count % 5 == 0:
            gx, gy, gz = data.sensor("imu_gyro").data
            ax, ay, az = data.sensor("imu_accel").data
            lib.sim_step(gx, gy, gz, ax, ay, az, 0.005, out_buf)

        T_cmd = out_buf[0]
        dt = model.opt.timestep
        tau = tau_up if T_cmd > actual_thrust else tau_dn
        actual_thrust += (T_cmd - actual_thrust) * (dt / tau)

        data.ctrl[0] = out_buf[1]      # Gimbal Pitch [deg]
        data.ctrl[1] = out_buf[2]      # Gimbal Roll [deg]
        data.ctrl[2] = actual_thrust   # Coaxial Thrust [N]
        data.ctrl[3] = out_buf[3]      # Differential Yaw Torque [N*m]

        mujoco.mj_step(model, data)
        step_count += 1

    # Render frame pixels and store them
    renderer.update_scene(data)
    pixels = renderer.render()
    frames.append(pixels)

# Save frames to an MP4 video file
output_filename = "weeble_first_10s.mp4"
imageio.mimsave(output_filename, frames, fps=fps)
print(f"Video successfully recorded and saved as '{output_filename}'!")

# (Optional) Continue running the interactive viewer normally after recording
with mujoco.viewer.launch_passive(model, data) as viewer:
    while viewer.is_running():
        # regular interactive loop...
        mujoco.mj_step(model, data)
        viewer.sync()