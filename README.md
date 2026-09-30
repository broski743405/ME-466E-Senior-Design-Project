# Weeble Phase 1 controls (ESP32, Arduino C++)

Scope: controlled descent + ground-to-ground hop. IMU only (MPU6050). Phase 2 (Pixracer/ArduCopter) not covered.

## Files
- `WeebleControls/weeble_core.h` – all control logic, hardware-independent. Every tunable number is in `struct Params` and is tagged ASSUMED / TODO.
- `WeebleControls/WeebleControls.ino` – ESP32 glue: MPU6050 driver, ESC/servo PWM, serial commands, RAM logging, bench tests. Needs the **ESP32Servo** library. Put both files in one folder named `WeebleControls`.
- `sim/sim_test.cpp` – laptop test of the real controller code against an *assumed* planar physics model. `g++ -O2 -std=c++17 sim_test.cpp -o sim_test && ./sim_test` (extra modes: `sweep`…`sweep7`, `hoptrace`). Output saved in `sim/sim_results.txt`.

## Status
- Logic tested in simulation only. The sketch was syntax-checked against stub headers, **not** built for an ESP32 and **not** run on hardware.
- First power-up: props OFF. Throttle cap boots at 15 %. Nothing spins until `cal` → `arm` → a mission command. `k` kills instantly.

## Suggested bench order
1. `imu` – check WHO_AM_I = 0x68 and that tilting the drone changes ax/ay/az the way you expect; fix `IMU_AXIS/IMU_SIGN`.
2. `cal`, `status` – still + upright; tilt should read ~0.
3. `arm`, then `servo 10 0`, `servo 0 10` (props off) – check which way the pod moves and that servos don't bind. Fix `servo_sign_*`, `servo_us_per_pod_deg`, `tilt_limit_deg`.
4. Thrust stand (7 Oct): record thrust vs throttle and step-response lag with the shell on. Put the values in `thrust_max_N`, `thrust_curve_exp`, `thrust_lag_*`, `idle_throttle`.
5. `spin 30 3`, `spin 60 3`… with the drone restrained: prints the accelerometer shift caused by the motors. **This number decides whether the GENTLE descent preset is safe** (see below).
6. Tethered attitude tuning (`att_kp/kd/ki`), then hop tests at low `cap`, then drop tests.

## Things that are assumed, not from your documents
| Item | Used | Where to get it |
|---|---|---|
| Thrust at full throttle with shell | 27.25 N (static, no download loss) | thrust stand |
| Thrust curve, spin-up/down lag | linear, 0.25 s | thrust stand step test |
| Pitch inertia | 0.04 kg·m² (sim only) | CAD |
| Pod pivot above CG | 0.13 m (sim only; from CG diagram) | CAD |
| Servo µs per pod degree | 5 µs/° (deliberately small) | horn/rod geometry (CDR A.8) + bench |
| Tilt limit | 25° (CDR mechanical) – requirement says ±30° | range-of-motion test |
| ESC protocol | standard 50 Hz PWM 1000–2000 µs, already calibrated | ESC datasheet |
| IMU mounting / axis map | identity | how the GY-521 is mounted |
| Release | true drop from rest (zero initial velocity) | – |
| Release height | 6.096 m (20 ft), set with `h` | tape measure |

## Descent presets
- **ROBUST** (default): brakes late and hard, ~1.5–3 s powered. Hidden accelerometer errors have little time to grow. Touchdown ~0.4–2 m/s in the sim (up to ~5 m/s if thrust lag is 0.4 s).
- **GENTLE**: long slow creep, ~0.2 m/s touchdown in the sim, **but** if the motors make the accelerometer under-read by more than about 0.05 m/s² the drone can drift upward and then fall. Use only if the `spin` test shows the shift is small and positive/near zero, or after adding a rangefinder/barometer.
