# Weeble Phase 1 controls, v2 (ESP32, Arduino C++)

Scope: every routine in `Controls_State_Conditions_Specific_Command_Routines.csv`. IMU only (MPU6050). Phase 2 (Pixracer/ArduCopter) not covered.

## Files
- `WeebleControls/weeble_core.h` – all control logic, hardware-independent: estimator, attitude loop, descent, horizontal hop, vertical hop, macrostate supervisor, mixer, six-position calibration solver. Every tunable number is in `struct Params`, tagged ASSUMED / TODO / TBD where it is not from your documents.
- `WeebleControls/WeebleControls.ino` – ESP32 glue: MPU6050 driver, ESC/servo PWM, serial (and optional Bluetooth) commands, safety lock, calibration storage in flash, RAM log, bench tests. Needs the **ESP32Servo** library (Preferences and BluetoothSerial come with the ESP32 core). Put both files in a folder named `WeebleControls`.
- `sim/sim_test.cpp` – laptop test of the real controller code against an *assumed* planar physics model. `g++ -O2 -std=c++17 sim_test.cpp -o sim_test && ./sim_test`. Extra modes: `sweep` (descent preset tuning), `trace` (tick-by-tick run of each routine). Output saved in `sim/sim_results.txt`.
- `Controls_State_Conditions_Specific_Command_Routines.csv` – your table with the "Status in Code" column updated.

## Status
- Logic tested in simulation only (planar: pitch axis only, no yaw or sideways motion, no drag, no ground effect).
- The sketch was compiled against stub headers that mimic the Arduino/ESP32 APIs, and its command parser and state flow were driven with a fake IMU on a laptop. It has **not** been built for an ESP32 and **not** run on hardware.
- First power-up: props OFF. Boot state is Rest, safety lock engaged, throttle cap 15 %. Nothing spins until `cal` → `unlock` → `arm` → a routine command. `k` kills instantly.

## Routine → command
| CSV routine | Command | Macrostates |
|---|---|---|
| Controlled Landing | `h <m>`, `payload <kg>`, `cap <pct>`, `preset robust\|gentle`, then `descent` | Armed → Await Release → Descending → Landing → Amp Down → Rest |
| Launch Horizontal Distance | `hop <ft> [bearing] [lean=deg] [land=ballistic\|brake] [scale=x]` | Armed → Launch → Amp Up → Ascending → Descending → [Landing] → Amp Down → Rest |
| Launch Vertical Distance | `vhop <ft> [land=brake\|ballistic]` | Armed → Launch → Amp Up → Ascending → Descending → Landing → Amp Down → Rest |
| Calibrate IMU | `cal` (every session), `cal6 start/pose/solve` (once, stored in flash) | Rest |
| Arm / Disarm / Kill | `unlock`, `arm`, `disarm`, `k` | Rest ↔ Armed; any → Fault; `disarm`: Fault → Rest |
| Bench Gimbal Check | `servo <x> <y>`, `servo off` | Armed |
| Bench Spin / Vibration | `spin <pct> <s>` | Armed |

Every mission ends in Rest (auto-disarm, lock re-engaged) and prints a summary. `log` dumps the last 18 s at 100 Hz as CSV.

## What changed from v1, and why
Bugs found in v1:
1. **Throttle cap ignored by every thrust budget (safety).** With the boot cap of 15 %, v1 started a descent believing it had full thrust: sim touchdown 9.9 m/s. The "refuse below 0.5 m/s² spare" check used 100 % thrust and ran only *after* release. Now all feasibility checks use thrust at the cap and run when the command is typed.
2. **Attitude estimate pulled toward the thrust axis in flight.** The accelerometer only reads "up" when the drone is not accelerating; in near-hover it reads the thrust direction and passes the 1 g gate. Sim: ~3° steady tilt error in v1, ~0.3° in v2. Accel correction is now used only on the ground and while hanging.
3. **Descent ignored the measured thrust lag.** The braking dead time was a fixed 0.30 s regardless of `thrust_lag_up_s`. Now dead time = measured lag + preset margin, so entering the thrust-stand value actually changes the braking.
4. Smaller: braking integrator wound up at the cap; descent yaw reference was 0 instead of the heading at release; `cal` warnings did not block arming (CSV says they should); "Amp Down" and "Rest" did not exist; failed hops ended with no reason.

The simulator was also wrong in one important way: at touchdown it changed velocity in a single step, so the simulated IMU **never saw an impact**. v1's "touchdown detected" results were really timeouts or the height backstop. v2 models a 15 ms contact (ASSUMED).

Design change you should review: **default hop lean is now 15°, not 25°** (still settable per hop with `lean=`). In the sim at TWR 1.5, both work at exactly 5 ft, but at 25° the drone is already fast enough at liftoff to fly ~4 ft, so the cut fires on the first airborne tick. Other distances then scatter (3 ft target → 2.95–4.52 ft across noise seeds), horizontal touchdown speed is about 40 % higher (3.0–4.2 m/s), peak tilt 39°, and the hop is refused at TWR 1.30 (the CDR thrust). At 15°: 2–8 ft targets land within about 0.3 ft (occasional 0.5 ft from the 20 ms PWM frame), 1.9–2.9 m/s horizontal, peak tilt 22°. Cost: a higher arc, so vertical touchdown is harder (~2 m/s vs ~1 m/s at 5 ft).

The descent presets are unchanged: a sweep found no setting that is both gentle with perfect sensing and safe with sensor errors.

## Things that are assumed, not from your documents
| Item | Used | Where to get it |
|---|---|---|
| Thrust at full throttle with shell | 27.25 N (TWR 1.30 at 2.144 kg) | thrust stand |
| Thrust curve, spin-up/down lag | linear, 0.25 s | thrust stand step test |
| Pitch inertia, pod pivot above CG | 0.04 kg·m², 0.13 m (sim only) | CAD |
| Servo µs per pod degree | 5 µs/° (deliberately small) | horn/rod geometry + bench |
| Tilt limit | 25° (CDR mechanical); requirement says ±30° | range-of-motion test |
| ESC protocol | 50 Hz PWM 1000–2000 µs, already calibrated | ESC datasheet |
| IMU mounting / axis map | identity | how the GY-521 is mounted |
| Base drone mass | 1.690 kg (CDR total 2.144 − payload 0.454) | scale |
| Impact contact time (sim) | 15 ms | drop test log |
| Calibration "gyro quiet" | std < 0.02 rad/s | CSV gives no number |
| Vertical hop max height | 1.0 m | team (CSV: TBD) |
| Await-release timeout | 25 s from the `descent` command | mothership team |

## Suggested bench order
1. `imu` – WHO_AM_I = 0x68; tilting the drone changes the body accelerations the way you expect; fix `IMU_AXIS/IMU_SIGN`.
2. `cal6 start`, then `cal6 pose` in 8–12 still orientations (every axis up and down), `cal6 solve`. Stored in flash.
3. `cal`, `status` – must print CALIBRATION OK.
4. `unlock`, `arm`, then `servo 10 0`, `servo 0 10` (props off) – fix `servo_sign_*`, `servo_us_per_pod_deg`, `tilt_limit_deg`.
5. Thrust stand: thrust vs throttle and step-response lag with the shell on → `thrust_max_N`, `thrust_curve_exp`, `thrust_lag_*`, `idle_throttle`. Re-run the sim with the measured values.
6. `spin 30 3`, `spin 60 3`… restrained – decides whether GENTLE is ever safe.
7. Tethered attitude tuning, then `vhop 0.5` at low height on a tether, then hops, then drops.
