Weeble Phase 1 controls, v2 (ESP32, Arduino C++)

Scope: every routine in Controls_State_Conditions_Specific_Command_Routines.csv. IMU only (MPU6050). Phase 2 (Pixracer/ArduCopter) not covered.

Files
WeebleControls/weeble_core.h – all control logic, hardware-independent: estimator, attitude loop, descent, horizontal hop, vertical hop, macrostate supervisor, mixer, six-position calibration solver. Every tunable number is in struct Params, tagged ASSUMED / TODO / TBD where it is not from your documents.
WeebleControls/WeebleControls.ino – ESP32 glue: MPU6050 driver, ESC/servo PWM, serial (and optional Bluetooth) commands, safety lock, calibration storage in flash, RAM log, bench tests. Needs the ESP32Servo library (Preferences and BluetoothSerial come with the ESP32 core). Put both files in a folder named WeebleControls.
sim/sim_test.cpp – laptop test of the real controller code against an assumed planar physics model. g++ -O2 -std=c++17 sim_test.cpp -o sim_test && ./sim_test. Extra modes: sweep (descent preset tuning), trace (tick-by-tick run of each routine). Output saved in sim/sim_results.txt.
Controls_State_Conditions_Specific_Command_Routines.csv – your table with the "Status in Code" column updated.
