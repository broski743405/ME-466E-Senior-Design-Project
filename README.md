# ME-466E-Senior-Design-Project
Weeble Based Drone Controls Architecture

High Level:

Sensors: IMU, GPS and barometer
ESP32 Drives commands to 2 Servos and 2 ESCs for the brushless motors

Flight States:
Drone Macros States:       State Value:         Entry Condition(s):         Exit Condition(s):          Desired Transistion:
    
        Rest                    0
        Launch                  1
        Amp Up                  2
        Ascending               3
        Descending              4
        Landing                 5
        Amp Down                6
        Damaged                 7

Flight state is determined by a combination of the IMU settings and routine sequences
IMU Data: {ax, ay, az, gx, gy, gz} = Linear acceleration and radial velocity for/around each axis

Aside from Flight state, there is the "Armed" status, which tells whether safety is engaged or not

Safety Engaged(1) --> Drone enters Rest, cutting off pulsing power to the ESCs and cmds to the Servos.
Safety Disengaged(0) --> Drone is allowed to use its full functionality

