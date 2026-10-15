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

x
