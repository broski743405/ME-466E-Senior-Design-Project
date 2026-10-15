/*
This code runs the IMU inputs
Using the inputs, the overall STATE of the drone is determined

Drone Macros States:       State Value:         Entry Condition(s):         Exit Condition(s):          Desired Transistion:         Timeout/Fallback:
    
        Rest                    0
        Launch                  1
        Amp Up                  2
        Ascending               3
        Descending              4
        Landing                 5
        Amp Down                6
        Damaged                 7

*/

#include <cmath>
#include <vector>
#include <optional>
#include <Arduino.h>

struct StateValues {        // This is the information useful for the state determination
    float roll = 0;
    float pitch = 0;
    float yaw = 0;
    float rollrate = 0;
    float pitchrate = 0;
    float yawrate = 0;
};

struct RawIMUData {         // Uncalibrated data 
    
    float ax=0, ay=0, az = 0;
    float wx=0, wy=0, wz = 0;
    float TempC = 0;

};

enum class FlightState : uint8_t {
  // Next:  Launch (on command)      Fallback: Damaged
  Rest = 0,
 
  // Next:  AmpUp                    Fallback: Rest (if exit not met in time)
  Launch = 1,
 
  // Next:  Ascending                Fallback: AmpDown (if exit not met in time)
  AmpUp = 2,
 
  // Next:  Descending               Fallback: Descending
  Ascending = 3,
 
  // Next:  Landing                  Fallback: Landing
  Descending = 4,
 
  // Next:  AmpDown                  Fallback: AmpDown
  Landing = 5,
 
  // Next:  Rest                     Fallback: Rest
  AmpDown = 6,
 
  // Next:  Rest 
  Damaged = 7,
 
  // Next:  Rest 
  Recharge = 8

};