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
#include <Wire.h>

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

enum class Armed : uint8_t {            // Defines the safety engagement of the drone
    SafetyLockEngaged = 1,              // Safety Engaged: Drone stays in a rest/damaged state, ESCs cannot send pulse cmds
    NoSafetyLockEngaged = 0             // Safety Not engaged: Freedom to engage thrust
};

class IMUdefine {
public:
    explicit IMUdefine(TwoWire& wire = Wire, uint8_t address = 0x68);

    // Esp32 GPIO Pins: SDA = 21 and SCL = 22 for the I2C
    bool begin(int sdaPin = 21, int sclPin = 22, uint32_t i2cClock = 40000);
    bool calibrate(uint16_t samples = 1000);
    bool update();

    StateValues getAttitude()           const { return _att; }
    RawIMUData getRaw()                 const      { return _raw; }
    
    FlightState   getStatus()             const    { return _state; }
    Armed getSafety()                   const {return _safety; }
    const char* statusString()          const;
    bool        isHealthy()             const    { return _state == FlightState::Rest; }
    uint32_t    getErrorCount()         const { return _totalErrors; }
    float       getLastDt()             const    { return _lastDt; }
    uint8_t     getWhoAmI()             const    { return _whoAmI; }
    
    // Higher alpha = trust gyro more (smoother, slower drift correction).
    void setFilterAlpha(float alpha) { _alpha = constrain(alpha, 0.0f, 1.0f); }
    
    // Re-seed roll/pitch from the accelerometer on the next update().
    void resetAttitude() { _seeded = false; }
    
    // Prints in Arduino Serial Plotter format.
    void printDebug(Stream& out) const;

private:
    bool writeRegisters(uint8_t reg, uint8_t value);
    bool readRegisters();
    bool readSensors(RawIMUData & out);

    TwoWire& _wire;
    uint8_t _addr;
    uint8_t _whoAmI = 0;

    FlightState _state = FlightState::Rest;
    Armed _safety = Armed::SafetyLockEngaged;
    bool _started = false;
    bool _calibrated = false;
    bool _seeded = false;

    float _alpha = 0.98f;
    float _gyroBias[3] = {0,0,0};

    StateValues _att;
    RawIMUData _raw;

    uint32_t _lastMicros = 0;
    float _lastDt = 0;
    uint8_t _consecutiveErrors = 0;
    uint32_t _totalErrors = 0;

};
