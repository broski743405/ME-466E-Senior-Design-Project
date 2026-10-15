/* Closed loop controller of a brushless motor RPM through the ESC

- ESC pulse signals generated with a LEDC hardware PWN peripheral
- RPM measured by a tachometer plus
- Feed-forward (KV * V) + PID with anti-windup

*/

#pragma once
#include <cmath>
#include <vector>
#include <optional>
#include <Arduino.h>

class EscController {
public:                            // Accessible anywhere in the program
    struct Config {
        uint8_t escPin;            // 
        uint8_t tachPin;
        uint8_t pulsesPerRev;

        float motorKv;                  // RPM per volt
        float battVolt;                 // nominal battery cell voltage
        float maxRpm;                   // max safe RPM
        uint8_t ledcChannel = 0;        // IDK what this is tbh
        uint16_t pwnFreq = 50;          // can be up 490Hz, look at the motor model
        uint16_t minPulseUs = 1000;
        uint16_t maxPulseUs = 2000;
        float kp = 0.000005f;
        float ki;
        float kd;
        uint16_t controlPeriodMs = 5; // 200 Hz control loop
        uint16_t stallTimeoutMs = 200;
        float maxThrottleslewpersec = 2.0f;
    
    };

explicit EscController(const Config & cfg);   // Blocks the compiler from doing type conversions, when the constructer type is different

    void begin();
    void arm(uint32_t holdMs = 3000);
    void setTargetRPM(float rpm);
    void stop();
    void update();

    float rpm() const {
        return currentRpm;
    }
    float targetRPM() const {
        return targetRpm;
    }
    float throtfunc() const {
        return throttle;
    }

private:                                           // Only accessible through the class functions when they are called, and specific to the object
    static void IRAM_ATTR tachISr(void* arg);
    void writePulseUs(uint16_t us);
    float readRpm();

    Config cfg_;
    bool armed = false;
    float targetRpm = 0;
    float currentRpm = 0;
    float throttle = 0;
    float integral = 0;
    float prevRpm = 0;
    uint32_t lastUpdateMs = 0;

    portMUX_TYPE      mux_ = portMUX_INITIALIZER_UNLOCKED;
    volatile uint32_t lastPulseUs_ = 0;
    volatile uint32_t periodUs_    = 0;
};