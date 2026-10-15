/*
This establishes the Servo classes
*/

#pragma once
#include <cmath>
#include <vector>
#include <optional>
#include <Arduino.h>

class Servo {
public:
    // minUs/maxUs: pulse widths at the two ends of travel (check your servo's datasheet)
    // travelRad:   shaft rotation between minUs and maxUs (often ~180 deg, varies by model)
    Servo(int pin, float minUs = 500.0f, float maxUs = 2500.0f, float travelRad = PI)
        : pin_(pin), minUs_(minUs), maxUs_(maxUs), travel_(travelRad) {}

    bool begin() { return ledcAttach(pin_, kFreqHz, kResbit); }

    void setAngle(float shaftRad) {
        const float t = std::clamp(shaftRad / travel_, 0.0f, 1.0f);   // hard travel limit
        const float us = minUs_ + t * (maxUs_ - minUs_);

        ledcWrite(pin_, static_cast<uint32_t>(us * kTicksPerUs + 0.5f));
    }
    
private:
    static constexpr uint32_t kFreqHz  = 50;   // standard hobby servo frame rate
    static constexpr uint8_t  kResBits = 14;   // 14 bits works on ESP32, S2, S3 and C3
    static constexpr float kTicksPerUs = float((1u << kResBits) - 1) / (1e6f / kFreqHz);
 
    int   pin_;
    float minUs_, maxUs_, travel_;
    
};