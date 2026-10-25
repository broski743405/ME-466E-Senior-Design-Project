#pragma once
#include <math.h>
#include <stdint.h>

#if defined(ARDUINO)
#include <Arduino.h>
#if !defined(ESP32)
#error "Wrong board selected! In Arduino IDE, go to Tools -> Board -> esp32 -> ESP32 Dev Module."
#endif

namespace weeble {

static inline bool ledcSetupPin(uint8_t pin, uint8_t ch, uint32_t freqHz, uint8_t resBits) {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  (void)ch;
  return ledcAttach(pin, freqHz, resBits);
#else
  ledcSetup(ch, freqHz, resBits);
  ledcAttachPin(pin, ch);
  return true;
#endif
}

static inline void ledcWritePinOrCh(uint8_t pin, uint8_t ch, uint32_t duty) {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  (void)ch;
  ledcWrite(pin, duty);
#else
  (void)pin;
  ledcWrite(ch, duty);
#endif
}

class LedcServo {
 public:
  LedcServo(int pin, float minUs = 500.0f, float maxUs = 2500.0f, float travelRad = 3.14159265f, uint8_t ledcCh = 2)
      : pin_(pin), minUs_(minUs), maxUs_(maxUs), travel_(travelRad), ch_(ledcCh) {}

  bool begin() { return ledcSetupPin(pin_, ch_, kFreqHz, kResBits); }

  void setAngle(float shaftRad) {
    float t = shaftRad / travel_;
    if (t < 0.0f) t = 0.0f;
    if (t > 1.0f) t = 1.0f;
    writeUs(minUs_ + t * (maxUs_ - minUs_));
  }

  void writeUs(float us) {
    if (us < minUs_) us = minUs_;
    if (us > maxUs_) us = maxUs_;
    uint32_t ticks = static_cast<uint32_t>(us * kTicksPerUs + 0.5f);
    ledcWritePinOrCh(pin_, ch_, ticks);
  }

 private:
  static constexpr uint32_t kFreqHz = 50;
  static constexpr uint8_t  kResBits = 14;
  static constexpr float    kTicksPerUs = float((1u << kResBits) - 1) / (1e6f / kFreqHz);
  int pin_; float minUs_, maxUs_, travel_; uint8_t ch_;
};

class EscController {
 public:
  struct Config {
    uint8_t escPin = 18, tachPin = 4, pulsesPerRev = 2, ledcChannel = 0;
    float motorKv = 920.0f, battVolt = 11.1f, maxRpm = 9000.0f;
    uint16_t pwmFreq = 50, minPulseUs = 1000, maxPulseUs = 2000;
    float kp = 0.00005f, ki = 0.00005f, kd = 0.000001f;
    float maxThrottleSlewPerSec = 50.0f; // <--- BUG FIXED: Changed from 2.0 to 50.0 so motors respond instantly!
    uint16_t controlPeriodMs = 5, stallTimeoutMs = 200;
  };

  explicit EscController(const Config& cfg) : cfg_(cfg) {}

  void begin() {
    ledcSetupPin(cfg_.escPin, cfg_.ledcChannel, cfg_.pwmFreq, 14);
    writePulseUs(cfg_.minPulseUs);
    if (cfg_.tachPin < 255) {
      pinMode(cfg_.tachPin, INPUT_PULLUP);
      attachInterruptArg(digitalPinToInterrupt(cfg_.tachPin), &EscController::tachIsr, this, RISING);
    }
    lastUpdateMs_ = millis();
  }

  void arm(uint32_t holdMs = 3000) { writePulseUs(cfg_.minPulseUs); if (holdMs > 0) delay(holdMs); armed_ = true; }
  void setTargetRPM(float rpm) { targetRpm_ = (!armed_ || rpm <= 0.0f) ? 0.0f : ((rpm > cfg_.maxRpm) ? cfg_.maxRpm : rpm); }
  void setDirectPulseUs(uint16_t us) { writePulseUs(us); currentRpm_ = readRpm(); }
  void stop() { targetRpm_ = 0.0f; throttle_ = 0.0f; integral_ = 0.0f; writePulseUs(cfg_.minPulseUs); }
  float rpm() const { return currentRpm_; }

  void update() {
    uint32_t now = millis(); float dt = (now - lastUpdateMs_) * 1e-3f;
    if (dt < cfg_.controlPeriodMs * 1e-3f) return;
    lastUpdateMs_ = now;
    currentRpm_ = readRpm();
    if (!armed_ || targetRpm_ <= 10.0f) { stop(); prevRpm_ = currentRpm_; return; }

    float nominalMaxRpm = fmaxf(cfg_.motorKv * cfg_.battVolt, 1000.0f);
    float err = targetRpm_ - currentRpm_;
    float dRpm = (currentRpm_ - prevRpm_) / fmaxf(dt, 0.001f);
    prevRpm_ = currentRpm_;

    integral_ += cfg_.ki * err * dt;
    if (integral_ > 0.35f) integral_ = 0.35f; if (integral_ < -0.35f) integral_ = -0.35f;

    float rawThrottle = (targetRpm_ / nominalMaxRpm) + cfg_.kp * err + integral_ - cfg_.kd * dRpm;
    if (rawThrottle < 0.05f) rawThrottle = 0.05f; if (rawThrottle > 1.0f) rawThrottle = 1.0f;

    float maxStep = cfg_.maxThrottleSlewPerSec * dt;
    float delta = rawThrottle - throttle_;
    if (delta > maxStep) delta = maxStep; if (delta < -maxStep) delta = -maxStep;
    throttle_ += delta;
    writePulseUs(static_cast<uint16_t>(cfg_.minPulseUs + throttle_ * (cfg_.maxPulseUs - cfg_.minPulseUs)));
  }

 private:
  static void IRAM_ATTR tachIsr(void* arg) {
    auto* self = static_cast<EscController*>(arg);
    uint32_t nowUs = micros();
    portENTER_CRITICAL_ISR(&self->mux_);
    uint32_t dtUs = nowUs - self->lastPulseUs_;
    if (dtUs > 150) { self->periodUs_ = dtUs; self->lastPulseUs_ = nowUs; }
    portEXIT_CRITICAL_ISR(&self->mux_);
  }
  float readRpm() {
    uint32_t lastUs, perUs; portENTER_CRITICAL(&mux_); lastUs = lastPulseUs_; perUs = periodUs_; portEXIT_CRITICAL(&mux_);
    if (perUs == 0 || (micros() - lastUs)/1000 > cfg_.stallTimeoutMs) return 0.0f;
    return 60.0e6f / (static_cast<float>(perUs) * static_cast<float>(cfg_.pulsesPerRev));
  }
  void writePulseUs(uint16_t us) {
    if (us < cfg_.minPulseUs) us = cfg_.minPulseUs; if (us > cfg_.maxPulseUs) us = cfg_.maxPulseUs;
    uint32_t ticks = static_cast<uint32_t>((static_cast<float>(us) / (1e6f/cfg_.pwmFreq)) * ((1u<<14)-1) + 0.5f);
    ledcWritePinOrCh(cfg_.escPin, cfg_.ledcChannel, ticks);
  }
  Config cfg_; bool armed_ = false; float targetRpm_ = 0, currentRpm_ = 0, throttle_ = 0, integral_ = 0, prevRpm_ = 0;
  uint32_t lastUpdateMs_ = 0; portMUX_TYPE mux_ = portMUX_INITIALIZER_UNLOCKED; volatile uint32_t lastPulseUs_ = 0, periodUs_ = 0;
};

} // namespace weeble
#endif
