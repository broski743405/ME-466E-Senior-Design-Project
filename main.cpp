/*
Main functionality of the UTME Weeble drone: main.cpp

Hardware:
  ESP32 dev board (PlatformIO)
  2 Brushless motors
  IMU on 2IC (SDA 21, SCL 22, address 0x68)

*/
#include <vector>
#include <Arduino.h>
#include <espcontroller.h>
#include <imu.h>
#include <servo.h>
#include <gimblecontrol.h>

// ===================================================== CONFIG ==========================
namespace config {
  constexpr int kSdaPin = 21;
  constexpr int kSclPin = 22;
  constexpr uint16_t kI2CClockHz = 400000; //400 kHz

  // Loop timing
  constexpr uint32_t kControlPeriodUs = 5000; // 200Hz attitude/state loop
  constexpr uint32_t kTelemetryPeriodMs = 100; // 10Hz serial telemetry

  // Safety
  constexpr float kMaxTiltDeg = 45.0f; //
  constexpr uint8_t kMaxImuFailStreak = 10; // Consecutive failed IMU reads --> fault
  constexpr float kLaunchleveltoDeg = 5.0f;

  // Flight profile
  constexpr float kAscendRPM = 7000.0f;    // Tune these based on drone weights and payload
  constexpr float kDescendRPM = 5000.0f;
  constexpr float kLandingRPM = 4500.0f;

  constexpr uint32_t kLaunchTimeoutMs = 3000;
  constexpr uint32_t kAmpUpRampMs = 2000;
  constexpr uint32_t kAmpUpTimeoutMs = 5000;
  constexpr uint32_t kAmpupRpmTolerance = 0.05f;
  constexpr uint32_t kAmpUpSettleMs = 300;

  constexpr uint32_t kLandingTimeoutMs = 5000;
  constexpr float kTouchdownAccelG = 1.6f;
  constexpr uint32_t kAmpDownRampMs    = 1500;
  constexpr uint32_t kAmpDownTimeoutMs = 4000;
  constexpr float kStoppedRpm = 300.0f;

  // Gimbal
  constexpr float kMaxGinbalTiltDeg = 15.0f;                  // Tommy said this is the max tilt of the gimbal in either direction
  constexpr float kTh2InitialDeg = 25.0f;                     // CHANGE once the gimbal design is finalized
  const std::vector<float> LinkageLengths = {2.5, 3.0, 4.1};  // Investigate this further

  // Yaw


  // =============================================== HARDWARE ===========================================================

  static EscController::Config EscConfig() {

    EscController::Config c{};
    c.escPin = 
    c.motorKv = 
    c.battVolt = 11.1f;                              // Check this logic, there should be a way to monitor the battery voltage 
    c.maxRpm =
    c.controlPeriodMs =
    c.maxThrottleslewpersec =
    c.ledcChannel = 

    c.kp = 0.00005f;
    c.ki = 0.00005f;
    c.kp = 0.00005f;
    return c;

  };

};

// put function declarations here:

void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:

}
