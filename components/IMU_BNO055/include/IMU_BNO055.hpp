// #pragma once
// #include <cstdint>

// #include "i2c.hpp"  // espp::I2c — add "espp/i2c" as a dependency (idf.py add-dependency
// "espp/i2c")

// // Minimal quaternion, replaces Adafruit's imu::Quaternion so this component
// // has no dependency on the Arduino/Adafruit sensor libraries.
// struct Quaternion {
//   float w = 1.0f, x = 0.0f, y = 0.0f, z = 0.0f;
// };

// // Native ESP-IDF driver for the Bosch BNO055 9-DOF absolute orientation
// // sensor, talking to it directly over I2C via espp::I2c (no Arduino/Adafruit
// // dependency). Ported from an Arduino/Adafruit_BNO055-based implementation;
// // the Euler-angle conversion, swing extraction and low-pass filtering logic
// // is unchanged from that original.
// class Imu {
//  public:
//   static constexpr uint8_t kDefaultAddress = 0x28;  // 0x29 if COM3 is pulled high

//   // `i2c` must already be constructed/initialized (see init() below for the
//   // sensor-side setup). The Imu does not own the bus.
//   explicit Imu(espp::I2c &i2c, uint8_t device_address = kDefaultAddress);

//   // Verifies the chip ID, resets the sensor, and puts it into NDOF
//   // (9-DOF sensor fusion) mode. Returns false (and logs why) on failure —
//   // check the wiring/address if this fails.
//   bool init();

//   // Reads quaternion + accelerometer data over I2C, updates all the public
//   // fields below (pitch/roll/yaw, filtered pitchf/rollf, ax/ay/az,
//   // current_quat). Call this periodically from your own task/loop.
//   void retrieve_readings();

//   // --- Outputs / tunables (same fields/units as the original Arduino code) ---
//   float pitch = 0.0f, pitch_offset = 0.0f, pitchrd = 0.0f;
//   float roll = 0.0f, rollrd = 0.0f, yaw = 0.0f;
//   float pitchf = 0.0f, rollf = 0.0f;  // low-pass filtered pitch/roll (degrees)
//   float am = 0.001f;
//   float K = 0.08f;  // low-pass filter coefficient
//   float ax = 0.0f, ay = 0.0f, az = 0.0f;  // linear acceleration, m/s^2
//   Quaternion current_quat;  // yaw-removed ("swing") quaternion for self-leveling kinematics

//  private:
//   bool write_register(uint8_t reg, uint8_t value);
//   static Quaternion extract_swing(const Quaternion &q);

//   espp::I2c &i2c_;
//   uint8_t address_;
// };