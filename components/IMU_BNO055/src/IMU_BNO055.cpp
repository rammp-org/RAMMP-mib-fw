// #include "imu.hpp"

// #include <cmath>

// #include "esp_log.h"
// #include "freertos/FreeRTOS.h"
// #include "freertos/task.h"

// namespace {
// static const char *TAG = "Imu";

// // --- BNO055 register map (page 0) ---
// constexpr uint8_t REG_CHIP_ID = 0x00;
// constexpr uint8_t CHIP_ID_VALUE = 0xA0;
// constexpr uint8_t REG_PAGE_ID = 0x07;
// constexpr uint8_t REG_ACC_DATA_X_LSB = 0x08;   // 6 bytes: X,Y,Z, LSB first, int16
// constexpr uint8_t REG_QUA_DATA_W_LSB = 0x20;   // 8 bytes: W,X,Y,Z, LSB first, int16
// constexpr uint8_t REG_SYS_TRIGGER = 0x3F;
// constexpr uint8_t REG_PWR_MODE = 0x3E;
// constexpr uint8_t REG_OPR_MODE = 0x3D;

// constexpr uint8_t OPR_MODE_CONFIG = 0x00;
// constexpr uint8_t OPR_MODE_NDOF = 0x0C;
// constexpr uint8_t PWR_MODE_NORMAL = 0x00;
// constexpr uint8_t SYS_TRIGGER_RESET = 0x20;

// // Datasheet default units: quaternion is Q14 fixed point (1 unit = 1/2^14),
// // linear acceleration defaults to m/s^2 at 100 LSB per m/s^2.
// constexpr float QUAT_SCALE = 1.0f / 16384.0f;
// constexpr float ACCEL_SCALE = 1.0f / 100.0f;

// int16_t le16(uint8_t lsb, uint8_t msb) { return static_cast<int16_t>((static_cast<uint16_t>(msb)
// << 8) | lsb); } }  // namespace

// Imu::Imu(espp::I2c &i2c, uint8_t device_address) : i2c_(i2c), address_(device_address) {}

// bool Imu::write_register(uint8_t reg, uint8_t value) {
//   uint8_t buf[2] = {reg, value};
//   return i2c_.write(address_, buf, sizeof(buf));
// }

// bool Imu::init() {
//   uint8_t chip_id = 0;
//   if (!i2c_.read_at_register(address_, REG_CHIP_ID, &chip_id, 1) || chip_id != CHIP_ID_VALUE) {
//     ESP_LOGE(TAG, "BNO055 not found at 0x%02X (chip id read as 0x%02X) - check wiring/address",
//              address_, chip_id);
//     return false;
//   }

//   // Reset. Bosch datasheet: allow up to 650ms for power-on/reset to complete.
//   write_register(REG_SYS_TRIGGER, SYS_TRIGGER_RESET);
//   vTaskDelay(pdMS_TO_TICKS(650));

//   if (!i2c_.read_at_register(address_, REG_CHIP_ID, &chip_id, 1) || chip_id != CHIP_ID_VALUE) {
//     ESP_LOGE(TAG, "BNO055 did not come back after reset");
//     return false;
//   }

//   write_register(REG_PWR_MODE, PWR_MODE_NORMAL);
//   vTaskDelay(pdMS_TO_TICKS(10));

//   write_register(REG_PAGE_ID, 0x00);
//   write_register(REG_OPR_MODE, OPR_MODE_CONFIG);
//   vTaskDelay(pdMS_TO_TICKS(20));

//   write_register(REG_SYS_TRIGGER, 0x00);  // use internal oscillator
//   vTaskDelay(pdMS_TO_TICKS(10));

//   write_register(REG_OPR_MODE, OPR_MODE_NDOF);  // 9-DOF sensor fusion
//   vTaskDelay(pdMS_TO_TICKS(20));

//   ESP_LOGI(TAG, "BNO055 initialized (addr 0x%02X, NDOF mode)", address_);
//   return true;
// }

// void Imu::retrieve_readings() {
//   // --- Quaternion ---
//   uint8_t quat_buf[8] = {0};
//   if (!i2c_.read_at_register(address_, REG_QUA_DATA_W_LSB, quat_buf, sizeof(quat_buf))) {
//     ESP_LOGW(TAG, "quaternion read failed");
//     return;
//   }

//   Quaternion quat{
//       .w = le16(quat_buf[0], quat_buf[1]) * QUAT_SCALE,
//       .x = le16(quat_buf[2], quat_buf[3]) * QUAT_SCALE,
//       .y = le16(quat_buf[4], quat_buf[5]) * QUAT_SCALE,
//       .z = le16(quat_buf[6], quat_buf[7]) * QUAT_SCALE,
//   };

//   // Expose non-yaw quaternion for self-leveling kinematics.
//   current_quat = extract_swing(quat);

//   // Convert quaternion to Euler angles (standard aerospace sequence).
//   // X-axis rotation
//   double sinr_cosp = 2.0 * (quat.w * quat.x + quat.y * quat.z);
//   double cosr_cosp = 1.0 - 2.0 * (quat.x * quat.x + quat.y * quat.y);
//   double raw_x = atan2(sinr_cosp, cosr_cosp) * (180.0 / M_PI);

//   // Y-axis rotation
//   double sinp = 2.0 * (quat.w * quat.y - quat.z * quat.x);
//   double raw_y;
//   if (fabs(sinp) >= 1)
//     raw_y = copysign(M_PI / 2, sinp) * (180.0 / M_PI);
//   else
//     raw_y = asin(sinp) * (180.0 / M_PI);

//   // Z-axis rotation
//   double siny_cosp = 2.0 * (quat.w * quat.z + quat.x * quat.y);
//   double cosy_cosp = 1.0 - 2.0 * (quat.y * quat.y + quat.z * quat.z);
//   double raw_z = atan2(siny_cosp, cosy_cosp) * (180.0 / M_PI);

//   // --- Linear acceleration ---
//   uint8_t accel_buf[6] = {0};
//   if (i2c_.read_at_register(address_, REG_ACC_DATA_X_LSB, accel_buf, sizeof(accel_buf))) {
//     ax = le16(accel_buf[0], accel_buf[1]) * ACCEL_SCALE;
//     ay = le16(accel_buf[2], accel_buf[3]) * ACCEL_SCALE;
//     az = le16(accel_buf[4], accel_buf[5]) * ACCEL_SCALE;
//   } else {
//     ESP_LOGW(TAG, "accelerometer read failed");
//   }

//   // Map to the encoder axes expected downstream.
//   double raw_pitch = raw_x;
//   double raw_roll = raw_y;
//   double raw_yaw = raw_z;

//   // Software fix for upside-down mounting (moves the +/-180 discontinuity
//   // away from the flat resting position).
//   raw_roll += 180.0;
//   if (raw_roll > 180.0) raw_roll -= 360.0;

//   pitch = raw_pitch;
//   roll = raw_roll;
//   yaw = raw_yaw;

//   // Apply low-pass filter using shortest path for continuous angles.
//   double diff_pitch = pitch - pitchf;
//   while (diff_pitch > 180.0) diff_pitch -= 360.0;
//   while (diff_pitch < -180.0) diff_pitch += 360.0;
//   pitchf = pitchf + K * diff_pitch;
//   while (pitchf > 180.0) pitchf -= 360.0;
//   while (pitchf < -180.0) pitchf += 360.0;

//   double diff_roll = roll - rollf;
//   while (diff_roll > 180.0) diff_roll -= 360.0;
//   while (diff_roll < -180.0) diff_roll += 360.0;
//   rollf = rollf + K * diff_roll;
//   while (rollf > 180.0) rollf -= 360.0;
//   while (rollf < -180.0) rollf += 360.0;

//   pitchrd = pitchf * (M_PI / 180.0);
//   rollrd = -1.0 * rollf * (M_PI / 180.0);
// }

// Quaternion Imu::extract_swing(const Quaternion &q) {
//   // 1. Magnitude of the yaw (twist) rotation.
//   float twist_norm = sqrtf((q.w * q.w) + (q.z * q.z));

//   // 2. Handle the singularity (divide-by-zero) — only happens if pitched
//   // exactly +/-180 degrees.
//   if (twist_norm < 0.0001f) {
//     return Quaternion{1.0f, 0.0f, 0.0f, 0.0f};
//   }

//   // 3. Algebraically simplified cancellation: Inverse(Twist) * Measured.
//   Quaternion swing;
//   swing.w = twist_norm;
//   swing.x = (q.w * q.x + q.z * q.y) / twist_norm;
//   swing.y = (q.w * q.y - q.z * q.x) / twist_norm;
//   swing.z = 0.0f;  // yaw is zeroed out
//   return swing;
// }