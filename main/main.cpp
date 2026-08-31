#include <atomic>
#include <chrono>
#include <thread>

#include "bno055.h"
#include "imu.hpp"
#include "logger.hpp"
#include "task.hpp"

using namespace std::chrono_literals;

extern "C" void app_main(void) {
  espp::Logger logger({.tag = "Template", .level = espp::Logger::Verbosity::DEBUG});

  logger.info("Bootup");

  IMU imu;
  if (!imu.init()) {
    logger.error("IMU init failed");
    return;
  }

  while (true) {
    bno055_t &dev = imu.device();
    esp_err_t ret = bno055_get_readings(&dev, EULER_ANGLE);
    if (ret != ESP_OK) {
      ESP_LOGE("MAIN", "bno055_get_readings() failed: %s", esp_err_to_name(ret));
    } else {
      ESP_LOGI("MAIN", "pitch=%.1f roll=%.1f yaw=%.1f", dev.euler_angle.pitch, dev.euler_angle.roll,
               dev.euler_angle.yaw);
    }
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}
