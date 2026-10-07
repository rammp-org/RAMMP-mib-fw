#include "bno055.h"
#include "imu.hpp"
#include "logger.hpp"
#include "task.hpp"

using namespace std::chrono_literals;

extern "C" void app_main(void) {
  static espp::Logger logger({.tag = "MAIN", .level = espp::Logger::Verbosity::INFO});
  static IMU imu;

  if (!imu.init()) {
    logger.error("IMU init failed");
    return;
  }

  if (bno055_get_readings(&imu.bno055_, EULER_ANGLE) == ESP_OK) {
    logger.info("pitch={:.1f} roll={:.1f} yaw={:.1f}", imu.bno055_.euler_angle.pitch,
                imu.bno055_.euler_angle.roll, imu.bno055_.euler_angle.yaw);
  } else {
    logger.error("bno055_get_readings() failed");
  }
}
