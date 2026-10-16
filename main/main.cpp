#include "mib_system.hpp"
#include <atomic>
#include <chrono>
#include <cstdlib>
#include <thread>

#include "bno055.h"
#include "canopen_client.hpp"
#include "imu.hpp"
#include "logger.hpp"
#include "mcp266.hpp"
#include "task.hpp"
#include "twai.hpp"

using namespace std::chrono_literals;

extern "C" void app_main(void) {
  static espp::Logger logger({.tag = "MAIN", .level = espp::Logger::Verbosity::INFO});
  static IMU imu;

  if (!imu.init()) {
    logger.error("IMU init failed");
    return;
  }

  static espp::Task task({
      .callback = [](std::mutex &m, std::condition_variable &cv) -> bool {
        if (bno055_get_readings(&imu.bno055_, EULER_ANGLE) == ESP_OK) {
          logger.info("pitch={:.1f} roll={:.1f} yaw={:.1f}", imu.bno055_.euler_angle.pitch,
                      imu.bno055_.euler_angle.roll, imu.bno055_.euler_angle.yaw);
        } else {
          logger.error("bno055_get_readings() failed");
        }
        std::unique_lock lock(m);
        cv.wait_for(lock, 100ms);
        return false;
      },
      .task_config = {.name = "imu_task"},
  });
  task.start();

  MibSystem system;
  system.start(); // blocks forever
}
