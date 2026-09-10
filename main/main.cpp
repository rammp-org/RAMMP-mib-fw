#include <atomic>
#include <chrono>
#include <thread>

#include "bsp.hpp"
#include "logger.hpp"
#include "task.hpp"

using namespace std::chrono_literals;

extern "C" void app_main(void) {
  espp::Logger logger({.tag = "RAMMP_MIB", .level = espp::Logger::Verbosity::INFO});

  logger.info("RAMMP MIB booting...");

  auto &mib = mib::bsp::MIB::instance();
  if (!mib.init()) {
    logger.error("Failed to initialize ESP32-P4-ETH board");
    return;
  }

  logger.info("ESP32-P4-ETH init complete");
  logger.info("Board Ethernet status: connected={}", mib.ethernet_connected());
  logger.info("Board IP: {}", mib.ethernet_ip_string());

  std::atomic<int> counter = 0;

  espp::Task task({
      .callback = [&](auto &m, auto &cv) -> bool {
        logger.info("[{}] MIB heartbeat: board online, ETH {}", counter++,
                    mib.ethernet_connected() ? "connected" : "disconnected");
        std::unique_lock<std::mutex> lock(m);
        cv.wait_for(lock, 1s);
        return false;
      },
      .task_config = {
          .name = "MIB heartbeat",
          .stack_size_bytes = 4096,
      }
  });
  task.start();

  while (true) {
    logger.info("[{}] RAMMP MIB active", counter++);
    std::this_thread::sleep_for(2s);
  }
}
