#include <atomic>
#include <chrono>
#include <thread>

#include "logger.hpp"
#include "task.hpp"

using namespace std::chrono_literals;

extern "C" void app_main(void) {
  espp::Logger logger({.tag = "RAMMP_MIB", .level = espp::Logger::Verbosity::DEBUG});

  logger.info("RAMMP MIB booting...");
  logger.info("Initializing drive control and interface subsystems");

  std::atomic<int> counter = 0;

  espp::Task task({
      .callback = [&](auto &m, auto &cv) -> bool {
        logger.debug("[{}] MIB heartbeat: subsystem status check", counter++);
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
    logger.debug("[{}] RAMMP MIB active", counter++);
    std::this_thread::sleep_for(1s);
  }
}
