#include <chrono>
#include <thread>

#include "bsp.hpp"
#include "logger.hpp"
#include "messages.hpp"
#include "rtps_pubsub.hpp"

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

  using Reliability = espp::RtpsParticipant::Reliability;
  espp::Subscriber<rammp_xy_twist_t> joystick_subscriber(
      mib.rtps_participant(),
      {.topic = RAMMP_TOPIC_JOYSTICK_XY_TWIST,
       .type_name = RAMMP_TYPE_XY_TWIST,
       .reliability = Reliability::BEST_EFFORT,
       .on_message = [&](const rammp_xy_twist_t &sample) {
         logger.info("Joystick x={} y={} twist={} buttons={} drive_mode={}", sample.x,
                     sample.y, sample.twist, sample.buttons, sample.drive_mode);
       }});

  if (!joystick_subscriber.is_valid()) {
    logger.error("Failed to create MIB RTPS joystick subscriber");
    return;
  }

  while (true) {
    logger.info("RAMMP MIB active, waiting for XYTwist messages");
    std::this_thread::sleep_for(2s);
  }
}
