#include <atomic>
#include <chrono>
#include <mutex>
#include <thread>

#include "arbiter.hpp"
#include "arbitration_server.hpp"
#include "bsp.hpp"
#include "logger.hpp"
#include "messages/control_message.hpp"
#include "rtps_pubsub.hpp"

using namespace std::chrono_literals;

// Serves the base's arbitration on the MIB's Ethernet, in ENFORCED mode, for a ROS 2
// client to exercise (tools/ros2_bench). A tokened twist is admitted or refused, and
// the admitted ones are counted instead of driven: nothing here moves the chair.

extern "C" void app_main(void) {
  espp::Logger logger(
      {.tag = "Arbitration Server Example", .level = espp::Logger::Verbosity::INFO});

  auto &mib = mib::bsp::MIB::instance();
  if (!mib.init()) {
    logger.error("Board init failed");
    return;
  }
  logger.info("Waiting for Ethernet link...");
  while (!mib.ethernet_connected()) {
    std::this_thread::sleep_for(100ms);
  }
  if (!mib.init_rtps()) {
    logger.error("RTPS init failed");
    return;
  }
  logger.info("RTPS up on {}", mib.ethernet_ip_string());

  //! [arbitration server example]
  std::atomic<int> admitted{0};
  std::atomic<int> halts{0};
  std::mutex twist_mutex;
  rammp::Twist latest_twist{};

  mib::Arbiter arbiter({
      .mode = mib::ArbitrationMode::ENFORCED,
      .on_halt =
          [&](mib::HaltReason reason) {
            // What MibSystem would do: stop the drive. Here, forget the last twist.
            ++halts;
            std::lock_guard<std::mutex> lock(twist_mutex);
            latest_twist = {};
            logger.warn("on_halt({})", mib::to_string(reason));
          },
  });

  mib::ArbitrationServer server({.arbiter = arbiter, .participant = mib.rtps_participant()});
  if (!server.is_valid()) {
    logger.error("Arbitration server failed to register its endpoints");
    return;
  }

  espp::Subscriber<rammp::TwistSetpoint> twist_subscriber(
      mib.rtps_participant(), {.topic = rammp::kBaseTwistSetpoint.name,
                               .type_name = rammp::kBaseTwistSetpoint.type,
                               .reliability = espp::RtpsParticipant::Reliability::BEST_EFFORT,
                               .on_message = [&](const rammp::TwistSetpoint &setpoint) {
                                 arbiter.run_if_admitted(setpoint.token, [&] {
                                   ++admitted;
                                   std::lock_guard<std::mutex> lock(twist_mutex);
                                   latest_twist = setpoint.twist;
                                 });
                               }});
  if (!twist_subscriber.is_valid()) {
    logger.error("Twist subscriber failed to register");
    return;
  }
  //! [arbitration server example]

  while (true) {
    std::this_thread::sleep_for(2s);
    const auto status = arbiter.status();
    rammp::Twist twist{};
    {
      std::lock_guard<std::mutex> lock(twist_mutex);
      twist = latest_twist;
    }
    logger.info("owned={} owner='{}' generation={} estopped={} admitted={} rejected={} "
                "halts={} twist=({:.3f}, {:.3f})",
                status.owned, status.owner_id, status.generation, status.estopped, admitted.load(),
                status.rejected_count, halts.load(), twist.linear.x, twist.angular.z);
  }
}
