#include "mib_system.hpp"

#include <chrono>
#include <condition_variable>
#include <mutex>
#include <thread>

using namespace std::chrono_literals;

MibSystem::MibSystem()
    : BaseComponent("RAMMP_MIB", espp::Logger::Verbosity::INFO),
      mib_(mib::bsp::MIB::instance()) {}

void MibSystem::start() {
  state_task_ = std::make_unique<espp::Task>(espp::Task::Config{
      .callback = [this](std::mutex &mutex,
                         std::condition_variable &condition_variable) -> bool {
        run_state_step();

        std::unique_lock<std::mutex> lock(mutex);
        condition_variable.wait_for(lock, 20ms);
        return false;
      },
      .task_config = {.name = "MIB system state", .stack_size_bytes = 8 * 1024},
  });
  state_task_->start();

  while (state_.load() == SystemState::INIT) {
    std::this_thread::sleep_for(10ms);
  }

  if (state_.load() == SystemState::ERROR) {
    return;
  }

  publication_task_ = std::make_unique<espp::Task>(espp::Task::Config{
      .callback = [this](std::mutex &mutex,
                         std::condition_variable &condition_variable) -> bool {
        publish_system_state();
        publish_seat_state();

        std::unique_lock<std::mutex> lock(mutex);
        condition_variable.wait_for(lock, 200ms);
        return false;
      },
      .task_config = {.name = "MIB state publication", .stack_size_bytes = 4 * 1024},
  });
  publication_task_->start();

  while (true) {
    logger_.info("RAMMP MIB active, waiting for XYTwist messages");
    std::this_thread::sleep_for(2s);
  }
}

bool MibSystem::initialize_pubsub() {
  using Reliability = espp::RtpsParticipant::Reliability;

  joystick_subscriber_ = std::make_unique<espp::Subscriber<rammp_xy_twist_t>>(
      mib_.rtps_participant(),
      espp::Subscriber<rammp_xy_twist_t>::Config{
          .topic = RAMMP_TOPIC_JOYSTICK_XY_TWIST,
          .type_name = RAMMP_TYPE_XY_TWIST,
          .reliability = Reliability::BEST_EFFORT,
          .on_message = [this](const rammp_xy_twist_t &sample) {
            handle_joystick_message(sample);
          }});

  if (!joystick_subscriber_->is_valid()) {
    logger_.error("Failed to create MIB RTPS joystick subscriber");
    return false;
  }

  return true;
}

void MibSystem::handle_joystick_message(const rammp_xy_twist_t &sample) {
  logger_.info("Joystick x={} y={} twist={} buttons={} drive_mode={}", sample.x, sample.y,
               sample.twist, sample.buttons, sample.drive_mode);
}

void MibSystem::handle_seat_control_command() {
  logger_.info("Seat control command received - placeholder");
}

void MibSystem::handle_motor_status() {
  logger_.info("Motor status received - placeholder");
}

void MibSystem::publish_system_state() {
  logger_.debug("Publishing system state - placeholder");
}

void MibSystem::publish_seat_state() {
  logger_.debug("Publishing seat state - placeholder");
}

void MibSystem::run_state_step() {
  switch (state_.load()) {
  case SystemState::INIT:
    logger_.info("System state: INIT - initializing MIB");
    if (!mib_.init()) {
      logger_.error("Failed to initialize ESP32-P4-ETH board");
      state_.store(SystemState::ERROR);
      return;
    }

    logger_.info("ESP32-P4-ETH init complete");
    logger_.info("Board Ethernet status: connected={}", mib_.ethernet_connected());
    logger_.info("Board IP: {}", mib_.ethernet_ip_string());
    state_.store(initialize_pubsub() ? SystemState::IDLE : SystemState::ERROR);
    break;
  case SystemState::IDLE:
    logger_.debug("System state: IDLE - running idle placeholder");
    break;
  case SystemState::CALIBRATE:
    logger_.info("System state: CALIBRATE - running calibration placeholder");
    break;
  case SystemState::DRIVE_ENABLED:
    logger_.info("System state: DRIVE_ENABLED - running drive placeholder");
    break;
  case SystemState::ERROR:
    logger_.error("System state: ERROR - running error placeholder");
    break;
  }

}