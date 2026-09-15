#pragma once

#include <atomic>
#include <memory>

#include "bsp.hpp"
#include "base_component.hpp"
#include "messages.hpp"
#include "rtps_pubsub.hpp"
#include "task.hpp"

class MibSystem : public espp::BaseComponent {
public:
  MibSystem();

  void start();

private:
  enum class SystemState {
    INIT,
    IDLE,
    CALIBRATE,
    DRIVE_ENABLED,
    ERROR,
  };

  bool initialize_pubsub();
  void run_state_step();
  void handle_joystick_message(const rammp_xy_twist_t &sample);
  void handle_seat_control_command();
  void handle_motor_status();
  void publish_system_state();
  void publish_seat_state();

  mib::bsp::MIB &mib_;
  std::atomic<SystemState> state_{SystemState::INIT};
  std::unique_ptr<espp::Subscriber<rammp_xy_twist_t>> joystick_subscriber_;
  std::unique_ptr<espp::Task> state_task_;
  std::unique_ptr<espp::Task> publication_task_;
};
