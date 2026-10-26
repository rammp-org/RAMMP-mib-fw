#include "mib_system.hpp"

#include <chrono>
#include <condition_variable>
#include <mutex>
#include <thread>

#include "MIBconfig.hpp"
#include "esp_system.h"

using namespace std::chrono_literals;

MibSystem::MibSystem()
    : BaseComponent("RAMMP_MIB", espp::Logger::Verbosity::INFO),
  mib_(mib::bsp::MIB::instance()), drive_controller_() {}

void MibSystem::start() {
  state_task_ = std::make_unique<espp::Task>(espp::Task::Config{
      .callback = [this](std::mutex &mutex,
                         std::condition_variable &condition_variable) -> bool {
        run_state_step();

        std::unique_lock<std::mutex> lock(mutex);
        condition_variable.wait_for(lock, mib::config::state_task_interval);
        return false;
      },
      .task_config = {.name = "MIB system state", .stack_size_bytes = 8 * 1024},
  });
  state_task_->start();

  // Publishing is only meaningful once RTPS and the endpoints exist, so wait out INIT.
  while (state_.load() == SystemState::INIT) {
    std::this_thread::sleep_for(10ms);
  }

  if (state_.load() == SystemState::ERROR) {
    // Nothing to publish. The state task reboots the MIB after error_reboot_delay; keep
    // this object (and that task) alive until it does.
    while (true) {
      std::this_thread::sleep_for(2s);
    }
  }

  publication_task_ = std::make_unique<espp::Task>(espp::Task::Config{
      .callback = [this](std::mutex &mutex,
                         std::condition_variable &condition_variable) -> bool {
        publish_system_state();
        publish_seat_state();
        log_drive_status();

        std::unique_lock<std::mutex> lock(mutex);
        condition_variable.wait_for(lock, mib::config::publication_task_interval);
        return false;
      },
      .task_config = {.name = "MIB state publication", .stack_size_bytes = 4 * 1024},
  });
  publication_task_->start();

  motor_command_task_ = std::make_unique<espp::Task>(espp::Task::Config{
      .callback = [this](std::mutex &mutex,
                         std::condition_variable &condition_variable) -> bool {
        publish_motor_commands();

        std::unique_lock<std::mutex> lock(mutex);
        condition_variable.wait_for(lock, mib::config::motor_command_task_interval);
        return false;
      },
      .task_config = {
          .name = "MIB motor commands",
          .stack_size_bytes = 4 * 1024,
          .priority = 2,
      },
  });
  motor_command_task_->start();

  while (true) {
    std::this_thread::sleep_for(2s);
  }
}

bool MibSystem::initialize_pubsub() {
  using Reliability = espp::RtpsParticipant::Reliability;

    joystick_subscriber_ = std::make_unique<espp::Subscriber<rammp::XYTwist>>(
      mib_.rtps_participant(),
      espp::Subscriber<rammp::XYTwist>::Config{
        .topic = rammp::kJoystickXYTwist.name,
        .type_name = rammp::kJoystickXYTwist.type,
          .reliability = Reliability::BEST_EFFORT,
          .on_message = [this](const rammp::XYTwist &sample) {
            handle_joystick_message(sample);
          }});

  if (!joystick_subscriber_->is_valid()) {
    logger_.error("Failed to create MIB RTPS joystick subscriber");
    return false;
  }

  seat_command_subscriber_ = std::make_unique<espp::Subscriber<rammp::SeatCommand>>(
      mib_.rtps_participant(),
      espp::Subscriber<rammp::SeatCommand>::Config{
          .topic = rammp::kJoystickSeatCommand.name,
          .type_name = rammp::kJoystickSeatCommand.type,
          .reliability = Reliability::BEST_EFFORT,
          .on_message = [this](const rammp::SeatCommand &command) {
            handle_seat_control_command(command);
          }});

  if (!seat_command_subscriber_->is_valid()) {
    logger_.error("Failed to create MIB RTPS seat command subscriber");
    return false;
  }

  drive_command_subscriber_ = std::make_unique<espp::Subscriber<rammp::DriveCommand>>(
      mib_.rtps_participant(),
      espp::Subscriber<rammp::DriveCommand>::Config{
          .topic = rammp::kJoystickDriveCommand.name,
          .type_name = rammp::kJoystickDriveCommand.type,
          .reliability = Reliability::BEST_EFFORT,
          .on_message = [this](const rammp::DriveCommand &command) {
            handle_drive_command(command);
          }});

  if (!drive_command_subscriber_->is_valid()) {
    logger_.error("Failed to create MIB RTPS drive command subscriber");
    return false;
  }

  status_publisher_ = std::make_unique<espp::Publisher<MIB::MibStatus>>(
      mib_.rtps_participant(),
      espp::Publisher<MIB::MibStatus>::Config{
          .topic = MIB::kMibStatus.name,
          .type_name = MIB::kMibStatus.type,
          .reliability = Reliability::BEST_EFFORT});

  if (!status_publisher_->is_valid()) {
    logger_.error("Failed to create MIB status publisher");
    return false;
  }

  left_motor_publisher_ = std::make_unique<espp::Publisher<rammp::MotorCommand>>(
      mib_.rtps_participant(),
      espp::Publisher<rammp::MotorCommand>::Config{
          .topic = rammp::axis(rammp::AxisId::DRIVE_LEFT).command.name,
          .type_name = rammp::axis(rammp::AxisId::DRIVE_LEFT).command.type,
          .reliability = Reliability::BEST_EFFORT});

  right_motor_publisher_ = std::make_unique<espp::Publisher<rammp::MotorCommand>>(
      mib_.rtps_participant(),
      espp::Publisher<rammp::MotorCommand>::Config{
          .topic = rammp::axis(rammp::AxisId::DRIVE_RIGHT).command.name,
          .type_name = rammp::axis(rammp::AxisId::DRIVE_RIGHT).command.type,
          .reliability = Reliability::BEST_EFFORT});

  if (!left_motor_publisher_->is_valid() || !right_motor_publisher_->is_valid()) {
    logger_.error("Failed to create drive motor command publisher");
    return false;
  }

  return true;
}

void MibSystem::handle_joystick_message(const rammp::XYTwist &sample) {
  {
    std::lock_guard<std::mutex> lock(joystick_mutex_);
    latest_joystick_ = sample;
  }

  logger_.debug("Joystick x={} y={} twist={} buttons={}", sample.x, sample.y, sample.twist,
                static_cast<uint32_t>(sample.buttons));

  switch (state_.load()) {
  case SystemState::DRIVE_ENABLED:
    // The MIB maps y to linear velocity and x to angular velocity. Joystick x is
    // positive to the right, but positive angular velocity is counter-clockwise
    // (a left turn), so x is negated.
    drive_controller_.set_target(sample.y, -sample.x);
    break;
  case SystemState::AUTONOMOUS:
    // Touching the joystick ends autonomy and hands control back to the joystick.
    if (joystick_touched(sample)) {
      leave_autonomy("joystick touched in AUTONOMOUS");
    }
    break;
  case SystemState::SHARED:
    // The joystick commands nothing in shared mode, even when touched. Drive input
    // comes from the Jetson through handle_autonomy_setpoint().
    break;
  case SystemState::CALIBRATE:
    // The joystick is ignored while calibrating.
    break;
  case SystemState::INIT:
  case SystemState::IDLE:
  case SystemState::ESTOP:
  case SystemState::ERROR:
    drive_controller_.stop();
    break;
  }
}

bool MibSystem::enter_autonomy(SystemState autonomy_state, const char *name) {
  const auto current_state = state_.load();
  if (current_state != SystemState::IDLE && current_state != SystemState::DRIVE_ENABLED) {
    logger_.warn("Ignoring {} request outside IDLE or DRIVE_ENABLED", name);
    return false;
  }
  drive_controller_.stop();
  state_.store(autonomy_state);
  logger_.info("Entering {} - placeholder", name);
  return true;
}

void MibSystem::leave_autonomy(const char *reason) {
  drive_controller_.stop();
  state_.store(SystemState::IDLE);
  logger_.info("Autonomy ended ({}) - back to IDLE", reason);
}

void MibSystem::handle_autonomous_request() { enter_autonomy(SystemState::AUTONOMOUS, "AUTONOMOUS"); }

void MibSystem::handle_autonomous_stop() {
  if (state_.load() != SystemState::AUTONOMOUS) {
    logger_.warn("Ignoring autonomous stop outside AUTONOMOUS state");
    return;
  }
  leave_autonomy("AutonomousStop");
}

void MibSystem::handle_shared_request() { enter_autonomy(SystemState::SHARED, "SHARED"); }

void MibSystem::handle_shared_stop() {
  if (state_.load() != SystemState::SHARED) {
    logger_.warn("Ignoring shared stop outside SHARED state");
    return;
  }
  leave_autonomy("SharedStop");
}

void MibSystem::handle_autonomy_setpoint(float linear_mps, float angular_radps) {
  switch (state_.load()) {
  case SystemState::AUTONOMOUS:
  case SystemState::SHARED:
    // Placeholder: the Jetson setpoint drives the base in both autonomy states.
    logger_.debug("Autonomy setpoint linear={} angular={} - placeholder", linear_mps,
                  angular_radps);
    break;
  default:
    break;
  }
}

void MibSystem::handle_seat_control_command(const rammp::SeatCommand &command) {
  if (state_.load() != SystemState::IDLE) {
    logger_.warn("Ignoring seat control command while system is not IDLE");
    return;
  }

  std::lock_guard<std::mutex> lock(seat_state_mutex_);
  switch (command.axis) {
  case rammp::SeatAxis::FRONT_BACK_TILT:
    seat_state_.front_back_tilt = command.target;
    break;
  case rammp::SeatAxis::LATERAL_TILT:
    seat_state_.lateral_tilt = command.target;
    break;
  case rammp::SeatAxis::ELEVATION:
    seat_state_.elevation = command.target;
    break;
  case rammp::SeatAxis::TRANSLATION:
    seat_state_.translation = command.target;
    break;
  default:
    logger_.warn("Ignoring unknown seat axis {}", static_cast<uint8_t>(command.axis));
    return;
  }

  drive_controller_.set_seat_target(static_cast<uint8_t>(command.axis), command.target);
  logger_.info("Seat control command applied: axis={} target={} - placeholder",
               static_cast<uint8_t>(command.axis), command.target);
}

void MibSystem::handle_drive_command(const rammp::DriveCommand &command) {
  const auto current_state = state_.load();
  if (drive_enable_requested(command)) {
    // In DRIVE_ENABLED this is profile_change_requested(): same request, profile only.
    if (current_state != SystemState::IDLE && current_state != SystemState::DRIVE_ENABLED) {
      logger_.warn("Ignoring drive enable command outside IDLE state");
      return;
    }
    if (!drive_controller_.update_drive_profile(command.profile)) {
      logger_.error("Failed to update drive profile");
      return;
    }
    drive_profile_.store(command.profile);
    if (current_state == SystemState::IDLE) {
      state_.store(SystemState::DRIVE_ENABLED);
      logger_.info("Drive enabled with profile={}", static_cast<uint8_t>(command.profile));
    } else {
      logger_.info("Drive profile updated to {}", static_cast<uint8_t>(command.profile));
    }
  } else if (drive_disable_requested(command)) {
    if (current_state != SystemState::DRIVE_ENABLED && current_state != SystemState::AUTONOMOUS &&
        current_state != SystemState::SHARED) {
      logger_.warn("Ignoring drive disable command outside a drive state");
      return;
    }
    drive_controller_.stop();
    state_.store(SystemState::IDLE);
    logger_.info("Drive disabled");
  } else {
    logger_.warn("Ignoring unknown drive request {}", static_cast<uint8_t>(command.request));
  }
}

void MibSystem::handle_motor_status() {
  logger_.info("Motor status received - placeholder");
}

void MibSystem::publish_system_state() {
  MIB::MibStatus status{};
  status.activeProfile = drive_profile_.load();
  switch (state_.load()) {
  case SystemState::INIT:
  case SystemState::CALIBRATE:  // TODO: Handle calibration state separately if needed.
    status.systemState = MIB::MibSystemState::INITIALIZING;
    break;
  case SystemState::IDLE:
    status.systemState = MIB::MibSystemState::IDLE;
    break;
  case SystemState::DRIVE_ENABLED:
    status.systemState = MIB::MibSystemState::ENABLED;
    break;
  case SystemState::AUTONOMOUS:
    status.systemState = MIB::MibSystemState::ENABLED;
    status.status_text = "AUTONOMOUS";
    break;
  case SystemState::SHARED:
    status.systemState = MIB::MibSystemState::ENABLED;
    status.status_text = "SHARED";
    break;
  case SystemState::ESTOP:
    status.systemState = MIB::MibSystemState::ERROR;
    status.status_text = "ESTOP";
    status.error_message = "Emergency stop";
    break;
  case SystemState::ERROR:
    status.systemState = MIB::MibSystemState::ERROR;
    status.error_message = "MIB system error";
    break;
  }

  {
    std::lock_guard<std::mutex> lock(seat_state_mutex_);
    status.currentSeatState = seat_state_;
  }
  status.seq = ++status_sequence_;
  if (!status_publisher_->publish(status)) {
    logger_.warn("Failed to publish MIB status");
  }
}

void MibSystem::publish_seat_state() {
  logger_.debug("Publishing seat state - placeholder");
}

void MibSystem::log_drive_status() {
  rammp::XYTwist joystick{};
  {
    std::lock_guard<std::mutex> lock(joystick_mutex_);
    joystick = latest_joystick_;
  }

  const auto current_state = state_.load();
  const char *state_name = "UNKNOWN";
  switch (current_state) {
  case SystemState::INIT:
    state_name = "INIT";
    break;
  case SystemState::IDLE:
    state_name = "IDLE";
    break;
  case SystemState::CALIBRATE:
    state_name = "CALIBRATE";
    break;
  case SystemState::DRIVE_ENABLED:
    state_name = "DRIVE_ENABLED";
    break;
  case SystemState::AUTONOMOUS:
    state_name = "AUTONOMOUS";
    break;
  case SystemState::SHARED:
    state_name = "SHARED";
    break;
  case SystemState::ESTOP:
    state_name = "ESTOP";
    break;
  case SystemState::ERROR:
    state_name = "ERROR";
    break;
  }

  float left_rad_per_second = 0.0f;
  float right_rad_per_second = 0.0f;
  if (current_state == SystemState::DRIVE_ENABLED) {
    const auto wheel_speeds = drive_controller_.wheel_speeds();
    constexpr float kRpmToRadPerSecond = 2.0f * 3.14159265358979323846f / 60.0f;
    left_rad_per_second = wheel_speeds.left_rpm * kRpmToRadPerSecond;
    right_rad_per_second = wheel_speeds.right_rpm * kRpmToRadPerSecond;
  }

  logger_.info("Drive status: state={} joystick x={} y={} twist={} motor left={} rad/s right={} rad/s",
               state_name, joystick.x, joystick.y, joystick.twist, left_rad_per_second,
               right_rad_per_second);
}

void MibSystem::publish_motor_commands() {
  rammp::MotorCommand command{};
  command.seq = ++motor_command_sequence_;
  command.requested_state = state_.load() == SystemState::DRIVE_ENABLED
                                ? rammp::RequestedState::ARMED
                                : rammp::RequestedState::DISARMED;
  command.mode = rammp::ControlMode::VELOCITY;

  if (state_.load() == SystemState::DRIVE_ENABLED) {
    const auto wheel_speeds = drive_controller_.wheel_speeds();
    constexpr float kRpmToRadPerSecond = 2.0f * 3.14159265358979323846f / 60.0f;
    command.velocity = wheel_speeds.left_rpm * kRpmToRadPerSecond;
    if (!left_motor_publisher_->publish(command)) {
      logger_.warn("Failed to publish left motor command");
    }

    command.velocity = wheel_speeds.right_rpm * kRpmToRadPerSecond;
    if (!right_motor_publisher_->publish(command)) {
      logger_.warn("Failed to publish right motor command");
    }
    return;
  }

  command.velocity = 0.0f;
  if (!left_motor_publisher_->publish(command)) {
    logger_.warn("Failed to publish left motor stop command");
  }
  if (!right_motor_publisher_->publish(command)) {
    logger_.warn("Failed to publish right motor stop command");
  }
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

    // RTPS discovery multicast fails on an interface with no carrier.
    logger_.info("Waiting for Ethernet link...");
    while (!mib_.ethernet_connected()) {
      std::this_thread::sleep_for(100ms);
    }

    logger_.info("ESP32-P4-ETH init complete");
    logger_.info("Board Ethernet status: connected={}", mib_.ethernet_connected());
    logger_.info("Board IP: {}", mib_.ethernet_ip_string());
    if (!mib_.init_rtps()) {
      logger_.error("Failed to initialize RTPS");
      state_.store(SystemState::ERROR);
      return;
    }
    state_.store(initialize_pubsub() ? SystemState::IDLE : SystemState::ERROR);
    break;
  case SystemState::IDLE:
    logger_.debug("System state: IDLE - running idle placeholder");
    break;
  case SystemState::CALIBRATE:
    // Placeholder routine: completes at once. A real calibration stays here until done.
    logger_.info("System state: CALIBRATE - calibration placeholder done, returning to IDLE");
    state_.store(SystemState::IDLE);
    break;
  case SystemState::DRIVE_ENABLED:
    logger_.debug("System state: DRIVE_ENABLED - running drive placeholder");
    break;
  case SystemState::AUTONOMOUS:
    logger_.debug("System state: AUTONOMOUS - running autonomous placeholder");
    break;
  case SystemState::SHARED:
    logger_.debug("System state: SHARED - running shared control placeholder");
    break;
  case SystemState::ESTOP:
    run_estop_step();
    break;
  case SystemState::ERROR:
    run_error_step();
    break;
  }
}

void MibSystem::run_error_step() {
  const auto now = std::chrono::steady_clock::now();
  if (error_entered_at_ == std::chrono::steady_clock::time_point{}) {
    error_entered_at_ = now;
    logger_.error("System state: ERROR - rebooting in {} s to retry bring-up",
                  std::chrono::duration_cast<std::chrono::seconds>(
                      mib::config::error_reboot_delay)
                      .count());
    return;
  }
  if (now - error_entered_at_ >= mib::config::error_reboot_delay) {
    logger_.error("Rebooting MIB");
    esp_restart();
  }
}

void MibSystem::enter_estop() {
  const auto current_state = state_.load();
  if (current_state == SystemState::INIT || current_state == SystemState::ERROR) {
    logger_.warn("Ignoring e-stop while system is in INIT or ERROR");
    return;
  }

  drive_controller_.stop();
  {
    std::lock_guard<std::mutex> lock(estop_mutex_);
    estop_entered_at_ = std::chrono::steady_clock::now();
  }
  state_.store(SystemState::ESTOP);
  logger_.warn("E-stop: entering ESTOP, back to IDLE after {} ms",
               std::chrono::duration_cast<std::chrono::milliseconds>(
                   mib::config::estop_hold_duration)
                   .count());
}

void MibSystem::enter_calibrate() {
  if (state_.load() != SystemState::IDLE) {
    logger_.warn("Ignoring calibrate request outside IDLE state");
    return;
  }
  drive_controller_.stop();
  state_.store(SystemState::CALIBRATE);
  logger_.info("Entering CALIBRATE - placeholder");
}

void MibSystem::run_estop_step() {
  {
    std::lock_guard<std::mutex> lock(estop_mutex_);
    if (std::chrono::steady_clock::now() - estop_entered_at_ < mib::config::estop_hold_duration) {
      return;
    }
  }
  state_.store(SystemState::IDLE);
  logger_.info("E-stop hold elapsed - back to IDLE");
}
