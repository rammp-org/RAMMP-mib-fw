#include "mib_system.hpp"

#include <chrono>
#include <condition_variable>
#include <mutex>
#include <thread>

#include "MIBconfig.hpp"
#include "sdkconfig.h"

using namespace std::chrono_literals;

// The RTPS endpoint pool in sdkconfig.defaults: one MotorCommand writer and one MotorState
// reader per axis-table board, plus the fixed endpoints below. A row added to the axis
// table needs the limits raised; these checks fail the build until they are.
namespace {
/// Fixed writers: MibStatus, Motors, estop-core's EStop and SPDP.
constexpr size_t kFixedWriters = 4;
/// Fixed readers: the joystick's XYTwist, SeatCommand, DriveCommand and MotorJog,
/// estop-core's EStop and SPDP.
constexpr size_t kFixedReaders = 6;
/// SEDP's publication and subscription endpoints: in the per-participant limits only.
constexpr size_t kSedpEndpoints = 2;
} // namespace

static_assert(CONFIG_RTPS_LIMIT_NUM_STATELESS_WRITERS >= rammp::kAxisCount + kFixedWriters,
              "raise CONFIG_RTPS_LIMIT_NUM_STATELESS_WRITERS with the axis table");
static_assert(CONFIG_RTPS_LIMIT_NUM_STATELESS_READERS >= rammp::kAxisCount + kFixedReaders,
              "raise CONFIG_RTPS_LIMIT_NUM_STATELESS_READERS with the axis table");
static_assert(CONFIG_RTPS_LIMIT_MAX_NUM_READER_CALLBACKS >= rammp::kAxisCount + kFixedReaders,
              "raise CONFIG_RTPS_LIMIT_MAX_NUM_READER_CALLBACKS with the axis table");
static_assert(CONFIG_RTPS_LIMIT_NUM_WRITERS_PER_PARTICIPANT >=
                  rammp::kAxisCount + kFixedWriters + kSedpEndpoints,
              "raise CONFIG_RTPS_LIMIT_NUM_WRITERS_PER_PARTICIPANT with the axis table");
static_assert(CONFIG_RTPS_LIMIT_NUM_READERS_PER_PARTICIPANT >=
                  rammp::kAxisCount + kFixedReaders + kSedpEndpoints,
              "raise CONFIG_RTPS_LIMIT_NUM_READERS_PER_PARTICIPANT with the axis table");

MibSystem::MibSystem()
    : BaseComponent("RAMMP_MIB", espp::Logger::Verbosity::INFO),
  mib_(mib::bsp::MIB::instance()), drive_controller_(),
  latch_({.clear_input = {.gpio_num = mib::config::estop_clear_gpio}}) {}

void MibSystem::start() {
  latch_.start(); // first: it starts latched, so nothing moves until a clear succeeds
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

  if (state_.load() == SystemState::ERROR && !estop_error_.load()) {
    return;
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
        if (std::chrono::steady_clock::now() - last_motor_state_publish_timestamp_ >=
            mib::config::motor_states_publish_interval) {
          publish_motor_states();
        }

        std::unique_lock<std::mutex> lock(mutex);
        condition_variable.wait_for(lock, mib::config::motor_command_task_interval);
        return false;
      },
      .task_config = {
          .name = "MIB motor commands",
          .stack_size_bytes = 6 * 1024, // Motors serializes a string per board
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

  for (size_t i = 0; i < rammp::kAxisCount; i++) {
    const rammp::AxisSpec &axis = rammp::kAxes[i];
    motor_publishers_[i] = std::make_unique<espp::Publisher<rammp::MotorCommand>>(
        mib_.rtps_participant(),
        espp::Publisher<rammp::MotorCommand>::Config{.topic = axis.command.name,
                                                     .type_name = axis.command.type,
                                                     .reliability = Reliability::BEST_EFFORT});
    motor_state_subscribers_[i] = std::make_unique<espp::Subscriber<rammp::MotorState>>(
        mib_.rtps_participant(),
        espp::Subscriber<rammp::MotorState>::Config{
            .topic = axis.state.name,
            .type_name = axis.state.type,
            .reliability = Reliability::BEST_EFFORT,
            .on_message = [this, i](const rammp::MotorState &state) {
              handle_motor_state(i, state);
            }});
    if (!motor_publishers_[i]->is_valid() || !motor_state_subscribers_[i]->is_valid()) {
      logger_.error("Failed to create the {} motor endpoints", axis.label);
      return false;
    }
  }

  motor_states_publisher_ = std::make_unique<espp::Publisher<rammp::Motors>>(
      mib_.rtps_participant(),
      espp::Publisher<rammp::Motors>::Config{.topic = rammp::kMibMotors.name,
                                             .type_name = rammp::kMibMotors.type,
                                             .reliability = Reliability::BEST_EFFORT});
  motor_jog_subscriber_ = std::make_unique<espp::Subscriber<rammp::MotorJog>>(
      mib_.rtps_participant(),
      espp::Subscriber<rammp::MotorJog>::Config{
          .topic = rammp::kJoystickMotorJog.name,
          .type_name = rammp::kJoystickMotorJog.type,
          .reliability = Reliability::BEST_EFFORT,
          .on_message = [this](const rammp::MotorJog &jog) { handle_motor_jog(jog); }});
  if (!motor_states_publisher_->is_valid() || !motor_jog_subscriber_->is_valid()) {
    logger_.error("Failed to create the motor debug endpoints");
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
    // The MIB maps y to linear velocity and x to angular velocity.
    drive_controller_.set_target(sample.y, sample.x);
    break;
  case SystemState::INIT:
  case SystemState::IDLE:
  case SystemState::CALIBRATE:
  case SystemState::ERROR:
    drive_controller_.stop();
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
  switch (command.request) {
  case rammp::DriveRequest::ENABLE:
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
    break;
  case rammp::DriveRequest::DISABLE:
    if (current_state != SystemState::DRIVE_ENABLED) {
      logger_.warn("Ignoring drive disable command outside DRIVE_ENABLED state");
      return;
    }
    drive_controller_.stop();
    state_.store(SystemState::IDLE);
    logger_.info("Drive disabled");
    break;
  default:
    logger_.warn("Ignoring unknown drive request {}", static_cast<uint8_t>(command.request));
    return;
  }
}

void MibSystem::handle_motor_state(size_t index, const rammp::MotorState &state) {
  std::lock_guard<std::mutex> lock(motor_state_mutex_);
  motor_states_[index] = state;
  motor_state_times_[index] = std::chrono::steady_clock::now();
}

void MibSystem::handle_motor_jog(const rammp::MotorJog &jog) {
  if (jog.axis_id >= rammp::kAxisCount) {
    logger_.warn("Ignoring motor jog for unknown axis {}", jog.axis_id);
    return;
  }
  std::lock_guard<std::mutex> lock(jog_mutex_);
  latest_jog_ = jog;
  latest_jog_time_ = std::chrono::steady_clock::now();
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
  const auto now = std::chrono::steady_clock::now();
  const auto current_state = state_.load();
  // The e-stop puts state_ in ERROR; latch_.is_active() is a backstop until #37:
  // state_ has two writers, so an RTPS ENABLE can overwrite ERROR for one state step.
  const bool stopped = latch_.is_active();
  const bool drive = current_state == SystemState::DRIVE_ENABLED && !stopped;

  rammp::MotorJog jog{};
  bool jog_fresh = false;
  {
    std::lock_guard<std::mutex> lock(jog_mutex_);
    jog = latest_jog_;
    jog_fresh = now - latest_jog_time_ < mib::config::motor_jog_timeout;
  }
  // A jog moves one board, only while IDLE: never while driving, never through the e-stop.
  const bool jog_requested = jog_fresh && jog.direction != 0;
  const bool jogging = jog_requested && current_state == SystemState::IDLE && !stopped;
  if (jog_requested != jog_requested_) {
    jog_requested_ = jog_requested;
    if (jog_requested) {
      logger_.info("Motor jog {} {}: {}", rammp::kAxes[jog.axis_id].label,
                   jog.direction > 0 ? "+" : "-",
                   jogging ? "moving" : "refused (not IDLE, or the e-stop is active)");
    } else {
      logger_.info("Motor jog stopped");
    }
  }

  mib::DifferentialDrive::WheelSpeeds wheel_speeds{};
  if (drive) {
    wheel_speeds = drive_controller_.wheel_speeds();
  }
  constexpr float kRpmToRadPerSecond = 2.0f * 3.14159265358979323846f / 60.0f;

  rammp::MotorCommand command{};
  command.seq = ++motor_command_sequence_;
  command.mode = rammp::ControlMode::VELOCITY;
  for (size_t i = 0; i < rammp::kAxisCount; i++) {
    const rammp::AxisId id = rammp::kAxes[i].id;
    command.requested_state = rammp::RequestedState::DISARMED;
    command.velocity = 0.0f;
    if (drive && id == rammp::AxisId::DRIVE_LEFT) {
      command.requested_state = rammp::RequestedState::ARMED;
      command.velocity = wheel_speeds.left_rpm * kRpmToRadPerSecond;
    } else if (drive && id == rammp::AxisId::DRIVE_RIGHT) {
      command.requested_state = rammp::RequestedState::ARMED;
      command.velocity = wheel_speeds.right_rpm * kRpmToRadPerSecond;
    } else if (jogging && jog.axis_id == i) {
      command.requested_state = rammp::RequestedState::ARMED;
      command.velocity = jog.direction > 0 ? mib::config::motor_jog_velocity_radps
                                           : -mib::config::motor_jog_velocity_radps;
    }
    if (!motor_publishers_[i]->publish(command)) {
      logger_.warn("Failed to publish {} motor command", rammp::kAxes[i].label);
    }
  }
}

void MibSystem::publish_motor_states() {
  const auto now = std::chrono::steady_clock::now();
  last_motor_state_publish_timestamp_ = now;
  rammp::Motors message{};
  message.seq = ++motor_states_sequence_;
  message.motors.reserve(rammp::kAxisCount);
  {
    std::lock_guard<std::mutex> lock(motor_state_mutex_);
    for (size_t i = 0; i < rammp::kAxisCount; i++) {
      const rammp::MotorState &state = motor_states_[i];
      const bool online = motor_state_times_[i] != std::chrono::steady_clock::time_point{} &&
                          now - motor_state_times_[i] < mib::config::motor_state_timeout;
      if (online != motor_online_[i]) {
        motor_online_[i] = online;
        logger_.info("Motor {} {}", rammp::kAxes[i].label, online ? "online" : "offline");
      }
      message.motors.push_back(rammp::MotorSummary{
          .axis_id = static_cast<uint8_t>(i),
          .online = static_cast<uint8_t>(online ? 1 : 0),
          .state = state.state,
          .fault_code = state.fault_code,
          .position = state.position,
          .velocity = state.velocity,
          .label = rammp::kAxes[i].label,
      });
    }
  }
  if (!motor_states_publisher_->publish(message)) {
    logger_.warn("Failed to publish Motors");
  }
}

void MibSystem::run_state_step() {
  // E-stop: ERROR while the latch is active, then IDLE once it clears (drive must be re-enabled).
  const auto state = state_.load();
  if (latch_.is_active() && state != SystemState::INIT && state != SystemState::ERROR) {
    estop_error_.store(true);
    drive_controller_.stop();
    state_.store(SystemState::ERROR);
  }
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
    if (!latch_.attach(mib_.rtps_participant())) {
      logger_.error("Failed to attach the e-stop latch");
      state_.store(SystemState::ERROR);
      return;
    }
    state_.store(initialize_pubsub() ? SystemState::IDLE : SystemState::ERROR);
    break;
  case SystemState::IDLE:
    logger_.debug("System state: IDLE - running idle placeholder");
    break;
  case SystemState::CALIBRATE:
    logger_.debug("System state: CALIBRATE - running calibration placeholder");
    break;
  case SystemState::DRIVE_ENABLED:
    logger_.debug("System state: DRIVE_ENABLED - running drive placeholder");
    break;
  case SystemState::ERROR:
    if (estop_error_.load() && !latch_.is_active()) {
      estop_error_.store(false);
      state_.store(SystemState::IDLE);
      break;
    }
    logger_.error("System state: ERROR - running error placeholder");
    break;
  }

}