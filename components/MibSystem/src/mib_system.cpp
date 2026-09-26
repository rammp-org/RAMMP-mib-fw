#include "mib_system.hpp"

#include <chrono>
#include <condition_variable>
#include <mutex>
#include <thread>

#include "MIBconfig.hpp"

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
    return;
  }

  publication_task_ = std::make_unique<espp::Task>(espp::Task::Config{
      .callback = [this](std::mutex &mutex,
                         std::condition_variable &condition_variable) -> bool {
        publish_system_state();
        publish_seat_state();
        publish_param_state();
        publish_can_status();
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

  // Parameter writes are rare and must not be lost, so this one endpoint is reliable.
  param_set_subscriber_ = std::make_unique<espp::Subscriber<mib::ParamSet>>(
      mib_.rtps_participant(),
      espp::Subscriber<mib::ParamSet>::Config{
          .topic = RAMMP_TOPIC_JOYSTICK_PARAM_SET,
          .type_name = RAMMP_TYPE_PARAM_SET,
          .reliability = Reliability::RELIABLE,
          .on_message = [this](const mib::ParamSet &request) { handle_param_set(request); }});

  if (!param_set_subscriber_->is_valid()) {
    logger_.error("Failed to create MIB RTPS parameter subscriber");
    return false;
  }

  param_state_publisher_ = std::make_unique<espp::Publisher<mib::ParamState>>(
      mib_.rtps_participant(),
      espp::Publisher<mib::ParamState>::Config{
          .topic = RAMMP_TOPIC_MIB_PARAMS,
          .type_name = RAMMP_TYPE_PARAM_STATE,
          .reliability = Reliability::BEST_EFFORT});

  if (!param_state_publisher_->is_valid()) {
    logger_.error("Failed to create MIB parameter state publisher");
    return false;
  }

  // CAN bridge: frames from the bench tool must not be lost, so that reader is reliable.
  can_tx_subscriber_ = std::make_unique<espp::Subscriber<mib::CanFrame>>(
      mib_.rtps_participant(),
      espp::Subscriber<mib::CanFrame>::Config{
          .topic = RAMMP_TOPIC_JOYSTICK_CAN_TX,
          .type_name = RAMMP_TYPE_CAN_FRAME,
          .reliability = Reliability::RELIABLE,
          .on_message = [this](const mib::CanFrame &frame) { handle_can_tx(frame); }});

  can_rx_publisher_ = std::make_unique<espp::Publisher<mib::CanFrame>>(
      mib_.rtps_participant(),
      espp::Publisher<mib::CanFrame>::Config{.topic = RAMMP_TOPIC_MIB_CAN_RX,
                                             .type_name = RAMMP_TYPE_CAN_FRAME,
                                             .reliability = Reliability::BEST_EFFORT});

  can_status_publisher_ = std::make_unique<espp::Publisher<mib::CanStatus>>(
      mib_.rtps_participant(),
      espp::Publisher<mib::CanStatus>::Config{.topic = RAMMP_TOPIC_MIB_CAN_STATUS,
                                              .type_name = RAMMP_TYPE_CAN_STATUS,
                                              .reliability = Reliability::BEST_EFFORT});

  if (!can_tx_subscriber_->is_valid() || !can_rx_publisher_->is_valid() ||
      !can_status_publisher_->is_valid()) {
    logger_.error("Failed to create MIB CAN bridge endpoints");
    return false;
  }

  return true;
}

bool MibSystem::initialize_can_bridge() {
  twai_ = std::make_unique<espp::Twai>(espp::Twai::Config{
      .tx_gpio = mib::can_config::tx_gpio,
      .rx_gpio = mib::can_config::rx_gpio,
      .baudrate = mib::can_config::bitrate,
      .mode = espp::Twai::Mode::NORMAL,
      .tx_queue_depth = 8,
      // A frame nobody acknowledges is retried a few times, then reported as failed
      // rather than retransmitted forever and blocking the next one.
      .tx_retry_count = 3,
      .on_receive =
          [this](const espp::Twai::Message &message) {
            mib::CanFrame frame{};
            frame.seq = ++can_rx_sequence_;
            frame.flags = (message.extended ? mib::kCanFlagExtended : 0) |
                          (message.rtr ? mib::kCanFlagRtr : 0);
            frame.dlc = message.dlc;
            frame.id = message.id;
            frame.data = message.data;
            if (can_rx_publisher_->publish(frame)) {
              ++can_rx_frames_;
            } else {
              ++can_rx_dropped_;
            }
          },
      .on_error = [this](twai_error_flags_t) { ++can_bus_errors_; },
      .log_level = espp::Logger::Verbosity::WARN,
  });

  std::error_code ec;
  if (!twai_->initialize(ec)) {
    logger_.error("CAN bridge: TWAI init failed on tx={} rx={}: {}", mib::can_config::tx_gpio,
                  mib::can_config::rx_gpio, ec.message());
    twai_.reset();
    return false;
  }
  can_initialized_.store(true);
  logger_.info("CAN bridge up: tx={} rx={} {} bit/s", mib::can_config::tx_gpio,
               mib::can_config::rx_gpio, mib::can_config::bitrate);
  return true;
}

void MibSystem::handle_can_tx(const mib::CanFrame &frame) {
  if (!can_initialized_.load() || !twai_) {
    ++can_tx_failed_;
    return;
  }
  espp::Twai::Message message{};
  message.id = frame.id;
  message.extended = (frame.flags & mib::kCanFlagExtended) != 0;
  message.rtr = (frame.flags & mib::kCanFlagRtr) != 0;
  message.dlc = frame.dlc > 8 ? 8 : frame.dlc;
  message.data = frame.data;

  std::error_code ec;
  if (twai_->transmit(message, ec, /*timeout_ms=*/50)) {
    ++can_tx_ok_;
  } else {
    ++can_tx_failed_;
    logger_.debug("CAN bridge: tx of id 0x{:X} failed: {}", frame.id, ec.message());
  }
}

void MibSystem::publish_can_status() {
  mib::CanStatus status{};
  status.initialized = can_initialized_.load() ? 1 : 0;
  status.bitrate = mib::can_config::bitrate;
  status.tx_ok = can_tx_ok_.load();
  status.tx_failed = can_tx_failed_.load();
  status.rx_frames = can_rx_frames_.load();
  status.rx_dropped = can_rx_dropped_.load();
  status.bus_errors = can_bus_errors_.load();
  status.tx_gpio = static_cast<int16_t>(mib::can_config::tx_gpio);
  status.rx_gpio = static_cast<int16_t>(mib::can_config::rx_gpio);
  if (twai_) {
    status.enabled = twai_->is_enabled() ? 1 : 0;
    twai_node_status_t node_status{};
    std::error_code ec;
    if (twai_->get_status(node_status, ec)) {
      status.bus_state = static_cast<uint8_t>(node_status.state);
      status.tx_error_count = node_status.tx_error_count;
      status.rx_error_count = node_status.rx_error_count;
    }
  }
  if (!can_status_publisher_->publish(status)) {
    logger_.warn("Failed to publish CAN bridge status");
  }
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
    // The MIB maps y to linear velocity and x to angular velocity. The sign of x is a
    // runtime parameter so the turn direction can be checked on the bench.
    drive_controller_.set_target(
        sample.y, sample.x * mib::Params::instance().get(mib::ParamId::JOYSTICK_X_SIGN));
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

void MibSystem::handle_motor_status() {
  logger_.info("Motor status received - placeholder");
}

void MibSystem::handle_param_set(const mib::ParamSet &request) {
  const auto index = static_cast<size_t>(request.id);
  bool ok = mib::Params::instance().set(request.id, request.value);
  if (ok) {
    ok = drive_controller_.apply_params();
  }
  last_param_set_seq_.store(request.seq);
  last_param_set_ok_.store(ok);
  if (ok) {
    logger_.info("Param {} set to {} (seq {})",
                 index < mib::kParamCount ? mib::kParamSpecs[index].name : "?", request.value,
                 request.seq);
  } else {
    logger_.warn("Rejected param id {} value {} (seq {})", index, request.value, request.seq);
  }
}

void MibSystem::publish_param_state() {
  mib::ParamState state{};
  state.seq = ++param_state_sequence_;
  state.last_set_seq = last_param_set_seq_.load();
  state.last_set_ok = last_param_set_ok_.load() ? 1 : 0;
  state.count = static_cast<uint8_t>(mib::kParamCount);
  state.values = mib::Params::instance().snapshot();
  if (!param_state_publisher_->publish(state)) {
    logger_.warn("Failed to publish parameter state");
  }
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
    if (state_.load() == SystemState::IDLE) {
      initialize_can_bridge(); // bench bridge; the MIB runs without it
    }
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
    logger_.error("System state: ERROR - running error placeholder");
    break;
  }

}