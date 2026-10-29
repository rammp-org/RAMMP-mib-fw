#include "mib_system.hpp"

#include <chrono>
#include <condition_variable>
#include <mutex>
#include <span>
#include <string_view>
#include <thread>

#include "MIBconfig.hpp"

using namespace std::chrono_literals;

// The RTPS endpoint pool in sdkconfig.defaults (the same checks as #45): a MotorCommand and a
// MotorTuning writer and a MotorState reader per axis-table board, plus the fixed endpoints
// below. A row added to the axis table needs the limits raised; these fail the build until
// they are. An sdkconfig made before the limits were raised keeps its old values: delete it.
namespace {
/// Fixed writers: MibStatus, estop-core's EStop and SPDP.
constexpr size_t kFixedWriters = 3;
/// Fixed readers: the joystick's XYTwist, SeatCommand and DriveCommand, estop-core's EStop
/// and SPDP.
constexpr size_t kFixedReaders = 5;
/// SEDP's publication and subscription endpoints: in the per-participant limits only.
constexpr size_t kSedpEndpoints = 2;
constexpr size_t kWriters = 2 * rammp::kAxisCount + kFixedWriters;
constexpr size_t kReaders = rammp::kAxisCount + kFixedReaders;
/// No watched nodes: estop-core's default watches the HMI as a deadman.
constexpr std::string_view kNoWatchedNodes[] = {""};
} // namespace

static_assert(CONFIG_RTPS_LIMIT_NUM_STATELESS_WRITERS >= kWriters,
              "raise CONFIG_RTPS_LIMIT_NUM_STATELESS_WRITERS with the axis table");
static_assert(CONFIG_RTPS_LIMIT_NUM_STATELESS_READERS >= kReaders,
              "raise CONFIG_RTPS_LIMIT_NUM_STATELESS_READERS with the axis table");
static_assert(CONFIG_RTPS_LIMIT_MAX_NUM_READER_CALLBACKS >= kReaders,
              "raise CONFIG_RTPS_LIMIT_MAX_NUM_READER_CALLBACKS with the axis table");
static_assert(CONFIG_RTPS_LIMIT_NUM_WRITERS_PER_PARTICIPANT >= kWriters + kSedpEndpoints,
              "raise CONFIG_RTPS_LIMIT_NUM_WRITERS_PER_PARTICIPANT with the axis table");
static_assert(CONFIG_RTPS_LIMIT_NUM_READERS_PER_PARTICIPANT >= kReaders + kSedpEndpoints,
              "raise CONFIG_RTPS_LIMIT_NUM_READERS_PER_PARTICIPANT with the axis table");

MibSystem::MibSystem()
    : BaseComponent("RAMMP_MIB", espp::Logger::Verbosity::INFO)
    , mib_(mib::bsp::MIB::instance())
    , drive_controller_()
    , latch_({.watched = mib::config::estop_watch_hmi
                             ? std::span<const std::string_view>(estop::kDefaultWatched)
                             : std::span<const std::string_view>(kNoWatchedNodes, 0),
              .clear_input = {.gpio_num = mib::config::estop_clear_gpio}}) {}

void MibSystem::start() {
  latch_.start(); // first: it starts latched, so nothing moves until a clear succeeds
  start_service_link(); // before INIT, which waits on the Ethernet link
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

  // Every axis in the table: command and tuning out, state in. Axes the MIB does not
  // drive still get DISARMED commands, so their boards hold no WATCHDOG fault.
  for (size_t i = 0; i < rammp::kAxisCount; i++) {
    const auto &spec = rammp::kAxes[i];
    auto &axis = axes_[i];
    axis.command_publisher = std::make_unique<espp::Publisher<rammp::MotorCommand>>(
        mib_.rtps_participant(),
        espp::Publisher<rammp::MotorCommand>::Config{.topic = spec.command.name,
                                                     .type_name = spec.command.type,
                                                     .reliability = Reliability::BEST_EFFORT});
    axis.tuning_publisher = std::make_unique<espp::Publisher<rammp::MotorTuning>>(
        mib_.rtps_participant(),
        espp::Publisher<rammp::MotorTuning>::Config{.topic = spec.tuning.name,
                                                    .type_name = spec.tuning.type,
                                                    .reliability = Reliability::BEST_EFFORT});
    axis.state_subscriber = std::make_unique<espp::Subscriber<rammp::MotorState>>(
        mib_.rtps_participant(), espp::Subscriber<rammp::MotorState>::Config{
                                     .topic = spec.state.name,
                                     .type_name = spec.state.type,
                                     .reliability = Reliability::BEST_EFFORT,
                                     .on_message = [this, i](const rammp::MotorState &state) {
                                       std::lock_guard<std::mutex> lock(axes_mutex_);
                                       axes_[i].state = state;
                                       axes_[i].state_time = std::chrono::steady_clock::now();
                                       axes_[i].state_seen = true;
                                     }});
    if (!axis.command_publisher->is_valid() || !axis.tuning_publisher->is_valid() ||
        !axis.state_subscriber->is_valid()) {
      logger_.error("Failed to create the RTPS endpoints for axis {}", spec.segment);
      return false;
    }
  }

  endpoints_ready_.store(true);
  return true;
}

void MibSystem::handle_joystick_message(const rammp::XYTwist &sample) {
  {
    std::lock_guard<std::mutex> lock(joystick_mutex_);
    latest_joystick_ = sample;
  }

  logger_.debug("Joystick x={} y={} twist={} buttons={}", sample.x, sample.y, sample.twist,
                static_cast<uint32_t>(sample.buttons));

  {
    // The control panel's stick, while it is live, drives instead of the HMI's.
    std::lock_guard<std::mutex> lock(joystick_mutex_);
    if (std::chrono::steady_clock::now() < panel_stick_until_) {
      return;
    }
  }
  apply_stick(sample.x, sample.y);
}

void MibSystem::apply_stick(float x, float y) {
  switch (state_.load()) {
  case SystemState::DRIVE_ENABLED:
    // The MIB maps y to linear velocity and x to angular velocity.
    drive_controller_.set_target(y, x);
    break;
  case SystemState::INIT:
  case SystemState::IDLE:
  case SystemState::CALIBRATE:
  case SystemState::ERROR:
    drive_controller_.stop();
    break;
  }
}

bool MibSystem::handle_seat_control_command(const rammp::SeatCommand &command) {
  if (state_.load() != SystemState::IDLE) {
    logger_.warn("Ignoring seat control command while system is not IDLE");
    return false;
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
    return false;
  }

  drive_controller_.set_seat_target(static_cast<uint8_t>(command.axis), command.target);
  logger_.info("Seat control command applied: axis={} target={} - placeholder",
               static_cast<uint8_t>(command.axis), command.target);
  return true;
}

bool MibSystem::handle_drive_command(const rammp::DriveCommand &command) {
  const auto current_state = state_.load();
  switch (command.request) {
  case rammp::DriveRequest::ENABLE:
    if (current_state != SystemState::IDLE && current_state != SystemState::DRIVE_ENABLED) {
      logger_.warn("Ignoring drive enable command outside IDLE state");
      return false;
    }
    if (any_override_active()) {
      logger_.warn("Ignoring drive enable while the control panel overrides a motor");
      return false;
    }
    if (ota_active_.load()) {
      logger_.warn("Ignoring drive enable during a firmware update");
      return false;
    }
    if (!drive_controller_.update_drive_profile(command.profile)) {
      logger_.error("Failed to update drive profile");
      return false;
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
      return false;
    }
    drive_controller_.stop();
    state_.store(SystemState::IDLE);
    logger_.info("Drive disabled");
    break;
  default:
    logger_.warn("Ignoring unknown drive request {}", static_cast<uint8_t>(command.request));
    return false;
  }
  return true;
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
  if (!endpoints_ready_.load()) {
    return;
  }
  // The e-stop puts state_ in ERROR; latch_.is_active() is a backstop until #37:
  // state_ has two writers, so an RTPS ENABLE can overwrite ERROR for one state step.
  const auto state = state_.load();
  const bool estop = latch_.is_active();
  const bool drive = state == SystemState::DRIVE_ENABLED && !estop;
  // Panel overrides run only while the chair is IDLE: never alongside the stick.
  const bool service = state == SystemState::IDLE && !estop;
  const auto wheel_speeds = drive_controller_.wheel_speeds();
  constexpr float kRpmToRadPerSecond = 2.0f * 3.14159265358979323846f / 60.0f;
  const auto now = std::chrono::steady_clock::now();
  const uint8_t seq = ++motor_command_sequence_;

  for (size_t i = 0; i < rammp::kAxisCount; i++) {
    const auto id = rammp::kAxes[i].id;
    rammp::MotorCommand command{};
    bool tuning_due = false;
    rammp::MotorTuning tuning{};
    {
      std::lock_guard<std::mutex> lock(axes_mutex_);
      auto &axis = axes_[i];
      if (service && now < axis.override_until) {
        command = axis.override_command;
      } else if (drive && (id == rammp::AxisId::DRIVE_LEFT || id == rammp::AxisId::DRIVE_RIGHT)) {
        command.requested_state = rammp::RequestedState::ARMED;
        command.mode = rammp::ControlMode::VELOCITY;
        command.velocity =
            (id == rammp::AxisId::DRIVE_LEFT ? wheel_speeds.left_rpm : wheel_speeds.right_rpm) *
            kRpmToRadPerSecond;
      } else {
        command.requested_state = rammp::RequestedState::DISARMED;
        command.mode = rammp::ControlMode::VELOCITY;
      }
      command.seq = seq;
      command.clear_fault_req_id = axis.clear_fault_req;
      axis.last_command = command;
      // Tuning is a one-shot by req id over best-effort transport: resend until echoed.
      if (axis.tuning.tuning_req_id != 0 && now < axis.tuning_until &&
          !(axis.state_seen && axis.state.tuning_ack == axis.tuning.tuning_req_id)) {
        tuning_due = true;
        tuning = axis.tuning;
      }
    }
    if (!axes_[i].command_publisher->publish(command)) {
      logger_.warn("Failed to publish {} motor command", rammp::kAxes[i].segment);
    }
    if (tuning_due && !axes_[i].tuning_publisher->publish(tuning)) {
      logger_.warn("Failed to publish {} motor tuning", rammp::kAxes[i].segment);
    }
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

    // The TCP/IP stack is up: open the panel's Ethernet port (it listens without a link).
    if (service_) {
      service_->start_tcp();
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
    if (!mib::config::estop_watch_hmi) {
      // Bench mode (MIBconfig.hpp): no HMI deadman, so clear the boot latch here.
      logger_.warn("E-stop: not watching the HMI (estop_watch_hmi = false); clearing the boot "
                   "latch");
      latch_.start_clear_attempt();
    }
    if (initialize_pubsub()) {
      state_.store(SystemState::IDLE);
      // RTPS and the endpoints are up: a new image that got this far is a keeper.
      if (service_) {
        service_->confirm_image_after(std::chrono::seconds{20});
      }
    } else {
      state_.store(SystemState::ERROR);
    }
    break;
  case SystemState::IDLE:
    logger_.debug("System state: IDLE - running idle placeholder");
    break;
  case SystemState::CALIBRATE:
    logger_.debug("System state: CALIBRATE - running calibration placeholder");
    break;
  case SystemState::DRIVE_ENABLED: {
    logger_.debug("System state: DRIVE_ENABLED - running drive placeholder");
    // A panel stick that went quiet (tab closed, cable pulled) stops the chair.
    bool lapsed = false;
    {
      std::lock_guard<std::mutex> lock(joystick_mutex_);
      if (panel_stick_until_.time_since_epoch().count() != 0 &&
          std::chrono::steady_clock::now() >= panel_stick_until_) {
        panel_stick_until_ = {};
        lapsed = true;
      }
    }
    if (lapsed) {
      drive_controller_.stop();
      logger_.warn("Control panel stick went quiet: stopping");
    }
    break;
  }
  case SystemState::ERROR:
    if (estop_error_.load() && !latch_.is_active()) {
      estop_error_.store(false);
      state_.store(SystemState::IDLE);
      break;
    }
    // Once per entry into ERROR: this runs every 20 ms.
    if (!error_reported_.exchange(true)) {
      if (estop_error_.load()) {
        logger_.error("System state: ERROR - e-stop latched (cause: {}); clear it from the HMI "
                      "or the control panel",
                      latch_.status().cause);
      } else {
        logger_.error("System state: ERROR - not operational; see the first error above");
      }
    }
    break;
  }
  if (state_.load() != SystemState::ERROR) {
    error_reported_.store(false);
  }
}