#include "mib_system.hpp"

#include <chrono>
#include <cmath>
#include <condition_variable>
#include <mutex>
#include <span>
#include <string_view>
#include <thread>

#include "MIBconfig.hpp"
#include "sdkconfig.h"

using namespace std::chrono_literals;

// The RTPS endpoint pool in sdkconfig.defaults: a MotorCommand and a MotorTuning writer and a
// MotorState reader per axis-table board, plus the fixed endpoints below. A row added to the
// axis table needs the limits raised; these checks fail the build until they are. An sdkconfig
// made before the limits were raised keeps its old values: delete it.
namespace {
/// Fixed writers: MibStatus, Diagnostics, Motors, estop-core's EStop and SPDP.
constexpr size_t kFixedWriters = 5;
/// Fixed readers: the joystick's XYTwist, SeatCommand, DriveCommand and MotorJog,
/// estop-core's EStop and SPDP.
constexpr size_t kFixedReaders = 6;
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

  if (state_.load() == SystemState::ERROR) {
    return;
  }

  publication_task_ = std::make_unique<espp::Task>(espp::Task::Config{
      .callback = [this](std::mutex &mutex,
                         std::condition_variable &condition_variable) -> bool {
        publish_system_state();
        publish_seat_state();
        publish_diagnostics();
        log_drive_status();

        std::unique_lock<std::mutex> lock(mutex);
        condition_variable.wait_for(lock, mib::config::publication_task_interval);
        return false;
      },
      // 6 KB: also builds and publishes Diagnostics.
      .task_config = {.name = "MIB state publication", .stack_size_bytes = 6 * 1024},
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
          .topic = rammp::kDriveCommand.name,
          .type_name = rammp::kDriveCommand.type,
          .reliability = Reliability::BEST_EFFORT,
          .on_message = [this](const rammp::DriveCommand &command) {
            handle_drive_command(command);
          }});

  if (!drive_command_subscriber_->is_valid()) {
    logger_.error("Failed to create MIB RTPS drive command subscriber");
    return false;
  }

  status_publisher_ = std::make_unique<espp::Publisher<rammp::MibStatus>>(
      mib_.rtps_participant(),
      espp::Publisher<rammp::MibStatus>::Config{
          .topic = rammp::kMibStatus.name,
          .type_name = rammp::kMibStatus.type,
          .reliability = Reliability::BEST_EFFORT});

  if (!status_publisher_->is_valid()) {
    logger_.error("Failed to create MIB status publisher");
    return false;
  }

  diagnostics_publisher_ = std::make_unique<espp::Publisher<rammp::Diagnostics>>(
      mib_.rtps_participant(),
      espp::Publisher<rammp::Diagnostics>::Config{.topic = rammp::kMcbDiagnostics.name,
                                                  .type_name = rammp::kMcbDiagnostics.type,
                                                  .reliability = Reliability::BEST_EFFORT});

  if (!diagnostics_publisher_->is_valid()) {
    logger_.error("Failed to create MIB diagnostics publisher");
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
    // The control panel's gains (MotorTuning), resent by publish_motor_commands().
    axes_[i].tuning_publisher = std::make_unique<espp::Publisher<rammp::MotorTuning>>(
        mib_.rtps_participant(),
        espp::Publisher<rammp::MotorTuning>::Config{.topic = axis.tuning.name,
                                                    .type_name = axis.tuning.type,
                                                    .reliability = Reliability::BEST_EFFORT});
    if (!motor_publishers_[i]->is_valid() || !motor_state_subscribers_[i]->is_valid() ||
        !axes_[i].tuning_publisher->is_valid()) {
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
  apply_stick(sample);
}

void MibSystem::apply_stick(const rammp::XYTwist &sample) {
  switch (state_.load()) {
  case SystemState::DRIVE_ENABLED:
    // The MIB maps y to linear velocity and x to angular velocity.
    drive_controller_.set_target(sample.y, sample.x);
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
  if (drive_enable_requested(command)) {
    // In DRIVE_ENABLED this is profile_change_requested(): same request, profile only.
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
  } else if (drive_disable_requested(command)) {
    // DISABLE with push_assist asks for PUSH_ASSIST (rammp-rtps#18), which the MIB does not
    // have yet: the drive is disabled as asked, and the joystick sees IDLE, not PUSH_ASSIST.
    if (command.push_assist) {
      logger_.warn("Push assist requested: not implemented on the MIB, disabling drive only");
    }
    if (current_state != SystemState::DRIVE_ENABLED && current_state != SystemState::AUTONOMOUS &&
        current_state != SystemState::SHARED) {
      logger_.warn("Ignoring drive disable command outside a drive state");
      return false;
    }
    drive_controller_.stop();
    state_.store(SystemState::IDLE);
    logger_.info("Drive disabled");
  } else if (autonomy_requested(command)) {
    // The Jetson's request (rammp-rtps#18): AUTONOMOUS or SHARED from IDLE or DRIVE_ENABLED,
    // driving with the profile it asked for. enter_autonomy() stops the drive first.
    if (current_state != SystemState::IDLE && current_state != SystemState::DRIVE_ENABLED) {
      logger_.warn("Ignoring autonomy request outside IDLE or DRIVE_ENABLED");
      return false;
    }
    if (any_override_active()) {
      logger_.warn("Ignoring autonomy request while the control panel overrides a motor");
      return false;
    }
    if (ota_active_.load()) {
      logger_.warn("Ignoring autonomy request during a firmware update");
      return false;
    }
    if (!drive_controller_.update_drive_profile(command.profile)) {
      logger_.error("Failed to update drive profile");
      return false;
    }
    drive_profile_.store(command.profile);
    if (command.request == rammp::DriveRequest::AUTONOMOUS) {
      return enter_autonomy(SystemState::AUTONOMOUS, "AUTONOMOUS");
    }
    return enter_autonomy(SystemState::SHARED, "SHARED");
  } else {
    logger_.warn("Ignoring unknown drive request {}", static_cast<uint8_t>(command.request));
    return false;
  }
  return true;
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
  rammp::MibStatus status{};
  status.active_profile = drive_profile_.load();
  switch (state_.load()) {
  case SystemState::INIT:
  case SystemState::CALIBRATE:  // TODO: Handle calibration state separately if needed.
    status.system_state = rammp::MibSystemState::INITIALIZING;
    break;
  case SystemState::IDLE:
    status.system_state = rammp::MibSystemState::IDLE;
    break;
  case SystemState::DRIVE_ENABLED:
    status.system_state = rammp::MibSystemState::ENABLED;
    break;
  case SystemState::AUTONOMOUS:
    status.system_state = rammp::MibSystemState::AUTONOMOUS;
    break;
  case SystemState::SHARED:
    status.system_state = rammp::MibSystemState::SHARED;
    break;
  case SystemState::ESTOP:
    status.system_state = rammp::MibSystemState::ESTOP;
    status.error_message = "Emergency stop";
    break;
  case SystemState::ERROR:
    status.system_state = rammp::MibSystemState::ERROR;
    status.error_message = "MIB system error";
    break;
  }

  {
    std::lock_guard<std::mutex> lock(seat_state_mutex_);
    status.current_seat_state = seat_state_;
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

void MibSystem::start_imu_task() {
  if (!mib_.init_imu()) {
    logger_.warn("IMU unavailable; continuing without it");
    return;
  }
  imu_reading_ok_.store(true);

  imu_task_ = std::make_unique<espp::Task>(espp::Task::Config{
      .callback = [this](std::mutex &mutex, std::condition_variable &condition_variable) -> bool {
        const bool ok = update_imu();

        // Fixed-period schedule: waiting a full interval after each read would
        // stretch the period by the read time. espp::Task copies the callback
        // every iteration, so the deadline lives in a member, not the lambda.
        const auto now = std::chrono::steady_clock::now();
        imu_next_wake_ += ok ? mib::config::imu_task_interval : mib::config::imu_retry_interval;
        if (imu_next_wake_ < now) {
          imu_next_wake_ = now; // fell behind; skip the missed periods instead of bursting
        }
        std::unique_lock<std::mutex> lock(mutex);
        condition_variable.wait_until(lock, imu_next_wake_);
        return false;
      },
      // 6 KB: the IMU log line left 508 B free of 4 KB on the bench.
      .task_config = {.name = "MIB IMU", .stack_size_bytes = 6 * 1024},
  });
  imu_next_wake_ = std::chrono::steady_clock::now();
  imu_task_->start();
}

bool MibSystem::update_imu() {
  auto *imu = mib_.imu();
  std::error_code ec;
  // A BNO055 that lost power comes back in CONFIG mode and reads all zeros
  // without a bus error, so after any failed read, run init() again first.
  // Probe once before init(): init() polls the chip ID 20 times, and each
  // failed poll logs an I2C error, which floods the console while it is unplugged.
  bool ok = imu_reading_ok_.load();
  if (!ok) {
    if (imu->probe(ec)) {
      ok = imu->init(ec);
    } else {
      ec = std::make_error_code(std::errc::no_such_device);
    }
  }
  if (ok) {
    ok = imu->update(ec);
  }
  imu_reading_ok_.store(ok);

  // Logging shares the publication rate so a failing bus does not flood the console.
  const auto now = std::chrono::steady_clock::now();
  if (now - last_imu_log_ < mib::config::publication_task_interval) {
    return ok;
  }
  last_imu_log_ = now;

  if (!ok) {
    logger_.warn("IMU read failed: {}", ec.message());
    return false;
  }

  const auto data = imu->data();
  constexpr float kRadToDeg = 180.0f / 3.14159265358979323846f;
  logger_.info("IMU: heading={:.1f} roll={:.1f} pitch={:.1f} deg gyro=({:.2f}, {:.2f}, {:.2f}) "
               "rad/s accel=({:.2f}, {:.2f}, {:.2f}) m/s^2 calib sys={} gyr={} acc={} mag={}",
               data.euler.heading * kRadToDeg, data.euler.roll * kRadToDeg,
               data.euler.pitch * kRadToDeg, data.gyro_radps.x, data.gyro_radps.y,
               data.gyro_radps.z, data.accel_mps2.x, data.accel_mps2.y, data.accel_mps2.z,
               data.calibration.system, data.calibration.gyro, data.calibration.accel,
               data.calibration.mag);
  return true;
}

void MibSystem::publish_diagnostics() {
  // Raw readings are integers in 10^-decimals of the row's unit (RAMMP_DIAG_TABLE).
  auto to_raw = [](float value, rammp::DiagId id, size_t field) {
    const auto decimals = rammp::kDiagItems[static_cast<size_t>(id)].decimals[field];
    return static_cast<int32_t>(std::lround(value * std::pow(10.0f, decimals)));
  };

  rammp::Diagnostics diagnostics{};
  diagnostics.seq = ++diagnostics_sequence_;
  diagnostics.items.resize(rammp::kDiagCount);

  // A failed read leaves the last good sample in the driver; send nothing rather than stale data.
  auto *imu = mib_.imu();
  if (imu != nullptr && imu_reading_ok_.load()) {
    const auto data = imu->data();
    constexpr float kRadToDeg = 180.0f / 3.14159265358979323846f;
    diagnostics.items[static_cast<size_t>(rammp::DiagId::IMU_TILT)].values = {
        to_raw(data.euler.roll * kRadToDeg, rammp::DiagId::IMU_TILT, 0),
        to_raw(data.euler.pitch * kRadToDeg, rammp::DiagId::IMU_TILT, 1),
        to_raw(data.euler.heading * kRadToDeg, rammp::DiagId::IMU_TILT, 2),
    };
    diagnostics.items[static_cast<size_t>(rammp::DiagId::IMU_STATUS)].values = {
        to_raw(data.gyro_radps.z * kRadToDeg, rammp::DiagId::IMU_STATUS, 0),
        to_raw(data.calibration.gyro, rammp::DiagId::IMU_STATUS, 1),
        to_raw(data.temperature_c, rammp::DiagId::IMU_STATUS, 2),
    };
  }

  if (!diagnostics_publisher_->publish(diagnostics)) {
    logger_.warn("Failed to publish diagnostics");
  }
}

void MibSystem::publish_motor_commands() {
  if (!endpoints_ready_.load()) {
    return;
  }
  const auto now = std::chrono::steady_clock::now();
  const auto current_state = state_.load();
  // The e-stop puts state_ in ESTOP; latch_.is_active() is a backstop until #37:
  // state_ has two writers, so an RTPS ENABLE can overwrite ESTOP for one state step.
  const bool stopped = latch_.is_active();
  const bool drive = current_state == SystemState::DRIVE_ENABLED && !stopped;
  // Panel overrides run only while the chair is IDLE: never alongside the stick.
  const bool service = current_state == SystemState::IDLE && !stopped;

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

  // Each board's last tuning echo, to stop resending a MotorTuning it has applied.
  std::array<uint8_t, rammp::kAxisCount> tuning_acks{};
  {
    std::lock_guard<std::mutex> lock(motor_state_mutex_);
    for (size_t i = 0; i < rammp::kAxisCount; i++) {
      tuning_acks[i] = motor_states_[i].tuning_ack;
    }
  }

  const uint8_t seq = ++motor_command_sequence_;
  for (size_t i = 0; i < rammp::kAxisCount; i++) {
    const rammp::AxisId id = rammp::kAxes[i].id;
    rammp::MotorCommand command{};
    command.seq = seq;
    command.mode = rammp::ControlMode::VELOCITY;
    command.requested_state = rammp::RequestedState::DISARMED;
    bool tuning_due = false;
    rammp::MotorTuning tuning{};
    {
      std::lock_guard<std::mutex> lock(axes_mutex_);
      auto &axis = axes_[i];
      if (service && now < axis.override_until) {
        command = axis.override_command;
        command.seq = seq;
      } else if (drive && id == rammp::AxisId::DRIVE_LEFT) {
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
      command.clear_fault_req_id = axis.clear_fault_req;
      axis.last_command = command;
      // Tuning is a one-shot by req id over best-effort transport: resend until echoed.
      if (axis.tuning.tuning_req_id != 0 && now < axis.tuning_until &&
          tuning_acks[i] != axis.tuning.tuning_req_id) {
        tuning_due = true;
        tuning = axis.tuning;
      }
    }
    if (!motor_publishers_[i]->publish(command)) {
      logger_.warn("Failed to publish {} motor command", rammp::kAxes[i].label);
    }
    if (tuning_due && !axes_[i].tuning_publisher->publish(tuning)) {
      logger_.warn("Failed to publish {} motor tuning", rammp::kAxes[i].label);
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
  // E-stop: the latch has no callback, so check it every step. ESTOP while it is active.
  // Not from INIT (it enters ESTOP itself for the boot latch) or ERROR (not operational).
  const auto state = state_.load();
  if (latch_.is_active() && state != SystemState::ESTOP && state != SystemState::INIT &&
      state != SystemState::ERROR) {
    enter_estop();
  }
  switch (state_.load()) {
  case SystemState::INIT:
    logger_.info("System state: INIT - initializing MIB");
    if (!mib_.init()) {
      logger_.error("Failed to initialize ESP32-P4-ETH board");
      state_.store(SystemState::ERROR);
      return;
    }

    start_imu_task();

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
    if (!initialize_pubsub()) {
      state_.store(SystemState::ERROR);
      return;
    }
    // RTPS and the endpoints are up: a new image that got this far is a keeper.
    if (service_) {
      service_->confirm_image_after(std::chrono::seconds{20});
    }
    // The latch starts latched ("boot"): straight to ESTOP, with no hold, until it clears.
    state_.store(latch_.is_active() ? SystemState::ESTOP : SystemState::IDLE);
    break;
  case SystemState::IDLE:
    logger_.debug("System state: IDLE - running idle placeholder");
    break;
  case SystemState::CALIBRATE:
    // Placeholder routine: completes at once. A real calibration stays here until done.
    logger_.info("System state: CALIBRATE - calibration placeholder done, returning to IDLE");
    state_.store(SystemState::IDLE);
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
    // Once per entry into ERROR: this runs every 20 ms.
    if (!error_reported_.exchange(true)) {
      logger_.error("System state: ERROR - not operational; see the first error above");
    }
    break;
  }
  if (state_.load() != SystemState::ERROR) {
    error_reported_.store(false);
  }
}

void MibSystem::enter_estop() {
  const auto current_state = state_.load();
  if (current_state == SystemState::INIT || current_state == SystemState::ERROR) {
    logger_.warn("Ignoring e-stop while system is in INIT or ERROR");
    return;
  }

  drive_controller_.stop();
  estop_hold_until_ = std::chrono::steady_clock::now() + mib::config::estop_hold_duration;
  state_.store(SystemState::ESTOP);
  logger_.warn("E-stop: entering ESTOP, asking the latch to clear after {} ms",
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
  if (!latch_.is_active()) {
    state_.store(SystemState::IDLE);
    logger_.info("E-stop: latch cleared - back to IDLE (drive must be re-enabled)");
    return;
  }
  const auto now = std::chrono::steady_clock::now();
  if (now < estop_hold_until_) {
    return;
  }
  // Hold elapsed: ask the latch to clear. The latch refuses while a node's switch is held
  // or a watched node has not been heard (the boot latch, before the joystick is discovered),
  // so keep asking; the state leaves ESTOP once the latch has cleared.
  if (latch_.status().clear_attempt_running ||
      now - estop_last_clear_attempt_ < mib::config::estop_clear_retry) {
    return;
  }
  estop_last_clear_attempt_ = now;
  latch_.start_clear_attempt();
  logger_.info("E-stop: hold elapsed, asking the latch to clear");
}
