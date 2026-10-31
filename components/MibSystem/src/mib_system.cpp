#include "mib_system.hpp"

#include <chrono>
#include <cmath>
#include <condition_variable>
#include <mutex>
#include <thread>
#include <utility>

#include "MIBconfig.hpp"

using namespace std::chrono_literals;

namespace {

/// Current time as a ROS 2 stamp, from the system clock.
/// TODO(#48): nothing syncs the MIB clock to the ROS host yet, so this is time
/// since boot. Once the clock is set, the stamps are wall time with no change here.
mib::ros::builtin_interfaces::Time ros_time_now() {
  const auto since_epoch = std::chrono::system_clock::now().time_since_epoch();
  const auto seconds = std::chrono::duration_cast<std::chrono::seconds>(since_epoch);
  const auto nanoseconds =
      std::chrono::duration_cast<std::chrono::nanoseconds>(since_epoch - seconds);
  return {.sec = static_cast<int32_t>(seconds.count()),
          .nanosec = static_cast<uint32_t>(nanoseconds.count())};
}

} // namespace

MibSystem::MibSystem()
    : BaseComponent("RAMMP_MIB", espp::Logger::Verbosity::INFO)
    , mib_(mib::bsp::MIB::instance())
    , drive_controller_()
    , odometry_({
          .wheel_diameter_m = mib::config::differential_drive.wheel_diameter_m,
          .wheel_separation_m = mib::config::differential_drive.wheel_separation_m,
          .invert_left = mib::config::differential_drive.invert_left,
          .invert_right = mib::config::differential_drive.invert_right,
      }) {}

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

  odometry_task_ = std::make_unique<espp::Task>(espp::Task::Config{
      .callback = [this](std::mutex &mutex, std::condition_variable &condition_variable) -> bool {
        publish_odometry();

        std::unique_lock<std::mutex> lock(mutex);
        condition_variable.wait_for(lock, mib::config::odometry_task_interval);
        return false;
      },
      .task_config = {.name = "MIB odometry", .stack_size_bytes = 8 * 1024},
  });
  odometry_task_->start();

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

  left_motor_state_subscriber_ = std::make_unique<espp::Subscriber<rammp::MotorState>>(
      mib_.rtps_participant(), espp::Subscriber<rammp::MotorState>::Config{
                                   .topic = rammp::axis(rammp::AxisId::DRIVE_LEFT).state.name,
                                   .type_name = rammp::axis(rammp::AxisId::DRIVE_LEFT).state.type,
                                   .reliability = Reliability::BEST_EFFORT,
                                   .on_message = [this](const rammp::MotorState &state) {
                                     handle_motor_status(rammp::AxisId::DRIVE_LEFT, state);
                                   }});

  right_motor_state_subscriber_ = std::make_unique<espp::Subscriber<rammp::MotorState>>(
      mib_.rtps_participant(), espp::Subscriber<rammp::MotorState>::Config{
                                   .topic = rammp::axis(rammp::AxisId::DRIVE_RIGHT).state.name,
                                   .type_name = rammp::axis(rammp::AxisId::DRIVE_RIGHT).state.type,
                                   .reliability = Reliability::BEST_EFFORT,
                                   .on_message = [this](const rammp::MotorState &state) {
                                     handle_motor_status(rammp::AxisId::DRIVE_RIGHT, state);
                                   }});

  if (!left_motor_state_subscriber_->is_valid() || !right_motor_state_subscriber_->is_valid()) {
    logger_.error("Failed to create drive motor state subscriber");
    return false;
  }

  // RELIABLE: tf2 listeners subscribe reliable, and a best-effort writer never
  // matches a reliable reader. A reliable writer still serves best-effort readers.
  odometry_publisher_ = std::make_unique<espp::Publisher<mib::ros::nav_msgs::Odometry>>(
      mib_.rtps_participant(),
      espp::Publisher<mib::ros::nav_msgs::Odometry>::Config{.topic = mib::ros::kOdom.name,
                                                            .type_name = mib::ros::kOdom.type,
                                                            .reliability = Reliability::RELIABLE});

  tf_publisher_ = std::make_unique<espp::Publisher<mib::ros::tf2_msgs::TFMessage>>(
      mib_.rtps_participant(),
      espp::Publisher<mib::ros::tf2_msgs::TFMessage>::Config{.topic = mib::ros::kTf.name,
                                                             .type_name = mib::ros::kTf.type,
                                                             .reliability = Reliability::RELIABLE});

  if (!odometry_publisher_->is_valid() || !tf_publisher_->is_valid()) {
    logger_.error("Failed to create odometry publisher");
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

void MibSystem::handle_motor_status(rammp::AxisId axis, const rammp::MotorState &state) {
  std::lock_guard<std::mutex> lock(motor_state_mutex_);
  auto &latest = axis == rammp::AxisId::DRIVE_LEFT ? left_motor_state_ : right_motor_state_;
  // A board counts its shaft angle from boot, so a first report or a reboot
  // (uptime going backwards) restarts the angle. uptime_ms wrapping also looks
  // like a reboot, which only costs one odometry step.
  if (!latest || state.uptime_ms < latest->uptime_ms) {
    odometry_resync_ = true;
  }
  latest = state;
  motor_state_updated_ = true;
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

void MibSystem::publish_odometry() {
  rammp::MotorState left{};
  rammp::MotorState right{};
  {
    std::lock_guard<std::mutex> lock(motor_state_mutex_);
    // Publish nothing until both wheels have reported, and nothing new while
    // neither has, so ROS sees /odom go quiet rather than a stale velocity.
    if (!left_motor_state_ || !right_motor_state_ || !motor_state_updated_) {
      return;
    }
    left = *left_motor_state_;
    right = *right_motor_state_;
    if (std::exchange(odometry_resync_, false)) {
      odometry_.resync();
    }
    motor_state_updated_ = false;
  }

  odometry_.update(left.position, right.position);
  const auto &pose = odometry_.pose();
  const auto velocity = odometry_.velocity(left.velocity, right.velocity);

  // Planar base: the only rotation is yaw about z.
  const mib::ros::geometry_msgs::Quaternion orientation{
      .z = std::sin(pose.yaw_rad / 2.0),
      .w = std::cos(pose.yaw_rad / 2.0),
  };
  const mib::ros::std_msgs::Header header{
      .stamp = ros_time_now(),
      .frame_id = mib::config::odom_frame_id,
  };

  mib::ros::nav_msgs::Odometry odometry{};
  odometry.header = header;
  odometry.child_frame_id = mib::config::base_frame_id;
  odometry.pose.pose.position = {.x = pose.x_m, .y = pose.y_m};
  odometry.pose.pose.orientation = orientation;
  odometry.twist.twist.linear.x = velocity.linear_mps;
  odometry.twist.twist.angular.z = velocity.angular_radps;
  for (size_t i = 0; i < 6; i++) {
    odometry.pose.covariance[i * 7] = mib::config::odometry_pose_covariance_diagonal[i];
    odometry.twist.covariance[i * 7] = mib::config::odometry_twist_covariance_diagonal[i];
  }
  if (!odometry_publisher_->publish(odometry)) {
    logger_.warn("Failed to publish odometry");
  }

  mib::ros::tf2_msgs::TFMessage tf{};
  tf.transforms.push_back({
      .header = header,
      .child_frame_id = mib::config::base_frame_id,
      .transform = {.translation = {.x = pose.x_m, .y = pose.y_m}, .rotation = orientation},
  });
  if (!tf_publisher_->publish(tf)) {
    logger_.warn("Failed to publish odom -> base_link transform");
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