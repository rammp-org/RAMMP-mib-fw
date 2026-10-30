#pragma once

#include <atomic>
#include <memory>
#include <mutex>
#include <optional>

#include "base_component.hpp"
#include "bsp.hpp"
#include "diff_drive_odometry.hpp"
#include "drive_controller.hpp"
#include "messages.hpp"
#include "ros_messages.hpp"
#include "rtps_pubsub.hpp"
#include "task.hpp"

/// @brief Top-level MIB application: owns the system state machine and RTPS endpoints.
///
/// Brings the board up through mib::bsp::MIB, then runs two periodic tasks: a
/// fast state task that advances the system state machine, and a slower
/// publication task that reports system / seat state to the rest of the robot.
///
/// \note Construction does no hardware work; call start() to bring the system up.
class MibSystem : public espp::BaseComponent {
public:
  /// Construct the MIB system. Binds to the MIB board singleton.
  MibSystem();

  /// Bring the system up and run it.
  /// Starts the state task, waits for initialization to finish, then starts the
  /// publication, motor command, and odometry tasks.
  /// \note Blocks the calling thread for the lifetime of the application, and
  ///       returns early if initialization fails (state becomes ERROR).
  void start();

private:
  /// High-level operating state of the MIB.
  enum class SystemState {
    INIT,          ///< Bringing up the board, Ethernet, RTPS, and pub/sub.
    IDLE,          ///< Initialized and waiting; drive is disabled.
    CALIBRATE,     ///< Running a calibration routine.
    DRIVE_ENABLED, ///< Drive commands are accepted and acted on.
    ERROR,         ///< Initialization or runtime failure; not operational.
  };

  /// Create the RTPS publishers and subscribers.
  /// \note The RTPS participant must already be started.
  /// \return True if every endpoint was registered successfully.
  bool initialize_pubsub();

  /// Run one iteration of the state machine, acting on the current state.
  /// \note Runs on the state task.
  void run_state_step();

  /// Handle one joystick sample from the HMI.
  /// \param sample The received normalized stick position, buttons, and drive mode.
  /// \note Runs on an RTPS receive thread - return quickly, do not block.
  void handle_joystick_message(const rammp::XYTwist &sample);

  /// Handle a seat control command.
  /// \param command The requested seat axis and target position.
  /// \note Runs on an RTPS receive thread - return quickly, do not block.
  void handle_seat_control_command(const rammp::SeatCommand &command);

  /// Handle a drive enable or disable command.
  /// \param command The requested drive state and response profile.
  /// \note Runs on an RTPS receive thread - return quickly, do not block.
  void handle_drive_command(const rammp::DriveCommand &command);

  /// Handle a drive motor controller state report.
  /// \param axis The drive axis whose state topic the report arrived on.
  /// \param state The reported shaft position, velocity, and board status.
  /// \note Runs on an RTPS receive thread - return quickly, do not block.
  void handle_motor_status(rammp::AxisId axis, const rammp::MotorState &state);

  /// Publish the current system state.
  /// \note Runs on the publication task.
  void publish_system_state();

  /// Publish the current seat state.
  /// \note Runs on the publication task.
  void publish_seat_state();

  /// Publish velocity commands for the two drive motor controllers.
  /// \note Runs on the publication task and feeds only the motor command topics.
  void publish_motor_commands();

  /// Advance wheel odometry and publish it to ROS 2 as /odom and the
  /// odom -> base_link transform on /tf (REP-105).
  /// \note Runs on the odometry task.
  void publish_odometry();

  /// Log the latest joystick input, system state, and motor output.
  /// \note Runs at the slower publication-task rate.
  void log_drive_status();

  mib::bsp::MIB &mib_;                         ///< Board support: Ethernet and RTPS participant.
  mib::DriveController drive_controller_;     ///< Chassis and seat motion controller.
  std::atomic<SystemState> state_{SystemState::INIT}; ///< Current state; shared across tasks.
  std::atomic<MIB::DriveProfile> drive_profile_{MIB::DriveProfile::NORMAL}; ///< Active drive profile.
  MIB::seatState seat_state_{};                ///< Placeholder for the current seat position.
  mutable std::mutex seat_state_mutex_;        ///< Protects seat state across RTPS and publish tasks.
  rammp::XYTwist latest_joystick_{};           ///< Most recent joystick sample for diagnostics.
  mutable std::mutex joystick_mutex_;          ///< Protects the latest joystick sample.
  mib::DiffDriveOdometry odometry_;            ///< Wheel odometry; used only on the odometry task.
  std::optional<rammp::MotorState> left_motor_state_;  ///< Latest Drive L report, once one arrives.
  std::optional<rammp::MotorState> right_motor_state_; ///< Latest Drive R report, once one arrives.
  bool motor_state_updated_{false};      ///< A drive report arrived since the last odometry step.
  bool odometry_resync_{false};          ///< A drive wheel's angle restarted; resync odometry.
  mutable std::mutex motor_state_mutex_; ///< Protects the drive reports and flags above.
  uint8_t status_sequence_{0};                 ///< Sequence number for MIB status samples.
  uint8_t motor_command_sequence_{0};          ///< Sequence number for motor command samples.
  std::unique_ptr<espp::Publisher<MIB::MibStatus>> status_publisher_; ///< MIB status output.
  std::unique_ptr<espp::Publisher<rammp::MotorCommand>> left_motor_publisher_; ///< Drive L output.
  std::unique_ptr<espp::Publisher<rammp::MotorCommand>> right_motor_publisher_; ///< Drive R output.
  std::unique_ptr<espp::Publisher<mib::ros::nav_msgs::Odometry>> odometry_publisher_; ///< /odom.
  std::unique_ptr<espp::Publisher<mib::ros::tf2_msgs::TFMessage>> tf_publisher_;      ///< /tf.
  std::unique_ptr<espp::Subscriber<rammp::MotorState>> left_motor_state_subscriber_;  ///< Drive L.
  std::unique_ptr<espp::Subscriber<rammp::MotorState>> right_motor_state_subscriber_; ///< Drive R.
  std::unique_ptr<espp::Subscriber<rammp::XYTwist>> joystick_subscriber_; ///< HMI joystick input.
  std::unique_ptr<espp::Subscriber<rammp::SeatCommand>> seat_command_subscriber_; ///< Seat input.
  std::unique_ptr<espp::Subscriber<rammp::DriveCommand>> drive_command_subscriber_; ///< Drive input.
  std::unique_ptr<espp::Task> state_task_;       ///< Advances the state machine.
  std::unique_ptr<espp::Task> publication_task_; ///< Publishes state at a lower rate.
  std::unique_ptr<espp::Task> motor_command_task_; ///< Publishes motor commands at 20 Hz.
  std::unique_ptr<espp::Task> odometry_task_;      ///< Publishes odometry at 20 Hz.
};
