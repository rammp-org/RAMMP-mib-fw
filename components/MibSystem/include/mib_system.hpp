#pragma once

#include <array>
#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>

#include "bsp.hpp"
#include "base_component.hpp"
#include "drive_controller.hpp"
#include "estop/estop.hpp"
#include "messages.hpp"
#include "rtps_pubsub.hpp"
#include "task.hpp"

/// @brief Top-level MIB application: owns the system state machine and RTPS endpoints.
///
/// Brings the board up through mib::bsp::MIB, then runs three periodic tasks: a
/// fast state task that advances the system state machine, a slower
/// publication task that reports system / seat state to the rest of the robot,
/// and the motor command task that commands every rammp::kAxes board and
/// reports them to the joystick as rammp::Motors.
///
/// \note Construction does no hardware work; call start() to bring the system up.
class MibSystem : public espp::BaseComponent {
public:
  /// Construct the MIB system. Binds to the MIB board singleton.
  MibSystem();

  /// Bring the system up and run it.
  /// Starts the state task, waits for initialization to finish, then starts the
  /// publication task.
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

  /// Handle one board's state report.
  /// \param index The board's row in rammp::kAxes.
  /// \param state The received state.
  /// \note Runs on an RTPS receive thread - return quickly, do not block.
  void handle_motor_state(size_t index, const rammp::MotorState &state);

  /// Handle a motor jog request from the joystick (hold-to-run, motor debug).
  /// \param jog The requested board and direction.
  /// \note Runs on an RTPS receive thread - return quickly, do not block.
  void handle_motor_jog(const rammp::MotorJog &jog);

  /// Publish the current system state.
  /// \note Runs on the publication task.
  void publish_system_state();

  /// Publish the current seat state.
  /// \note Runs on the publication task.
  void publish_seat_state();

  /// Publish a command to every rammp::kAxes board: the wheel speeds while
  /// driving, a jog while IDLE, otherwise DISARMED.
  /// \note Runs on the motor command task.
  void publish_motor_commands();

  /// Publish the motor debug list: every board with its latest state.
  /// \note Runs on the motor command task.
  void publish_motors();

  /// Log the latest joystick input, system state, and motor output.
  /// \note Runs at the slower publication-task rate.
  void log_drive_status();

  mib::bsp::MIB &mib_;                         ///< Board support: Ethernet and RTPS participant.
  mib::DriveController drive_controller_;     ///< Chassis and seat motion controller.
  estop::Latch latch_;                         ///< Robot-wide e-stop latch (~6.4 KB: keep MibSystem static).
  std::atomic<SystemState> state_{SystemState::INIT}; ///< Current state; shared across tasks.
  std::atomic<bool> estop_error_{false};       ///< ERROR came from the e-stop: left once it clears.
  std::atomic<MIB::DriveProfile> drive_profile_{MIB::DriveProfile::NORMAL}; ///< Active drive profile.
  MIB::seatState seat_state_{};                ///< Placeholder for the current seat position.
  mutable std::mutex seat_state_mutex_;        ///< Protects seat state across RTPS and publish tasks.
  rammp::XYTwist latest_joystick_{};           ///< Most recent joystick sample for diagnostics.
  mutable std::mutex joystick_mutex_;          ///< Protects the latest joystick sample.
  uint8_t status_sequence_{0};                 ///< Sequence number for MIB status samples.
  uint8_t motor_command_sequence_{0};          ///< Sequence number for motor command samples.
  uint8_t motors_sequence_{0};                 ///< Sequence number for Motors samples.
  std::chrono::steady_clock::time_point motors_published_{}; ///< Last Motors sample.
  /// Latest state of each kAxes board, and when it arrived (epoch = never).
  std::array<rammp::MotorState, rammp::kAxisCount> motor_states_{};
  std::array<std::chrono::steady_clock::time_point, rammp::kAxisCount> motor_state_times_{};
  std::array<bool, rammp::kAxisCount> motor_online_{}; ///< As last reported, for logging.
  mutable std::mutex motor_state_mutex_;       ///< Protects motor states across RTPS and tasks.
  rammp::MotorJog latest_jog_{};               ///< Most recent jog request.
  std::chrono::steady_clock::time_point latest_jog_time_{}; ///< When it arrived.
  mutable std::mutex jog_mutex_;               ///< Protects the latest jog request.
  bool jog_requested_{false};                  ///< A fresh jog asked for; motor task only.
  std::unique_ptr<espp::Publisher<MIB::MibStatus>> status_publisher_; ///< MIB status output.
  /// One command output per kAxes row, in table order.
  std::array<std::unique_ptr<espp::Publisher<rammp::MotorCommand>>, rammp::kAxisCount>
      motor_publishers_;
  /// One state input per kAxes row, in table order.
  std::array<std::unique_ptr<espp::Subscriber<rammp::MotorState>>, rammp::kAxisCount>
      motor_state_subscribers_;
  std::unique_ptr<espp::Publisher<rammp::Motors>> motors_publisher_; ///< Motor debug list.
  std::unique_ptr<espp::Subscriber<rammp::MotorJog>> motor_jog_subscriber_; ///< Jog input.
  std::unique_ptr<espp::Subscriber<rammp::XYTwist>> joystick_subscriber_; ///< HMI joystick input.
  std::unique_ptr<espp::Subscriber<rammp::SeatCommand>> seat_command_subscriber_; ///< Seat input.
  std::unique_ptr<espp::Subscriber<rammp::DriveCommand>> drive_command_subscriber_; ///< Drive input.
  std::unique_ptr<espp::Task> state_task_;       ///< Advances the state machine.
  std::unique_ptr<espp::Task> publication_task_; ///< Publishes state at a lower rate.
  std::unique_ptr<espp::Task> motor_command_task_; ///< Publishes motor commands at 20 Hz.
};
