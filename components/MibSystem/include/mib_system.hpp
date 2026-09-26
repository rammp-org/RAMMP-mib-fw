#pragma once

#include <atomic>
#include <memory>
#include <mutex>

#include "bsp.hpp"
#include "base_component.hpp"
#include "drive_controller.hpp"
#include "messages.hpp"
#include "mib_can_bridge.hpp"
#include "mib_params.hpp"
#include "rtps_pubsub.hpp"
#include "task.hpp"
#include "twai.hpp"

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

  /// Handle a motor controller status report.
  void handle_motor_status();

  /// Validate and apply one runtime parameter change, then re-apply it to the drive.
  /// \note Runs on an RTPS receive thread.
  void handle_param_set(const mib::ParamSet &request);

  /// Publish the whole parameter table with the outcome of the last ParamSet.
  /// \note Runs on the publication task.
  void publish_param_state();

  /// Bring up the TWAI peripheral for the CAN bridge. Failure is logged, not fatal:
  /// the MIB runs without a transceiver, it just reports the bridge as down.
  /// \return True if the TWAI node is on the bus.
  bool initialize_can_bridge();

  /// Transmit one frame from the bench tool on the CAN bus.
  /// \note Runs on an RTPS receive thread; blocks for at most one frame timeout.
  void handle_can_tx(const mib::CanFrame &frame);

  /// Publish the CAN bridge health.
  /// \note Runs on the publication task.
  void publish_can_status();

  /// Publish the current system state.
  /// \note Runs on the publication task.
  void publish_system_state();

  /// Publish the current seat state.
  /// \note Runs on the publication task.
  void publish_seat_state();

  /// Publish velocity commands for the two drive motor controllers.
  /// \note Runs on the publication task and feeds only the motor command topics.
  void publish_motor_commands();

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
  uint8_t status_sequence_{0};                 ///< Sequence number for MIB status samples.
  uint8_t motor_command_sequence_{0};          ///< Sequence number for motor command samples.
  uint8_t param_state_sequence_{0};            ///< Sequence number for ParamState samples.
  std::atomic<uint8_t> last_param_set_seq_{0}; ///< seq of the last ParamSet handled.
  std::atomic<bool> last_param_set_ok_{false}; ///< Whether that ParamSet was applied.
  std::unique_ptr<espp::Publisher<MIB::MibStatus>> status_publisher_; ///< MIB status output.
  std::unique_ptr<espp::Publisher<rammp::MotorCommand>> left_motor_publisher_; ///< Drive L output.
  std::unique_ptr<espp::Publisher<rammp::MotorCommand>> right_motor_publisher_; ///< Drive R output.
  std::unique_ptr<espp::Subscriber<rammp::XYTwist>> joystick_subscriber_; ///< HMI joystick input.
  std::unique_ptr<espp::Subscriber<rammp::SeatCommand>> seat_command_subscriber_; ///< Seat input.
  std::unique_ptr<espp::Subscriber<rammp::DriveCommand>> drive_command_subscriber_; ///< Drive input.
  std::unique_ptr<espp::Subscriber<mib::ParamSet>> param_set_subscriber_;   ///< Parameter writes.
  std::unique_ptr<espp::Publisher<mib::ParamState>> param_state_publisher_; ///< Parameter table.
  std::unique_ptr<espp::Twai> twai_;                                         ///< CAN bridge bus.
  std::unique_ptr<espp::Subscriber<mib::CanFrame>> can_tx_subscriber_;  ///< Frames to transmit.
  std::unique_ptr<espp::Publisher<mib::CanFrame>> can_rx_publisher_;    ///< Frames received.
  std::unique_ptr<espp::Publisher<mib::CanStatus>> can_status_publisher_; ///< Bridge health.
  std::atomic<bool> can_initialized_{false};  ///< TWAI node created and enabled.
  std::atomic<uint32_t> can_tx_ok_{0};        ///< CanStatus counters, see mib_can_bridge.hpp.
  std::atomic<uint32_t> can_tx_failed_{0};
  std::atomic<uint32_t> can_rx_frames_{0};
  std::atomic<uint32_t> can_rx_dropped_{0};
  std::atomic<uint32_t> can_bus_errors_{0};
  std::atomic<uint8_t> can_rx_sequence_{0};   ///< Sequence number for republished frames.
  std::unique_ptr<espp::Task> state_task_;       ///< Advances the state machine.
  std::unique_ptr<espp::Task> publication_task_; ///< Publishes state at a lower rate.
  std::unique_ptr<espp::Task> motor_command_task_; ///< Publishes motor commands at 20 Hz.
};
