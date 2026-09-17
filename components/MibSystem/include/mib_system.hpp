#pragma once

#include <atomic>
#include <memory>

#include "bsp.hpp"
#include "base_component.hpp"
#include "messages.hpp"
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
  void handle_joystick_message(const rammp_xy_twist_t &sample);

  /// Handle a seat control command.
  void handle_seat_control_command();

  /// Handle a motor controller status report.
  void handle_motor_status();

  /// Publish the current system state.
  /// \note Runs on the publication task.
  void publish_system_state();

  /// Publish the current seat state.
  /// \note Runs on the publication task.
  void publish_seat_state();

  mib::bsp::MIB &mib_;                         ///< Board support: Ethernet and RTPS participant.
  std::atomic<SystemState> state_{SystemState::INIT}; ///< Current state; shared across tasks.
  std::unique_ptr<espp::Subscriber<rammp_xy_twist_t>> joystick_subscriber_; ///< HMI joystick input.
  std::unique_ptr<espp::Task> state_task_;       ///< Advances the state machine.
  std::unique_ptr<espp::Task> publication_task_; ///< Publishes state at a lower rate.
};
