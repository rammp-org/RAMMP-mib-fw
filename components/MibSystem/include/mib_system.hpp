#pragma once

#include <array>
#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>

#include "bsp.hpp"
#include "base_component.hpp"
#include "drive_controller.hpp"
#include "estop/estop.hpp"
#include "messages.hpp"
#include "rtps_pubsub.hpp"
#include "service_link.hpp"
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
    AUTONOMOUS,    ///< The Jetson drives the base; any joystick input ends it (back to IDLE).
    SHARED,        ///< Shared control; the joystick commands nothing, even when touched.
    ESTOP,         ///< Emergency stop: the e-stop latch is active, drive stopped; IDLE once it clears.
    ERROR,         ///< Initialization or runtime failure; not operational.
  };

  /// Guards shared with the state-machine model (tools/hfsm/mib_system.json).
  /// \return True if the command asks to enable driving.
  static bool drive_enable_requested(const rammp::DriveCommand &command) {
    return command.request == rammp::DriveRequest::ENABLE;
  }
  /// \return True if the command asks to disable driving.
  static bool drive_disable_requested(const rammp::DriveCommand &command) {
    return command.request == rammp::DriveRequest::DISABLE;
  }
  /// \return True if the command changes the profile while already driving (an ENABLE
  ///         received in DRIVE_ENABLED).
  static bool profile_change_requested(const rammp::DriveCommand &command) {
    return command.request == rammp::DriveRequest::ENABLE;
  }
  /// \return True if the stick is off centre or any button is pressed.
  static bool joystick_touched(const rammp::XYTwist &sample) {
    return sample.x != 0.0f || sample.y != 0.0f || sample.twist != 0.0f ||
           sample.buttons != rammp::Buttons::NONE;
  }

  /// Create the RTPS publishers and subscribers.
  /// \note The RTPS participant must already be started.
  /// \return True if every endpoint was registered successfully.
  bool initialize_pubsub();

  /// Run one iteration of the state machine, acting on the current state.
  /// \note Runs on the state task.
  void run_state_step();

  /// One PACE RACER axis as the MIB sees it: its endpoints, its last state, and any
  /// control panel override or pending tuning.
  struct AxisLink {
    std::unique_ptr<espp::Publisher<rammp::MotorCommand>> command_publisher;
    std::unique_ptr<espp::Publisher<rammp::MotorTuning>> tuning_publisher;
    std::unique_ptr<espp::Subscriber<rammp::MotorState>> state_subscriber;
    rammp::MotorState state{};                              ///< Last MotorState received.
    std::chrono::steady_clock::time_point state_time{};     ///< When it arrived.
    bool state_seen{false};                                 ///< A MotorState has ever arrived.
    rammp::MotorCommand override_command{};                 ///< The panel's command.
    std::chrono::steady_clock::time_point override_until{}; ///< Override lapses here.
    rammp::MotorTuning tuning{};                            ///< Last tuning requested.
    std::chrono::steady_clock::time_point tuning_until{};   ///< Resend it until here.
    uint8_t clear_fault_req{0};                             ///< Carried in every command.
    rammp::MotorCommand last_command{};                     ///< What the MIB last sent.
  };

  /// Bring up the service link (WebUSB + UART) the control panel talks to.
  /// \note Runs before INIT, so the panel can connect without an Ethernet link.
  void start_service_link();

  /// Handle one frame of the control module (see mib_panel_protocol.hpp).
  /// \note Runs on a service-link worker task.
  void handle_panel_frame(const espp::stream_frame::Frame &frame,
                          const mib::ServiceLink::reply_fn &reply);

  /// The MIB's state for the panel's status view, as a JSON object.
  std::string status_json();

  /// Emit one telemetry sample on the service link.
  /// \note Runs on the telemetry task.
  void emit_telemetry();

  /// Apply a stick sample (the HMI joystick's or the control panel's) to the state: it drives
  /// in DRIVE_ENABLED, and a touch ends AUTONOMOUS.
  void apply_stick(const rammp::XYTwist &sample);

  /// Disable drive and drop every override before a firmware update starts.
  /// \note Runs on a service-link worker task.
  void make_safe_for_ota();

  /// True while the panel overrides at least one axis.
  bool any_override_active() const;

  /// Enter ESTOP from the current state and stop the drive. ESTOP holds for
  /// mib::config::estop_hold_duration, then asks the e-stop latch to clear (repeated every
  /// mib::config::estop_clear_retry while the latch refuses), and returns to IDLE once the
  /// latch has cleared. Called by run_state_step() while latch_.is_active().
  /// \note Ignored in INIT (the blocking INIT step would overwrite the state; INIT enters
  ///       ESTOP itself for the boot latch, without the hold) and in ERROR (bring-up
  ///       failed; not operational).
  void enter_estop();

  /// Jetson control requests. The RTPS messages (AutonomousRequest, AutonomousStop,
  /// SharedRequest, SharedStop) are not defined yet and their names may change, so these
  /// take no payload and nothing calls them. They run on an RTPS receive thread once wired.
  /// Autonomy starts from IDLE or DRIVE_ENABLED and always ends in IDLE, so driving has
  /// to be re-enabled from the joystick afterwards.
  /// IDLE or DRIVE_ENABLED -> AUTONOMOUS. Ignored elsewhere.
  void handle_autonomous_request();
  /// AUTONOMOUS -> IDLE. Ignored elsewhere.
  void handle_autonomous_stop();
  /// IDLE or DRIVE_ENABLED -> SHARED. Ignored elsewhere.
  void handle_shared_request();
  /// SHARED -> IDLE. Ignored elsewhere.
  void handle_shared_stop();

  /// Enter AUTONOMOUS or SHARED from a manual state.
  /// \return False (and logs) if the current state is not IDLE or DRIVE_ENABLED.
  bool enter_autonomy(SystemState autonomy_state, const char *name);

  /// Leave autonomy: stop the drive and go to IDLE.
  void leave_autonomy(const char *reason);

  /// Handle one drive setpoint from the Jetson (autonomy).
  /// \param linear_mps Forward velocity in metres per second.
  /// \param angular_radps Angular velocity in radians per second, positive counter-clockwise.
  /// \note Acted on in AUTONOMOUS and SHARED only. The RTPS topic for this is not defined
  ///       yet, so nothing calls this. Runs on an RTPS receive thread once it does.
  void handle_autonomy_setpoint(float linear_mps, float angular_radps);

  /// Enter CALIBRATE from IDLE.
  /// \note The trigger (a joystick command or another source) is not finalized; nothing
  ///       calls this yet. Only valid from IDLE.
  void enter_calibrate();

  /// Leave ESTOP for IDLE once the e-stop latch has cleared. Until then, after the hold,
  /// ask the latch to clear every mib::config::estop_clear_retry. Nothing is resumed:
  /// driving must be re-enabled and the Jetson must request control again.
  /// \note Runs on the state task.
  void run_estop_step();

  /// Handle one joystick sample from the HMI.
  /// \param sample The received normalized stick position, buttons, and drive mode.
  /// \note Runs on an RTPS receive thread - return quickly, do not block.
  void handle_joystick_message(const rammp::XYTwist &sample);

  /// Handle a seat control command.
  /// \param command The requested seat axis and target position.
  /// \note Runs on an RTPS receive thread - return quickly, do not block.
  /// \return False if the command was refused.
  bool handle_seat_control_command(const rammp::SeatCommand &command);

  /// Handle a drive enable or disable command.
  /// \param command The requested drive state and response profile.
  /// \note Runs on an RTPS receive thread - return quickly, do not block.
  /// \return False if the command was refused.
  bool handle_drive_command(const rammp::DriveCommand &command);

  /// Handle a motor controller status report.
  void handle_motor_status();

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
  estop::Latch latch_;                         ///< Robot-wide e-stop latch (~6.4 KB: keep MibSystem static).
  std::atomic<SystemState> state_{SystemState::INIT}; ///< Current state; shared across tasks.
  std::atomic<bool> error_reported_{false};    ///< This stay in ERROR has been logged.
  std::chrono::steady_clock::time_point estop_hold_until_{}; ///< End of the ESTOP hold; default = no hold (boot latch). State task only.
  std::chrono::steady_clock::time_point estop_last_clear_attempt_{}; ///< Last start_clear_attempt(). State task only.
  std::atomic<MIB::DriveProfile> drive_profile_{MIB::DriveProfile::NORMAL}; ///< Active drive profile.
  MIB::seatState seat_state_{};                ///< Placeholder for the current seat position.
  mutable std::mutex seat_state_mutex_;        ///< Protects seat state across RTPS and publish tasks.
  rammp::XYTwist latest_joystick_{};           ///< Most recent joystick sample for diagnostics.
  mutable std::mutex joystick_mutex_;          ///< Protects the latest joystick sample.
  uint8_t status_sequence_{0};                 ///< Sequence number for MIB status samples.
  uint8_t motor_command_sequence_{0};          ///< Sequence number for motor command samples.
  std::unique_ptr<espp::Publisher<MIB::MibStatus>> status_publisher_; ///< MIB status output.
  std::array<AxisLink, rammp::kAxisCount> axes_{}; ///< Every PACE RACER axis, in table order.
  mutable std::mutex axes_mutex_;                  ///< Protects axes_ state and overrides.
  std::atomic<bool> ota_active_{false};            ///< A firmware update is being written.
  std::atomic<bool> endpoints_ready_{false};       ///< initialize_pubsub() succeeded.
  uint8_t tuning_req_{0};                          ///< Last MotorTuning::tuning_req_id.
  float panel_stick_x_{0.0f};                      ///< Panel stick, guarded by joystick_mutex_.
  float panel_stick_y_{0.0f};
  std::chrono::steady_clock::time_point panel_stick_until_{}; ///< Panel stick lapses here.
  std::unique_ptr<mib::ServiceLink> service_;                 ///< WebUSB / UART control panel link.
  std::unique_ptr<espp::Task> telemetry_task_;                ///< Feeds the panel's plots.
  std::unique_ptr<espp::Subscriber<rammp::XYTwist>> joystick_subscriber_; ///< HMI joystick input.
  std::unique_ptr<espp::Subscriber<rammp::SeatCommand>> seat_command_subscriber_; ///< Seat input.
  std::unique_ptr<espp::Subscriber<rammp::DriveCommand>> drive_command_subscriber_; ///< Drive input.
  std::unique_ptr<espp::Task> state_task_;       ///< Advances the state machine.
  std::unique_ptr<espp::Task> publication_task_; ///< Publishes state at a lower rate.
  std::unique_ptr<espp::Task> motor_command_task_; ///< Publishes motor commands at 20 Hz.
};
