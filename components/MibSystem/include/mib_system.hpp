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
  /// \return True if the command asks for AUTONOMOUS or SHARED (the Jetson's requests).
  static bool autonomy_requested(const rammp::DriveCommand &command) {
    return command.request == rammp::DriveRequest::AUTONOMOUS ||
           command.request == rammp::DriveRequest::SHARED;
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

  /// The control panel's side of one axis-table board: its tuning writer, and any panel
  /// override, pending tuning or clear-fault. The board's command writer and last
  /// MotorState are motor_publishers_ and motor_states_.
  struct AxisLink {
    std::unique_ptr<espp::Publisher<rammp::MotorTuning>> tuning_publisher;
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

  /// Jetson control requests. The Jetson asks with a rammp::DriveCommand whose request is
  /// AUTONOMOUS or SHARED (handle_drive_command(), which calls enter_autonomy() directly)
  /// and ends it with DISABLE, so these stop-and-start helpers have no caller of their own
  /// yet. They run on an RTPS receive thread. Autonomy starts from IDLE or DRIVE_ENABLED
  /// and always ends in IDLE, so driving has to be re-enabled from the joystick afterwards.
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

  /// Handle one board's state report.
  /// \param index The board's row in rammp::kAxes.
  /// \param state The received state.
  /// \note Runs on an RTPS receive thread - return quickly, do not block.
  void handle_motor_state(size_t index, const rammp::MotorState &state);
  /// @brief Both drive boards have sent a MotorState within motor_state_timeout and
  ///        neither is in FAULT (always true when drive_requires_both_boards is off).
  bool drive_boards_ready() const;

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

  /// Start the IMU read task if the IMU initialized.
  /// \note Runs on the state task during INIT. A missing IMU is logged, not fatal.
  void start_imu_task();

  /// Read the IMU once and periodically log its orientation and calibration.
  /// \note Runs on the IMU task.
  /// \return True if the read succeeded.
  bool update_imu();

  /// Publish one Diagnostics sample, one item per RAMMP_DIAG_TABLE row.
  /// Rows the MIB has no reading for are sent with no values.
  /// \note Runs on the publication task.
  void publish_diagnostics();

  /// Publish the motor debug list: every board with its latest state.
  /// \note Runs on the motor command task.
  void publish_motor_states();

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
  std::atomic<rammp::DriveProfile> drive_profile_{rammp::DriveProfile::NORMAL}; ///< Active drive profile.
  rammp::SeatState seat_state_{};                ///< Placeholder for the current seat position.
  mutable std::mutex seat_state_mutex_;        ///< Protects seat state across RTPS and publish tasks.
  rammp::XYTwist latest_joystick_{};           ///< Most recent joystick sample for diagnostics.
  mutable std::mutex joystick_mutex_;          ///< Protects the latest joystick sample.
  uint8_t status_sequence_{0};                 ///< Sequence number for MIB status samples.
  uint8_t motor_command_sequence_{0};          ///< Sequence number for motor command samples.
  uint8_t diagnostics_sequence_{0};            ///< Sequence number for diagnostics samples.
  uint8_t motor_states_sequence_{0};           ///< Sequence number for Motors samples.
  /// When publish_motor_states() last sent a Motors sample (epoch = never).
  std::chrono::steady_clock::time_point last_motor_state_publish_timestamp_{};
  /// Latest state of each kAxes board, and when it arrived (epoch = never).
  std::array<rammp::MotorState, rammp::kAxisCount> motor_states_{};
  std::array<std::chrono::steady_clock::time_point, rammp::kAxisCount> motor_state_times_{};
  std::array<bool, rammp::kAxisCount> motor_online_{}; ///< As last reported, for logging.
  /// drive_boards_ready() as the state task last saw it, to log only its changes.
  bool drive_boards_were_ready_{true};
  /// Inside-wheel torque cap as last sent, to log only its changes (motor command task).
  bool inside_cap_left_{false};
  bool inside_cap_right_{false};
  /// Largest |iq| each board reported since the last drive status line, in A.
  std::array<float, rammp::kAxisCount> motor_peak_iq_{};
  mutable std::mutex motor_state_mutex_;       ///< Protects motor states across RTPS and tasks.
  rammp::MotorJog latest_jog_{};               ///< Most recent jog request.
  std::chrono::steady_clock::time_point latest_jog_time_{}; ///< When it arrived.
  mutable std::mutex jog_mutex_;               ///< Protects the latest jog request.
  bool jog_requested_{false};                  ///< A fresh jog asked for; motor task only.
  std::unique_ptr<espp::Publisher<rammp::MibStatus>> status_publisher_; ///< MIB status output.
  std::unique_ptr<espp::Publisher<rammp::Diagnostics>> diagnostics_publisher_; ///< Readings output.
  /// One command output per kAxes row, in table order.
  std::array<std::unique_ptr<espp::Publisher<rammp::MotorCommand>>, rammp::kAxisCount>
      motor_publishers_;
  /// One state input per kAxes row, in table order.
  std::array<std::unique_ptr<espp::Subscriber<rammp::MotorState>>, rammp::kAxisCount>
      motor_state_subscribers_;
  std::unique_ptr<espp::Publisher<rammp::Motors>> motor_states_publisher_; ///< Motor debug list.
  std::unique_ptr<espp::Subscriber<rammp::MotorJog>> motor_jog_subscriber_; ///< Jog input.
  std::array<AxisLink, rammp::kAxisCount> axes_{}; ///< Every PACE RACER axis, in table order.
  mutable std::mutex axes_mutex_;                  ///< Protects axes_.
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
  std::unique_ptr<espp::Task> imu_task_;           ///< Reads the IMU at 100 Hz.
  std::chrono::steady_clock::time_point last_imu_log_{};  ///< Last IMU log time; IMU task only.
  std::chrono::steady_clock::time_point imu_next_wake_{}; ///< Next IMU read time; IMU task only.
  std::atomic<bool> imu_reading_ok_{false}; ///< True if the latest IMU read succeeded.
};
