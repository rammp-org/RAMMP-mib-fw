#pragma once

#include <chrono>
#include <cstdint>
#include <functional>
#include <mutex>
#include <optional>
#include <system_error>

#include "actuator_store.hpp"
#include "base_component.hpp"
#include "mcp266.hpp"

namespace mib {

/// @brief One positioned axis of a Basicmicro MCP266.
///
/// The controller does the work: it closes the position loop, runs the motion profile,
/// enforces its own clamp and, where wired, stops on limit switches. This class is the
/// thin, reusable layer above it. It knows the axis's calibrated travel in encoder
/// counts, its default motion profile and a jog step, and it turns "go there" into the
/// CiA 402 profile-position exchange that espp::Mcp266 implements.
///
/// Positions are encoder counts, always. Engineering units (metres, degrees) and the
/// kinematics behind them are not this class's concern; an application layer above the
/// BSP maps them onto counts using the calibrated range it can read from range().
///
/// Two kinds of encoder, two separate procedures that must not be confused:
/// - An **absolute** encoder reports the joint's position from the moment of power-up.
///   It is calibrated once per installation (set_calibration_mode, set_range) to learn
///   the counts at the mechanical ends, and never again.
/// - An **incremental** (AB) encoder reports an arbitrary count at power-up. It must be
///   homed every boot (home(): drive into the limit switch, install a known count) or
///   have its last known count restored from NVS (restore_position, written by the
///   periodic save_position).
///
/// Methods return true on success and log the reason on failure, like the rest of the
/// MIB firmware. Every method that talks to the controller blocks for one or more SDO
/// round trips (a few milliseconds each, up to the client's SDO timeout when the
/// controller is silent) and must be called from a task that may wait, never from an
/// RTPS callback. The current position is not held as state: read_position() asks the
/// encoder, and current_position() returns whatever the last read produced, with its age.
///
/// Two actuators can share one Mcp266 (its M1 and M2 channels), and must then be given
/// the same mutex. What it protects: espp::CanopenClient already serialises single SDO
/// transactions internally, but espp::Mcp266 and Ds402Drive do not serialise their
/// multi-step sequences (a move is ~10 SDOs, the clamp setup is a read-modify-write), so
/// two sequences on one controller could interleave. Every call here holds the mutex for
/// one sequence and releases it between calls. espp's own Mcp266Service does the same.
/// The mutex can go once espp::Mcp266 serialises its sequences itself.
///
/// A controller that stops answering is marked offline: further calls fail at once,
/// without touching the bus, until the retry interval has passed. A controller that
/// answers but has lost its limits (it reset while the MIB ran) is detected before each
/// move and re-initialised.
class Actuator : public espp::BaseComponent {
public:
  using Axis = espp::Mcp266::Axis;
  using DriveState = espp::Ds402Drive::State;
  using Range = ActuatorStore::Range;

  /// Motion profile for a move, in the controller's units. The Actuator runs the
  /// trapezoid itself: a move streams setpoints from poll() that ramp at `acceleration`,
  /// cruise at `velocity` and slow at `deceleration`, because on the MCP266 firmware
  /// tested a profile-position target is taken as a step (the CiA 402 profile objects are
  /// accepted but no ramp is applied; see DESIGN.md). The controller's stored default
  /// speed caps how fast it follows and initialize() installs the config profile's
  /// velocity as that ceiling every boot, so a per-move profile can ask for less, not
  /// more. Too low a velocity makes the velocity loop cog (seen at 100 counts/s with
  /// QPPS 2750).
  struct Profile {
    uint32_t velocity;     ///< Cruise speed, counts per second.
    uint32_t acceleration; ///< Counts per second squared.
    uint32_t deceleration; ///< Counts per second squared.
  };

  /// How to home an incremental-encoder joint against its limit switch.
  struct Homing {
    bool required{false};            ///< True for an incremental (AB) encoder.
    int direction{-1};               ///< +1 or -1: which way the home switch lies.
    int32_t home_count{0};           ///< Count installed once the switch has stopped the motor.
    Profile profile{500, 500, 500};  ///< Approach profile. Too slow a speed makes the
                                     ///< velocity loop cog; use a normal move speed.
    int32_t stall_counts{2};         ///< Movement below this over stall_time means stopped.
    std::chrono::milliseconds stall_time{500};
    std::chrono::seconds timeout{60};
    /// Length of each approach segment. A target a billion counts away makes the
    /// controller's trajectory maths overflow and the motion stutter, so the approach is
    /// commanded in segments of this many counts, re-issued until the switch stops it.
    int32_t segment{50000};
    /// Polled during the approach; returning true aborts homing with a quick stop.
    std::function<bool()> cancel{};
  };

  struct Config {
    const char *name;         ///< Short name for logs and NVS keys (11 characters max).
    Axis axis;                ///< Which channel of the shared controller.
    Range range;              ///< Calibrated travel in counts. Targets are clamped to it.
    Profile profile;          ///< Default motion profile.
    int32_t jog_step;         ///< Counts moved by increment() and decrement().
    int32_t tolerance;        ///< Counts within which a position counts as at target, and
                              ///< the deadband of the rest watch (see poll()).
    bool hardware_limits;     ///< Limit switches are wired to the controller for this axis.
    Homing homing{};          ///< Leave required=false for an absolute encoder.
    ActuatorStore *store{nullptr};    ///< Where ranges and positions persist; may be null.
    std::chrono::milliseconds offline_retry{2000}; ///< How long to skip a silent controller.
    std::chrono::milliseconds position_save_interval{5000}; ///< Throttle for save_position().
    int32_t position_save_threshold{20};                    ///< Counts moved before saving.
    espp::Logger::Verbosity log_level{espp::Logger::Verbosity::INFO};
  };

  /// @param mcp The controller this axis lives on. Must outlive the actuator. Needs the
  ///        set_encoder() / read_position_limits() API (esp-cpp/espp#842).
  /// @param mcp_mutex Shared by every actuator on the same controller.
  Actuator(espp::Mcp266 &mcp, std::mutex &mcp_mutex, const Config &config);

  /// @name Lifecycle
  /// @{

  /// Prepare the axis. Clears latched faults and installs the range as the controller's
  /// clamp and software limits; the MCP266 reverts to its EEPROM at power-up, so this is
  /// once per boot. An incremental-encoder joint also tries restore_position(); if no
  /// saved position exists it stays not homed until home() runs.
  bool initialize();

  /// Initialized, and homed where homing is required. Moves are refused otherwise.
  bool ready() const;
  bool initialized() const;
  bool online() const;
  /// @}

  /// @name Position
  /// @{

  /// Ask the encoder. Refreshes current_position().
  bool read_position(int32_t &counts);

  /// The latest reading from any bus exchange, without one. Fresh only as often as the
  /// owner polls read_position(); check position_age().
  std::optional<int32_t> current_position() const;
  std::chrono::milliseconds position_age() const;

  /// The last count this actuator commanded, for layers that queue moves.
  std::optional<int32_t> last_target() const;
  /// @}

  /// @name Moves. All clamp to the range (unless calibrating) and use the default profile
  ///       unless one is given. Relative moves read the encoder first: "relative" means
  ///       relative to where the joint is now, not to the last target.
  /// @{
  bool move_absolute(int32_t counts);
  bool move_absolute(int32_t counts, const Profile &profile);
  bool move_relative(int32_t delta);
  bool move_relative(int32_t delta, const Profile &profile);
  bool increment();                     ///< One jog step, in counts.
  bool decrement();
  bool stop();                          ///< CiA 402 quick stop; the next move re-enables.
  /// The move engine; call it every few tens of milliseconds, only while the joint may
  /// move (the owner's IDLE), from a task that may block on CAN. While a move is in
  /// progress each call advances the trapezoid and sends the next setpoint (a few SDOs).
  /// Once the setpoints have reached the target, the joint counts as arrived when the
  /// controller reports target reached, or when its count has sat within `tolerance` of
  /// the target and still for half a second; a settle time later the drive is released:
  /// no duty, no loop, the joint rests unpowered. A joint that has not arrived three
  /// seconds after the stream ended is released where it stalled, with a warning that
  /// says how far short (a loop that cannot close the last counts is a tuning problem,
  /// not a reason to keep it humming). The resting joint is then watched at a slow rate:
  /// if the count moves more than `tolerance` from where it came to rest (load, a shock
  /// absorber in the linkage, a bump), the joint is moved back to its target and
  /// released again, at most once a second. A quick stop, calibration mode or a
  /// rewritten encoder count ends the watch; the next move restarts it. No bus traffic
  /// while there is nothing to do. False only on a bus failure.
  bool poll();
  bool is_target_reached(bool &reached);
  bool get_drive_state(DriveState &state);
  bool read_statusword(uint16_t &statusword); ///< Raw CiA 402 statusword; bit 11 = limit active.
  /// Whether the controller reports a limit switch active (statusword bit 11).
  bool limit_active(bool &active);

  /// Clear whatever the controller has latched: the CiA 402 fault, and the Basicmicro
  /// latched conditions (E-stop, and on some pin configurations a tripped switch) through
  /// the E-stop reset command. Harmless when nothing is latched.
  bool clear_faults();
  /// @}

  /// @name Calibration: absolute encoders, once per installation.
  /// @{

  /// Lift or restore the range clamp on the controller so the axis can be driven to its
  /// mechanical ends while a calibration script records the readings, and let an unhomed
  /// incremental-encoder axis be jogged. With the clamp lifted nothing limits travel but
  /// the current limit and any switches, so only jogs (increment / decrement) are
  /// accepted in this mode; absolute and relative moves are refused. Off at every boot;
  /// nothing enters it but an explicit call.
  bool set_calibration_mode(bool enabled);
  bool calibration_mode() const;

  /// Install a calibrated range: on the controller if initialised, and in the store.
  bool set_range(const Range &range);
  Range range() const;

  /// Clamp a count to the range (identity in calibration mode).
  int32_t clamp(int32_t counts) const;

  /// Read the MinPos/MaxPos clamp the controller currently holds (two SDOs). Normally
  /// equal to range(), or the lifted clamp in calibration mode; differs when the
  /// controller has reset or someone wrote it behind our back.
  bool read_controller_limits(int32_t &min_counts, int32_t &max_counts);
  /// @}

  /// @name Position loop tuning: the gain fields of the controller's position PID record
  ///       (Basicmicro command 61-64). Controller RAM only: the MCP266 reverts to its
  ///       EEPROM at power-up, so a gain set here lasts until the next power cycle (save
  ///       it on the controller with Motion Studio once tuned).
  /// @{

  /// The record's gains. Its MinPos/MaxPos clamp belongs to the range machinery
  /// (set_range(), read_controller_limits()) and is not part of this struct.
  struct PositionPid {
    float p{};           ///< Proportional gain (the controller stores it x1024).
    float i{};           ///< Integral gain.
    float d{};           ///< Derivative gain.
    uint32_t max_i{};    ///< Integral windup limit.
    uint32_t deadzone{}; ///< Counts: no correction inside it.
  };

  /// Read the gains the controller currently holds (seven SDOs).
  bool read_position_pid(PositionPid &pid);

  /// Write the gains to controller RAM, keeping the installed clamp (the setter writes
  /// the whole record). Refused while a move is in progress: it would rewrite the loop
  /// under way.
  bool set_position_pid(const PositionPid &pid);
  /// @}

  /// @name Homing: incremental encoders, every boot.
  /// @{

  /// Drive toward the home switch at the homing profile until the controller stops the
  /// motor on it, install homing.home_count as the encoder value, restore the clamp, and
  /// save the position. Blocks for the whole approach (up to homing.timeout).
  bool home();
  bool homed() const;

  /// Install the position saved by save_position() as the encoder value, so the joint is
  /// homed without moving. Valid only if the joint did not move while powered off.
  bool restore_position();

  /// Persist the current position for restore_position(). Throttled by
  /// position_save_interval and position_save_threshold unless forced; call it from the
  /// periodic poll. Does nothing for a joint that is not homed.
  bool save_position(bool force = false);

  /// Write the controller's encoder count (Basicmicro command 22/23, mirrored over
  /// CANopen; not yet verified on hardware).
  bool set_encoder(int32_t counts);
  /// @}

  const Config &config() const { return config_; }
  const char *name() const { return config_.name; }

private:
  bool check_online();
  void note_result(bool ok, const std::error_code &ec, const char *what);
  bool apply_limits(const Range &range, std::error_code &ec);
  /// Install the config profile's velocity and acceleration as the controller's stored
  /// default speed and accel, which its position moves run on. Caller must not hold
  /// mcp_mutex_.
  bool apply_motion_defaults(std::error_code &ec);
  /// Advance the trapezoid by the time since the last call and send the next setpoint.
  /// Nothing to do when no move is in progress. Caller must not hold mcp_mutex_.
  bool step_trajectory();
  /// Read the controller's clamp back and compare with what we installed; false if the
  /// controller has lost it (a reset), in which case initialize() is re-run.
  bool verify_limits();
  bool do_move(int32_t counts, const Profile &profile, bool jog = false);
  /// Leave "quick stop active" (where stop() parks the drive), which the standard enable
  /// sequence cannot exit: first try to re-enable in place (holds position), else disable
  /// voltage (coasts) so the enable sequence can start from "switch on disabled".
  bool release_quick_stop();
  /// Send a controlword and poll until the drive reports `wanted` (1 s). Caller holds
  /// mcp_mutex_.
  bool command_state(uint16_t controlword, DriveState wanted, std::error_code &ec);
  /// Take the drive out of any state in which its position loop is active (operation
  /// enabled, quick stop active) so the encoder count can be rewritten without the loop
  /// chasing the old target. Caller holds mcp_mutex_.
  bool disable_loop(std::error_code &ec);
  bool read_position_locked(int32_t &counts, std::error_code &ec);
  Range installed_limits() const;

  espp::Mcp266 &mcp_;
  std::mutex &mcp_mutex_;
  Config config_;

  mutable std::mutex state_mutex_; ///< Guards everything below.
  Range range_;
  std::optional<int32_t> current_position_;
  std::chrono::steady_clock::time_point position_time_{};
  std::optional<int32_t> last_target_;
  bool pending_release_{false}; ///< A move is out whose drive poll() has yet to release.
  bool watching_{false};        ///< Released at a target that poll() keeps the joint on.
  uint32_t move_serial_{0};     ///< Bumped per move, so poll() never releases a newer one.
  std::optional<std::chrono::steady_clock::time_point> reached_since_;
  std::chrono::steady_clock::time_point next_poll_{};
  std::chrono::steady_clock::time_point last_correction_{};
  /// For poll()'s stillness test: the count has not left a small band around
  /// still_reference_ since still_since_.
  std::optional<int32_t> still_reference_;
  std::chrono::steady_clock::time_point still_since_{};
  std::optional<std::chrono::steady_clock::time_point> stream_done_at_; ///< Last setpoint sent.
  std::optional<int32_t>
      rest_position_; ///< Where the joint sat when released; the watch's reference.
  /// The move in progress: the setpoint streamed to the controller and its velocity,
  /// advanced by step_trajectory() until the setpoint lands on the target.
  struct Trajectory {
    bool active{false};
    double setpoint{0};
    double velocity{0}; ///< Signed, counts per second.
    int32_t target{0};
    Profile profile{};
    std::chrono::steady_clock::time_point last{};
  } trajectory_;
  int32_t last_saved_position_{0};
  std::chrono::steady_clock::time_point last_save_time_{};
  bool initialized_{false};
  bool homed_{false};
  bool calibration_mode_{false};
  bool online_{true};
  std::chrono::steady_clock::time_point offline_since_{};
};

} // namespace mib
