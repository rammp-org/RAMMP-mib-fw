#pragma once

#include <chrono>
#include <cstdint>
#include <mutex>
#include <optional>
#include <system_error>

#include "actuator_store.hpp"
#include "base_component.hpp"
#include "canopen_client.hpp"
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
/// Positions come in two forms, interchangeable through the calibrated range:
/// - **counts**, the encoder's own units, used for calibration, homing and jogging;
/// - a **fraction** of the calibrated travel, 0 at the low end and 1 at the high end, for
///   layers that must not know counts.
/// Engineering units (metres, degrees) and the kinematics behind them are not this
/// class's concern; an application layer above the BSP maps them onto fractions.
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
/// Two actuators can share one Mcp266 (its M1 and M2 channels). The controller has a
/// single SDO channel, so both must be given the same mutex, which every call here holds
/// for the duration of its exchange.
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

  /// Motion profile for a move, in the controller's units.
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
    Profile profile{100, 200, 200};  ///< Slow approach.
    int32_t stall_counts{2};         ///< Movement below this over stall_time means stopped.
    std::chrono::milliseconds stall_time{500};
    std::chrono::seconds timeout{60};
  };

  struct Config {
    const char *name;         ///< Short name for logs and NVS keys (11 characters max).
    Axis axis;                ///< Which channel of the shared controller.
    Range range;              ///< Calibrated travel in counts. Targets are clamped to it.
    Profile profile;          ///< Default motion profile.
    int32_t jog_step;         ///< Counts moved by increment() and decrement().
    int32_t tolerance;        ///< Counts within which a position counts as at target.
    bool hardware_limits;     ///< Limit switches are wired to the controller for this axis.
    Homing homing{};          ///< Leave required=false for an absolute encoder.
    ActuatorStore *store{nullptr};    ///< Where ranges and positions persist; may be null.
    std::chrono::milliseconds offline_retry{2000}; ///< How long to skip a silent controller.
    std::chrono::milliseconds position_save_interval{5000}; ///< Throttle for save_position().
    int32_t position_save_threshold{20};                    ///< Counts moved before saving.
    espp::Logger::Verbosity log_level{espp::Logger::Verbosity::INFO};
  };

  /// @param mcp The controller this axis lives on. Must outlive the actuator.
  /// @param client The controller's CANopen client, for the few manufacturer objects
  ///        Mcp266 does not wrap (set encoder, clamp readback).
  /// @param mcp_mutex Shared by every actuator on the same controller.
  Actuator(espp::Mcp266 &mcp, espp::CanopenClient &client, std::mutex &mcp_mutex,
           const Config &config);

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

  /// Latest reading as a fraction of the calibrated travel. Empty when nothing has been
  /// read yet.
  std::optional<float> current_fraction() const;

  /// Bus read, then convert.
  bool read_fraction(float &fraction);

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
  bool move_fraction(float fraction);
  bool move_relative_fraction(float delta);
  bool increment();                     ///< One jog step, in counts.
  bool decrement();
  bool stop();                          ///< CiA 402 quick stop; the next move re-enables.
  bool is_target_reached(bool &reached);
  bool get_drive_state(DriveState &state);
  /// @}

  /// @name Calibration: absolute encoders, once per installation.
  /// @{

  /// Lift or restore the range clamp on the controller so the axis can be driven to its
  /// mechanical ends while a calibration script records the readings.
  bool set_calibration_mode(bool enabled);
  bool calibration_mode() const;

  /// Install a calibrated range: on the controller if initialised, and in the store.
  bool set_range(const Range &range);
  Range range() const;

  /// Clamp a count to the range (identity in calibration mode).
  int32_t clamp(int32_t counts) const;
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
  /// Read the controller's clamp back and compare with what we installed; false if the
  /// controller has lost it (a reset), in which case initialize() is re-run.
  bool verify_limits();
  bool do_move(int32_t counts, const Profile &profile);
  bool read_position_locked(int32_t &counts, std::error_code &ec);
  int32_t fraction_to_counts(float fraction) const;
  float counts_to_fraction(int32_t counts) const;
  Range installed_limits() const;

  espp::Mcp266 &mcp_;
  espp::CanopenClient &client_;
  std::mutex &mcp_mutex_;
  Config config_;
  const uint16_t position_pid_get_; ///< Manufacturer object for the clamp readback.
  const uint16_t set_encoder_object_;

  mutable std::mutex state_mutex_; ///< Guards everything below.
  Range range_;
  std::optional<int32_t> current_position_;
  std::chrono::steady_clock::time_point position_time_{};
  std::optional<int32_t> last_target_;
  int32_t last_saved_position_{0};
  std::chrono::steady_clock::time_point last_save_time_{};
  bool initialized_{false};
  bool homed_{false};
  bool calibration_mode_{false};
  bool online_{true};
  std::chrono::steady_clock::time_point offline_since_{};
};

} // namespace mib
