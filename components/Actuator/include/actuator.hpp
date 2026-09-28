#pragma once

#include <chrono>
#include <cstdint>
#include <mutex>
#include <optional>
#include <system_error>

#include "base_component.hpp"
#include "mcp266.hpp"

namespace mib {

/// @brief One positioned axis of a Basicmicro MCP266, addressed in encoder counts.
///
/// The controller does the work: it closes the position loop, runs the motion profile,
/// enforces its own clamp and, where wired, stops on limit switches. This class is the
/// thin, reusable layer above it: it knows the axis's calibrated travel, its default
/// motion profile and a jog step, and it turns "go to this count" into the CiA 402
/// profile-position exchange that espp::Mcp266 implements. It knows nothing about what
/// the axis moves; kinematics live above it.
///
/// Every method that talks to the controller blocks for one or more SDO round trips (a
/// few milliseconds each, up to the client's SDO timeout when the controller is silent)
/// and must be called from a task that may wait, never from an RTPS callback.
///
/// Two actuators can share one Mcp266 (its M1 and M2 channels). The controller has a
/// single SDO channel, so both must be given the same mutex, which every call here holds
/// for the duration of its exchange.
///
/// A controller that stops answering is marked offline: further calls fail at once with
/// std::errc::host_unreachable, without touching the bus, until the retry interval has
/// passed. One dead controller therefore costs the rest of the system one SDO timeout
/// per retry interval rather than one per call.
class Actuator : public espp::BaseComponent {
public:
  using Axis = espp::Mcp266::Axis;
  using DriveState = espp::Ds402Drive::State;

  /// Motion profile for a move, in the controller's units.
  struct Profile {
    uint32_t velocity;     ///< Cruise speed, counts per second.
    uint32_t acceleration; ///< Counts per second squared.
    uint32_t deceleration; ///< Counts per second squared.
  };

  /// Calibrated travel of the axis, inclusive, in encoder counts.
  struct Range {
    int32_t min;
    int32_t max;
  };

  struct Config {
    const char *name;         ///< For logs, e.g. "FC".
    Axis axis;                ///< Which channel of the shared controller.
    Range range;              ///< Calibrated travel. Targets are clamped to it.
    Profile profile;          ///< Default motion profile.
    int32_t jog_step;         ///< Counts moved by increment() and decrement().
    int32_t tolerance;        ///< Counts within which last_position() counts as at target.
    bool hardware_limits;     ///< Limit switches are wired to the controller for this axis.
    std::chrono::milliseconds offline_retry{2000}; ///< How long to skip a silent controller.
    espp::Logger::Verbosity log_level{espp::Logger::Verbosity::INFO};
  };

  /// @param mcp The controller this axis lives on. Must outlive the actuator.
  /// @param mcp_mutex Shared by every actuator on the same controller.
  Actuator(espp::Mcp266 &mcp, std::mutex &mcp_mutex, const Config &config);

  /// Prepare the axis for moves. Clears latched faults, then installs the calibrated
  /// range as the controller's position clamp and software limits. The MCP266 reverts
  /// to its EEPROM at power-up, so call once per boot before any move.
  /// @return True when the controller accepted everything.
  bool initialize(std::error_code &ec);

  /// Read the joint position from the controller.
  /// @param counts Out: encoder counts.
  bool get_position(int32_t &counts, std::error_code &ec);

  /// The last position read from the controller by any method, without a bus exchange.
  std::optional<int32_t> last_position() const;

  /// Move to an absolute count with the default profile.
  /// The target is clamped to the range unless calibration mode is on.
  bool move_absolute(int32_t target, std::error_code &ec);

  /// Move to an absolute count with a one-off profile.
  bool move_absolute(int32_t target, const Profile &profile, std::error_code &ec);

  /// Move by a signed number of counts from the current position (read from the
  /// controller first).
  bool move_relative(int32_t delta, std::error_code &ec);
  bool move_relative(int32_t delta, const Profile &profile, std::error_code &ec);

  /// Move one jog step up or down. Intended for calibration and manual positioning.
  bool increment(std::error_code &ec);
  bool decrement(std::error_code &ec);

  /// Stop the current move with a CiA 402 quick stop. The next move re-enables the axis.
  bool stop(std::error_code &ec);

  /// Whether the controller reports the last commanded target as reached.
  bool is_target_reached(bool &reached, std::error_code &ec);

  /// The CiA 402 drive state of the axis, decoded from its statusword.
  bool get_drive_state(DriveState &state, std::error_code &ec);

  /// Lift or restore the range clamp on the controller so the axis can be driven to its
  /// mechanical ends (or into its limit switches) while a calibration routine records the
  /// readings. Blocks: it rewrites the controller's limits.
  bool set_calibration_mode(bool enabled, std::error_code &ec);
  bool calibration_mode() const { return calibration_mode_; }

  /// Install a new calibrated range and, if the axis is initialized and not in
  /// calibration mode, write it to the controller.
  bool set_range(const Range &range, std::error_code &ec);
  Range range() const;

  /// Clamp a target to the range (identity in calibration mode).
  int32_t clamp(int32_t target) const;

  bool initialized() const { return initialized_; }
  bool online() const { return online_; }
  const Config &config() const { return config_; }
  const char *name() const { return config_.name; }

private:
  /// Fail fast while the controller is marked offline and the retry interval has not
  /// passed. Returns false and sets ec in that case.
  bool check_online(std::error_code &ec);

  /// Update the offline marking from the outcome of a controller exchange.
  void note_result(bool ok, const std::error_code &ec, const char *what);

  /// Write a clamp and software limits to the controller (under the mutex).
  bool apply_limits(const Range &range, std::error_code &ec);

  bool do_move(int32_t target, const Profile &profile, std::error_code &ec);

  espp::Mcp266 &mcp_;
  std::mutex &mcp_mutex_;
  Config config_;
  mutable std::mutex state_mutex_; ///< Guards range_, last_position_ and the flags.
  Range range_;
  std::optional<int32_t> last_position_;
  bool initialized_{false};
  bool calibration_mode_{false};
  bool online_{true};
  std::chrono::steady_clock::time_point offline_since_{};
};

} // namespace mib
