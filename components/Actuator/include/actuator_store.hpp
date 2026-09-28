#pragma once

#include <cstdint>
#include <string_view>

#include "base_component.hpp"
#include "nvs.hpp"

namespace mib {

/// @brief What an actuator needs to remember across boots, kept in the MIB's NVS flash.
///
/// Two kinds of record, deliberately separate because they have different lifetimes:
///
/// - A **range** is the once-per-installation calibration of an absolute-encoder joint:
///   the counts at its two mechanical ends. Written by the calibration script, read at
///   boot to override the compiled default in MIBconfig.hpp.
/// - A **position** is the last known count of an incremental-encoder joint, written
///   periodically while the machine runs so that a boot can restore the count instead of
///   homing. It is only as good as the assumption that the joint did not move while
///   powered off.
///
/// Keys are the actuator's short name plus a suffix, and NVS limits a key to 15
/// characters, so names must be 11 or fewer. Methods return true on success and log the
/// reason on failure. NVS is not human-readable: the calibration should also be exported
/// as text and committed, so a replaced board can be seeded (see DESIGN.md).
class ActuatorStore : public espp::BaseComponent {
public:
  static constexpr const char *kNamespace = "actuator";

  struct Range {
    int32_t min;
    int32_t max;
  };

  ActuatorStore();

  /// Initialise the NVS partition. Safe to call once per boot.
  bool init();
  bool ready() const { return ready_; }

  /// @name Calibrated range (absolute encoders, once per installation)
  /// @{
  bool load_range(std::string_view name, Range &range);
  bool save_range(std::string_view name, const Range &range);
  bool erase_range(std::string_view name);
  /// @}

  /// @name Last known position (incremental encoders, updated while running)
  /// @{
  bool load_position(std::string_view name, int32_t &counts);
  bool save_position(std::string_view name, int32_t counts);
  bool erase_position(std::string_view name);
  /// @}

private:
  bool load_i32(std::string_view name, const char *suffix, int32_t &value);
  bool save_i32(std::string_view name, const char *suffix, int32_t value);
  bool erase_key(std::string_view name, const char *suffix);

  espp::Nvs nvs_;
  bool ready_{false};
};

} // namespace mib
