#pragma once

#include <string_view>
#include <system_error>

#include "actuator.hpp"
#include "nvs.hpp"

namespace mib {

/// @brief Persists each actuator's calibrated range in the MIB's NVS flash.
///
/// The compiled defaults in MIBconfig.hpp seed an axis; a range saved here overrides them
/// at the next boot. Calibration is once per installation, so this is what makes it stick
/// across reflashes and controller swaps. Keys are the actuator's short name plus a
/// suffix, and NVS limits a key to 15 characters, so names must be 11 or fewer.
class ActuatorRangeStore {
public:
  static constexpr const char *kNamespace = "act_range";

  /// Initialise the NVS partition. Safe to call once per boot.
  bool init(std::error_code &ec);

  /// Fetch a saved range. Returns false, leaving `range` untouched, when none is saved.
  bool load(std::string_view name, Actuator::Range &range);

  /// Save a range for the next boot.
  bool save(std::string_view name, const Actuator::Range &range, std::error_code &ec);

  /// Forget a saved range so the compiled default applies again.
  bool erase(std::string_view name, std::error_code &ec);

private:
  espp::Nvs nvs_;
  bool ready_{false};
};

} // namespace mib
