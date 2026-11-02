#pragma once

#include <mutex>
#include <string_view>
#include <system_error>

#include "base_component.hpp"
#include "push_assist.hpp"

namespace mib::push_assist {

/// The live push-assist Gains, shared between the control panel (which tunes them) and
/// whatever drives from the handles (which reads them). A set takes effect at once; save()
/// keeps the current values in NVS for the next boot.
class Settings : public espp::BaseComponent {
public:
  Settings();

  /// Load the saved values from NVS, falling back to kDefaultGains for any missing key.
  /// \note Initializes the NVS partition if nothing has yet.
  void load();

  /// \return A copy of the current gains.
  Gains get() const;

  /// Set one param by key, clamped to its range.
  /// \return The spec it matched, or nullptr for an unknown key.
  const ParamSpec *set(std::string_view key, float value);

  /// Back to kDefaultGains (not saved).
  void reset_to_defaults();

  /// Write the current gains to NVS.
  /// \return True on success.
  bool save();

  /// \return True if the current gains match what is in NVS (or the defaults, before a save).
  bool saved() const;

private:
  mutable std::mutex mutex_;
  Gains gains_{kDefaultGains};
  Gains saved_{kDefaultGains};
};

} // namespace mib::push_assist
