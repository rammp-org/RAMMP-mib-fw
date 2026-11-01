#pragma once

#include <array>
#include <cstdint>
#include <filesystem>
#include <memory>
#include <mutex>

#include "MIBconfig.hpp"
#include "base_component.hpp"
#include "messages.hpp"
#include "persistent_data.hpp"

namespace mib {

/// @brief The limits of the LOW, NORMAL, and HIGH drive profiles, kept in flash.
///
/// Each profile is one file, written with espp::PersistentData: a versioned
/// header, a CRC, and an atomic save (temp file, then rename), so a reset
/// mid-save keeps the previous file. A profile with no saved file, or one that
/// cannot be used (corrupt, written by newer firmware, or failing
/// invalid_field()), runs on its compiled-in MIBconfig.hpp default, so a bad
/// file can never leave the chair without a profile. Defaults are never written
/// to flash, so a firmware update that changes a default reaches every profile
/// nobody has tuned.
///
/// \note Construction does no flash work; until load() runs, every profile is
///       its compiled-in default.
class DriveProfileStore : public espp::BaseComponent {
public:
  /// The saved form of one profile, current version first.
  using File = espp::PersistentData<config::DriveProfileConfig>;

  /// @brief Construct the store.
  /// @param directory Where the profile files live, normally the root of the
  ///        LittleFS partition (espp::FileSystem::get_root_path()).
  explicit DriveProfileStore(const std::filesystem::path &directory);

  /// @brief Mount the file system and load every profile's saved file.
  /// @details A profile whose file is missing or unusable keeps its compiled-in
  ///          default; the reason is logged. A file that cannot be used is left
  ///          in place, not overwritten, until a persistent set() or reset().
  void load();

  /// @brief The compiled-in MIBconfig.hpp limits of a profile.
  static const config::DriveProfileConfig &default_values(MIB::DriveProfile profile);

  /// @brief The limits a profile is using now.
  config::DriveProfileConfig get(MIB::DriveProfile profile) const;

  /// @brief Where the limits a profile is using now came from.
  MIB::DriveProfileSource source(MIB::DriveProfile profile) const;

  /// @brief Replace a profile's limits.
  /// @param profile The profile to change.
  /// @param values The new limits; rejected whole if invalid_field() finds one bad.
  /// @param persist Also save them to flash, so they survive a reboot.
  /// @return OK, INVALID (nothing changed), SAVE_FAILED (in use until reboot), or
  ///         BAD_REQUEST for an unknown profile.
  MIB::DriveProfileResult set(MIB::DriveProfile profile, const config::DriveProfileConfig &values,
                              bool persist);

  /// @brief Return a profile to its compiled-in default.
  /// @param profile The profile to reset.
  /// @param persist Also delete its saved file, so the default survives a reboot.
  /// @return OK, SAVE_FAILED (the file could not be deleted), or BAD_REQUEST.
  MIB::DriveProfileResult reset(MIB::DriveProfile profile, bool persist);

private:
  struct Slot {
    std::unique_ptr<File> file; ///< The profile's file; its in-memory copy is the limits in use.
    MIB::DriveProfileSource source{MIB::DriveProfileSource::DEFAULT};
    bool has_saved{false}; ///< A usable saved file exists, so a reboot would load it.
  };

  /// @return The slot for a profile, or nullptr for a value outside the enum.
  Slot *slot(MIB::DriveProfile profile);
  const Slot *slot(MIB::DriveProfile profile) const;

  void load_slot(MIB::DriveProfile profile, Slot &slot);

  mutable std::mutex mutex_; ///< Keeps each slot's values and source consistent.
  std::array<Slot, MIB::kDriveProfileCount> slots_;
};

} // namespace mib
