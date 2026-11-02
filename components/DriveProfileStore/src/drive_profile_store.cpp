#include "drive_profile_store.hpp"

#include "drive_profile.hpp"
#include "file_system.hpp"

namespace mib {

namespace {

const char *profile_name(rammp::DriveProfile profile) {
  switch (profile) {
  case rammp::DriveProfile::LOW:
    return "LOW";
  case rammp::DriveProfile::NORMAL:
    return "NORMAL";
  case rammp::DriveProfile::HIGH:
    return "HIGH";
  }
  return "UNKNOWN";
}

const char *file_name(rammp::DriveProfile profile) {
  switch (profile) {
  case rammp::DriveProfile::LOW:
    return "drive_profile_low.bin";
  case rammp::DriveProfile::NORMAL:
    return "drive_profile_normal.bin";
  case rammp::DriveProfile::HIGH:
    return "drive_profile_high.bin";
  }
  return "drive_profile_unknown.bin";
}

} // namespace

DriveProfileStore::DriveProfileStore(const std::filesystem::path &directory)
    : BaseComponent("DriveProfileStore", espp::Logger::Verbosity::INFO) {
  for (size_t i = 0; i < slots_.size(); ++i) {
    const auto profile = static_cast<rammp::DriveProfile>(i);
    slots_[i].file = std::make_unique<File>(File::Config{
        .file_path = directory / file_name(profile),
        .auto_load = false,
    });
    slots_[i].file->set(default_values(profile));
  }
}

void DriveProfileStore::load() {
  // The first call mounts the LittleFS partition, formatting it if it has never
  // been mounted. A mount failure is logged there and every load below then
  // fails, leaving the compiled-in defaults.
  espp::FileSystem::get();

  std::lock_guard<std::mutex> lock(mutex_);
  for (size_t i = 0; i < slots_.size(); ++i) {
    load_slot(static_cast<rammp::DriveProfile>(i), slots_[i]);
  }
}

void DriveProfileStore::load_slot(rammp::DriveProfile profile, Slot &slot) {
  const char *name = profile_name(profile);
  File &file = *slot.file;
  std::error_code ec;
  if (!file.load(ec)) {
    if (ec == espp::PersistentDataErrc::file_not_found) {
      logger_.info("{}: no saved profile, using the compiled-in default", name);
    } else {
      logger_.error("{}: saved profile not usable ({}), using the compiled-in default", name,
                    ec.message());
    }
    file.set(default_values(profile));
    slot.source = rammp::DriveProfileSource::DEFAULT;
    slot.has_saved = false;
    return;
  }

  const auto values = file.get();
  if (const char *bad = invalid_field(values)) {
    logger_.error("{}: saved profile has an invalid {}, using the compiled-in default", name, bad);
    file.set(default_values(profile));
    slot.source = rammp::DriveProfileSource::DEFAULT;
    slot.has_saved = false;
    return;
  }

  slot.source = rammp::DriveProfileSource::SAVED;
  slot.has_saved = true;
  if (file.was_migrated()) {
    logger_.info("{}: migrated saved profile from version {} to {}", name, file.loaded_version(),
                 File::CURRENT_VERSION);
  }
  logger_.info("{}: loaded saved profile: forward {} m/s, reverse {} m/s, turn {} rad/s", name,
               values.max_linear_velocity_mps, values.max_reverse_linear_velocity_mps,
               values.max_angular_velocity_radps);
}

const config::DriveProfileConfig &DriveProfileStore::default_values(rammp::DriveProfile profile) {
  switch (profile) {
  case rammp::DriveProfile::LOW:
    return config::drive_profile_low;
  case rammp::DriveProfile::HIGH:
    return config::drive_profile_high;
  case rammp::DriveProfile::NORMAL:
  default:
    return config::drive_profile_normal;
  }
}

config::DriveProfileConfig DriveProfileStore::get(rammp::DriveProfile profile) const {
  std::lock_guard<std::mutex> lock(mutex_);
  const Slot *s = slot(profile);
  return s ? s->file->get() : default_values(profile);
}

rammp::DriveProfileSource DriveProfileStore::source(rammp::DriveProfile profile) const {
  std::lock_guard<std::mutex> lock(mutex_);
  const Slot *s = slot(profile);
  return s ? s->source : rammp::DriveProfileSource::DEFAULT;
}

rammp::DriveProfileResult DriveProfileStore::set(rammp::DriveProfile profile,
                                                 const config::DriveProfileConfig &values,
                                                 bool persist) {
  const char *name = profile_name(profile);
  std::lock_guard<std::mutex> lock(mutex_);
  Slot *s = slot(profile);
  if (!s) {
    logger_.warn("Ignoring a change to unknown drive profile {}", static_cast<uint8_t>(profile));
    return rammp::DriveProfileResult::BAD_REQUEST;
  }
  if (const char *bad = invalid_field(values)) {
    logger_.warn("{}: rejected new limits, invalid {}", name, bad);
    return rammp::DriveProfileResult::INVALID;
  }

  if (!persist) {
    s->file->set(values);
    s->source = rammp::DriveProfileSource::UNSAVED;
    logger_.info("{}: new limits in use until reboot", name);
    return rammp::DriveProfileResult::OK;
  }

  std::error_code ec;
  if (!s->file->save(values, ec)) {
    // save() keeps the new values in memory even when the write fails
    s->source = rammp::DriveProfileSource::UNSAVED;
    logger_.error("{}: new limits in use, but saving them failed: {}", name, ec.message());
    return rammp::DriveProfileResult::SAVE_FAILED;
  }
  s->source = rammp::DriveProfileSource::SAVED;
  s->has_saved = true;
  logger_.info("{}: new limits saved", name);
  return rammp::DriveProfileResult::OK;
}

rammp::DriveProfileResult DriveProfileStore::reset(rammp::DriveProfile profile, bool persist) {
  const char *name = profile_name(profile);
  std::lock_guard<std::mutex> lock(mutex_);
  Slot *s = slot(profile);
  if (!s) {
    logger_.warn("Ignoring a reset of unknown drive profile {}", static_cast<uint8_t>(profile));
    return rammp::DriveProfileResult::BAD_REQUEST;
  }

  s->file->set(default_values(profile));
  if (!persist) {
    // a reboot goes back to the saved file, if there is one
    s->source =
        s->has_saved ? rammp::DriveProfileSource::UNSAVED : rammp::DriveProfileSource::DEFAULT;
    logger_.info("{}: compiled-in default in use until reboot", name);
    return rammp::DriveProfileResult::OK;
  }

  std::error_code ec;
  if (!s->file->erase(ec)) {
    s->source = rammp::DriveProfileSource::UNSAVED;
    logger_.error("{}: compiled-in default in use, but deleting the saved profile failed: {}", name,
                  ec.message());
    return rammp::DriveProfileResult::SAVE_FAILED;
  }
  s->source = rammp::DriveProfileSource::DEFAULT;
  s->has_saved = false;
  logger_.info("{}: saved profile deleted, compiled-in default in use", name);
  return rammp::DriveProfileResult::OK;
}

DriveProfileStore::Slot *DriveProfileStore::slot(rammp::DriveProfile profile) {
  const auto index = static_cast<size_t>(profile);
  return index < slots_.size() ? &slots_[index] : nullptr;
}

const DriveProfileStore::Slot *DriveProfileStore::slot(rammp::DriveProfile profile) const {
  const auto index = static_cast<size_t>(profile);
  return index < slots_.size() ? &slots_[index] : nullptr;
}

} // namespace mib
