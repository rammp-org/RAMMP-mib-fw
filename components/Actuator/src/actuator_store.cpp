#include "actuator_store.hpp"

#include <string>
#include <system_error>

namespace mib {

namespace {
std::string key(std::string_view name, const char *suffix) {
  return std::string(name) + suffix;
}
} // namespace

ActuatorStore::ActuatorStore()
    : BaseComponent("ActuatorStore", espp::Logger::Verbosity::INFO) {}

bool ActuatorStore::init() {
  // nvs_flash_init() returns ESP_OK when the partition is already initialised (it looks
  // it up first), so another component initialising NVS before or after us is harmless.
  // espp's wrapper erases the partition and retries on NO_FREE_PAGES / NEW_VERSION_FOUND,
  // which loses saved records; both only arise when the partition is unreadable anyway.
  // The ESP-IDF NVS API takes a global lock in every call, so one store may be used from
  // several tasks.
  std::error_code ec;
  nvs_.init(ec);
  ready_ = !ec;
  if (!ready_) {
    logger_.warn("NVS unavailable, nothing will persist across boots: {}", ec.message());
  }
  return ready_;
}

// ------------------------------------------------------------------ primitives

bool ActuatorStore::load_i32(std::string_view name, const char *suffix, int32_t &value) {
  if (!ready_) {
    return false;
  }
  std::error_code ec;
  int32_t read = 0;
  nvs_.get_var(kNamespace, key(name, suffix), read, ec);
  if (ec) {
    return false; // nothing saved under this key, the normal case on a fresh board
  }
  value = read;
  return true;
}

bool ActuatorStore::save_i32(std::string_view name, const char *suffix, int32_t value) {
  if (!ready_) {
    logger_.error("{}: cannot save {}, NVS is not initialised", name, suffix);
    return false;
  }
  std::error_code ec;
  nvs_.set_var(kNamespace, key(name, suffix), value, ec);
  if (ec) {
    logger_.error("{}: saving {} failed: {}", name, suffix, ec.message());
    return false;
  }
  return true;
}

bool ActuatorStore::erase_key(std::string_view name, const char *suffix) {
  if (!ready_) {
    logger_.error("{}: cannot erase {}, NVS is not initialised", name, suffix);
    return false;
  }
  std::error_code ec;
  const bool ok = nvs_.erase(kNamespace, key(name, suffix), ec);
  if (!ok) {
    logger_.warn("{}: erasing {}: {}", name, suffix, ec.message());
  }
  return ok;
}

// ------------------------------------------------------------------ ranges

bool ActuatorStore::load_range(std::string_view name, Range &range) {
  int32_t min = 0;
  int32_t max = 0;
  if (!load_i32(name, "_min", min) || !load_i32(name, "_max", max)) {
    return false;
  }
  range = {min, max};
  return true;
}

bool ActuatorStore::save_range(std::string_view name, const Range &range) {
  if (!save_i32(name, "_min", range.min) || !save_i32(name, "_max", range.max)) {
    return false;
  }
  logger_.info("{}: range [{}, {}] saved", name, range.min, range.max);
  return true;
}

bool ActuatorStore::erase_range(std::string_view name) {
  const bool a = erase_key(name, "_min");
  const bool b = erase_key(name, "_max");
  return a && b;
}

// ------------------------------------------------------------------ positions

bool ActuatorStore::load_position(std::string_view name, int32_t &counts) {
  return load_i32(name, "_pos", counts);
}

bool ActuatorStore::save_position(std::string_view name, int32_t counts) {
  if (!save_i32(name, "_pos", counts)) {
    return false;
  }
  logger_.debug("{}: position {} saved", name, counts);
  return true;
}

bool ActuatorStore::erase_position(std::string_view name) { return erase_key(name, "_pos"); }

} // namespace mib
