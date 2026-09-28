#include "actuator_range_store.hpp"

#include <string>
#include <system_error>

namespace mib {

namespace {
std::string key(std::string_view name, const char *suffix) {
  return std::string(name) + suffix;
}
} // namespace

ActuatorRangeStore::ActuatorRangeStore()
    : BaseComponent("ActuatorRangeStore", espp::Logger::Verbosity::INFO) {}

bool ActuatorRangeStore::init() {
  std::error_code ec;
  nvs_.init(ec);
  ready_ = !ec;
  if (!ready_) {
    logger_.warn("NVS unavailable, calibrated ranges will not persist: {}", ec.message());
  }
  return ready_;
}

bool ActuatorRangeStore::load(std::string_view name, Actuator::Range &range) {
  if (!ready_) {
    return false;
  }
  std::error_code ec;
  int32_t min = 0;
  int32_t max = 0;
  nvs_.get_var(kNamespace, key(name, "_min"), min, ec);
  if (ec) {
    return false; // nothing saved for this name, the normal case on a fresh board
  }
  nvs_.get_var(kNamespace, key(name, "_max"), max, ec);
  if (ec) {
    return false;
  }
  range = {min, max};
  return true;
}

bool ActuatorRangeStore::save(std::string_view name, const Actuator::Range &range) {
  if (!ready_) {
    logger_.error("{}: cannot save range, NVS is not initialised", name);
    return false;
  }
  std::error_code ec;
  nvs_.set_var(kNamespace, key(name, "_min"), range.min, ec);
  if (!ec) {
    nvs_.set_var(kNamespace, key(name, "_max"), range.max, ec);
  }
  if (ec) {
    logger_.error("{}: saving range failed: {}", name, ec.message());
    return false;
  }
  logger_.info("{}: range [{}, {}] saved", name, range.min, range.max);
  return true;
}

bool ActuatorRangeStore::erase(std::string_view name) {
  if (!ready_) {
    logger_.error("{}: cannot erase range, NVS is not initialised", name);
    return false;
  }
  std::error_code ec;
  const bool a = nvs_.erase(kNamespace, key(name, "_min"), ec);
  const bool b = nvs_.erase(kNamespace, key(name, "_max"), ec);
  if (!(a && b)) {
    logger_.warn("{}: erasing range: {}", name, ec.message());
  }
  return a && b;
}

} // namespace mib
