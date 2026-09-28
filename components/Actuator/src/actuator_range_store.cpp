#include "actuator_range_store.hpp"

#include <string>

namespace mib {

namespace {
std::string key(std::string_view name, const char *suffix) {
  return std::string(name) + suffix;
}
} // namespace

bool ActuatorRangeStore::init(std::error_code &ec) {
  ec.clear();
  nvs_.init(ec);
  ready_ = !ec;
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
    return false;
  }
  nvs_.get_var(kNamespace, key(name, "_max"), max, ec);
  if (ec) {
    return false;
  }
  range = {min, max};
  return true;
}

bool ActuatorRangeStore::save(std::string_view name, const Actuator::Range &range,
                              std::error_code &ec) {
  ec.clear();
  if (!ready_) {
    ec = std::make_error_code(std::errc::not_connected);
    return false;
  }
  nvs_.set_var(kNamespace, key(name, "_min"), range.min, ec);
  if (ec) {
    return false;
  }
  nvs_.set_var(kNamespace, key(name, "_max"), range.max, ec);
  return !ec;
}

bool ActuatorRangeStore::erase(std::string_view name, std::error_code &ec) {
  ec.clear();
  if (!ready_) {
    ec = std::make_error_code(std::errc::not_connected);
    return false;
  }
  const bool a = nvs_.erase(kNamespace, key(name, "_min"), ec);
  const bool b = nvs_.erase(kNamespace, key(name, "_max"), ec);
  return a && b;
}

} // namespace mib
