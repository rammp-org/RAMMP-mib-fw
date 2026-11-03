#include "push_assist_settings.hpp"

#include <cstring>

#include "nvs.hpp"
#include "nvs_handle_espp.hpp"

namespace mib::push_assist {

namespace {
constexpr const char *kNamespace = "push_assist";
} // namespace

Settings::Settings()
    : BaseComponent("PushAssist", espp::Logger::Verbosity::INFO) {}

void Settings::load() {
  std::error_code ec;
  espp::Nvs().init(ec); // a no-op if the partition is already up
  if (ec) {
    logger_.error("NVS init failed ({}); push assist uses its defaults", ec.message());
    return;
  }
  espp::NvsHandle handle(kNamespace, ec);
  if (ec) {
    logger_.error("NVS open failed ({}); push assist uses its defaults", ec.message());
    return;
  }
  Gains loaded = kDefaultGains;
  int found = 0;
  for (const auto &p : kParams) {
    float value = 0.0f;
    std::error_code key_ec;
    handle.get(p.key.data(), value, key_ec);
    if (!key_ec) {
      loaded.*(p.field) = clamp_param(p, value);
      found++;
    }
  }
  std::lock_guard<std::mutex> lock(mutex_);
  gains_ = saved_ = loaded;
  logger_.info("Push assist gains: {} of {} from NVS, the rest default", found, kParams.size());
}

Gains Settings::get() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return gains_;
}

const ParamSpec *Settings::set(std::string_view key, float value) {
  const auto *spec = find_param(key);
  if (spec == nullptr) {
    return nullptr;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  gains_.*(spec->field) = clamp_param(*spec, value);
  return spec;
}

void Settings::reset_to_defaults() {
  std::lock_guard<std::mutex> lock(mutex_);
  gains_ = kDefaultGains;
}

bool Settings::save() {
  const Gains gains = get();
  std::error_code ec;
  espp::NvsHandle handle(kNamespace, ec);
  for (const auto &p : kParams) {
    if (!ec) {
      handle.set(p.key.data(), gains.*(p.field), ec);
    }
  }
  if (!ec) {
    handle.commit(ec);
  }
  if (ec) {
    logger_.error("Saving push assist gains failed: {}", ec.message());
    return false;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  saved_ = gains;
  logger_.info("Push assist gains saved");
  return true;
}

bool Settings::saved() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return std::memcmp(&gains_, &saved_, sizeof(Gains)) == 0;
}

} // namespace mib::push_assist
