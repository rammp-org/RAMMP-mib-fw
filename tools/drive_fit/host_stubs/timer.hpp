#pragma once
#include <chrono>
#include <functional>

#include "base_component.hpp"
#include "task.hpp"

namespace espp {
class Timer {
public:
  typedef std::function<bool()> callback_fn;
  struct AdvancedConfig {
    std::chrono::duration<float> period;
    std::chrono::duration<float> delay = std::chrono::duration<float>(0);
    callback_fn callback;
    bool auto_start{true};
    Task::BaseConfig task_config;
    Logger::Verbosity log_level = Logger::Verbosity::WARN;
  };
  explicit Timer(const AdvancedConfig &) {}
  void cancel() { running_ = false; }
  bool is_running() const { return running_; }

private:
  bool running_{true};
};
} // namespace espp
