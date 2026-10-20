#pragma once
#include <cstddef>
#include <functional>
#include <memory>
#include <string>

namespace espp {
class Task {
public:
  struct BaseConfig {
    std::string name;
    size_t stack_size_bytes{4096};
    size_t priority{0};
    int core_id{-1};
  };
  struct Config {
    std::function<bool()> callback;
    BaseConfig task_config;
  };
  static std::unique_ptr<Task> make_unique(const Config &) { return std::make_unique<Task>(); }
  bool start() { return started_ = true; }
  bool stop() { return !(started_ = false); }
  bool is_started() const { return started_; }

private:
  bool started_{false};
};
} // namespace espp
