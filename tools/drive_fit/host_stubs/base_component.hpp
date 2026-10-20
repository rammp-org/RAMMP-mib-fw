#pragma once
#include <string_view>

#include "format.hpp"

namespace espp {
class Logger {
public:
  enum class Verbosity { DEBUG, INFO, WARN, ERROR, NONE };
  template <typename... Args> void debug(Args &&...) {}
  template <typename... Args> void info(Args &&...) {}
  template <typename... Args> void warn(Args &&...) {}
  template <typename... Args> void error(Args &&...) {}
};

class BaseComponent {
public:
  BaseComponent(std::string_view, Logger::Verbosity) {}

protected:
  Logger logger_;
};
} // namespace espp
