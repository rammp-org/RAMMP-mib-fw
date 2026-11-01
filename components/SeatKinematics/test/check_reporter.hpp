#pragma once

#include <cmath>
#include <cstdio>

namespace mib::test {

/// @brief Reports one "PASS name" / "FAIL name" line per check and counts the failures.
///
/// Shared by the check files, so the host test and the ESP-IDF example print the same lines.
class CheckReporter {
public:
  /// @param report Called with each result line, e.g. "PASS neutral pose round trip".
  explicit CheckReporter(void (*report)(const char *line))
      : report_(report) {}

  /// @brief Record one check.
  void operator()(bool passed, const char *name) {
    char line[160];
    std::snprintf(line, sizeof(line), "%s %s", passed ? "PASS" : "FAIL", name);
    report_(line);
    if (!passed) {
      ++failures_;
    }
  }

  /// @brief Number of failed checks so far (0 = all passed).
  int failures() const { return failures_; }

private:
  void (*report_)(const char *line);
  int failures_{0};
};

/// @brief True if a and b differ by no more than tolerance.
inline bool near(float a, float b, float tolerance) { return std::fabs(a - b) <= tolerance; }

} // namespace mib::test
