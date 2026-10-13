// Host-side test for SeatKinematics. No ESP-IDF needed. Run: ./run.sh
#include <cstdio>

#include "actuator_stage_checks.hpp"

int main() {
  auto print_line = [](const char *line) { std::printf("%s\n", line); };

  const int failures = mib::test::run_actuator_stage_checks(print_line);

  std::printf("%d failure(s)\n", failures);
  return failures == 0 ? 0 : 1;
}
