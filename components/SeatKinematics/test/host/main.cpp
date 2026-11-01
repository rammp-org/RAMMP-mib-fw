// Host-side test for SeatKinematics. No ESP-IDF needed. Run: ./run.sh
#include <cstdio>

#include "MIBconfig.hpp"
#include "actuator_stage_checks.hpp"
#include "seat_kinematics_checks.hpp"

int main() {
  auto print_line = [](const char *line) { std::printf("%s\n", line); };

  int failures = mib::test::run_actuator_stage_checks(print_line);

  const mib::SeatKinematics kinematics(mib::config::seat_geometry);
  failures += mib::test::run_seat_kinematics_checks(kinematics, print_line);

  std::printf("%d failure(s)\n", failures);
  return failures == 0 ? 0 : 1;
}
