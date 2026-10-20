// Host-side replay of a joystick log through the firmware drive path: the
// MIBconfig profile, DriveController's joystick shaping, and the real
// espp::TrajectoryPlanner source, stepped at its planning period.
//
// Input CSV rows:  t_s,linear_stick,side_stick   (side_stick + = right)
// Output CSV rows: t_s,linear_mps,angular_radps
//
// Usage: replay_drive_profile INPUT.csv low|normal|high DELAY_S

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <vector>

#include "drive_profile.hpp"

namespace {
// update() is protected; the firmware drives it from the planner's own timer.
class ReplayPlanner : public espp::TrajectoryPlanner {
public:
  using espp::TrajectoryPlanner::TrajectoryPlanner;
  using espp::TrajectoryPlanner::update;
};
} // namespace

int main(int argc, char **argv) {
  if (argc != 4) {
    std::fprintf(stderr, "usage: %s INPUT.csv low|normal|high DELAY_S\n", argv[0]);
    return 2;
  }

  const mib::config::DriveProfileConfig *profile = &mib::config::drive_profile_normal;
  if (std::strcmp(argv[2], "low") == 0) {
    profile = &mib::config::drive_profile_low;
  } else if (std::strcmp(argv[2], "high") == 0) {
    profile = &mib::config::drive_profile_high;
  }
  const auto planner_config = mib::planner_config(*profile);
  const auto shaping = mib::input_shaping(*profile);
  ReplayPlanner planner(planner_config);
  const double delay_s = std::atof(argv[3]);

  std::FILE *input = std::fopen(argv[1], "r");
  if (input == nullptr) {
    std::perror(argv[1]);
    return 1;
  }
  std::vector<double> t, linear, side;
  double row_t, row_linear, row_side;
  while (std::fscanf(input, "%lf,%lf,%lf", &row_t, &row_linear, &row_side) == 3) {
    t.push_back(row_t);
    linear.push_back(row_linear);
    side.push_back(row_side);
  }
  std::fclose(input);
  if (t.empty()) {
    return 1;
  }

  const double step_s = planner_config.planning_period.count();
  size_t j = 0;
  for (double now = 0.0; now <= t.back(); now += step_s) {
    const double stick_time = now - delay_s;
    while (j + 1 < t.size() && t[j + 1] <= stick_time) {
      j++;
    }
    const bool have_stick = stick_time >= t.front();
    // MibSystem passes (y, -x) to DriveController::set_target: stick + right is
    // a clockwise (negative) turn.
    const auto shaped = mib::shape_drive_input(have_stick ? linear[j] : 0.0f,
                                               have_stick ? -side[j] : 0.0f, shaping);
    planner.set_target(shaped.linear, shaped.angular);
    planner.update(static_cast<float>(step_s));
    const auto out = planner.output();
    std::printf("%.3f,%.5f,%.5f\n", now, out.linear_velocity, out.angular_velocity);
  }
  return 0;
}
