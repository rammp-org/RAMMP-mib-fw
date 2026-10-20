// Host-side replay of a joystick log through the firmware drive path: the
// MIBconfig profile, DriveController's reverse scaling, and espp's
// TrajectoryPlanner from the cross-platform library, stepped at its planning
// period.
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
// The planner starts its own timer thread on construction. Stop it so the
// replay is the only caller of update(), stepping a fixed dt deterministically.
class ReplayPlanner : public espp::TrajectoryPlanner {
public:
  explicit ReplayPlanner(const Config &config)
      : espp::TrajectoryPlanner(config) {
    stop_task();
  }
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
  auto planner_config = mib::planner_config(*profile);
  // The planner logs to stdout, which carries the CSV output.
  planner_config.log_level = espp::Logger::Verbosity::NONE;
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
    // Same as DriveController::set_target(y, -x) from MibSystem: stick + right
    // is a clockwise (negative) turn.
    const float stick = have_stick ? static_cast<float>(linear[j]) : 0.0f;
    const float turn = have_stick ? static_cast<float>(-side[j]) : 0.0f;
    planner.set_target(mib::scale_reverse_input(stick, profile->reverse_scale), turn);
    planner.update(static_cast<float>(step_s));
    const auto out = planner.output();
    std::printf("%.3f,%.5f,%.5f\n", now, out.linear_velocity, out.angular_velocity);
  }
  return 0;
}
