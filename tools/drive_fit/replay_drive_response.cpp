// Host-side replay of a joystick log through the firmware's DriveResponse and
// MIBconfig profiles, stepped at config::drive_response_interval.
//
// Input CSV rows:  t_s,linear_stick,side_stick   (side_stick + = right)
// Output CSV rows: t_s,linear_mps,angular_radps
//
// Usage: replay_drive_response INPUT.csv low|normal|high DELAY_S

#include <cstdio>
#include <chrono>
#include <cstdlib>
#include <cstring>
#include <vector>

#include "MIBconfig.hpp"
#include "drive_response.hpp"

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
  mib::DriveResponse response({
      .max_forward_mps = profile->max_forward_mps,
      .max_reverse_mps = profile->max_reverse_mps,
      .max_angular_radps = profile->max_angular_radps,
      .turn_reduction = profile->turn_reduction,
      .linear_accel_mps2 = profile->linear_accel_mps2,
      .linear_decel_mps2 = profile->linear_decel_mps2,
      .angular_accel_radps2 = profile->angular_accel_radps2,
      .angular_decel_radps2 = profile->angular_decel_radps2,
  });
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

  const double step_s = std::chrono::duration<double>(mib::config::drive_response_interval).count();
  size_t j = 0;
  for (double now = 0.0; now <= t.back(); now += step_s) {
    const double stick_time = now - delay_s;
    while (j + 1 < t.size() && t[j + 1] <= stick_time) {
      j++;
    }
    const bool have_stick = stick_time >= t.front();
    // MibSystem passes (y, -x): stick + right is a clockwise (negative) turn.
    response.set_target(have_stick ? linear[j] : 0.0f, have_stick ? -side[j] : 0.0f);
    response.update(static_cast<float>(step_s));
    const auto out = response.output();
    std::printf("%.3f,%.5f,%.5f\n", now, out.linear_mps, out.angular_radps);
  }
  return 0;
}
