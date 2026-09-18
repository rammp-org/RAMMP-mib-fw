#include <cmath>
#include <numbers>

#include "differential_drive.hpp"
#include "logger.hpp"

extern "C" void app_main(void) {
  espp::Logger logger({.tag = "DiffDrive Example", .level = espp::Logger::Verbosity::INFO});

  //! [differential drive example]
  // 8 inch wheels, 550 mm apart. The right motor faces the opposite way, so its
  // output is negated for the platform to drive straight.
  mib::DifferentialDrive drive({
      .wheel_diameter_m = 0.2032f,
      .wheel_separation_m = 0.55f,
      .invert_left = false,
      .invert_right = true,
  });

  logger.info("Drive valid: {}", drive.is_valid());
  logger.info("Wheel diameter: {} m, separation: {} m", drive.wheel_diameter(),
              drive.wheel_separation());

  struct Command {
    const char *description;
    float linear_mps;
    float angular_radps;
  };

  static constexpr float quarter_turn_per_s = std::numbers::pi_v<float> / 2.0f;
  const Command commands[] = {
      {"stopped", 0.0f, 0.0f},
      {"forward 1.0 m/s", 1.0f, 0.0f},
      {"reverse 0.5 m/s", -0.5f, 0.0f},
      {"spin left in place", 0.0f, quarter_turn_per_s},
      {"spin right in place", 0.0f, -quarter_turn_per_s},
      {"forward + left turn", 1.0f, quarter_turn_per_s},
  };

  for (const auto &command : commands) {
    auto speeds = drive.compute(command.linear_mps, command.angular_radps);
    logger.info("{:<20} v={:+.2f} m/s w={:+.2f} rad/s -> left={:+.1f} rpm right={:+.1f} rpm",
                command.description, command.linear_mps, command.angular_radps, speeds.left_rpm,
                speeds.right_rpm);
  }
  //! [differential drive example]

  // A platform with no geometry logs a warning on construction and always
  // produces zero speeds, so a bad config cannot command the motors.
  mib::DifferentialDrive invalid_drive({});
  auto invalid_speeds = invalid_drive.compute(1.0f, 0.0f);
  logger.info("Invalid drive valid={} -> left={} rpm right={} rpm", invalid_drive.is_valid(),
              invalid_speeds.left_rpm, invalid_speeds.right_rpm);
}
