#include "MIBconfig.hpp"
#include "actuator_stage_checks.hpp"
#include "logger.hpp"
#include "seat_kinematics.hpp"
#include "seat_kinematics_checks.hpp"

namespace {

espp::Logger &example_logger() {
  static espp::Logger logger(
      {.tag = "SeatKinematics Example", .level = espp::Logger::Verbosity::INFO});
  return logger;
}

void log_line(const char *line) { example_logger().info("{}", line); }

} // namespace

extern "C" void app_main(void) {
  auto &logger = example_logger();

  //! [seat kinematics example]
  const mib::SeatKinematics kinematics(mib::config::seat_geometry);

  // Tilt back 3 degrees, seat origin 120 mm above the ground, carriages 50 mm forward.
  const mib::SeatPose pose{
      .pitch_deg = 3.0f, .roll_deg = 0.0f, .elevation_mm = 120.0f, .translation_mm = 50.0f};

  auto joints = kinematics.inverse(pose);
  if (!joints) {
    logger.warn("Pose is unreachable");
  } else {
    logger.info("Arms: main left {:.2f} deg, main right {:.2f} deg, rear caster {:.2f} deg",
                joints->main_left_arm_deg, joints->main_right_arm_deg, joints->rear_caster_arm_deg);
    logger.info("Carriages: left {:.1f} mm, right {:.1f} mm", joints->left_carriage_mm,
                joints->right_carriage_mm);

    auto actuators = kinematics.to_actuators(*joints);
    if (actuators) {
      logger.info("Actuators (deg): main left {:.1f}, main right {:.1f}, rear caster {:.1f}, "
                  "left carriage {:.1f}, right carriage {:.1f}",
                  actuators->main_left_deg, actuators->main_right_deg, actuators->rear_caster_deg,
                  actuators->left_carriage_deg, actuators->right_carriage_deg);
    }
  }
  //! [seat kinematics example]

  int failures = mib::test::run_actuator_stage_checks(log_line);
  failures += mib::test::run_seat_kinematics_checks(kinematics, log_line);
  logger.info("{} failure(s)", failures);
}
