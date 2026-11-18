// Host checks for the Actuator's motion profile: the trapezoid the MIB streams to the
// MCP266, the target lead, never writing past the target, and continuing a move in flight.
// The numbers are the right main leg's (docs/actuator-revamp/bench-findings.md): 800 counts/s,
// 1,600 counts/s^2, lead 250 ms (1 / position P 4.0).

#include <cmath>
#include <cstdio>
#include <functional>

#include "motion_profile.hpp"

using mib::MotionProfile;

namespace {

int failures = 0;

#define CHECK(condition)                                                                           \
  do {                                                                                             \
    if (!(condition)) {                                                                            \
      std::printf("  FAIL %s:%d: %s\n", __FILE__, __LINE__, #condition);                           \
      ++failures;                                                                                  \
    }                                                                                              \
  } while (0)

bool near(double a, double b, double tolerance = 1e-6) { return std::fabs(a - b) <= tolerance; }

constexpr MotionProfile::Limits kLeg{.max_speed = 800.0, .acceleration = 1600.0};
constexpr double kLead = 0.25;
constexpr double kTick = 0.01; // the Actuator's 10 ms tick

/// Walk a profile tick by tick and check what must hold at every tick: speed within the
/// limit, speed and position continuous (no step bigger than the acceleration allows),
/// and the end at rest on the target.
void check_walk(const MotionProfile &profile, const MotionProfile::Limits &limits,
                double start_speed) {
  MotionProfile::Sample previous = profile.at(0.0);
  CHECK(near(previous.speed, start_speed));
  const double speed_cap = std::max(limits.max_speed, std::fabs(start_speed));
  for (double t = kTick; t <= profile.duration() + 2 * kTick; t += kTick) {
    const auto s = profile.at(t);
    CHECK(std::fabs(s.speed) <= speed_cap + 1e-6);
    CHECK(std::fabs(s.speed - previous.speed) <= limits.acceleration * kTick + 1e-6);
    // Moved no further than the speed allows (which may peak inside the tick).
    CHECK(std::fabs(s.position - previous.position) <=
          (std::max(std::fabs(s.speed), std::fabs(previous.speed)) + limits.acceleration * kTick) *
                  kTick +
              1e-6);
    previous = s;
  }
  const auto end = profile.at(profile.duration());
  CHECK(end.done);
  CHECK(near(end.position, profile.target()));
  CHECK(end.speed == 0.0);
}

void rest_to_rest_trapezoid() {
  std::printf("rest to rest, 1,000 counts up: accelerate, cruise at 800, decelerate\n");
  MotionProfile p;
  CHECK(p.plan(13000, 0, 14000, kLeg));
  // 0.5 s to 800 counts/s (200 counts), 600 counts at 800 (0.75 s), 0.5 s to rest.
  CHECK(near(p.duration(), 1.75));
  CHECK(near(p.at(0.25).speed, 400.0));
  CHECK(near(p.at(0.5).position, 13200.0));
  CHECK(near(p.at(0.5).speed, 800.0));
  CHECK(near(p.at(0.875).position, 13500.0));
  CHECK(near(p.at(1.25).position, 13800.0));
  CHECK(near(p.at(1.5).speed, 400.0));
  CHECK(!p.at(1.7).done);
  CHECK(p.at(1.75).done);
  CHECK(near(p.at(5.0).position, 14000.0));
  CHECK(near(p.at(-1.0).position, 13000.0));
  check_walk(p, kLeg, 0.0);
}

void rest_to_rest_down_and_triangle() {
  std::printf("rest to rest, down, and a short move that never reaches cruise\n");
  MotionProfile down;
  CHECK(down.plan(14000, 0, 13000, kLeg));
  CHECK(near(down.duration(), 1.75));
  CHECK(near(down.at(0.5).speed, -800.0));
  CHECK(near(down.at(0.5).position, 13800.0));
  check_walk(down, kLeg, 0.0);

  MotionProfile tri;
  CHECK(tri.plan(0, 0, 100, kLeg));
  // Peak sqrt(100 x 1600) = 400 counts/s after 0.25 s, at the midpoint.
  CHECK(near(tri.duration(), 0.5));
  CHECK(near(tri.at(0.25).speed, 400.0));
  CHECK(near(tri.at(0.25).position, 50.0));
  check_walk(tri, kLeg, 0.0);
}

void zero_and_invalid() {
  std::printf("a move to where it is, and limits that are not positive\n");
  MotionProfile p;
  CHECK(p.plan(500, 0, 500, kLeg));
  CHECK(p.duration() == 0.0);
  CHECK(p.at(0.0).done);
  CHECK(near(p.setpoint(0.0, kLead), 500.0));

  MotionProfile bad;
  CHECK(!bad.plan(0, 0, 100, {.max_speed = 0.0, .acceleration = 1600.0}));
  CHECK(!bad.plan(0, 0, 100, {.max_speed = 800.0, .acceleration = -1.0}));
  CHECK(bad.at(1.0).done);
  CHECK(near(bad.at(1.0).position, 0.0)); // left at rest on the start, never toward 100
}

void lead_never_past_target() {
  std::printf("setpoint = profile + speed x lead, never past the target\n");
  for (const double target : {14000.0, 12000.0, 13050.0, 12990.0}) {
    MotionProfile p;
    CHECK(p.plan(13000, 0, target, kLeg));
    const double direction = target > 13000 ? 1.0 : -1.0;
    bool saw_lead = false;
    for (double t = 0; t <= p.duration() + 0.1; t += kTick) {
      const double sp = p.setpoint(t, kLead);
      const auto s = p.at(t);
      CHECK((sp - target) * direction <= 1e-9);      // never beyond the target
      CHECK((sp - s.position) * direction >= -1e-9); // never behind the profile
      if (near(sp, s.position + s.speed * kLead)) {
        saw_lead = saw_lead || std::fabs(s.speed) > 0;
      }
    }
    CHECK(saw_lead);
    CHECK(near(p.setpoint(p.duration(), kLead), target));
  }
  // In the cruise the setpoint leads by 800 x 0.25 = 200 counts.
  MotionProfile p;
  CHECK(p.plan(13000, 0, 15000, kLeg));
  CHECK(near(p.setpoint(1.0, kLead), p.at(1.0).position + 200.0));
  // While decelerating below 2 x accel x lead = 800 counts/s the lead would pass the
  // target, so the setpoint is the target for the whole of the final ramp.
  CHECK(near(p.setpoint(p.duration() - 0.3, kLead), 15000.0));
  // No lead is the profile itself.
  CHECK(near(p.setpoint(1.0, 0.0), p.at(1.0).position));
}

/// Plan A, then at `t_switch` plan B from where A is: B must start exactly there, at A's
/// speed, and end on its own target.
void retarget(double first_target, double second_target, const char *what) {
  std::printf("retarget mid-move: %s\n", what);
  MotionProfile a;
  CHECK(a.plan(13000, 0, first_target, kLeg));
  const double t_switch = 0.8;
  const auto here = a.at(t_switch);
  MotionProfile b;
  CHECK(b.plan(here.position, here.speed, second_target, kLeg));
  CHECK(near(b.at(0.0).position, here.position));
  CHECK(near(b.at(0.0).speed, here.speed));
  CHECK(near(b.at(kTick).speed, here.speed, kLeg.acceleration * kTick + 1e-6));
  check_walk(b, kLeg, here.speed);
  CHECK(near(b.at(b.duration()).position, second_target));
}

void retargets() {
  // At t = 0.8 s the first move is cruising at 800 counts/s around 13,440.
  retarget(15000, 16000, "further on, same way: keeps cruising");
  retarget(15000, 14000, "nearer, still ahead and stoppable");
  retarget(15000, 13500, "nearer than the stopping distance: passes it and comes back");
  retarget(15000, 12000, "reverse: brakes, then turns back");

  std::printf("retarget with the speed limit lowered below the current speed\n");
  MotionProfile a;
  CHECK(a.plan(0, 0, 10000, kLeg));
  const auto here = a.at(1.0); // cruising at 800
  const MotionProfile::Limits slow{.max_speed = 200.0, .acceleration = 1600.0};
  MotionProfile b;
  CHECK(b.plan(here.position, here.speed, 10000, slow));
  CHECK(near(b.at(0.5).speed, 200.0)); // slowed from 800 to 200 in 0.375 s, then held
  check_walk(b, slow, here.speed);

  std::printf("retarget while braking past a target: the overshoot comes back\n");
  MotionProfile c;
  CHECK(c.plan(here.position, here.speed, here.position + 50, kLeg)); // stop needs 200
  double furthest = here.position;
  for (double t = 0; t <= c.duration(); t += kTick) {
    furthest = std::max(furthest, c.at(t).position);
  }
  CHECK(near(furthest, here.position + 200.0, 1.0)); // braked to rest 200 counts on
  CHECK(near(c.at(c.duration()).position, here.position + 50));
}

void setpoint_follows_a_retarget() {
  std::printf("setpoints across a retarget stay continuous\n");
  MotionProfile a;
  CHECK(a.plan(13000, 0, 15000, kLeg));
  const double t_switch = 1.0;
  const double before = a.setpoint(t_switch, kLead);
  const auto here = a.at(t_switch);
  MotionProfile b;
  CHECK(b.plan(here.position, here.speed, 16000, kLeg));
  // Same position and speed, so the same setpoint: the controller sees no step.
  CHECK(near(b.setpoint(0.0, kLead), before));
}

} // namespace

int main() {
  rest_to_rest_trapezoid();
  rest_to_rest_down_and_triangle();
  zero_and_invalid();
  lead_never_past_target();
  retargets();
  setpoint_follows_a_retarget();
  if (failures != 0) {
    std::printf("%d check(s) FAILED\n", failures);
    return 1;
  }
  std::printf("all motion profile checks passed\n");
  return 0;
}
