#include "motion_profile.hpp"

#include <algorithm>
#include <cmath>

namespace mib {

void MotionProfile::add(double &t, double &p, double &v, double a, double duration) {
  if (!(duration > 0.0)) {
    return;
  }
  segments_[count_++] = {.t0 = t, .p0 = p, .v0 = v, .a = a, .duration = duration};
  p += v * duration + 0.5 * a * duration * duration;
  v += a * duration;
  t += duration;
}

bool MotionProfile::plan(double start, double start_speed, double target, const Limits &limits) {
  count_ = 0;
  start_ = start;
  target_ = start;
  duration_ = 0.0;
  if (!(limits.max_speed > 0.0) || !(limits.acceleration > 0.0)) {
    return false;
  }
  target_ = target;
  const double a = limits.acceleration;
  double t = 0.0;
  double p = start;
  double v = start_speed;

  // 1. Brake to rest first if the start speed heads away from the target, or is too fast
  //    to stop before reaching it. The profile then starts again from rest.
  const double to_go = target - p;
  if (v != 0.0 && (v * to_go < 0.0 || v * v / (2.0 * a) > std::fabs(to_go))) {
    add(t, p, v, v > 0.0 ? -a : a, std::fabs(v) / a);
    v = 0.0;
  }

  // 2. From (p, v), with v zero or toward the target and stoppable in time: reach the peak
  //    speed, cruise, and decelerate to rest on the target.
  const double distance = std::fabs(target - p);
  if (distance > 0.0) {
    const double direction = target > p ? 1.0 : -1.0;
    const double u = std::fabs(v); // speed toward the target
    // The peak of a triangle that ends at rest on the target; capped at max_speed. If u is
    // above max_speed (the limits were lowered mid-move), the first segment slows to it.
    const double triangle_peak = std::sqrt((2.0 * a * distance + u * u) / 2.0);
    const double peak = std::min(limits.max_speed, triangle_peak);
    const double ramp_distance = std::fabs(peak * peak - u * u) / (2.0 * a);
    const double stop_distance = peak * peak / (2.0 * a);
    const double cruise_distance = std::max(0.0, distance - ramp_distance - stop_distance);
    add(t, p, v, direction * (peak >= u ? a : -a), std::fabs(peak - u) / a);
    if (peak > 0.0) {
      add(t, p, v, 0.0, cruise_distance / peak);
    }
    add(t, p, v, -direction * a, peak / a);
  }
  duration_ = t;
  return true;
}

MotionProfile::Sample MotionProfile::at(double t) const {
  if (count_ == 0 || t >= duration_) {
    return {.position = count_ == 0 ? start_ : target_, .speed = 0.0, .done = true};
  }
  if (t <= 0.0) {
    return {.position = start_, .speed = segments_[0].v0, .done = false};
  }
  for (size_t i = 0; i < count_; ++i) {
    const Segment &s = segments_[i];
    if (t < s.t0 + s.duration || i + 1 == count_) {
      const double dt = t - s.t0;
      return {.position = s.p0 + s.v0 * dt + 0.5 * s.a * dt * dt,
              .speed = s.v0 + s.a * dt,
              .done = false};
    }
  }
  return {.position = target_, .speed = 0.0, .done = true}; // not reached
}

double MotionProfile::setpoint(double t, double lead_s) const {
  const Sample s = at(t);
  if (s.done) {
    return target_;
  }
  const double led = s.position + s.speed * std::max(lead_s, 0.0);
  const double to_target = target_ - s.position;
  const double to_led = led - s.position;
  // Ahead of the profile, toward the target, but beyond it: hold the target instead.
  if (to_target * to_led > 0.0 && std::fabs(to_led) > std::fabs(to_target)) {
    return target_;
  }
  return led;
}

double MotionProfile::shaped_setpoint(double t, double lead_s, double shaper_s) const {
  if (shaper_s <= 0.0) {
    return setpoint(t, lead_s);
  }
  return 0.5 * (setpoint(t, lead_s) + setpoint(t - shaper_s, lead_s));
}

} // namespace mib
