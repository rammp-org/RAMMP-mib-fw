// Host checks for mib::CalibrationHold (calibration_hold.hpp): when a hold may begin, how it
// is renewed and capped, and how it ends. MibSystem's side (state_, the motor command gate,
// the e-stop) needs the board and is not covered here.

#include <chrono>
#include <cstdio>

#include "calibration_hold.hpp"

using namespace std::chrono_literals;
using mib::CalibrationHold;
using End = CalibrationHold::End;
using Refusal = CalibrationHold::Refusal;

namespace {

int failures = 0;

#define CHECK(condition)                                                                           \
  do {                                                                                             \
    if (!(condition)) {                                                                            \
      std::printf("  FAIL %s:%d: %s\n", __FILE__, __LINE__, #condition);                           \
      ++failures;                                                                                  \
    }                                                                                              \
  } while (0)

constexpr CalibrationHold::Config kConfig{.default_duration = 60s, .max_total = 300s};
constexpr CalibrationHold::Conditions kIdle{.idle = true};
const CalibrationHold::Clock::time_point t0 = CalibrationHold::Clock::time_point{} + 1000s;
constexpr uint8_t kLeft = 1u << 2;
constexpr uint8_t kRight = 1u << 3;

void begins_only_from_a_quiet_idle() {
  std::printf("begins_only_from_a_quiet_idle\n");
  CalibrationHold hold(kConfig);
  CHECK(hold.check(kLeft, kIdle, t0) == Refusal::NONE);
  CHECK(hold.check(0, kIdle, t0) == Refusal::NO_AXES);
  CHECK(hold.check(kLeft, {.idle = false}, t0) == Refusal::NOT_IDLE);
  CHECK(hold.check(kLeft, {.idle = true, .estop = true}, t0) == Refusal::ESTOP);
  CHECK(hold.check(kLeft, {.idle = true, .ota = true}, t0) == Refusal::OTA);
  CHECK(hold.check(kLeft, {.idle = true, .override_active = true}, t0) == Refusal::OVERRIDE);
  CHECK(hold.check(kLeft, {.idle = true, .jogging = true}, t0) == Refusal::JOGGING);
  // check() changes nothing.
  CHECK(!hold.active(t0));
  CHECK(hold.mask(t0) == 0);
}

void default_duration_and_expiry() {
  std::printf("default_duration_and_expiry\n");
  CalibrationHold hold(kConfig);
  CHECK(hold.grant(kLeft, 0ms, t0)); // began
  CHECK(hold.active(t0));
  CHECK(hold.mask(t0) == kLeft);
  CHECK(hold.remaining(t0) == 60s);
  CHECK(hold.active(t0 + 59s));
  CHECK(hold.ended(t0 + 59s) == End::NONE);
  CHECK(!hold.active(t0 + 60s));
  CHECK(hold.mask(t0 + 60s) == 0);
  CHECK(hold.remaining(t0 + 60s) == 0ms);
  CHECK(hold.ended(t0 + 60s) == End::TIMED_OUT);
  hold.clear(t0 + 60s);
  CHECK(hold.ended(t0 + 60s) == End::NONE);
}

void renewal_needs_no_idle_and_adds_axes() {
  std::printf("renewal_needs_no_idle_and_adds_axes\n");
  CalibrationHold hold(kConfig);
  hold.grant(kLeft, 0ms, t0);
  // The state is CALIBRATE by now, not IDLE: a renewal is still allowed...
  CHECK(hold.check(kLeft, {.idle = false}, t0 + 30s) == Refusal::NONE);
  // ...but never through the e-stop or a firmware update.
  CHECK(hold.check(kLeft, {.idle = false, .estop = true}, t0 + 30s) == Refusal::ESTOP);
  CHECK(hold.check(kLeft, {.idle = false, .ota = true}, t0 + 30s) == Refusal::OTA);
  CHECK(!hold.grant(kRight, 0ms, t0 + 30s)); // renewed, not begun
  CHECK(hold.mask(t0 + 30s) == (kLeft | kRight));
  CHECK(hold.remaining(t0 + 30s) == 60s);
  CHECK(hold.active(t0 + 89s));
  CHECK(!hold.active(t0 + 90s));
}

void total_cap_holds_however_often_renewed() {
  std::printf("total_cap_holds_however_often_renewed\n");
  CalibrationHold hold(kConfig);
  hold.grant(kLeft, 0ms, t0);
  for (auto t = t0 + 20s; t < t0 + 300s; t += 20s) {
    CHECK(hold.active(t));
    hold.grant(kLeft, 0ms, t);
  }
  CHECK(hold.active(t0 + 299s));
  CHECK(!hold.active(t0 + 300s));
  CHECK(hold.ended(t0 + 300s) == End::CAPPED);
  // A request after the cap is a new hold, and needs IDLE again.
  CHECK(hold.check(kLeft, {.idle = false}, t0 + 301s) == Refusal::NOT_IDLE);
  CHECK(hold.grant(kLeft, 0ms, t0 + 301s));
  CHECK(hold.remaining(t0 + 301s) == 60s);
}

void durations_are_clamped() {
  std::printf("durations_are_clamped\n");
  CalibrationHold hold(kConfig);
  hold.grant(kLeft, 100ms, t0);
  CHECK(hold.remaining(t0) == 1s); // min_duration
  CalibrationHold longer(kConfig);
  longer.grant(kLeft, 3600s, t0);
  CHECK(longer.remaining(t0) == 300s); // max_total
}

void release_per_axis_and_all() {
  std::printf("release_per_axis_and_all\n");
  CalibrationHold hold(kConfig);
  hold.grant(kLeft | kRight, 0ms, t0);
  CHECK(hold.release(kLeft, t0 + 1s) == kRight);
  CHECK(hold.active(t0 + 1s));
  CHECK(hold.mask(t0 + 1s) == kRight);
  CHECK(hold.release(0xFF, t0 + 2s) == 0);
  CHECK(!hold.active(t0 + 2s));
  CHECK(hold.ended(t0 + 2s) == End::RELEASED);
  // Releasing with no hold is harmless.
  CHECK(hold.release(0xFF, t0 + 3s) == 0);
  CHECK(hold.ended(t0 + 3s) == End::RELEASED);
}

void end_is_immediate() {
  std::printf("end_is_immediate\n");
  CalibrationHold hold(kConfig);
  hold.grant(kLeft, 0ms, t0);
  hold.end(End::ESTOP, t0 + 5s);
  CHECK(!hold.active(t0 + 5s));
  CHECK(hold.mask(t0 + 5s) == 0);
  CHECK(hold.ended(t0 + 5s) == End::ESTOP);
  // After the e-stop the next request is a new hold from IDLE, not a renewal.
  CHECK(hold.check(kLeft, {.idle = false}, t0 + 6s) == Refusal::NOT_IDLE);
  CHECK(hold.grant(kLeft, 0ms, t0 + 6s));
  CHECK(hold.ended(t0 + 6s) == End::NONE);
}

} // namespace

int main() {
  begins_only_from_a_quiet_idle();
  default_duration_and_expiry();
  renewal_needs_no_idle_and_adds_axes();
  total_cap_holds_however_often_renewed();
  durations_are_clamped();
  release_per_axis_and_all();
  end_is_immediate();
  if (failures != 0) {
    std::printf("%d failure(s)\n", failures);
    return 1;
  }
  std::printf("all calibration hold checks passed\n");
  return 0;
}
