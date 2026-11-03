// Host test for push_assist.hpp: the conversion against hand-worked cases.
// Run: components/PushAssist/test/run.sh
#include <cmath>
#include <cstdio>
#include <cstdlib>

#include "push_assist.hpp"

using namespace mib::push_assist;

static int failures = 0;

static void expect_near(const char *what, float got, float want, float tol = 1e-5f) {
  if (!(std::fabs(got - want) <= tol)) {
    std::printf("FAIL %s: got %.6f, want %.6f\n", what, got, want);
    failures++;
  }
}

static void expect_twist(const char *what, float l, float r, float lin, float ang,
                         const Gains &g = kDefaultGains) {
  const auto t = push_to_twist(l, r, g);
  char buf[160];
  std::snprintf(buf, sizeof buf, "%s linear", what);
  expect_near(buf, t.linear, lin);
  std::snprintf(buf, sizeof buf, "%s angular", what);
  expect_near(buf, t.angular, ang);
}

int main() {
  // Defaults: dead band 10 N, forward 60 N, turn 30 N, reverse cap 0.3.
  expect_twist("rest", 0, 0, 0, 0);
  expect_twist("inside the dead band", 9.9f, -10.0f, 0, 0);
  // 25 N each: 15 past the band each, 30/60 forward, no turn.
  expect_twist("even push", 25, 25, 0.5f, 0);
  // 40 + 10: 30 and 0 past the bands. Linear 30/60, angular -(30)/30 = -1 (clockwise).
  expect_twist("left harder", 40, 10, 0.5f, -1.0f);
  // Right alone: 0 and 30 past the bands. Linear 0.5, angular +1 (counter-clockwise).
  expect_twist("right alone", 10, 40, 0.5f, 1.0f);
  // Over full scale: 100 + 100 = 180 past the bands, linear 3: scaled to 1.
  expect_twist("hard push", 100, 100, 1.0f, 0);
  // Over full scale with a turn: l 90, r 30 past the bands: linear 2, angular -2, scale 2.
  expect_twist("hard push turning", 100, 40, 1.0f, -1.0f);
  // Pull: -40 each, -30 each past the bands: linear -1, capped at -0.3.
  expect_twist("pull", -40, -40, -0.3f, 0);
  // One amp missing: NaN stops.
  expect_twist("left NaN", NAN, 20, 0, 0);
  expect_twist("right NaN", 20, NAN, 0, 0);

  // Custom gains: no dead band, so the forces map straight through.
  Gains g = kDefaultGains;
  g.dead_band_n = 0;
  expect_twist("no dead band", 15, 15, 0.5f, 0, g);

  // Param table: every key fits NVS, every default is inside its range, and lookups work.
  for (const auto &p : kParams) {
    if (p.key.size() > 15) {
      std::printf("FAIL key %.*s is longer than an NVS key\n", int(p.key.size()), p.key.data());
      failures++;
    }
    const float d = kDefaultGains.*(p.field);
    if (d < p.min || d > p.max) {
      std::printf("FAIL default for %.*s is outside its range\n", int(p.key.size()), p.key.data());
      failures++;
    }
  }
  if (find_param("turn_full_n") == nullptr || find_param("nope") != nullptr) {
    std::printf("FAIL find_param\n");
    failures++;
  }
  expect_near("clamp high", clamp_param(*find_param("reverse_max"), 5.0f), 1.0f);
  expect_near("clamp NaN", clamp_param(*find_param("forward_full_n"), NAN), 10.0f);

  if (failures) {
    std::printf("%d failure(s)\n", failures);
    return EXIT_FAILURE;
  }
  std::printf("push_assist host test: all passed\n");
  return EXIT_SUCCESS;
}
