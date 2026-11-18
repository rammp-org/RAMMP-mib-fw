// Host test of the transmit filter (main/bus_policy.hpp). No ESP-IDF needed:
//   c++ -std=c++20 -I../main bus_policy_test.cpp -o /tmp/bus_policy_test && /tmp/bus_policy_test

#include <cstdio>
#include <initializer_list>

#include "bus_policy.hpp"

using namespace sysid;

static int failures = 0;

static bool allowed(uint32_t id, std::initializer_list<uint8_t> bytes, const NodeSet &ro,
                    uint8_t heartbeat = 0) {
  uint8_t data[8] = {};
  uint8_t n = 0;
  for (uint8_t b : bytes) {
    data[n++] = b;
  }
  return check_frame({.id = id, .extended = false, .rtr = false, .dlc = n, .data = data}, ro,
                     heartbeat)
      .allow;
}

static void expect(bool got, bool want, const char *what) {
  if (got != want) {
    std::printf("FAIL: %s (got %s)\n", what, got ? "allow" : "block");
    ++failures;
  }
}

int main() {
  const NodeSet ro2 = to_node_set(parse_node_list("2"));
  NodeSet all;
  for (int id = 1; id < 128; ++id) {
    all.set(id);
  }

  // Read-only node 2: reads pass, everything else is blocked.
  expect(allowed(0x602, {0x40, 0x41, 0x60, 0, 0, 0, 0, 0}, ro2), true, "upload initiate to 2");
  expect(allowed(0x602, {0x60, 0, 0, 0, 0, 0, 0, 0}, ro2), true, "upload segment (t=0) to 2");
  expect(allowed(0x602, {0x70, 0, 0, 0, 0, 0, 0, 0}, ro2), true, "upload segment (t=1) to 2");
  expect(allowed(0x602, {0x2B, 0x40, 0x60, 0, 0, 0, 0, 0}, ro2), false, "controlword write to 2");
  expect(allowed(0x602, {0x23, 0x7A, 0x60, 0, 1, 0, 0, 0}, ro2), false, "target write to 2");
  expect(allowed(0x602, {0x80, 0, 0, 0, 0, 0, 0, 0}, ro2), false, "SDO abort to 2");
  expect(allowed(0x202, {1, 2}, ro2), false, "RPDO1 to 2");
  expect(allowed(0x502, {1, 2}, ro2), false, "RPDO4 to 2");
  expect(allowed(0x000, {0x01, 0x02}, ro2), false, "NMT start node 2");
  // Broadcasts reach node 2: always blocked.
  expect(allowed(0x000, {0x01, 0x00}, ro2), false, "NMT start all");
  expect(allowed(0x080, {}, ro2), false, "SYNC");
  expect(allowed(0x100, {0, 0, 0, 0, 0, 0}, ro2), false, "TIME");
  expect(allowed(0x7E5, {0x04, 0x01}, ro2), false, "LSS");
  // Writable node 3.
  expect(allowed(0x603, {0x2B, 0x40, 0x60, 0, 0x0F, 0, 0, 0}, ro2), true, "controlword write to 3");
  expect(allowed(0x000, {0x01, 0x03}, ro2), true, "NMT start node 3");
  // Heartbeat: only our own id, only when we have one.
  expect(allowed(0x77F, {0x05}, ro2, 127), true, "own heartbeat");
  expect(allowed(0x77F, {0x05}, ro2, 0), false, "heartbeat when none configured");
  // Passive build: every node read-only.
  expect(allowed(0x603, {0x40, 0x64, 0x60, 0, 0, 0, 0, 0}, all), true, "passive upload to 3");
  expect(allowed(0x603, {0x2B, 0x40, 0x60, 0, 0, 0, 0, 0}, all), false, "passive write to 3");
  expect(allowed(0x000, {0x01, 0x03}, all), false, "passive NMT to 3");
  // Parsing.
  const auto ids = parse_node_list("1, 3,0,200,x,127");
  expect(ids.size() == 3 && ids[0] == 1 && ids[1] == 3 && ids[2] == 127, true, "parse list");

  std::printf("%s (%d failure(s))\n", failures ? "FAILED" : "all passed", failures);
  return failures ? 1 : 0;
}
