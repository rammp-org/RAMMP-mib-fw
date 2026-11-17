#!/usr/bin/env bash
# Build and run the SeatControl checks on the host (macOS or Linux). No ESP-IDF needed: the
# SeatKinematics host test's stubs stand in for the headers MIBconfig.hpp includes from ESP-IDF
# and the Arbiter component.
set -euo pipefail
here="$(cd "$(dirname "$0")" && pwd)"
component="$here/../.."
kinematics="$component/../SeatKinematics"
out="$(mktemp -d)"
c++ -std=c++20 -Wall -Wextra -Wshadow -Werror \
  -I "$component/include" -I "$kinematics/include" -I "$kinematics/test" \
  -I "$component/../MIBConfig/include" -I "$kinematics/test/host/stubs" \
  "$here/main.cpp" "$component/src/seat_leg_map.cpp" "$kinematics/src/seat_kinematics.cpp" \
  -o "$out/seat_control_host_test"
"$out/seat_control_host_test"
