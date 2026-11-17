#!/usr/bin/env bash
# Build and run the SeatKinematics checks on the host (macOS or Linux). No ESP-IDF needed:
# stubs/ stands in for the headers MIBconfig.hpp includes from ESP-IDF and the Arbiter component.
set -euo pipefail
shopt -s nullglob
here="$(cd "$(dirname "$0")" && pwd)"
component="$here/../.."
sources=("$here/main.cpp" "$component"/src/*.cpp)
out="$(mktemp -d)"
c++ -std=c++20 -Wall -Wextra -Wshadow -Werror \
  -I "$component/include" -I "$component/test" -I "$component/../MIBConfig/include" \
  -I "$here/stubs" \
  "${sources[@]}" -o "$out/seat_kinematics_host_test"
"$out/seat_kinematics_host_test"
