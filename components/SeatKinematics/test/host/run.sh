#!/usr/bin/env bash
# Build and run the SeatKinematics checks on the host (macOS or Linux). No ESP-IDF needed.
set -euo pipefail
shopt -s nullglob
here="$(cd "$(dirname "$0")" && pwd)"
component="$here/../.."
sources=("$here/main.cpp" "$component"/src/*.cpp)
out="$(mktemp -d)"
c++ -std=c++20 -Wall -Wextra -Werror \
  -I "$component/include" -I "$component/test" -I "$component/../MIBConfig/include" \
  "${sources[@]}" -o "$out/seat_kinematics_host_test"
"$out/seat_kinematics_host_test"
