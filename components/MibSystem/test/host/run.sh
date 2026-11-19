#!/usr/bin/env bash
# Build and run the calibration hold checks on the host (macOS or Linux), with sanitizers.
# calibration_hold.hpp is plain C++20: no ESP-IDF toolchain and no espp headers needed.
set -euo pipefail
here="$(cd "$(dirname "$0")" && pwd)"
component="$here/../.."
out="${OUT_DIR:-$(mktemp -d)}"
# ASan deadlocks at startup under Apple clang on recent macOS, so it runs on Linux only.
sanitizers=undefined
if [ "$(uname)" = Linux ]; then sanitizers=address,undefined; fi
c++ -std=c++20 -Wall -Wextra -Werror -g -fsanitize="$sanitizers" \
  -I "$component/include" \
  "$here/main.cpp" -o "$out/calibration_hold_host_test"
"$out/calibration_hold_host_test"
