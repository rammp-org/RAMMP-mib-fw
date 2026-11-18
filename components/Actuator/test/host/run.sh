#!/usr/bin/env bash
# Build and run the Actuator's motion-profile checks on the host (macOS or Linux), with
# sanitizers. The profile is plain C++ with no espp or ESP-IDF dependency, so unlike the
# Arbiter checks this needs no `idf.py reconfigure` first.
set -euo pipefail
here="$(cd "$(dirname "$0")" && pwd)"
component="$here/../.."
out="${OUT_DIR:-$(mktemp -d)}"
# ASan deadlocks at startup under Apple clang on recent macOS, so it runs on Linux only.
sanitizers=undefined
if [ "$(uname)" = Linux ]; then sanitizers=address,undefined; fi
c++ -std=c++20 -Wall -Wextra -Werror -g -fsanitize="$sanitizers" \
  -I "$component/include" \
  "$here/main.cpp" "$component/src/motion_profile.cpp" \
  -o "$out/actuator_host_test"
"$out/actuator_host_test"
