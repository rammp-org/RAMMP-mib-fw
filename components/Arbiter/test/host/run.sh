#!/usr/bin/env bash
# Build and run the Arbiter checks on the host (macOS or Linux), with sanitizers.
# No ESP-IDF toolchain needed, but the espp headers come from the component manager:
# run `idf.py reconfigure` (or any build) in the repo root once to fetch them.
set -euo pipefail
here="$(cd "$(dirname "$0")" && pwd)"
component="$here/../.."
managed="${MANAGED_COMPONENTS:-$component/../../managed_components}"
if [ ! -d "$managed/espp__logger" ]; then
  echo "espp headers not found in $managed; run 'idf.py reconfigure' in the repo root" >&2
  exit 1
fi
out="${OUT_DIR:-$(mktemp -d)}"
# ASan deadlocks at startup under Apple clang on recent macOS, so it runs on Linux only.
sanitizers=undefined
if [ "$(uname)" = Linux ]; then sanitizers=address,undefined; fi
c++ -std=c++20 -Wall -Wextra -Werror -g -pthread -fsanitize="$sanitizers" \
  -I "$component/include" \
  -I "$managed/espp__base_component/include" \
  -I "$managed/espp__logger/include" \
  -I "$managed/espp__format/include" \
  -isystem "$managed/espp__format/detail/fmt/include" \
  "$here/main.cpp" "$component/src/arbiter.cpp" "$managed/espp__logger/src/logger.cpp" \
   -o "$out/arbiter_host_test"
"$out/arbiter_host_test"
