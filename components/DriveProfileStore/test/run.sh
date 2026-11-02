#!/usr/bin/env bash
# Build and run the DriveProfileStore host test (macOS or Linux, no ESP-IDF needed).
#
# espp's headers come from, in order: ESPP_DIR (an espp checkout with its fmt and alpaca
# submodules), managed_components/ after an `idf.py build`, or a shallow clone of espp at
# ESPP_REF. stubs/ stands in for the one ESP-IDF header MIBconfig.hpp includes.
set -euo pipefail
here="$(cd "$(dirname "$0")" && pwd)"
repo="$(cd "$here/../../.." && pwd)"
out="$(mktemp -d)"
managed="$repo/managed_components"

if [[ -z "${ESPP_DIR:-}" && ! -d "$managed/espp__persistent_data" ]]; then
  ESPP_DIR="$out/espp"
  git -c advice.detachedHead=false clone --quiet --depth 1 --branch "${ESPP_REF:-v1.3.11}" \
    https://github.com/esp-cpp/espp "$ESPP_DIR"
  git -C "$ESPP_DIR" submodule update --quiet --init --depth 1 \
    components/format/detail/fmt components/serialization/detail/alpaca
fi

# One espp component's directory, in either layout.
espp() {
  if [[ -n "${ESPP_DIR:-}" ]]; then
    echo "$ESPP_DIR/components/$1"
  else
    echo "$managed/espp__$1"
  fi
}

includes=()
for component in base_component logger format persistent_data serialization file_system \
  trajectory_planner task timer; do
  includes+=(-isystem "$(espp "$component")/include")
done
includes+=(-isystem "$(espp format)/detail/fmt/include")
includes+=(-isystem "$(espp serialization)/detail/alpaca/include")

"${CXX:-c++}" -std=c++23 -Wall -Wextra -Werror \
  -I "$repo/components/DriveProfileStore/include" -I "$repo/components/DriveController/include" \
  -I "$repo/components/MIBConfig/include" \
  -I "$repo/external/rammp-rtps/components/rammp_rtps_messages/include" \
  -I "$here/stubs" "${includes[@]}" -DFMT_HEADER_ONLY \
  "$here/drive_profile_store_host_test.cpp" "$repo/components/DriveProfileStore/src/drive_profile_store.cpp" \
  "$(espp file_system)/src/file_system.cpp" "$(espp logger)/src/logger.cpp" \
  -o "$out/drive_profile_store_test"
"$out/drive_profile_store_test"
