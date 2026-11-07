#!/usr/bin/env bash
# Fetch what the bench needs, pinned, and build it. Run through pixi: `pixi run setup`.
# - rammp-interfaces-ros2 at the v2.0.0 release: AcquireControl's pre-supplied token,
#   rammp_base_interfaces' TwistSetpoint, DriveCommand and MibStatus, built with colcon.
# - The arm's arbitration conformance checks from kinova-gen3-ros2, run unchanged.
set -euo pipefail
here="$(cd "$(dirname "$0")" && pwd)"
rev=c2e3c0d # rammp-interfaces-ros2 main, v2.0.0: adds DriveCommand and MibStatus
src="$here/ws/src/rammp-interfaces-ros2"
if [ ! -d "$src" ]; then
  git clone -q https://github.com/rammp-org/rammp-interfaces-ros2.git "$src"
fi
git -C "$src" fetch -q origin
git -C "$src" checkout -q "$rev"
checks_rev=228c48ea36486aa74a71853f3aca178ac563fd4d # kinova-gen3-ros2, PR #58
curl -fsSL -o "$here/conformance/checks_arbitration.py" \
  "https://raw.githubusercontent.com/rammp-org/kinova-gen3-ros2/$checks_rev/kinova_gen3_ros2/test/conformance/checks_arbitration.py"

cd "$here/ws"
colcon build --packages-select rammp_common_interfaces rammp_base_interfaces \
  --cmake-args -DBUILD_TESTING=OFF -G Ninja
