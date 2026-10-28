#!/usr/bin/env bash
# Run the arbitration conformance checks against a running MIB. Through pixi:
# `pixi run conformance [--serial <port>]`.
set -eo pipefail # no -u: the ROS setup scripts read unset variables
here="$(cd "$(dirname "$0")" && pwd)"
source "$here/env.sh"
cd "$here/conformance"
exec python3 run_conformance.py "$@"
