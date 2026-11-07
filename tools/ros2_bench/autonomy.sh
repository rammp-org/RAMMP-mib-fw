#!/usr/bin/env bash
# Drive an autonomy session on a running MIB (THE CHAIR MOVES). Through pixi:
# `pixi run autonomy [--manual] [--speed m/s] [--nudge s]`.
set -eo pipefail # no -u: the ROS setup scripts read unset variables
here="$(cd "$(dirname "$0")" && pwd)"
source "$here/env.sh"
cd "$here/conformance"
exec python3 run_autonomy.py "$@"
