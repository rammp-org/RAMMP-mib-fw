#!/bin/sh
# Start the debugger. Pass --interface <name> to choose the port facing the board,
# or --local to talk to the simulator started by sim.sh.
cd "$(dirname "$0")"
[ -x .venv/bin/python ] || { echo "Run ./setup.sh first." >&2; exit 1; }
exec .venv/bin/python -u app.py "$@"
