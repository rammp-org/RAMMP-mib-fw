#!/bin/sh
# Start the debugger. Pass --interface <name> to choose the port facing the board,
# or --local to talk to the simulator started by sim.sh.
cd "$(dirname "$0")"
VENV_PY=.venv/bin/python
[ -x "$VENV_PY" ] || VENV_PY=.venv/Scripts/python.exe   # Git Bash on Windows
[ -x "$VENV_PY" ] || { echo "Run ./setup.sh first (setup.cmd on Windows)." >&2; exit 1; }
exec "$VENV_PY" -u app.py "$@"
