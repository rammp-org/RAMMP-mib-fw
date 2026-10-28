#!/bin/sh
# Start the firmware stand-in on loopback. Use with:  ./run.sh --local
cd "$(dirname "$0")"
VENV_PY=.venv/bin/python
[ -x "$VENV_PY" ] || VENV_PY=.venv/Scripts/python.exe   # Git Bash on Windows
[ -x "$VENV_PY" ] || { echo "Run ./setup.sh first (setup.cmd on Windows)." >&2; exit 1; }
exec "$VENV_PY" -u fake_mib.py --local "$@"
