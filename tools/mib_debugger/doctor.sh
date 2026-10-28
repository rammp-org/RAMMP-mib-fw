#!/bin/sh
# Check the path to the MIB step by step. Same arguments as doctor.py.
cd "$(dirname "$0")"
VENV_PY=.venv/bin/python
[ -x "$VENV_PY" ] || VENV_PY=.venv/Scripts/python.exe
[ -x "$VENV_PY" ] || { echo "Run ./setup.sh first (setup.cmd on Windows)." >&2; exit 1; }
exec "$VENV_PY" -u doctor.py "$@"
