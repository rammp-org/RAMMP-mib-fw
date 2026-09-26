#!/bin/sh
# Start the firmware stand-in on loopback. Use with:  ./run.sh --local
cd "$(dirname "$0")"
[ -x .venv/bin/python ] || { echo "Run ./setup.sh first." >&2; exit 1; }
exec .venv/bin/python -u fake_mib.py --local "$@"
