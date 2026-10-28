#!/bin/sh
# Create a virtual environment for the MIB debugger with a Python that has a
# prebuilt CycloneDDS wheel (CPython 3.9 or 3.10). Run once, then use run.sh.
# On Windows use setup.cmd instead.
set -e
cd "$(dirname "$0")"

pick_python() {
  for candidate in /usr/bin/python3 python3.10 python3.9 python3; do
    if command -v "$candidate" >/dev/null 2>&1; then
      version=$("$candidate" -c 'import sys; print(sys.version_info.minor)' 2>/dev/null || echo 99)
      if [ "$version" -ge 9 ] && [ "$version" -le 10 ]; then
        echo "$candidate"; return 0
      fi
    fi
  done
  return 1
}

PY=$(pick_python) || {
  echo "No Python 3.9 or 3.10 found. CycloneDDS 0.10 ships no wheel for other versions." >&2
  echo "Install one with: brew install python@3.10" >&2
  exit 1
}

echo "Using $PY ($("$PY" --version))"
"$PY" -m venv .venv
VENV_PY=.venv/bin/python
[ -x "$VENV_PY" ] || VENV_PY=.venv/Scripts/python.exe   # Git Bash on Windows
"$VENV_PY" -m pip install --quiet --upgrade pip
"$VENV_PY" -m pip install --quiet -r requirements.txt
echo
echo "Ready. Start the debugger with:  ./run.sh"
echo "Or test without hardware:        ./run.sh --local   (and ./sim.sh in another terminal)"
