#!/bin/sh
# Create a virtual environment for the MIB debugger with a Python that has a
# prebuilt CycloneDDS wheel. Run once, then use run.sh.
set -e
cd "$(dirname "$0")"

pick_python() {
  for candidate in /usr/bin/python3 python3.12 python3.11 python3.10 python3.9 python3; do
    if command -v "$candidate" >/dev/null 2>&1; then
      version=$("$candidate" -c 'import sys; print(sys.version_info.minor)' 2>/dev/null || echo 99)
      if [ "$version" -ge 9 ] && [ "$version" -le 12 ]; then
        echo "$candidate"; return 0
      fi
    fi
  done
  return 1
}

PY=$(pick_python) || {
  echo "No Python 3.9 to 3.12 found. CycloneDDS ships no wheel for newer versions." >&2
  echo "Install one with: brew install python@3.12" >&2
  exit 1
}

echo "Using $PY ($("$PY" --version))"
"$PY" -m venv .venv
.venv/bin/pip install --quiet --upgrade pip
.venv/bin/pip install --quiet -r requirements.txt
echo
echo "Ready. Start the debugger with:  ./run.sh"
echo "Or test without hardware:        ./run.sh --local   (and ./sim.sh in another terminal)"
