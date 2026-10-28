@echo off
rem Create a virtual environment for the MIB debugger with a Python that has a
rem prebuilt espp wheel (CPython 3.10 to 3.14). Run once, then run.cmd.
setlocal
cd /d "%~dp0"
set "PY="
for %%v in (3.14 3.13 3.12 3.11 3.10) do (
  if not defined PY (
    py -%%v -c "import sys" >nul 2>&1 && set "PY=py -%%v"
  )
)
if not defined PY (
  python -c "import sys; sys.exit(0 if (3,10) <= sys.version_info[:2] <= (3,14) else 1)" >nul 2>&1 && set "PY=python"
)
if not defined PY (
  echo No Python 3.10 to 3.14 found; the espp wheel needs one of those. 1>&2
  echo Install one with:  winget install Python.Python.3.12   ^(then reopen this terminal^) 1>&2
  exit /b 1
)
echo Using %PY%
%PY% -m venv .venv || exit /b 1
.venv\Scripts\python.exe -m pip install --quiet --upgrade pip || exit /b 1
.venv\Scripts\python.exe -m pip install --quiet -r requirements.txt || exit /b 1
echo.
echo Ready. Start the debugger with:  run.cmd
echo Or test without hardware:        run.cmd --local   ^(and sim.cmd in another terminal^)
echo Windows Firewall asks once whether Python may receive on the network: allow it on
echo both private and public networks, or the board's packets are dropped.
