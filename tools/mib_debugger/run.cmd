@echo off
rem Start the debugger. Pass --interface <name or address> to choose the port
rem facing the board, or --local to talk to the simulator started by sim.cmd.
cd /d "%~dp0"
if not exist .venv\Scripts\python.exe (
  echo Run setup.cmd first. 1>&2
  exit /b 1
)
.venv\Scripts\python.exe -u app.py %*
