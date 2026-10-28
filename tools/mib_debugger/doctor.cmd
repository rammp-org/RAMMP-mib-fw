@echo off
rem Check the path to the MIB step by step. Same arguments as doctor.py.
cd /d "%~dp0"
if not exist .venv\Scripts\python.exe (
  echo Run setup.cmd first. 1>&2
  exit /b 1
)
.venv\Scripts\python.exe -u doctor.py %*
