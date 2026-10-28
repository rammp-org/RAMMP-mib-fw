@echo off
rem Start the firmware stand-in on loopback. Use with:  run.cmd --local
cd /d "%~dp0"
if not exist .venv\Scripts\python.exe (
  echo Run setup.cmd first. 1>&2
  exit /b 1
)
.venv\Scripts\python.exe -u fake_mib.py --local %*
