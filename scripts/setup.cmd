@echo off
rem Double-click to set up the environment and run all checks (see README.md).
powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0setup.ps1" %*
echo.
pause
