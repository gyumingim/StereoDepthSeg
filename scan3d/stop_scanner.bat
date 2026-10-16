@echo off
rem 3D scanner: stop recording cleanly (STOP broadcast), then stop relay, WSL scanner and phone app
powershell.exe -NoProfile -ExecutionPolicy Bypass -File "%~dp0start_scanner.ps1" -Stop
pause
