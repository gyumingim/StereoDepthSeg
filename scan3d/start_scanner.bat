@echo off
rem 3D scanner: start the idle server if needed, then open http://localhost:8795 (normally not needed - autostart does it)
powershell.exe -NoProfile -ExecutionPolicy Bypass -File "%~dp0start_scanner.ps1"
