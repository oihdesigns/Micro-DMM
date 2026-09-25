@echo off
powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0Capture-Scope.ps1" %*
pause
