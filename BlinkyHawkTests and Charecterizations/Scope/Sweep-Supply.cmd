@echo off
powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0Sweep-Supply.ps1" %*
pause
