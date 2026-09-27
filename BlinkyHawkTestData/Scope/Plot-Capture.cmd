@echo off
rem Double-click to plot the newest capture, or drag one or more CSVs onto this file.
powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0Plot-Capture.ps1" %*
if errorlevel 1 pause
