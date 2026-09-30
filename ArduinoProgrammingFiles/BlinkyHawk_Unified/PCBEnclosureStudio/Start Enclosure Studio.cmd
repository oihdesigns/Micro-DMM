@echo off
cd /d "%~dp0"
set "STUDIO_PYTHON=%USERPROFILE%\.cache\codex-runtimes\codex-primary-runtime\dependencies\python\python.exe"
if exist "%STUDIO_PYTHON%" (
  "%STUDIO_PYTHON%" launcher.py
) else (
  python launcher.py
)
if errorlevel 1 pause
