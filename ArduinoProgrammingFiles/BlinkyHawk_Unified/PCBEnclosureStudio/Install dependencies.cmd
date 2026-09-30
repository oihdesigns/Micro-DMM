@echo off
cd /d "%~dp0"
set "STUDIO_PYTHON=%USERPROFILE%\.cache\codex-runtimes\codex-primary-runtime\dependencies\python\python.exe"
if not exist "%STUDIO_PYTHON%" set "STUDIO_PYTHON=python"
"%STUDIO_PYTHON%" -m pip install --target .runtime -r requirements.txt
if errorlevel 1 (
  echo Installation failed. Python 3.12 or 3.13, 64-bit, is recommended.
  pause
  exit /b 1
)
echo Installation finished. Open Start Enclosure Studio.cmd.
pause
