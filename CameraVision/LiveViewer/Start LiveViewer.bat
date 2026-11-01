@echo off
rem Starts the viewer in its own window and opens the page. Builds it first if needed.
cd /d "%~dp0"
if not exist bin\LiveViewer.exe "%SystemRoot%\System32\WindowsPowerShell\v1.0\powershell.exe" -NoProfile -ExecutionPolicy Bypass -File build.ps1
start "LiveViewer - close from the web page" bin\LiveViewer.exe
rem Gives the viewer two seconds to start before the browser asks for the page.
"%SystemRoot%\System32\ping.exe" -n 3 127.0.0.1 >nul
start "" http://localhost:8765/
