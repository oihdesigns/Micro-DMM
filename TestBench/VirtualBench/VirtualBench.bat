@echo off
rem Virtual Bench -- opens http://127.0.0.1:8765 in the browser.  Ctrl+C here to quit.
cd /d "%~dp0"
py -3 virtual_bench.py %*
pause
