@echo off
setlocal
cd /d "%~dp0"

if not exist venv\Scripts\python.exe (
    echo First run: installing...
    call install.bat || exit /b 1
)

if not exist app.pid goto launch
set /p PID=<app.pid
tasklist /FI "PID eq %PID%" 2>nul | find "%PID%" >nul && (
    echo Obstacle Detection is already running. Use stop.bat to close it.
    timeout /t 4 >nul
    exit /b 0
)

:launch

rem Runs in a minimized console; if the app crashes, the console stays open to show the error.
start "Obstacle Detection (console)" /min cmd /c "venv\Scripts\python.exe app.py  %* || pause"
