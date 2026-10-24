@echo off
setlocal
cd /d "%~dp0"
title Obstacle Detection - Install

rem Prefer the "py" launcher; fall back to "python" on PATH.
set "PY="
where py >nul 2>nul && set "PY=py -3"
if not defined PY where python >nul 2>nul && set "PY=python"
if not defined PY (
    echo Python was not found.
    echo Install Python 3.10 or newer from https://www.python.org/downloads/
    echo and tick "Add python.exe to PATH" during setup. Then run install.bat again.
    pause
    exit /b 1
)

if not exist venv (
    echo Creating virtual environment...
    %PY% -m venv venv || (echo Failed to create the virtual environment. & pause & exit /b 1)
)

echo Installing packages...
venv\Scripts\python.exe -m pip install --upgrade pip
venv\Scripts\python.exe -m pip install -r requirements.txt || (echo Package install failed. & pause & exit /b 1)

echo.
echo YOLO gives much better detection but is a large download (about 2 GB, includes PyTorch).
choice /c YN /m "Install YOLO now"
if errorlevel 2 goto done
venv\Scripts\python.exe -m pip install -r requirements-yolo.txt || echo YOLO install failed - the app will use the basic detector.

:done
echo.
echo Done. Start the app with run.bat
pause
