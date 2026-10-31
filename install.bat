@echo off
setlocal
cd /d "%~dp0"
title Obstacle Detection - Install

rem Prefer a Python that numpy / opencv / PyTorch ship ready-made packages for (a brand-new
rem Python makes pip try to compile numpy, which fails). Fall back to "python" on PATH.
set "PY="
for %%v in (3.12 3.11 3.13 3.10) do if not defined PY py -%%v -c "" >nul 2>nul && set "PY=py -%%v"
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
venv\Scripts\python.exe --version
rem --only-binary: never compile; fail with a clear message instead.
venv\Scripts\python.exe -m pip install --only-binary=:all: -r requirements.txt || (
    echo Package install failed. Your Python is probably too new or 32-bit.
    echo Install 64-bit Python 3.12 from https://www.python.org/downloads/ , delete the venv folder,
    echo then run install.bat again.
    pause & exit /b 1
)

echo.
echo YOLO gives much better detection but is a large download (about 2 GB, includes PyTorch).
choice /c YN /m "Install YOLO now"
if errorlevel 2 goto done
venv\Scripts\python.exe -m pip install -r requirements-yolo.txt || echo YOLO install failed - the app will use the basic detector.

:done
echo.
echo Done. Start the app with run.bat
pause
