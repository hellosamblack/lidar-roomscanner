@echo off
rem ---------------------------------------------------------------------------
rem view-latest-scan.bat — replay the newest scan recording in the 3D viewer.
rem Bootstraps the Python venv and dependencies on first run (needs Python
rem 3.11 or 3.12 installed; open3d has no wheels for 3.13+).
rem ---------------------------------------------------------------------------
setlocal
cd /d "%~dp0"

set "VENV_PY=host\.venv\Scripts\python.exe"

if not exist "%VENV_PY%" (
    echo [setup] Creating virtual environment...
    py -3.12 -m venv host\.venv 2>nul || py -3.11 -m venv host\.venv || (
        echo [error] Python 3.11 or 3.12 is required ^(py launcher couldn't find one^).
        pause
        exit /b 1
    )
)

"%VENV_PY%" -c "import open3d, numpy, serial, roomscan" 2>nul || (
    echo [setup] Installing dependencies ^(open3d is ~100 MB, first run takes a few minutes^)...
    "%VENV_PY%" -m pip install --quiet --upgrade pip
    "%VENV_PY%" -m pip install --quiet -e host || (
        echo [error] Dependency installation failed.
        pause
        exit /b 1
    )
)

set "LATEST="
for /f "delims=" %%f in ('dir /b /o-d recordings\*.bin 2^>nul') do (
    set "LATEST=recordings\%%f"
    goto :found
)
echo [error] No recordings found under recordings\
pause
exit /b 1

:found
echo [run] Replaying %LATEST% at 11 fps ^(close the window to exit^)...
"%VENV_PY%" -m roomscan.viewer --replay "%LATEST%" --replay-fps 11
if errorlevel 1 pause
endlocal
