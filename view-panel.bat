@echo off
rem ---------------------------------------------------------------------------
rem view-panel.bat — open the NEW gui control panel against the connected scanner.
rem Buttons/sliders + a live 2D IR monitor pane, instead of the keyboard-only
rem window (that's view-live.bat). Auto-finds the scanner's USB CDC port
rem (VID:PID CAFE:4001). Bootstraps the Python venv/dependencies on first run
rem (needs Python 3.11 or 3.12).
rem Extra args pass through, e.g.:  view-panel.bat --color reflectance
rem                                 view-panel.bat --replay recordings\scan.bin
rem ---------------------------------------------------------------------------
setlocal EnableDelayedExpansion
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

"%VENV_PY%" -c "from roomscan.slam.config import SlamConfig; import sys; sys.exit(0 if SlamConfig.load().backend == 'remote' else 1)" 2>nul
if %errorlevel% equ 0 (
    wslc list 2>nul | findstr /i "roomscan-slam" >nul
    if errorlevel 1 (
        echo.
        echo [slam] Remote SLAM backend configured but container is not running.
        set /p START_CONTAINER="Do you want to start the GPU container now? [Y/n] "
        if /i "!START_CONTAINER!" neq "N" (
            pwsh tools\slam-container\start.ps1
        )
        echo.
    )
)

echo [run] Opening control panel ^(close the window to exit^)...
echo [tip] Everything is on-screen: Device buttons, View/Near-contrast, IR Monitor.
echo       Press H or click Help for the guide. Near contrast defaults to "window"
echo       ^(greys past 1.5 m^) to make a close subject stand out.
"%VENV_PY%" -m roomscan.panel %*
if errorlevel 1 (
    echo.
    echo [hint] No scanner port found? Check the board's USER USB cable ^(CDC CAFE:4001^),
    echo        and make sure nothing else has the port open. Press the black RESET
    echo        button on the board if the stream doesn't start.
    pause
)
endlocal
