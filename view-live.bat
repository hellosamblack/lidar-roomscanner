@echo off
rem ---------------------------------------------------------------------------
rem view-live.bat — open the live 3D viewer against the connected scanner.
rem Auto-finds the scanner's USB CDC port (VID:PID CAFE:4001). Bootstraps the
rem Python venv/dependencies on first run (needs Python 3.11 or 3.12).
rem Extra args pass through, e.g.:  view-live.bat --color reflectance
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

echo [run] Opening live viewer ^(close the window to exit^)...
echo [keys] P=ping  C=calibration  R=reinit  1/2=usecase  H=help  ^| stats print in THIS console
"%VENV_PY%" -m roomscan.viewer %*
if errorlevel 1 (
    echo.
    echo [hint] No scanner port found? Check the board's USER USB cable ^(CDC CAFE:4001^),
    echo        and make sure nothing else has the port open. Press the black RESET
    echo        button on the board if the stream doesn't start.
    pause
)
endlocal
