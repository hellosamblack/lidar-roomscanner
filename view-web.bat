@echo off
rem ---------------------------------------------------------------------------
rem view-web.bat — start the roomscan WEB viewer and open it in a browser.
rem Serves the live 3D point cloud over a local WebSocket to a Three.js page at
rem http://localhost:8000/static/index.html (the app opens your browser for you
rem once the server is listening). Auto-finds the scanner's USB CDC port
rem (VID:PID CAFE:4001). Bootstraps the Python venv/dependencies on first run
rem (needs Python 3.11 or 3.12).
rem Extra args pass through, e.g.:  view-web.bat --color reflectance
rem                                 view-web.bat --replay recordings\scan.bin
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

"%VENV_PY%" -c "import fastapi, uvicorn, numpy, serial, roomscan" 2>nul || (
    echo [setup] Installing dependencies ^(first run takes a few minutes^)...
    "%VENV_PY%" -m pip install --quiet --upgrade pip
    "%VENV_PY%" -m pip install --quiet -e host || (
        echo [error] Dependency installation failed.
        pause
        exit /b 1
    )
)

echo [run] Starting web viewer on http://localhost:8000/static/index.html
echo [tip] Your browser opens automatically once the server is up. Press Ctrl+C here to stop.
"%VENV_PY%" -m roomscan.web %*
if errorlevel 1 (
    echo.
    echo [hint] No scanner port found? Check the board's USER USB cable ^(CDC CAFE:4001^),
    echo        and make sure nothing else has the port open. Press the black RESET
    echo        button on the board if the stream doesn't start.
    pause
)
endlocal
