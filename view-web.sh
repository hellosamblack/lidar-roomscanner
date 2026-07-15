#!/usr/bin/env bash
# ---------------------------------------------------------------------------
# view-web.sh — start the roomscan WEB viewer and open it in a browser (Linux/macOS).
# Serves the live 3D point cloud over a local WebSocket to a Three.js page at
# http://localhost:8000/static/index.html (the app opens your browser for you
# once the server is listening). Auto-finds the scanner's USB CDC port
# (VID:PID CAFE:4001). Bootstraps the Python venv/dependencies on first run
# (needs Python 3.11 or 3.12).
# Extra args pass through, e.g.:  ./view-web.sh --color reflectance
#                                 ./view-web.sh --replay recordings/scan.bin
# ---------------------------------------------------------------------------
set -euo pipefail

# cd to the directory this script lives in, so relative paths work from anywhere.
cd "$(dirname "$(readlink -f "$0")")"

VENV_PY="host/.venv/bin/python"

if [ ! -x "$VENV_PY" ]; then
    echo "[setup] Creating virtual environment..."
    if command -v python3.12 >/dev/null 2>&1; then
        python3.12 -m venv host/.venv
    elif command -v python3.11 >/dev/null 2>&1; then
        python3.11 -m venv host/.venv
    else
        echo "[error] Python 3.11 or 3.12 is required (neither python3.12 nor python3.11 found on PATH)."
        exit 1
    fi
fi

if ! "$VENV_PY" -c "import fastapi, uvicorn, numpy, serial, roomscan" >/dev/null 2>&1; then
    echo "[setup] Installing dependencies (first run takes a few minutes)..."
    "$VENV_PY" -m pip install --quiet --upgrade pip
    "$VENV_PY" -m pip install --quiet -e "host[web]"
fi

echo "[run] Starting web viewer on http://localhost:8000/static/index.html"
echo "[tip] Your browser opens automatically once the server is up. Press Ctrl+C here to stop."
exec "$VENV_PY" -m roomscan.web "$@"
