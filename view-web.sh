#!/usr/bin/env bash
# ---------------------------------------------------------------------------
# view-web.sh — start the roomscan WEB viewer (Linux/macOS).
# Serves the live 3D point cloud over a local WebSocket to a Three.js page at
# http://localhost:8000/static/index.html. Auto-finds the scanner's USB CDC port
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

check_and_handle_existing_server() {
    local port=8000
    local pids=""

    if command -v lsof >/dev/null 2>&1; then
        pids=$(lsof -ti :$port 2>/dev/null || true)
    elif command -v netstat >/dev/null 2>&1; then
        pids=$(netstat -tlnp 2>/dev/null | grep ":$port " | awk '{print $NF}' | cut -d'/' -f1 || true)
    fi

    if [ -n "$pids" ]; then
        local formatted_pids
        formatted_pids=$(echo "$pids" | tr '\n' ' ' | xargs)
        echo "[warn] A server is already running on port $port (PID: $formatted_pids)"
        read -p "Kill and restart? (y/N) " -n 1 -r response
        echo
        if [[ $response =~ ^[Yy]$ ]]; then
            echo "[cleanup] Killing existing server (PID: $formatted_pids)..."
            echo "$pids" | xargs -r kill -9 2>/dev/null || true
            sleep 1
        else
            echo "[abort] Keeping existing server running. Exiting."
            exit 0
        fi
    fi
}

check_and_handle_existing_server

echo "[run] Starting web viewer on http://localhost:8000/static/index.html"
echo "[tip] Open the URL above in a browser if desired. Press Ctrl+C here to stop."
ROOMSCAN_NO_BROWSER=1 exec "$VENV_PY" -m roomscan.web "$@"
