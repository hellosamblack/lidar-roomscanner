#!/usr/bin/env bash
# ---------------------------------------------------------------------------
# view-web.sh — start the roomscan WEB viewer (Linux/macOS).
# Serves the live 3D point cloud over a local WebSocket to a Three.js page at
# http://localhost:8000/static/index.html. Auto-finds the scanner's USB CDC port
# (VID:PID CAFE:4001). Bootstraps the Python venv/dependencies on first run
# (needs Python 3.11 or 3.12).
# Extra args pass through, e.g.:  ./view-web.sh --color reflectance
#                                 ./view-web.sh --replay recordings/scan.bin
#
# On a box where roomscan-web is installed as a systemd *user* service (#207,
# host/deploy/roomscan-web.service) this script DELEGATES to systemd rather than
# spawning a competing server: a plain `./view-web.sh` just ensures the managed
# service is up; passing args runs a one-off and restores the live service on
# exit. Manage it directly with `systemctl --user {start,stop,restart} roomscan-web`.
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

# simplejpeg (#197): missing from this probe means a pre-#197 venv never
# reinstalls, and every /ws-thin JPEG negotiation then silently degrades to
# raw ("a new default is invisible without a migration" -- see AGENTS.md).
if ! "$VENV_PY" -c "import fastapi, uvicorn, numpy, serial, roomscan, simplejpeg" >/dev/null 2>&1; then
    echo "[setup] Installing dependencies (first run takes a few minutes)..."
    "$VENV_PY" -m pip install --quiet --upgrade pip
    "$VENV_PY" -m pip install --quiet -e "host[web]"
fi

# --- Supervised-service scheme (#207) --------------------------------------
# On the dev box roomscan-web runs as an always-on systemd *user* service, and
# it -- not this script -- owns port 8000 + the device stream. Spawning a second
# bare server here would race Restart=always for both (a double-bind crash-loop).
# So when the unit is installed we delegate to systemd. Boxes without the unit
# (macOS, a plain Linux checkout) keep the old direct launch below.
WEB_UNIT="roomscan-web.service"

unit_installed() {
    command -v systemctl >/dev/null 2>&1 || return 1
    [ "$(systemctl --user show "$WEB_UNIT" -p LoadState --value 2>/dev/null)" = "loaded" ]
}

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

if unit_installed; then
    if [ "$#" -eq 0 ]; then
        # Plain launch: just ensure the supervised unit is up and point the user
        # at it. Never kill it -- systemd keeps it alive by design.
        echo "[service] roomscan-web is a supervised systemd user service on this box."
        systemctl --user start "$WEB_UNIT"
        echo "[service] active — http://localhost:8000/static/index.html"
        echo "[tip] logs:    journalctl --user -u roomscan-web -f"
        echo "[tip] reload:  systemctl --user restart roomscan-web"
        echo "[tip] stop:    systemctl --user stop roomscan-web"
        exit 0
    fi
    # Passthrough args (e.g. --replay, --color) are a one-off the managed service
    # can't carry. Stop the unit so this run can bind, and restore live on exit.
    echo "[service] one-off launch with args ($*) — stopping the managed unit first."
    systemctl --user stop "$WEB_UNIT" || true
    # Restore the live service however this run ends. Run the server in the
    # background and `wait` on it: a foreground child would defer signal handling
    # until it exits, but `wait` is interruptible, so Ctrl+C/TERM reaches the trap
    # immediately, is forwarded to the server, and the EXIT trap then restores.
    restore_live() { echo "[service] restoring live server..."; systemctl --user start "$WEB_UNIT" || true; }
    trap restore_live EXIT
    echo "[run] one-off web viewer on http://localhost:8000/static/index.html"
    echo "[tip] Ctrl+C to stop; the live server is restored automatically afterwards."
    ROOMSCAN_NO_BROWSER=1 "$VENV_PY" -m roomscan.web "$@" &
    child=$!
    trap 'kill -TERM "$child" 2>/dev/null || true' INT TERM
    wait "$child"
    exit $?
fi

# --- No service unit: direct launch (macOS / plain Linux checkout) ----------
check_and_handle_existing_server

echo "[run] Starting web viewer on http://localhost:8000/static/index.html"
echo "[tip] Open the URL above in a browser if desired. Press Ctrl+C here to stop."
ROOMSCAN_NO_BROWSER=1 exec "$VENV_PY" -m roomscan.web "$@"
