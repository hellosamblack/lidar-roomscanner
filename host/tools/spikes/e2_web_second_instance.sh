#!/usr/bin/env bash
# Second roomscan-web instance for the #224 live measurement: runs THIS
# worktree's code on ROOMSCAN_WEB_PORT (default 8010, never :8000 -- the systemd
# service owns that) against a private HOME whose roomscan.toml selects
# `[slam] backend = "remote"` (packet mode, spawned service on 127.0.0.1:5556).
#   e2_web_second_instance.sh [capture] [port]
# Foreground; Ctrl-C (or kill) stops it, and the lifespan hook ends the spawned
# slam service with it.
set -u
WT="$(cd "$(dirname "$0")/../../.." && pwd)"
CAP="${1:-$WT/captures/roomSweepFull20260730.bin}"
PORT="${2:-8010}"
HOME_DIR="${E2_HOME:-/tmp/rs224home}"
mkdir -p "$HOME_DIR/roomscan"
if [ ! -f "$HOME_DIR/roomscan/roomscan.toml" ]; then
  cat > "$HOME_DIR/roomscan/roomscan.toml" <<'TOML'
[viewer]
mode = "real_time"
slam_auto_record = false

[slam]
backend = "remote"
remote_addr = "127.0.0.1:5556"
remote_mesh_format = "packet"
remote_spawn = true
TOML
fi
cd "$WT"
export HOME="$HOME_DIR"
export ROOMSCAN_NO_BROWSER=1
export ROOMSCAN_WEB_PORT="$PORT"
export PYTHONPATH="$WT/host/src"
export ROOMSCAN_TRANSFORM_DLL=/home/sam/git/personal/lidar-roomscanner/host/transform/build/libroomscan_transform.so
exec /home/sam/git/personal/lidar-roomscanner/host/.venv/bin/python -m roomscan.web --replay "$CAP"
