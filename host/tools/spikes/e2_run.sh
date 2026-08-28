#!/usr/bin/env bash
# E2 spike runner (Refs #223): runs slam_stall_profile.py from this worktree with
# the main checkout's venv, writing JSON to $1 and a headline summary to stdout.
#   e2_run.sh <out.json> [profiler args...]
set -u
WT="$(cd "$(dirname "$0")/../../.." && pwd)"
PY=/home/sam/git/personal/lidar-roomscanner/host/.venv/bin/python
CAP=/home/sam/git/personal/lidar-roomscanner/captures/coffeeRoomCircuitNoMnt.bin
OUT="$1"; shift
export PYTHONPATH="$WT/host/src"
export ROOMSCAN_TRANSFORM_DLL=/home/sam/git/personal/lidar-roomscanner/host/transform/build/libroomscan_transform.so
"$PY" "$WT/host/tools/slam_stall_profile.py" "$CAP" --frames 1800 --device CUDA:0 --json "$@" \
  > "$OUT" 2> "${OUT%.json}.err"
rc=$?
echo "rc=$rc"
tail -5 "${OUT%.json}.err"
"$PY" - "$OUT" <<'PY'
import json, sys
r = json.load(open(sys.argv[1]))
if "error" in r:
    print("ERROR", r["error"]); sys.exit(0)
keys = ["mode", "backend", "wall_s", "effective_fps", "tick_share", "worst_stall_ms",
        "starved_pct_of_wall", "frames", "frames_integrated", "meshes", "final_payload_mb",
        "pose_age_ms", "ipc_overhead_ms", "mesh_cadence_s", "service"]
print({k: r[k] for k in keys if k in r})
for k, v in r.get("gil_starvation", {}).items():
    print("  gil", k, v)
for k, v in r.get("stages", {}).items():
    print("  stage", k, v)
PY
