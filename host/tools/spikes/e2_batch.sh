#!/usr/bin/env bash
# E2 measurement batch (Refs #223): runs the configurations named in $@ one
# after another (never concurrently -- they would share the GPU), 1800 frames
# of coffeeRoomCircuitNoMnt.bin on CUDA:0. Each name is <config>:<run>:
#   serial        shipped serial rig (per-stage attribution)
#   local         live rig, in-process SlamWorker + MeshPrep (what ships)
#   remote-arrays live rig, SlamService subprocess, prep in-process
#   remote-packet live rig, SlamService subprocess, prep in the service
HERE="$(cd "$(dirname "$0")" && pwd)"
for spec in "$@"; do
  cfg="${spec%%:*}"; run="${spec##*:}"
  out="/tmp/e2_${cfg}_run${run}.json"
  case "$cfg" in
    serial)        args="" ;;
    local)         args="--mode live --backend local" ;;
    remote-arrays) args="--mode live --backend remote --mesh-format arrays" ;;
    remote-packet) args="--mode live --backend remote --mesh-format packet" ;;
    *) echo "unknown config $cfg"; exit 2 ;;
  esac
  echo "=== $cfg run $run -> $out"
  bash "$HERE/e2_run.sh" "$out" $args
  sleep 3
done
