"""One-screen summary of a `slam_stall_profile --mode live --json` report
(E2 / #224 measurement bookkeeping).

    host/.venv/bin/python host/tools/spikes/e2_summary.py /tmp/e2_*.json
"""
from __future__ import annotations

import json
import sys

_TOP = ("backend", "mesh_format", "frames", "frames_processed", "frames_overwritten",
        "tracking_lost", "wall_s", "effective_fps", "tick_share", "worst_stall_ms",
        "starved_pct_of_wall", "stalls_over_300ms", "meshes", "meshes_received",
        "final_payload_mb", "peak_payload_mb", "client_cpu_pct", "service")


def main(paths: list[str]) -> int:
    for p in paths:
        # `raw_decode`: the local backend's Open3D prints its memory statistics
        # to stdout at exit, AFTER the JSON, so a plain `json.load` fails.
        r, _end = json.JSONDecoder().raw_decode(open(p).read())
        print(f"== {p}")
        if "error" in r:
            print("  error:", r["error"])
            continue
        for k in _TOP:
            print(f"  {k:22} {r.get(k)}")
        for k in ("pose_age_ms", "pose_transport_ms", "step_total_ms"):
            print(f"  {k:22} {r.get(k)}")
        for k, v in (r.get("ipc_ms") or {}).items():
            print(f"  ipc {k:18} {v}")
        for k, v in (r.get("gil_starvation") or {}).items():
            print(f"  gil {k:18} {v}")
        print(f"  {'mesh_cadence_s':22} {r.get('mesh_cadence_s')}")
        print(f"  {'mesh_delivery_ms':22} {r.get('mesh_delivery_ms')}")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
