"""#224 live measurement driver: arm SLAM on a running roomscan-web (default
the second instance on :8010), wait for the map to grow, then run
`ws_probe` for `--seconds` and print the pose-age / rate numbers.

    host/.venv/bin/python host/tools/spikes/e2_live_probe.py --url ws://localhost:8010 --seconds 20
"""
from __future__ import annotations

import argparse
import asyncio
import json
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from ws_probe import probe_async  # noqa: E402


RESTART = False


async def _arm(url: str, wait_s: float) -> dict:
    import websockets
    last = None
    async with websockets.connect(f"{url}/ws", max_size=None) as ws:
        if RESTART:
            # Loop + restart the replay so the map grows over the whole take.
            await ws.send(json.dumps({"type": "transport", "action": "loop", "value": True}))
            await ws.send(json.dumps({"type": "transport", "action": "restart"}))
            await asyncio.sleep(2.0)
        await ws.send(json.dumps({"type": "set_mode", "mode": "slam"}))
        t0 = time.monotonic()
        while time.monotonic() - t0 < wait_s:
            msg = await asyncio.wait_for(ws.recv(), timeout=wait_s)
            if isinstance(msg, (bytes, bytearray)):
                continue
            d = json.loads(msg)
            if d.get("type") == "slam":
                last = {k: d.get(k) for k in ("backend", "device", "frames_integrated",
                                              "mesh_seq", "mesh_verts", "traj_len")}
            elif d.get("type") == "log":
                txt = d.get("text") or d.get("message") or ""
                if "[slam" in txt:
                    print("  log:", txt)
    return last or {}


def main(argv=None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--url", default="ws://localhost:8010")
    ap.add_argument("--arm-wait", type=float, default=25.0)
    ap.add_argument("--seconds", type=float, default=20.0)
    ap.add_argument("--json", action="store_true")
    ap.add_argument("--restart", action="store_true", help="loop + restart the replay first")
    a = ap.parse_args(argv)
    global RESTART
    RESTART = a.restart
    armed = asyncio.run(_arm(a.url, a.arm_wait))
    print("armed:", armed)
    rep = asyncio.run(probe_async(seconds=a.seconds, url=a.url))
    keys = ("slam", "slam_msg_hz", "slam_age_ms", "meshes", "mesh_bytes", "mesh_rate_hz",
            "verdict", "json_counts", "binary_tags")
    out = {k: rep.get(k) for k in keys}
    out["mesh_parses_clean"] = (rep.get("mesh_decode") or {}).get("parses_clean")
    print(json.dumps(out, indent=2) if a.json else out)
    return 0


if __name__ == "__main__":
    sys.exit(main())
