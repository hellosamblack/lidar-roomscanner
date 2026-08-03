"""What is roomscan-web actually PUTTING ON THE WIRE, and is it renderable?

    host/.venv/bin/python host/tools/ws_probe.py
    host/.venv/bin/python host/tools/ws_probe.py --seconds 15 --json

Written 2026-08-02 during a "SLAM rendered nothing" report, to split a single
symptom into the three independent questions it collapses:

  1. Is the SERVER computing?     -- the `slam` message's traj_len /
                                     frames_integrated / mesh_seq / blocks_used.
  2. Is the TRANSPORT delivering? -- per-type JSON counts on `/ws`, per-tag
                                     binary counts, and MESH bytes actually
                                     received on `/ws-mesh`.
  3. Is the PAYLOAD well formed?  -- one MESH re-parsed with slam.js's exact
                                     layout, section by section.

All three read healthy and the fault is client-side; any one of them empty and
you have the stage. On the report that prompted this, all three were healthy --
the map was rendering and sitting behind two UI cards.

This is a pure CLIENT of the same `/ws` every browser tab uses. It never touches
the device stream (roomscan-web owns it), records nothing, and sends nothing but
`mesh_ack`. It is deliberately short-lived, because **connection count is a
performance variable on this server**: `_broadcast_bytes` awaits `send_bytes`
per client on the event loop, and `/ws` has no backpressure at all, so a probe
left connected is a probe that changes what it measures (BUG-060, BUG-061).

MESH ACKS ARE NOT OPTIONAL. `/ws-mesh` is credit-gated: one mesh in flight per
client, released by an inbound `{"type": "mesh_ack"}`. A probe that connects and
stays silent gets exactly one mesh and then the legacy trickle (1 per 5 s), which
reads identically to "the server has stopped sending" -- so this acks every mesh
and reports `meshes` as a RATE you can compare against `mesh_seq`.

READING THE MESH DECODE. `parses_clean` is the whole point: the 9x u32 header
(tag, mesh_seq, flags, then six counts) must account for every remaining byte,
because the client walks it with bare `new Float32Array(buffer, off, n)` views.
A header that disagrees with the payload by even one element does not degrade --
the view constructor throws, the handler dies, and the map silently stops
updating with no console error anyone will look at. `slack_bytes != 0` means the
firmware/server packer and slam.js's reader have drifted apart.
"""
from __future__ import annotations

import argparse
import asyncio
import collections
import json
import struct
import sys
from pathlib import Path

DEFAULT_URL = "ws://localhost:8000"
DEFAULT_SECONDS = 10.0

# slam.js: 9 x u32 header, then per submesh f32 pos / f32 col / u32 idx, then
# the floor's f32 points + u32 line indices. Kept as data so the decode below
# reads the same order the client does.
MESH_HEADER = ("tag", "mesh_seq", "flags", "nonwall_verts", "nonwall_tris",
               "wall_verts", "wall_tris", "floor_pts", "floor_lines")
MESH_SECTIONS = (
    ("nonwall_pos", "nonwall_verts", 3), ("nonwall_col", "nonwall_verts", 3),
    ("nonwall_idx", "nonwall_tris", 3),
    ("wall_pos", "wall_verts", 3), ("wall_col", "wall_verts", 3),
    ("wall_idx", "wall_tris", 3),
    ("floor_pos", "floor_pts", 3), ("floor_idx", "floor_lines", 2),
)
# Fields worth lifting out of the newest `slam` message -- "is the mapper
# working" in one line, rather than the whole ~40-field payload.
SLAM_KEYS = ("traj_len", "frames_integrated", "frames_submitted", "frames_processed",
             "frames_overwritten", "mesh_seq", "mesh_verts", "blocks_used",
             "blocks_capacity", "device", "backend", "tracking_lost", "slam_ms",
             "mesh_payload_bytes")


def decode_mesh(buf: bytes) -> dict:
    """Re-parse one MESH packet exactly as `slam.js`'s `hub.on('mesh')` does.

    Returns the header, a per-section offset/size walk, and `parses_clean` --
    True only when the sections consume the buffer to the byte.
    """
    if len(buf) < 36:
        return {"error": f"runt MESH: {len(buf)} bytes", "parses_clean": False}
    hdr = dict(zip(MESH_HEADER, struct.unpack_from("<9I", buf, 0)))
    off, sections, overrun = 36, [], False
    for name, count_key, per in MESH_SECTIONS:
        size = hdr[count_key] * per * 4
        end = off + size
        if end > len(buf):
            overrun = True
        sections.append({"name": name, "offset": off, "bytes": size, "end": end,
                         "overruns": end > len(buf)})
        off = end
    return {"header": hdr, "total_bytes": len(buf), "sections": sections,
            "parsed_end": off, "slack_bytes": len(buf) - off,
            "parses_clean": (not overrun) and off == len(buf)}


async def _probe_ws(url: str, seconds: float, out: dict) -> None:
    import websockets

    async with websockets.connect(f"{url}/ws", max_size=None) as ws:
        loop = asyncio.get_running_loop()
        end = loop.time() + seconds
        while (remain := end - loop.time()) > 0:
            try:
                msg = await asyncio.wait_for(ws.recv(), timeout=remain)
            except asyncio.TimeoutError:
                break
            except Exception as exc:            # server went away mid-probe
                out["ws_error"] = repr(exc)
                break
            if isinstance(msg, (bytes, bytearray)):
                out["binary_tags"][msg[0]] += 1
                out["binary_bytes"][msg[0]] += len(msg)
                continue
            data = json.loads(msg)
            mtype = data.get("type")
            out["json_counts"][mtype] += 1
            if mtype == "slam":
                out["slam"] = {k: data[k] for k in SLAM_KEYS if k in data}
            elif mtype == "state":
                out["state"] = {k: data.get(k) for k in
                                ("mode", "source", "display", "selected_capture",
                                 "view_mode", "slam_available")}
            elif mtype == "log":
                out["logs"].append(data.get("text") or data.get("message") or "")


async def _probe_mesh(url: str, seconds: float, out: dict) -> None:
    import websockets

    try:
        async with websockets.connect(f"{url}/ws-mesh", max_size=None) as ws:
            loop = asyncio.get_running_loop()
            end = loop.time() + seconds
            while (remain := end - loop.time()) > 0:
                try:
                    msg = await asyncio.wait_for(ws.recv(), timeout=remain)
                except asyncio.TimeoutError:
                    break
                if not isinstance(msg, (bytes, bytearray)):
                    continue
                out["meshes"] += 1
                out["mesh_bytes"] += len(msg)
                out["mesh_tags"][msg[0]] += 1
                if out["mesh_decode"] is None:
                    out["mesh_decode"] = decode_mesh(bytes(msg))
                # Release the credit, or the next mesh never comes.
                await ws.send(json.dumps({"type": "mesh_ack"}))
    except Exception as exc:
        out["mesh_error"] = repr(exc)


async def probe_async(seconds: float = DEFAULT_SECONDS, url: str = DEFAULT_URL) -> dict:
    """Watch `/ws` + `/ws-mesh` for `seconds` and report what arrived.

    The real implementation. Pure apart from the `mesh_ack`s that keep MESH
    flowing -- no device access, no recording, no server state changed.

    Async because the MCP tool awaits it from an already-running event loop;
    `probe()` is the sync front end the CLI uses.
    """
    out: dict = {
        "url": url, "seconds": seconds,
        "json_counts": collections.Counter(), "binary_tags": collections.Counter(),
        "binary_bytes": collections.Counter(), "logs": [],
        "slam": None, "state": None,
        "meshes": 0, "mesh_bytes": 0, "mesh_tags": collections.Counter(),
        "mesh_decode": None, "mesh_error": None, "ws_error": None,
    }

    await asyncio.gather(_probe_ws(url, seconds, out),
                         _probe_mesh(url, seconds, out))

    for key in ("json_counts", "binary_tags", "binary_bytes", "mesh_tags"):
        out[key] = {str(k): v for k, v in sorted(out[key].items(), key=lambda kv: str(kv[0]))}
    out["logs"] = out["logs"][-25:]
    out["mesh_rate_hz"] = round(out["meshes"] / seconds, 3) if seconds > 0 else None
    out["slam_msg_hz"] = round(out["json_counts"].get("slam", 0) / seconds, 2) if seconds > 0 else None
    # The three questions, answered. `payload_ok` is None when no mesh arrived,
    # which is a different finding from "a mesh arrived and was malformed".
    dec = out["mesh_decode"]
    out["verdict"] = {
        "server_computing": bool(out["slam"] and out["slam"].get("frames_integrated")),
        "transport_delivering": out["meshes"] > 0,
        "payload_ok": None if dec is None else bool(dec.get("parses_clean")),
    }
    return out


def probe(seconds: float = DEFAULT_SECONDS, url: str = DEFAULT_URL) -> dict:
    """Sync front end for the CLI. Do not call from inside an event loop."""
    return asyncio.run(probe_async(seconds=seconds, url=url))


def _print(rep: dict) -> None:
    v = rep["verdict"]
    mark = {True: "ok", False: "FAIL", None: "n/a"}
    print(f"probe {rep['url']} for {rep['seconds']:g}s")
    print(f"  server computing    {mark[v['server_computing']]}")
    print(f"  transport delivering {mark[v['transport_delivering']]}")
    print(f"  payload well formed  {mark[v['payload_ok']]}")
    if rep["slam"]:
        s = rep["slam"]
        print(f"\n  slam: {s.get('frames_integrated')} frames integrated, "
              f"mesh_seq {s.get('mesh_seq')}, {s.get('mesh_verts')} verts, "
              f"{s.get('blocks_used')}/{s.get('blocks_capacity')} blocks "
              f"on {s.get('device')} ({s.get('backend')})")
    else:
        print("\n  slam: no `slam` message seen -- SLAM is not the active display, "
              "or the mapper never armed")
    print(f"  /ws json: {rep['json_counts']}")
    print(f"  /ws binary tags: {rep['binary_tags']}  bytes: {rep['binary_bytes']}")
    print(f"  /ws-mesh: {rep['meshes']} meshes ({rep['mesh_rate_hz']} Hz), "
          f"{rep['mesh_bytes']} bytes")
    if rep["mesh_error"]:
        print(f"  /ws-mesh error: {rep['mesh_error']}")
    dec = rep["mesh_decode"]
    if dec and "sections" in dec:
        print(f"\n  MESH decode (slam.js layout), {dec['total_bytes']} bytes:")
        for s in dec["sections"]:
            flag = "  OVERRUN" if s["overruns"] else ""
            print(f"    {s['name']:<12} off={s['offset']:<10d} bytes={s['bytes']:<10d}{flag}")
        print(f"    slack {dec['slack_bytes']} bytes -> "
              f"{'parses clean' if dec['parses_clean'] else 'LAYOUT MISMATCH'}")
    for line in rep["logs"]:
        print(f"  log: {line}")


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--seconds", type=float, default=DEFAULT_SECONDS,
                    help=f"how long to watch (default {DEFAULT_SECONDS:g}); "
                         "keep it short, a connected client costs the server")
    ap.add_argument("--url", default=DEFAULT_URL, help=f"default {DEFAULT_URL}")
    ap.add_argument("--json", action="store_true")
    args = ap.parse_args(argv)

    rep = probe(seconds=args.seconds, url=args.url)
    if args.json:
        print(json.dumps(rep, indent=2))
    else:
        _print(rep)
    return 0


if __name__ == "__main__":
    sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
    raise SystemExit(main())
