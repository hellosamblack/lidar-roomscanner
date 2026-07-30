"""Capture inspection and host diagnostics.

Every function here delegates to the corresponding `host/tools/` script's pure
half -- this module contributes no analysis logic of its own.
"""
from __future__ import annotations

import sys
from pathlib import Path

from .paths import CAPTURES, HOST, RECORDINGS, REPO, WEB_WS, rel
from .server import mcp

sys.path.insert(0, str(HOST))  # `tools` is a top-level package rooted at host/


def _survey(path: Path, max_frames: int = 400) -> dict:
    """Cheap bounded header walk: which streams a capture carries, and roughly how many.

    Deliberately not `analyze_capture.scan()` -- that decodes and CRCs the whole
    file, which is far too slow to run across a whole directory just to answer
    "does this one have stream 9?".
    """
    from tools.analyze_capture import _HEADER, HEADER_SIZE, MAGIC, MAX_PAYLOAD, STREAMS

    counts: dict[str, int] = {}
    frames = 0
    truncated = False
    with open(path, "rb") as f:
        data = f.read(8 << 20)  # a bounded prefix is enough to survey stream presence
    n = len(data)
    pos = 0
    while pos < n and frames < max_frames:
        idx = data.find(MAGIC, pos)
        if idx < 0:
            break
        pos = idx
        if n - pos < HEADER_SIZE:
            truncated = True
            break
        _m, ver, _ft, stream, _fl, _seq, _t, _w, _h, plen, _r = _HEADER.unpack(
            data[pos:pos + HEADER_SIZE])
        if ver != 1 or plen > MAX_PAYLOAD:
            pos += 1
            continue
        total = HEADER_SIZE + plen + 4
        if n - pos < total:
            truncated = True
            break
        name = STREAMS.get(stream, str(stream))
        counts[name] = counts.get(name, 0) + 1
        frames += 1
        pos += total
    return {"streams": counts, "frames_sampled": frames, "prefix_truncated": truncated}


@mcp.tool()
def capture_list(surveyed: bool = True) -> dict:
    """List recorded captures in captures/ and recordings/, newest first.

    Reports size, mtime and -- when `surveyed` -- which streams each file carries,
    including `has_stream_9` (IMU_QUAT). SLAM and orientation work both require a
    stream-9 capture, and answering that question is otherwise a manual decode.

    Set `surveyed=False` for a fast listing that only stats the files.
    """
    out = []
    for d in (CAPTURES, RECORDINGS):
        if not d.is_dir():
            continue
        for p in d.glob("*.bin"):
            st = p.stat()
            entry = {
                "path": rel(p),
                "size_bytes": st.st_size,
                "size_mb": round(st.st_size / 1e6, 1),
                "mtime": int(st.st_mtime),
            }
            if surveyed:
                s = _survey(p)
                entry.update(s)
                entry["has_stream_9"] = "IMU_QUAT" in s["streams"]
            out.append(entry)
    out.sort(key=lambda e: e["mtime"], reverse=True)
    return {"count": len(out), "captures": out}


@mcp.tool()
def capture_analyze(path: str, min_zero_run: int = 50, zero_scan_frames: int = 8,
                    dump_bytes: int = 0, include_frame_log: bool = False) -> dict:
    """Byte-exact forensics over a capture: CRC failures, skip runs, truncation.

    Every anomaly is pinned to a file offset and carries the decoded header fields.
    `include_frame_log` adds the full per-frame inventory, which is thousands of
    entries on a real capture -- leave it off unless you need it.

    Wraps `host/tools/analyze_capture.py::scan()`.
    """
    from tools.analyze_capture import scan

    p = (REPO / path) if not Path(path).is_absolute() else Path(path)
    if not p.is_file():
        return {"error": f"no such capture: {rel(p)}"}
    r = scan(str(p), min_zero_run=min_zero_run, zero_scan_frames=zero_scan_frames,
             dump_bytes=dump_bytes)
    if not include_frame_log:
        r["frame_log_len"] = len(r.pop("frame_log"))
    r["path"] = rel(p)
    return r


@mcp.tool()
def doctor(build: bool = False, net: bool = True) -> dict:
    """Run the headless-host bring-up checks and return each verdict.

    Checks vendored 53L9A1 sources, the native transform .so, the live board UDP
    stream + mDNS, vendored three.js, and a WebGL-capable browser. `build=True`
    builds the native library if it is missing; `net=False` skips the board probe.

    Wraps `host/tools/headless_doctor.py::Doctor`.
    """
    from tools.headless_doctor import Doctor

    d = Doctor(quiet=True)
    failed = d.run(build=build, net=net)
    return {"failed": failed, "ok": failed == 0, "checks": d.results}


@mcp.tool()
async def orientation_probe(mode: str = "jitter", seconds: float = 15.0,
                            label: str = "", url: str = "") -> dict:
    """Measure orientation noise or stream health against a running roomscan-web.

    `mode="jitter"` reports per-frame rotation change (mean/median/p95/max degrees
    plus edge motion at 3 m); `mode="health"` reports per-stream Hz, drops and gaps.
    Needs the server up -- call `rig_status()` first.

    Wraps `host/tools/orientation_probe.py`.
    """
    from tools import orientation_probe as op

    url = url or WEB_WS
    if mode == "jitter":
        dirs = await op.collect_directions(url, seconds)
        if not dirs:
            return {"error": "no POINT_CLOUD messages seen — is the rig streaming?",
                    "url": url, "seconds": seconds}
        return op.summarize_jitter(dirs, seconds=seconds, label=label)
    if mode == "health":
        msgs = await op.collect_metrics(url, seconds)
        if not msgs:
            return {"error": "no metrics messages seen — is the server up?",
                    "url": url, "seconds": seconds}
        return op.summarize_health(msgs, seconds=seconds)
    return {"error": f"unknown mode {mode!r} (expected 'jitter' or 'health')"}
