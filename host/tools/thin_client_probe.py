"""Is `/ws-thin` actually putting a PICTURE on the wire, and do the commands move it?

    host/.venv/bin/python host/tools/thin_client_probe.py
    host/.venv/bin/python host/tools/thin_client_probe.py --frames 8 --json
    host/.venv/bin/python host/tools/thin_client_probe.py --record   # opt-in only

Written 2026-08-17 alongside `/ws-thin` (#194) to stand in for the CrowPanel we
do not have on the bench: it is a thin client, in Python, that connects to a
running `roomscan-web`, decodes `THIN_FRAME`, and writes the frames out as PNGs
a human (or an agent) can look at.

WHY PIXELS. Everything this reports about the inbound commands is measured on
the decoded image, never on a counter or a read-back of the camera state. A
`sceneOpacity` getter that returned the value its setter had just assigned
"verified" the See-Through slider end-to-end for four days while the renderer
ignored it (#106); the same trap is available here in the shape of a frame
counter that keeps ticking whatever the camera does. So `thin_orbit` is judged
by the fraction of pixels that changed, **against a control**: two consecutive
frames with no command sent measure how much the picture moves on its own
(live geometry, replay advancing, render noise), and the orbit has to beat that
by a wide margin to count. A null result cannot be explained away as "the scene
was static anyway" because the static number is right there next to it.

`thin_mode` is judged the same way -- a mode "worked" only if a frame actually
arrived after the *server's own* telemetry confirmed the switch. The render loop
sends `thin_telemetry` at the top of a tick and the frame at the bottom of the
same tick, so the first frame after a telemetry carrying the new mode is
genuinely rendered in that mode; a frame that merely arrives after the command
was sent may still be the previous mode's, rendered before it landed.

A mode that produces NO frame is a real finding, not an error: the render loop
deliberately sends nothing when it has no fresh generation-tagged data for the
requested mode (a live->replay switch must never show a stale picture). So "ir
produced no frame" means the IR stash is empty or stale, which is exactly what
you want to know.

This is a pure CLIENT of `roomscan-web`, like `ws_probe.py`: it never binds the
device stream, and it never starts a recording unless `--record` is passed
explicitly. Note `/ws-thin` caps concurrent thin clients (2), and each one costs
a full server-side render, so a probe left connected is expensive -- this one
connects, does its work and disconnects.
"""
from __future__ import annotations

import argparse
import asyncio
import json
import struct
import sys
import time
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]

DEFAULT_URL = "ws://localhost:8000"
DEFAULT_FRAMES = 5
DEFAULT_MODES = ("point_cloud", "slam", "ir")
DEFAULT_ORBIT_YAW = 120.0
#: per-message wait; the feed is 10 fps and telemetry 2 Hz, so this is generous
DEFAULT_TIMEOUT = 15.0
#: The orbit "after" frame used to be found by discarding a FIXED number of
#: frames post-command. That silently broke at high negotiated fps (#197
#: review, live finding 2026-08-18): the probe (decode + PNG-write per
#: frame) is a slower consumer than the server is a producer, so several
#: PRE-command frames can already be queued in the receive buffer by the
#: time the command is sent -- "discard the next 2" can still land on a
#: stale one when the backlog is deeper than 2. It is now TIME-ANCHORED
#: instead (`_ThinLink.drain`/`next_frame_after`): "after" must have been
#: RECEIVED at least one full negotiated interval past the send, however
#: many frames that takes to reach -- `orbit["frames_discarded"]` in the
#: report is the real count for a given run.
#: default un-negotiated cadence (matches `roomscan.web.THIN_INTERVAL`) --
#: used to size the time-anchor window when no `thin_hello` was negotiated.
DEFAULT_THIN_INTERVAL_S = 1.0 / 10.0
#: per-read timeout for `_ThinLink.drain()`: long enough that an ALREADY
#: buffered message (no real wait) reliably resolves first, short enough
#: that it can never itself be mistaken for "waiting for a new frame" --
#: safely below even the fastest negotiated interval (1/60 s = 16.7 ms).
DRAIN_POLL_TIMEOUT = 0.01

# Wire contract, mirrored from `roomscan.thin_render` (see docs/thin-client.md).
# Imported when the package is importable, so a contract change breaks loudly
# here instead of silently decoding garbage.
THIN_TAG = 1
THIN_HEADER = struct.Struct("<IHH")  # u32 tag, u16 width, u16 height
THIN_TAG_JPEG = 2
THIN_HEADER_JPEG = struct.Struct("<IHHI")  # u32 tag, u16 width, u16 height, u32 jpeg_len

#: a per-channel difference this small is dithering/compression, not a moved camera
PIXEL_DIFF_THRESHOLD = 8
#: the orbit must change at least this fraction of pixels...
ORBIT_MIN_CHANGED_FRAC = 0.10
#: ...and beat the no-command control by this factor, or the verdict is False
ORBIT_CONTROL_MARGIN = 3.0


def _import_contract() -> None:
    """Adopt the server's own header constants when `roomscan` is importable.

    Two tools disagreeing about the same bytes is a bug generator (memory:
    "Mirrored constants drift per-tool"), so the literals above are only a
    fallback for running this file straight out of a checkout without the venv.
    """
    global THIN_TAG, THIN_HEADER, THIN_TAG_JPEG, THIN_HEADER_JPEG
    try:
        from roomscan.thin_render import (
            THIN_HEADER as H, THIN_HEADER_JPEG as HJ, THIN_TAG as T,
            THIN_TAG_JPEG as TJ,
        )
    except Exception:  # noqa: BLE001 - the literals above still decode a frame
        return
    THIN_TAG, THIN_HEADER = T, H
    THIN_TAG_JPEG, THIN_HEADER_JPEG = TJ, HJ


_import_contract()


# --------------------------------------------------------------------------
# pure decode / measure helpers
# --------------------------------------------------------------------------


def rgb565_to_rgb(payload: bytes, width: int, height: int):
    """Decode little-endian RGB565 back to an (H, W, 3) uint8 array.

    The inverse of `thin_render.rgba_to_rgb565`, with the low bits replicated
    into the gaps (`>> 5` / `>> 6`) so a saturated channel decodes to 255 rather
    than 248 -- otherwise pure white would read as off-white and every
    brightness measure below would carry a constant bias.
    """
    import numpy as np

    px = np.frombuffer(payload, dtype="<u2", count=width * height)
    px = px.reshape(height, width)
    r = ((px >> 11) & 0x1F).astype(np.uint8)
    g = ((px >> 5) & 0x3F).astype(np.uint8)
    b = (px & 0x1F).astype(np.uint8)
    out = np.empty((height, width, 3), dtype=np.uint8)
    out[:, :, 0] = (r << 3) | (r >> 2)
    out[:, :, 1] = (g << 2) | (g >> 4)
    out[:, :, 2] = (b << 3) | (b >> 2)
    return out


def decode_thin_jpeg(payload: bytes, width: int, height: int) -> dict:
    """Decode a tag-2 JFIF payload back to an (H, W, 3) uint8 array.

    Returns `{"ok": False, "error": ...}` rather than raising when
    `simplejpeg` is unavailable or the bytes do not decode -- the same
    "report what happened" contract as `decode_thin_frame`.
    """
    try:
        import simplejpeg
    except Exception as exc:  # noqa: BLE001
        return {"ok": False, "error": f"simplejpeg not importable: {exc!r}"}
    try:
        rgb = simplejpeg.decode_jpeg(payload, colorspace="rgb")
    except Exception as exc:  # noqa: BLE001 - a torn JPEG is a finding
        return {"ok": False, "error": f"JPEG decode failed: {exc!r}"}
    if rgb.shape[:2] != (height, width):
        return {"ok": False, "error": (f"decoded {rgb.shape[1]}x{rgb.shape[0]} "
                                       f"!= header {width}x{height}")}
    return {"ok": True, "rgb": rgb}


def decode_thin_frame(buf: bytes) -> dict:
    """Parse one binary `/ws-thin` frame -- tag 1 (`<IHH` + RGB565) or tag 2
    (`<IHHI` + JFIF bytes, #197).

    Returns a dict with `ok` plus, when it parses, the decoded image under
    `rgb`. The length check is exact in both directions: a payload longer than
    the header claims means the packer and this reader have drifted, which on a
    real thin client shows up as a torn picture, not an exception.
    """
    if len(buf) < 4:
        return {"ok": False, "error": f"runt frame: {len(buf)} bytes"}
    (tag,) = struct.unpack_from("<I", buf, 0)

    if tag == THIN_TAG_JPEG:
        if len(buf) < THIN_HEADER_JPEG.size:
            return {"ok": False, "error": f"runt JPEG frame: {len(buf)} bytes"}
        _tag, width, height, jpeg_len = THIN_HEADER_JPEG.unpack_from(buf, 0)
        payload = buf[THIN_HEADER_JPEG.size:]
        info = {"tag": int(tag), "width": int(width), "height": int(height),
                "jpeg_len": int(jpeg_len), "payload_bytes": len(payload),
                "total_bytes": len(buf)}
        if len(payload) != jpeg_len:
            info["ok"] = False
            info["error"] = (f"payload is {len(payload)} bytes, expected "
                             f"{jpeg_len} jpeg_len")
            return info
        info.update(decode_thin_jpeg(payload, int(width), int(height)))
        return info

    if len(buf) < THIN_HEADER.size:
        return {"ok": False, "error": f"runt frame: {len(buf)} bytes"}
    tag, width, height = THIN_HEADER.unpack_from(buf, 0)
    payload = buf[THIN_HEADER.size:]
    expected = width * height * 2
    info = {"tag": int(tag), "width": int(width), "height": int(height),
            "payload_bytes": len(payload), "expected_payload_bytes": expected,
            "total_bytes": len(buf)}
    if tag != THIN_TAG:
        info["ok"] = False
        info["error"] = f"tag {tag} != THIN_FRAME tag {THIN_TAG} or JPEG tag {THIN_TAG_JPEG}"
        return info
    if len(payload) != expected:
        info["ok"] = False
        info["error"] = (f"payload is {len(payload)} bytes, expected {expected} "
                         f"for {width}x{height} RGB565")
        return info
    info["ok"] = True
    info["rgb"] = rgb565_to_rgb(payload, int(width), int(height))
    return info


def frame_stats(rgb) -> dict:
    """Cheap "is this an actual picture or a blank buffer" summary.

    `distinct_colors` is the one that matters: the offscreen spike's first
    success criterion was 23 599 distinct colours in a 480x480 PNG, because a
    render that fails silently returns a uniformly filled buffer that every
    mean/variance check happily calls a frame.
    """
    import numpy as np

    arr = np.asarray(rgb)
    flat = arr.reshape(-1, 3)
    packed = (flat[:, 0].astype(np.uint32) << 16 | flat[:, 1].astype(np.uint32) << 8
              | flat[:, 2].astype(np.uint32))
    nonblack = int((flat.max(axis=1) > PIXEL_DIFF_THRESHOLD).sum())
    return {"distinct_colors": int(np.unique(packed).size),
            "nonblack_frac": round(nonblack / max(len(flat), 1), 4),
            "mean_luma": round(float(flat.mean()), 2)}


def changed_frac(a, b, threshold: int = PIXEL_DIFF_THRESHOLD) -> float | None:
    """Fraction of pixels whose brightest channel moved by more than `threshold`."""
    import numpy as np

    if a is None or b is None:
        return None
    x = np.asarray(a, dtype=np.int16)
    y = np.asarray(b, dtype=np.int16)
    if x.shape != y.shape:
        return None
    diff = np.abs(x - y).max(axis=2)
    return round(float((diff > threshold).mean()), 4)


def save_png(rgb, path: Path) -> dict:
    """Write a decoded frame to `path`. Never raises -- the report says so."""
    try:
        from PIL import Image

        path.parent.mkdir(parents=True, exist_ok=True)
        Image.fromarray(rgb, mode="RGB").save(path)
    except Exception as exc:  # noqa: BLE001 - a missing PNG must not lose the numbers
        return {"path": str(path), "saved": False, "error": repr(exc)}
    entry = {"path": str(path), "saved": True, "bytes": path.stat().st_size}
    try:
        entry["url"] = "/results/" + str(path.resolve().relative_to(REPO / "results")).replace("\\", "/")
    except ValueError:
        pass  # saved outside results/, so the web server does not serve it
    return entry


def _normalize_modes(modes) -> list[str]:
    if modes is None:
        return list(DEFAULT_MODES)
    if isinstance(modes, str):
        modes = [m.strip() for m in modes.split(",")]
    return [m for m in modes if m]


def default_out_dir() -> Path:
    """`results/thin_probe/<timestamp>/` -- inside the tree roomscan-web already
    serves at `/results/`, so a saved frame is viewable without moving it."""
    return REPO / "results" / "thin_probe" / time.strftime("%Y%m%d_%H%M%S")


# --------------------------------------------------------------------------
# the probe
# --------------------------------------------------------------------------


class _ThinLink:
    """Receive side of one `/ws-thin` connection, sorting frames from JSON.

    Keeps the newest telemetry (the server's own authoritative view of mode and
    recording) so the command round-trips below can wait on the SERVER's
    confirmation rather than on a sleep.
    """

    def __init__(self, ws, report: dict):
        self.ws = ws
        self.report = report
        self.telemetry: dict | None = None
        self.telemetry_seen = 0
        self.frames_received = 0
        self.closed_error: str | None = None
        #: `time.monotonic()` at which the MOST RECENT message was actually
        #: received off the socket -- set by `next_message`, read by
        #: `next_frame_after` to time-anchor a capture against a send.
        self.last_recv_time: float = 0.0

    async def send(self, msg: dict) -> bool:
        try:
            await self.ws.send(json.dumps(msg))
            self.report["commands_sent"].append(msg)
            return True
        except Exception as exc:  # noqa: BLE001
            self.closed_error = repr(exc)
            self.report["errors"].append(f"send {msg.get('type')}: {exc!r}")
            return False

    async def next_message(self, timeout: float):
        """('frame', info) | ('json', obj) | ('timeout', None) | ('closed', repr)."""
        try:
            msg = await asyncio.wait_for(self.ws.recv(), timeout=timeout)
        except asyncio.TimeoutError:
            return "timeout", None
        except Exception as exc:  # noqa: BLE001 - server went away mid-probe
            self.closed_error = repr(exc)
            return "closed", repr(exc)
        self.last_recv_time = time.monotonic()
        if isinstance(msg, (bytes, bytearray)):
            info = decode_thin_frame(bytes(msg))
            self.frames_received += 1
            if not info.get("ok"):
                self.report["decode_errors"].append(info.get("error"))
            return "frame", info
        try:
            data = json.loads(msg)
        except Exception as exc:  # noqa: BLE001
            self.report["errors"].append(f"non-JSON text message: {exc!r}")
            return "json", {}
        if data.get("type") == "thin_telemetry":
            self.telemetry = data
            self.telemetry_seen += 1
        elif data.get("type") == "error":
            self.report["server_error"] = data
        return "json", data

    async def next_frame(self, timeout: float):
        """The next decoded frame, or None (timeout / close / undecodable)."""
        deadline = time.monotonic() + timeout
        while (remain := deadline - time.monotonic()) > 0:
            kind, payload = await self.next_message(remain)
            if kind == "frame":
                return payload if payload.get("ok") else None
            if kind in ("timeout", "closed"):
                return None
            if self.report.get("server_error"):
                return None
        return None

    async def drain(self, timeout: float = DRAIN_POLL_TIMEOUT) -> int:
        """Discard whatever is ALREADY sitting in the receive queue, without
        waiting for anything new. Returns how many messages were discarded.

        `websockets` keeps filling its receive queue in the background
        regardless of whether this probe reads promptly, and at a high
        negotiated fps the probe (decode + PNG-write per frame) is a SLOWER
        CONSUMER than the server is a producer -- so a burst of already-
        queued, pre-command frames can sit ahead of anything sent from here
        on out (#197 review, live finding 2026-08-18). A short per-read
        timeout is what makes this non-blocking rather than another fixed
        sleep: a message already in the queue resolves near-instantly, well
        under `timeout`; an empty queue means `recv()` has to actually wait
        for the network, so it hits `timeout` instead -- which is this
        method's stopping condition.
        """
        drained = 0
        while True:
            kind, _payload = await self.next_message(timeout)
            if kind in ("timeout", "closed"):
                return drained
            drained += 1

    async def next_frame_after(self, deadline_recv_time: float, timeout: float):
        """The next decoded frame RECEIVED at or after `deadline_recv_time`
        (a `time.monotonic()` value); returns `(frame_or_None, discarded)`.

        Preferred over counting a fixed number of "settle frames" (#197
        review): at a high negotiated fps the receive-buffer backlog is not
        bounded by frame count, so "discard the next N" can still resolve
        entirely within a pre-command backlog when N is smaller than that
        backlog's depth. Time-anchoring asks the one question that actually
        matters -- was this frame produced after the command had landed --
        and keeps discarding, however many that takes, until it is.
        """
        discarded = 0
        deadline = time.monotonic() + timeout
        while (remain := deadline - time.monotonic()) > 0:
            kind, payload = await self.next_message(remain)
            if kind in ("timeout", "closed"):
                return None, discarded
            if self.report.get("server_error"):
                return None, discarded
            if kind != "frame" or not payload.get("ok"):
                continue
            if self.last_recv_time >= deadline_recv_time:
                return payload, discarded
            discarded += 1
        return None, discarded

    async def wait_telemetry(self, predicate, timeout: float) -> dict | None:
        """Next telemetry satisfying `predicate` -- the server's own confirmation."""
        deadline = time.monotonic() + timeout
        while (remain := deadline - time.monotonic()) > 0:
            kind, payload = await self.next_message(remain)
            if kind in ("timeout", "closed"):
                return None
            if kind == "json" and payload.get("type") == "thin_telemetry":
                if predicate(payload):
                    return payload
        return None

    async def wait_hello_ack(self, timeout: float) -> dict | None:
        """Next `thin_hello_ack` -- the server's echo of the CLAMPED effective
        values, never the raw request (#197: "report what actually happened")."""
        deadline = time.monotonic() + timeout
        while (remain := deadline - time.monotonic()) > 0:
            kind, payload = await self.next_message(remain)
            if kind in ("timeout", "closed"):
                return None
            if kind == "json" and payload.get("type") == "thin_hello_ack":
                return payload
        return None


async def _probe_hello(link: _ThinLink, report: dict, *, fmt: str | None,
                       fps: int | None, width: int | None, height: int | None,
                       quality: int | None, timeout: float) -> None:
    """Negotiate `thin_hello` (#197) when any of `fmt`/`fps`/`width`/`height`/
    `quality` was passed. A default run (nothing passed) sends nothing and
    keeps today's behaviour -- tag-1 RGB565 480x480 @ 10 fps.

    `width`/`height`: the protocol is SQUARE-ONLY (`THIN_HELLO_RESOLUTIONS`),
    and `--width` alone with no `--height` used to build a hello the server
    silently ignores in full (it requires both, or neither -- #197 review
    finding 7). Chosen fix: when only one of the two is given, use it for
    BOTH, since a non-square pair is meaningless on this protocol anyway.
    """
    if width is not None and height is None:
        height = width
    elif height is not None and width is None:
        width = height

    hello: dict = {"type": "thin_hello"}
    if fmt is not None:
        hello["format"] = fmt
    if fps is not None:
        hello["fps"] = fps
    if width is not None:
        hello["width"] = width
    if height is not None:
        hello["height"] = height
    if quality is not None:
        hello["quality"] = quality
    if len(hello) == 1:
        report["hello"] = None
        return

    entry: dict = {"requested": hello, "acked": None, "degraded": False}
    report["hello"] = entry
    if not await link.send(hello):
        entry["error"] = "send failed"
        return
    ack = await link.wait_hello_ack(timeout)
    if ack is None:
        entry["error"] = f"no thin_hello_ack within {timeout:g}s"
        return
    entry["ack"] = ack
    entry["acked"] = {k: ack.get(k) for k in ("format", "fps", "width", "height", "quality")}
    # requested-vs-acked DRIFT, surfaced explicitly rather than left for the
    # reader to diff two dicts (#197 review finding 7): a clamp (e.g. fps
    # 999 -> 60) is expected and not "degraded"; only a FORMAT downgrade
    # (jpeg requested, raw acked -- the encoder was unavailable) counts.
    entry["degraded"] = bool(fmt == "jpeg" and ack.get("format") == "raw")


async def _collect_initial(link: _ThinLink, frames: int, out_dir: Path,
                           report: dict, timeout: float) -> None:
    """Receive `frames` frames in the default mode, timing the real cadence."""
    started = None
    last = None
    for i in range(frames):
        info = await link.next_frame(timeout)
        if info is None:
            why = ("connection closed" if link.closed_error else
                   "server sent an error" if report.get("server_error") else
                   f"no THIN_FRAME within {timeout:g}s")
            report["errors"].append(f"{why} (frame {i + 1} of {frames})")
            break
        now = time.monotonic()
        if started is None:
            started = now  # start the clock on the FIRST frame: everything before
            # it is connect + the server's first-tick latency, not feed cadence
        last = now
        entry = {"index": i, "tag": info.get("tag"), "width": info["width"],
                 "height": info["height"], "total_bytes": info["total_bytes"],
                 **frame_stats(info["rgb"])}
        entry.update(save_png(info["rgb"], out_dir / f"frame_{i:02d}.png"))
        report["frames"].append(entry)
    got = len(report["frames"])
    span = (last - started) if (started is not None and last is not None) else 0.0
    report["frames_received"] = got
    report["measured_fps"] = round((got - 1) / span, 2) if got > 1 and span > 0 else None


async def _probe_orbit(link: _ThinLink, out_dir: Path, report: dict,
                       dyaw: float, timeout: float,
                       interval_s: float = DEFAULT_THIN_INTERVAL_S) -> None:
    """Does `thin_orbit` move the PICTURE? Measured against a no-command control.

    Both halves drain the receive buffer immediately before reading, and the
    post-command frame is additionally TIME-ANCHORED rather than frame-
    counted -- see `_ThinLink.drain`/`next_frame_after`. `interval_s` should
    be the flow's actual negotiated cadence (1/fps from a `thin_hello` ack,
    or the server default) so the anchor window is "one real tick", not a
    guess.
    """
    orbit: dict = {"dyaw": dyaw, "interval_s": interval_s,
                  "settle_method": "time-anchored"}
    report["orbit"] = orbit

    # The control: two consecutive frames, no command in between. Draining
    # before EACH capture keeps it symmetric with how "after" is captured
    # below -- both are "freshest available at read time", never a frame
    # that had been sitting in a backlog since before this phase even
    # started (#197 review, live finding 2026-08-18).
    await link.drain()
    a = await link.next_frame(timeout)
    await link.drain()
    b = await link.next_frame(timeout)
    if a is None or b is None:
        orbit["error"] = "no frame pair for the control measurement"
        return
    # Whatever the scene does on its own -- replay advancing, live geometry,
    # render dither -- shows up here, and the orbit has to beat it.
    orbit["control_changed_frac"] = changed_frac(a["rgb"], b["rgb"])
    orbit["before"] = save_png(b["rgb"], out_dir / "orbit_before.png")
    orbit["before"].update(frame_stats(b["rgb"]))

    if not await link.send({"type": "thin_orbit", "dyaw": dyaw}):
        orbit["error"] = "thin_orbit send failed"
        return
    send_completed_at = time.monotonic()

    # 'after' must have been RECEIVED at least one full negotiated interval
    # past the send -- discarding anything received before that, however
    # many frames that takes, rather than trusting a fixed settle count that
    # a receive-buffer backlog can exhaust before the orbit ever landed.
    deadline_recv_time = send_completed_at + max(interval_s, 0.0)
    c, discarded = await link.next_frame_after(deadline_recv_time, timeout)
    orbit["frames_discarded"] = discarded
    if c is None:
        orbit["error"] = "no frame after thin_orbit"
        return
    orbit["after"] = save_png(c["rgb"], out_dir / "orbit_after.png")
    orbit["after"].update(frame_stats(c["rgb"]))
    orbit["changed_frac"] = changed_frac(b["rgb"], c["rgb"])

    ctrl = orbit.get("control_changed_frac")
    moved = orbit["changed_frac"]
    orbit["criterion"] = (f"changed_frac >= {ORBIT_MIN_CHANGED_FRAC} and "
                          f"> {ORBIT_CONTROL_MARGIN:g}x the no-command control")
    orbit["moved_pixels"] = bool(
        moved is not None and moved >= ORBIT_MIN_CHANGED_FRAC
        and (ctrl is None or moved > ORBIT_CONTROL_MARGIN * max(ctrl, 1e-4)))


async def _probe_modes(link: _ThinLink, modes, out_dir: Path, report: dict,
                       timeout: float) -> None:
    """Round-trip `thin_mode` for each mode and report which produced a frame."""
    results: dict = {}
    report["modes"] = results
    for mode in modes:
        entry: dict = {"requested": mode, "confirmed": False, "frame": False}
        results[mode] = entry
        if not await link.send({"type": "thin_mode", "mode": mode}):
            entry["error"] = "send failed"
            continue
        tel = await link.wait_telemetry(lambda t, m=mode: t.get("mode") == m, timeout)
        if tel is None:
            entry["error"] = (f"server telemetry never reported mode {mode!r} "
                              f"within {timeout:g}s")
            continue
        entry["confirmed"] = True
        entry["telemetry"] = {k: tel.get(k) for k in
                              ("mode", "fps", "point_count", "recording")}
        # Telemetry goes out at the top of a tick and the frame at the bottom of
        # the same tick, so this frame is genuinely rendered in `mode`.
        info = await link.next_frame(timeout)
        if info is None:
            entry["note"] = ("mode confirmed but no frame arrived -- the render loop "
                             "sends nothing when it has no fresh (generation-matching) "
                             "data for this mode")
            continue
        entry["frame"] = True
        entry.update({"width": info["width"], "height": info["height"]})
        entry.update(frame_stats(info["rgb"]))
        entry["png"] = save_png(info["rgb"], out_dir / f"mode_{mode}.png")


async def _probe_record(link: _ThinLink, report: dict, timeout: float) -> None:
    """Opt-in only: start recording via `thin_record`, confirm, stop, confirm.

    Never runs by default. This is the one command on `/ws-thin` with a side
    effect that outlives the probe -- it writes a capture file and changes what
    every browser tab shows.
    """
    rec: dict = {"requested": True}
    report["record"] = rec
    if not await link.send({"type": "thin_record", "on": True}):
        rec["error"] = "send failed"
        return
    on = await link.wait_telemetry(lambda t: bool(t.get("recording")), timeout)
    rec["started"] = on is not None
    if not await link.send({"type": "thin_record", "on": False}):
        rec["error"] = "stop send failed; a recording may still be running"
        return
    off = await link.wait_telemetry(lambda t: not t.get("recording"), timeout)
    rec["stopped"] = off is not None
    if not rec["stopped"]:
        rec["error"] = ("recording was not observed to stop; check rig_status() -- "
                        "a capture may still be running")


async def probe_async(frames: int = DEFAULT_FRAMES, url: str = DEFAULT_URL,
                      out_dir: str | Path | None = None,
                      orbit_yaw: float = DEFAULT_ORBIT_YAW,
                      modes=DEFAULT_MODES, record: bool = False,
                      timeout: float = DEFAULT_TIMEOUT,
                      format: str | None = None, fps: int | None = None,
                      width: int | None = None, height: int | None = None,
                      quality: int | None = None) -> dict:
    """Connect to `/ws-thin`, decode frames to PNG, round-trip the commands.

    The real implementation; `probe()` is the sync front end and
    `rig_thin_probe` the MCP one. Reports what happened rather than what was
    asked for: a mode with no data, a server at its client cap, an unavailable
    renderer and a server that is not running are all *results* here, not
    exceptions.

    `format`/`fps`/`width`/`height`/`quality` negotiate `thin_hello` (#197)
    when any is given; leaving all five `None` sends no hello at all, so a
    default run still proves the un-negotiated path -- today's tag-1 RGB565
    480x480 @ 10 fps, byte-identical.
    """
    ws_url = url.rstrip("/")
    if not ws_url.endswith("/ws-thin"):
        ws_url = f"{ws_url}/ws-thin"
    modes = _normalize_modes(modes)
    out = Path(out_dir) if out_dir else default_out_dir()

    report: dict = {
        "ok": False, "url": ws_url, "out_dir": str(out),
        "frames_requested": int(frames), "frames_received": 0,
        "measured_fps": None, "frames": [], "decode_errors": [],
        "hello": None, "orbit": None, "modes": {}, "record": None,
        "telemetry": None, "telemetry_seen": 0,
        "commands_sent": [], "server_error": None, "errors": [],
    }

    try:
        import websockets
    except Exception as exc:  # noqa: BLE001
        report["errors"].append(f"websockets is not importable: {exc!r}")
        return report

    try:
        ws_cm = websockets.connect(ws_url, max_size=None, open_timeout=timeout)
        conn = await ws_cm.__aenter__()
    except Exception as exc:  # noqa: BLE001 - server down is a finding, not a crash
        report["errors"].append(f"could not connect to {ws_url}: {exc!r}")
        report["hint"] = ("is roomscan-web running? rig_status() answers that; "
                          "rig_up() starts it")
        return report

    link = _ThinLink(conn, report)
    try:
        await _probe_hello(link, report, fmt=format, fps=fps, width=width,
                           height=height, quality=quality, timeout=timeout)
        await _collect_initial(link, max(int(frames), 0), out, report, timeout)

        # A refusal (`thin_client_limit`, `thin_render_unavailable`) is JSON
        # followed by a close, so it surfaces above as zero frames plus a
        # `server_error`. Stop here rather than sending commands into a socket
        # the server has already shut, which would only pile up send errors.
        if report["server_error"]:
            report["errors"].append(
                f"server refused the connection: {report['server_error'].get('error')}: "
                f"{report['server_error'].get('message')}")
            return report

        if report["frames_received"] == 0:
            report["errors"].append(
                "no THIN_FRAME arrived -- the render loop sends nothing when it has "
                "no fresh data for the current mode; is a source streaming?")
        else:
            # The negotiated cadence, if any -- sizes the orbit phase's
            # time-anchor window to "one real tick" rather than a guess.
            interval_s = DEFAULT_THIN_INTERVAL_S
            hello_ack = (report.get("hello") or {}).get("ack") or {}
            acked_fps = hello_ack.get("fps")
            if acked_fps:
                interval_s = 1.0 / float(acked_fps)
            await _probe_orbit(link, out, report, orbit_yaw, timeout,
                               interval_s=interval_s)
        await _probe_modes(link, modes, out, report, timeout)
        if record:
            await _probe_record(link, report, timeout)
        else:
            report["record"] = {"requested": False,
                                "note": "pass record=True to exercise thin_record; "
                                        "it starts a real capture"}
    finally:
        report["telemetry"] = link.telemetry
        report["telemetry_seen"] = link.telemetry_seen
        report["frames_total_seen"] = link.frames_received
        if link.closed_error:
            report["errors"].append(f"connection error: {link.closed_error}")
        try:
            await ws_cm.__aexit__(None, None, None)
        except Exception:  # noqa: BLE001 - closing a dead socket is not a finding
            pass

    report["png_paths"] = [e["path"] for e in report["frames"] if e.get("saved")]
    for key in ("before", "after"):
        entry = (report.get("orbit") or {}).get(key)
        if entry and entry.get("saved"):
            report["png_paths"].append(entry["path"])
    for entry in report["modes"].values():
        png = entry.get("png")
        if png and png.get("saved"):
            report["png_paths"].append(png["path"])
    report["modes_with_frames"] = sorted(m for m, e in report["modes"].items()
                                         if e.get("frame"))

    # `ok` used to ignore negotiation entirely -- a degraded (jpeg requested,
    # raw acked) or never-acked hello still reported ok:true (#197 review
    # finding 7). When a hello was actually requested, `ok` now also requires
    # the ack to have arrived AND every collected frame's tag to match the
    # ACKED format (frames are collected strictly after the hello round-trip
    # above, so this can see a server that acked jpeg but kept sending tag 1).
    hello_ok = True
    hello_entry = report.get("hello")
    if hello_entry is not None:
        hello_ok = bool(hello_entry.get("ack")) and not hello_entry.get("error")
        acked_fmt = (hello_entry.get("acked") or {}).get("format")
        if acked_fmt is not None:
            expected_tag = THIN_TAG_JPEG if acked_fmt == "jpeg" else THIN_TAG
            mismatched = [f["index"] for f in report["frames"]
                         if f.get("tag") not in (None, expected_tag)]
            if mismatched:
                hello_ok = False
                report["errors"].append(
                    f"frame(s) {mismatched} carried a tag != expected "
                    f"{expected_tag} for the ACKED format {acked_fmt!r}")

    report["ok"] = bool(report["frames_received"] and not report["server_error"]
                        and hello_ok)
    return report


def probe(**kwargs) -> dict:
    """Sync front end for the CLI. Do not call from inside an event loop."""
    return asyncio.run(probe_async(**kwargs))


def _print(rep: dict) -> None:
    print(f"thin probe {rep['url']} -> {rep['out_dir']}")
    if rep.get("hello"):
        h = rep["hello"]
        if h.get("ack"):
            a = h["ack"]
            print(f"  hello: requested {h['requested']} -> ack format={a.get('format')} "
                  f"fps={a.get('fps')} {a.get('width')}x{a.get('height')} "
                  f"quality={a.get('quality')}"
                  + ("  DEGRADED (jpeg unavailable server-side)" if h.get("degraded") else ""))
        elif h.get("error"):
            print(f"  hello: {h['error']}")
    if rep["server_error"]:
        e = rep["server_error"]
        print(f"  SERVER REFUSED: {e.get('error')}: {e.get('message')}")
    print(f"  frames {rep['frames_received']}/{rep['frames_requested']}"
          f"  fps {rep['measured_fps']}  telemetry msgs {rep['telemetry_seen']}")
    for f in rep["frames"]:
        print(f"    frame {f['index']:02d} {f.get('width')}x{f.get('height')} "
              f"{f.get('distinct_colors')} colors  nonblack {f.get('nonblack_frac')}"
              f"  -> {f.get('path')}")
    orb = rep.get("orbit") or {}
    if orb:
        if orb.get("error"):
            print(f"  orbit: {orb['error']}")
        else:
            print(f"  orbit dyaw={orb['dyaw']:g}: changed {orb.get('changed_frac')} "
                  f"of pixels vs control {orb.get('control_changed_frac')} "
                  f"-> {'MOVED' if orb.get('moved_pixels') else 'NO VISIBLE CHANGE'}")
    for mode, e in (rep.get("modes") or {}).items():
        state = ("frame" if e.get("frame") else
                 "confirmed, no frame" if e.get("confirmed") else "not confirmed")
        print(f"  mode {mode:<12} {state}"
              + (f"  {e.get('distinct_colors')} colors" if e.get("frame") else "")
              + (f"  ({e['error']})" if e.get("error") else ""))
    rec = rep.get("record") or {}
    if rec.get("requested"):
        print(f"  record: started={rec.get('started')} stopped={rec.get('stopped')}")
    if rep.get("telemetry"):
        t = rep["telemetry"]
        print(f"  telemetry: fps={t.get('fps')} mode={t.get('mode')} "
              f"points={t.get('point_count')} recording={t.get('recording')}")
        # `null` heading is a real state, not a gap: it means no compass
        # bearing exists right now (see docs/thin-client.md).
        print(f"  orientation: roll={t.get('roll_deg')} tilt={t.get('tilt_deg')} "
              f"heading={t.get('heading_deg')} valid={t.get('orientation_valid')}")
    for err in rep["decode_errors"]:
        print(f"  DECODE: {err}")
    for err in rep["errors"]:
        print(f"  error: {err}")
    if rep.get("hint"):
        print(f"  hint: {rep['hint']}")


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--frames", type=int, default=DEFAULT_FRAMES,
                    help=f"frames to receive and save (default {DEFAULT_FRAMES})")
    ap.add_argument("--url", default=DEFAULT_URL,
                    help=f"server base or full /ws-thin URL (default {DEFAULT_URL})")
    ap.add_argument("--out-dir", default=None,
                    help="where to write PNGs (default results/thin_probe/<timestamp>)")
    ap.add_argument("--orbit-yaw", type=float, default=DEFAULT_ORBIT_YAW,
                    help=f"yaw delta for the orbit test (default {DEFAULT_ORBIT_YAW:g} deg)")
    ap.add_argument("--modes", default=",".join(DEFAULT_MODES),
                    help="comma-separated modes to round-trip")
    ap.add_argument("--record", action="store_true",
                    help="also round-trip thin_record -- this STARTS A REAL RECORDING")
    ap.add_argument("--timeout", type=float, default=DEFAULT_TIMEOUT,
                    help=f"per-message wait in seconds (default {DEFAULT_TIMEOUT:g})")
    ap.add_argument("--format", choices=("raw", "jpeg"), default=None,
                    help="negotiate thin_hello format (#197); default sends no hello")
    ap.add_argument("--fps", type=int, default=None,
                    help="negotiate thin_hello fps (#197)")
    ap.add_argument("--width", type=int, default=None,
                    help="negotiate thin_hello width, one of 320/480 (#197); "
                         "alone (no --height) sends a SQUARE request of this size")
    ap.add_argument("--height", type=int, default=None,
                    help="negotiate thin_hello height, one of 320/480 (#197); "
                         "alone (no --width) sends a SQUARE request of this size")
    ap.add_argument("--quality", type=int, default=None,
                    help="negotiate thin_hello JPEG quality 40-95 (#197)")
    ap.add_argument("--json", action="store_true")
    args = ap.parse_args(argv)

    rep = probe(frames=args.frames, url=args.url, out_dir=args.out_dir,
                orbit_yaw=args.orbit_yaw, modes=args.modes, record=args.record,
                timeout=args.timeout, format=args.format, fps=args.fps,
                width=args.width, height=args.height, quality=args.quality)
    if args.json:
        print(json.dumps(rep, indent=2, default=str))
    else:
        _print(rep)
    return 0 if rep.get("ok") else 1


if __name__ == "__main__":
    sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
    _import_contract()
    raise SystemExit(main())
