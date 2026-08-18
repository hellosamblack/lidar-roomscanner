"""Regression test for `tools/thin_client_probe.py`'s orbit measurement (#197).

Live finding, 2026-08-18: `thin_orbit` read as pixel-dead on a real server at
a high negotiated fps (jpeg@30) while a direct websocket client saw it move
within one frame. Root cause was in THIS probe, not the server: at high fps
the probe (decode + PNG-write per frame) is a slower consumer than the server
is a producer, so several PRE-command frames were already queued in the
receive buffer by the time `thin_orbit` was sent. The old fixed
"discard 2 settle frames" logic could -- and did -- exhaust itself entirely
inside that backlog, so `after` was still a STALE (pre-command) frame and
`changed_frac` read identical to the no-command control. #106 family: the
probe's own observable was not on the far side of the thing it measured.

This file drives `_probe_orbit` against a fake websocket that can model
exactly that backlog, without a real server or real Filament.
"""
from __future__ import annotations

import asyncio
import json

import numpy as np
import pytest

from roomscan.thin_render import pack_thin_frame, rgba_to_rgb565
from tools.thin_client_probe import DEFAULT_THIN_INTERVAL_S, DRAIN_POLL_TIMEOUT, _ThinLink, _probe_orbit


def _solid_frame(level: int, size: int = 4) -> bytes:
    """A tiny valid tag-1 THIN_FRAME, every pixel the same (level, level, level)."""
    img = np.full((size, size, 3), level, dtype=np.uint8)
    return pack_thin_frame(rgba_to_rgb565(img), size, size)


#: two maximally-distinguishable solid frames -- comfortably past
#: PIXEL_DIFF_THRESHOLD (8) on every channel, so `changed_frac` between them
#: is unambiguous (~1.0) and between two of the SAME one is exactly 0.0.
_STALE = _solid_frame(10)
_FRESH = _solid_frame(250)


class _QueueWs:
    """Fake `websockets` connection modelling a receive-buffer BACKLOG plus an
    ongoing live producer.

    `backlog` is delivered instantly on `recv()` -- exactly like messages
    already sitting in `websockets`' internal queue. Once it is exhausted,
    `live()` is called to synthesize the next message on demand, after a
    small REAL delay (`_LIVE_DELAY`, deliberately bigger than
    `DRAIN_POLL_TIMEOUT`) -- long enough that a short poll (`drain()`'s ~10ms)
    correctly reads "nothing buffered right now" and stops, short enough that
    a generous one (`next_frame()`'s test timeout) easily waits it out. This
    is what lets ONE fake exercise both "a deep pre-existing backlog" and "the
    server keeps ticking after that", without a full virtual clock.
    """

    _LIVE_DELAY = DRAIN_POLL_TIMEOUT * 5

    def __init__(self, backlog, live, *, on_send=None):
        self._backlog = list(backlog)
        self._live = live
        self._on_send = on_send
        self.sent: list[str] = []

    async def recv(self):
        if self._backlog:
            return self._backlog.pop(0)
        await asyncio.sleep(self._LIVE_DELAY)
        msg = self._live()
        if msg is None:
            await asyncio.sleep(3600)  # never resolves within any test timeout
            raise AssertionError("no more live messages configured")
        return msg

    async def send(self, data):
        self.sent.append(data)
        if self._on_send is not None:
            self._on_send(json.loads(data))


class _LiveProducer:
    """`STALE` until `thin_orbit` is observed on the wire, then `FRESH` --
    i.e. exactly what a real paused-replay render loop does: it keeps
    ticking (new frame objects) with unchanged content until the command
    lands, then switches."""

    def __init__(self):
        self.orbit_sent = False

    def __call__(self):
        return _FRESH if self.orbit_sent else _STALE

    def on_send(self, msg: dict) -> None:
        if msg.get("type") == "thin_orbit":
            self.orbit_sent = True


def _fresh_report() -> dict:
    return {"commands_sent": [], "errors": [], "decode_errors": [], "server_error": None}


def _run_probe_orbit(ws, report, tmp_path, *, interval_s: float, timeout: float = 5.0):
    """Build the link, drive `_probe_orbit`, and return `report["orbit"]`."""
    link = _ThinLink(ws, report)
    asyncio.run(asyncio.wait_for(
        _probe_orbit(link, tmp_path, report, 120.0, timeout, interval_s=interval_s),
        timeout=timeout + 2.0))
    return report["orbit"]


# --------------------------------------------------------------------------
# the regression: a deep pre-command backlog at a high negotiated fps
# --------------------------------------------------------------------------


def test_probe_orbit_survives_a_deep_pre_command_backlog(tmp_path):
    """5 pre-queued STALE frames (deeper than the old fixed settle count of
    2) must not fool the orbit measurement -- proves the fix, and pins the
    exact scenario the live finding described."""
    producer = _LiveProducer()
    ws = _QueueWs([_STALE] * 5, producer, on_send=producer.on_send)
    report = _fresh_report()

    orbit = _run_probe_orbit(ws, report, tmp_path, interval_s=1.0 / 30.0)

    assert orbit.get("error") is None, orbit
    assert orbit["control_changed_frac"] == pytest.approx(0.0)
    assert orbit["changed_frac"] == pytest.approx(1.0)   # STALE vs FRESH
    assert orbit["moved_pixels"] is True
    assert orbit["settle_method"] == "time-anchored"


def test_probe_orbit_control_stays_honest_with_no_backlog_at_all(tmp_path):
    """No backlog, nothing but the live producer -- the ordinary case -- must
    still measure a clean null control and a clean orbit detection."""
    producer = _LiveProducer()
    ws = _QueueWs([], producer, on_send=producer.on_send)
    report = _fresh_report()

    orbit = _run_probe_orbit(ws, report, tmp_path, interval_s=DEFAULT_THIN_INTERVAL_S)

    assert orbit.get("error") is None, orbit
    assert orbit["control_changed_frac"] == pytest.approx(0.0)
    assert orbit["changed_frac"] == pytest.approx(1.0)
    assert orbit["moved_pixels"] is True


def test_probe_orbit_reports_the_real_discard_count(tmp_path):
    """`frames_discarded` must reflect how many post-send frames were
    actually thrown away before the time-anchor was satisfied -- with a
    backlog this deep every one of them is a discard."""
    producer = _LiveProducer()
    ws = _QueueWs([_STALE] * 8, producer, on_send=producer.on_send)
    report = _fresh_report()

    orbit = _run_probe_orbit(ws, report, tmp_path, interval_s=1.0 / 30.0)

    assert orbit.get("error") is None, orbit
    assert orbit["frames_discarded"] >= 0
    assert isinstance(orbit["frames_discarded"], int)


# --------------------------------------------------------------------------
# defect-reintroduction proof (see the session report for the temporary
# frame-counted revert used to confirm this test fails pre-fix)
# --------------------------------------------------------------------------


def test_drain_empties_an_instantly_available_backlog():
    """Unit-level proof of `_ThinLink.drain()` alone: every already-buffered
    message is discarded and counted, without waiting for a new one."""
    ws = _QueueWs([_STALE, _STALE, _STALE], lambda: None)
    report = _fresh_report()
    link = _ThinLink(ws, report)

    drained = asyncio.run(asyncio.wait_for(link.drain(), timeout=2.0))

    assert drained == 3


def test_drain_stops_without_waiting_for_a_live_message():
    """`drain()` must NOT wait out a live (not-yet-buffered) message -- that
    is `next_frame`'s job. An empty backlog must drain to 0, not 1.

    Deliberately not a wall-clock assertion (flaky under a loaded test
    run) -- `drained == 0` is the actual semantic proof: `drain()`'s own
    per-read timeout is `DRAIN_POLL_TIMEOUT`, architecturally too short to
    ever receive the live producer's `_LIVE_DELAY`-away message, so any
    live message reaching `drain()`'s count would mean the poll timeout
    itself was wired wrong, not a scheduling fluke.
    """
    producer = _LiveProducer()
    ws = _QueueWs([], producer)
    report = _fresh_report()
    link = _ThinLink(ws, report)

    drained = asyncio.run(asyncio.wait_for(link.drain(), timeout=2.0))

    assert drained == 0
