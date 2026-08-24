"""`/ws` backpressure + broadcaster cadence phasing (#217).

Two defects, one reported symptom. The owner saw "almost 7 seconds of latency on
the IR monitor, and the device and IMU frames have dropped as well". Everything
upstream measured healthy -- 30.03 ToF frames/s on the wire, 99.9 % complete,
zero kernel UDP drops, 34 ms from a `set_ir` command to changed pixels leaving
the server -- so neither symptom was a loss of data. Both were queueing:

* uvicorn's websocket send is `transport.write()` with no drain, so `/ws` piles
  unsent frames into the asyncio transport's write buffer without limit.
  `test_write_buffer_is_observable_and_grows` reproduces that against a real
  server and is the reason the fix can exist at all.
* the broadcaster re-anchored each sub-cadence to `now` on every send, which
  quantizes the achieved rate up to a multiple of the 33.3 ms tick.

The rate tests here deliberately assert on ACHIEVED INTERVALS, never on the
scheduling arithmetic -- a test that recomputes `due + interval` the way the
code does would agree with any phasing bug, which is exactly how the same defect
shipped twice (#216, then this).
"""
from __future__ import annotations

import asyncio
import threading
import time

import pytest

from roomscan import web


# --------------------------------------------------------------------------
# The transport walk -- pinned against a REAL uvicorn, not a double
# --------------------------------------------------------------------------

def test_write_buffer_is_observable_and_grows():
    """`ws_backlog_bytes` must resolve on the real stack, and must actually
    track the unsent backlog.

    This is the load-bearing assumption of the whole fix: if a vendor bump
    moves the protocol object out of reach of the closure walk,
    `_ws_transport` fails open and backpressure silently stops working. That
    must fail loudly here rather than quietly in production.

    It also pins the direction: a stalled client's backlog must GROW. An
    assertion that the number merely exists would pass against a transport
    that always reports 0 -- the read-back trap (#106).
    """
    uvicorn = pytest.importorskip("uvicorn")
    websockets = pytest.importorskip("websockets")
    from starlette.applications import Starlette
    from starlette.routing import WebSocketRoute

    observed: dict = {}
    port = 8774

    async def endpoint(ws):
        await ws.accept()
        observed["resolved"] = web.ws_backlog_bytes(ws) is not None
        payload = b"x" * 54_000
        samples = []
        for i in range(240):
            await ws.send_bytes(payload)
            if i % 40 == 0:
                samples.append(web.ws_backlog_bytes(ws))
            await asyncio.sleep(0.005)
        observed["samples"] = samples
        observed["writable_at_end"] = ws in web.writable_clients({ws})
        with pytest.raises(Exception):     # noqa: PT011 - closing a stalled peer
            await ws.close()

    app = Starlette(routes=[WebSocketRoute("/ws", endpoint)])
    server = uvicorn.Server(uvicorn.Config(
        app, host="127.0.0.1", port=port, log_level="critical",
        ws_per_message_deflate=False))
    thread = threading.Thread(target=server.run, daemon=True)
    thread.start()
    for _ in range(100):                   # wait for bind
        if getattr(server, "started", False):
            break
        time.sleep(0.05)

    async def stalled_client():
        async with websockets.connect(
                f"ws://127.0.0.1:{port}/ws", max_size=None) as c:
            await c.recv()
            await asyncio.sleep(3.0)       # stop reading; the server keeps writing
    try:
        asyncio.run(stalled_client())
    except Exception:                      # noqa: BLE001 - the stall ends in a reset
        pass
    server.should_exit = True
    thread.join(timeout=10)

    assert observed.get("resolved") is True, (
        "ws_backlog_bytes could not reach the asyncio transport on this "
        "uvicorn/websockets build -- backpressure is silently disabled")
    samples = [s for s in observed.get("samples", []) if s is not None]
    assert len(samples) >= 4
    assert max(samples) > 500_000, (
        f"a stalled client's write buffer never grew: {samples}")
    assert observed.get("writable_at_end") is False, (
        "a client holding megabytes of unsent data still counted as writable")


# --------------------------------------------------------------------------
# writable_clients -- the gate itself
# --------------------------------------------------------------------------

class _FakeTransport:
    def __init__(self, size):
        self._size = size

    def get_write_buffer_size(self):
        return self._size


class _FakeWs:
    """A websocket whose transport the walk finds via `_send.__self__`."""
    def __init__(self, size):
        self.transport = _FakeTransport(size)
        self._send = self._noop        # bound method -> __self__ is this object

    def _noop(self, *_a, **_kw):
        pass


def _fresh(size):
    ws = _FakeWs(size)
    web._WS_TRANSPORT_CACHE.pop(ws, None)
    return ws


def test_writable_clients_drops_only_the_backed_up_client():
    fast = _fresh(0)
    ok = _fresh(web.WS_BACKLOG_LIMIT_BYTES)          # exactly at the cap: still sent
    slow = _fresh(web.WS_BACKLOG_LIMIT_BYTES + 1)
    out = web.writable_clients({fast, ok, slow})
    assert fast in out and ok in out
    assert slow not in out, "a backed-up client must lose the frame, not queue it"


def test_unresolvable_transport_is_always_writable():
    """Fail-open: an environment where the walk cannot find a transport must
    behave exactly as it did before #217, not drop every frame."""
    class Opaque:
        _send = None
    ws = Opaque()
    assert web.ws_backlog_bytes(ws) is None
    assert web.writable_clients({ws}) == {ws}


def test_backpressure_drops_are_counted_not_silent():
    """A cap that drops silently reads as 'everything is fine' on every
    dashboard. `metrics.ws` has to show it."""
    class S:
        pass
    state = S()
    web._count_backpressure_drops(state, 0)
    assert getattr(state, "ws_backpressure_drops", 0) == 0
    web._count_backpressure_drops(state, 3)
    web._count_backpressure_drops(state, 2)
    assert state.ws_backpressure_drops == 5

    state.clients = {_fresh(1234), _fresh(99)}
    counters = web.ws_flow_counters(state)
    assert counters["backpressure_drops"] == 5
    assert counters["backlog_bytes_max"] == 1234


# --------------------------------------------------------------------------
# Cadence phasing -- asserted on achieved intervals, not on the arithmetic
# --------------------------------------------------------------------------

def _achieved_hz(interval: float, tick: float, seconds: float, *, rephase: bool):
    """Simulate the broadcaster's tick loop and report the ACHIEVED rate.

    `rephase=True` is the old `now - last >= interval; last = now` shape;
    `rephase=False` runs the shipped `_advance_due`. Same driver for both, so
    the comparison isolates the one change.
    """
    fires = []
    due = 0.0
    last = 0.0
    n = int(seconds / tick)
    for i in range(n):
        now = i * tick
        if rephase:
            if now - last >= interval:
                last = now
                fires.append(now)
        else:
            if now >= due:
                due = web._advance_due(due, now, interval)
                fires.append(now)
    span = fires[-1] - fires[0]
    return (len(fires) - 1) / span if span else 0.0


def test_ir_cadence_reaches_its_requested_rate():
    """IR is configured for 15 Hz and was measured on the live server at
    11.75 Hz. The fix has to actually deliver ~15."""
    hz = _achieved_hz(web.IR_INTERVAL, web.POINT_INTERVAL, 20.0, rephase=False)
    assert 14.5 <= hz <= 15.5, f"IR cadence achieved {hz:.2f} Hz, want ~15"


def test_sensor_cadence_reaches_its_requested_rate():
    """The `sensor` message carries the IMU/orientation readouts, so its
    cadence is what the owner read as 'IMU frames have dropped'."""
    hz = _achieved_hz(web.SENSOR_INTERVAL, web.POINT_INTERVAL, 20.0, rephase=False)
    assert 14.5 <= hz <= 15.5, f"sensor cadence achieved {hz:.2f} Hz, want ~15"


def test_rephasing_is_what_lost_the_rate():
    """A positive control for the two tests above.

    Without this, they would still pass if the interval constants were simply
    made smaller, or if the tick were faster -- the test would be measuring the
    driver rather than the fix. Running the OLD shape through the SAME driver
    shows the shortfall is the re-phasing and nothing else.
    """
    old = _achieved_hz(web.IR_INTERVAL, web.POINT_INTERVAL, 20.0, rephase=True)
    new = _achieved_hz(web.IR_INTERVAL, web.POINT_INTERVAL, 20.0, rephase=False)
    assert old < 12.5, f"expected the old shape to under-deliver, got {old:.2f} Hz"
    assert new > old + 2.0


def test_advance_due_resyncs_after_a_long_stall_instead_of_bursting():
    """A loop that stalled for a second must not then fire 15 catch-up IR
    frames back to back -- that burst is precisely the queue growth the
    backpressure half of this fix exists to prevent."""
    due = web._advance_due(0.0, 5.0, web.IR_INTERVAL)
    assert due == pytest.approx(5.0 + web.IR_INTERVAL)


def test_advance_due_does_not_drift_on_a_normal_tick():
    """The ordinary case: advance by exactly one interval, keeping the
    schedule's absolute phase."""
    due = 10.0
    nxt = web._advance_due(due, 10.001, web.IR_INTERVAL)
    assert nxt == pytest.approx(10.0 + web.IR_INTERVAL)
