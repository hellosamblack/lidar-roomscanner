"""`/api/camera/stream.mjpg` against a REAL uvicorn server and a real socket.

Why this file exists separately from `test_camera_routes.py`: the bound that
keeps a slow viewer from costing unbounded memory lives in uvicorn's HTTP
`send`, which calls `await self.flow.drain()` once the asyncio transport's write
buffer passes its 64 KiB high-water mark. `TestClient` runs the ASGI app
in-process and drains every chunk instantly, so it cannot see that bound at all
-- and cannot even hold an infinite response open. #219's lesson, exactly: a
fake is defined by what it omits, and transport bugs live in the omissions, so
one live probe is budgeted against the real thing.

It also cannot live in `test_camera_routes.py`: `conftest.py` derives the
`slow`/`net` markers from a file's own source, so importing `uvicorn.Server`
there would push every cheap camera-route test out of the fast lane.

The far-side observable is deliberately NOT "the send returned" (a read-back
proves only that the setter ran, #106). It is how many parts the generator
managed to select while the peer was not reading: with the bound in force that
is a handful, and without it, every frame published.
"""
from __future__ import annotations

import asyncio
import contextlib
import socket
import threading
import time

import pytest

from roomscan import camera as cam
from roomscan.domain import HostMono
from roomscan.server import routes as routes_mod

from test_camera import make_jpeg


def _frame(seq: int, payload: bytes) -> cam.CameraFrame:
    return cam.CameraFrame(generation=1, seq=seq, jpeg=payload,
                           host_recv_mono=HostMono(float(seq)),
                           width=1280, height=720)


def _publisher_lifespan(coro_fn):
    """Run `coro_fn` as a background task for the server's lifetime.

    A lifespan, not the removed `@app.on_event("startup")` hook -- and the task
    is cancelled and joined on shutdown so a failed assertion cannot leave a
    publisher writing into a torn-down loop.
    """
    @contextlib.asynccontextmanager
    async def lifespan(_app):
        task = asyncio.create_task(coro_fn())
        try:
            yield
        finally:
            task.cancel()
            with contextlib.suppress(asyncio.CancelledError):
                await task

    return lifespan


def _serve(app, port: int):
    uvicorn = pytest.importorskip("uvicorn")
    server = uvicorn.Server(uvicorn.Config(app, host="127.0.0.1", port=port,
                                           log_level="critical"))
    thread = threading.Thread(target=server.run, daemon=True)
    thread.start()
    for _ in range(200):
        if getattr(server, "started", False):
            break
        time.sleep(0.05)
    assert getattr(server, "started", False), "uvicorn never bound"
    return server, thread


def _read_until(sock: socket.socket, marker: bytes, limit: int = 1 << 20) -> bytes:
    buf = b""
    while marker not in buf and len(buf) < limit:
        chunk = sock.recv(65536)
        if not chunk:
            break
        buf += chunk
    return buf


def test_a_real_browser_style_client_reads_parseable_parts_off_a_socket():
    """End to end over TCP: the emitted framing must survive a real HTTP hop
    and parse with the same parser that reads the Pi daemon's stream."""
    from starlette.applications import Starlette
    from starlette.responses import StreamingResponse
    from starlette.routing import Route

    slot = cam.LatestFrameSlot()
    jpeg = make_jpeg(1280, 720, filler=33)

    async def endpoint(_request):
        return StreamingResponse(routes_mod._camera_parts(slot),
                                 media_type=routes_mod.CAMERA_MEDIA_TYPE,
                                 headers=routes_mod.CAMERA_NO_STORE)

    async def publisher():
        for seq in range(1, 40):
            slot.publish(_frame(seq, jpeg))
            await asyncio.sleep(0.01)

    app = Starlette(routes=[Route("/api/camera/stream.mjpg", endpoint)],
                    lifespan=_publisher_lifespan(publisher))
    server, thread = _serve(app, 8781)
    try:
        sock = socket.create_connection(("127.0.0.1", 8781), timeout=10)
        sock.settimeout(10)
        sock.sendall(b"GET /api/camera/stream.mjpg HTTP/1.1\r\n"
                     b"Host: 127.0.0.1\r\n\r\n")
        head = _read_until(sock, b"\r\n\r\n")
        assert b"200 OK" in head
        assert b"multipart/x-mixed-replace" in head
        assert b"no-store" in head

        parser = cam.MultipartParser(routes_mod.CAMERA_BOUNDARY)
        parts = parser.feed(head.split(b"\r\n\r\n", 1)[1])
        deadline = time.monotonic() + 10
        while len(parts) < 3 and time.monotonic() < deadline:
            parts.extend(parser.feed(sock.recv(65536)))
        sock.close()
        assert len(parts) >= 3, "fewer than three parts arrived over a real socket"
        assert all(p.body == jpeg for p in parts), "bytes changed in transit"
        seqs = [int(p.headers["x-roomscan-camera-seq"]) for p in parts]
        assert seqs == sorted(seqs) and len(set(seqs)) == len(seqs), seqs
        # Latest-wins: a viewer keeping up still must not be handed a backlog,
        # so the sequence numbers it sees are allowed to skip.
        assert parser.buffered < 2 * len(jpeg)
    finally:
        server.should_exit = True
        thread.join(timeout=10)

    for _ in range(100):                     # the generator's `finally` runs off-loop
        if slot.subscribers == 0:
            break
        time.sleep(0.05)
    assert slot.subscribers == 0, "a disconnected client stayed counted as a viewer"


def test_a_stalled_socket_does_not_let_the_proxy_advance_without_bound():
    """The load-bearing claim, measured on the real transport.

    A client that opens the stream and then stops reading must not cause the
    server to keep selecting frames: uvicorn's HTTP send blocks past the write
    buffer's high-water mark, and the generator only picks a new frame after
    the previous send completed. So while 400 frames are published, the
    generator must advance a handful of times, not 400.

    The assertion is that COUNT and the retained-frame count -- not "the send
    succeeded", which is true in both the bounded and unbounded case.
    """
    from starlette.applications import Starlette
    from starlette.responses import StreamingResponse
    from starlette.routing import Route

    slot = cam.LatestFrameSlot()
    payload = make_jpeg(1280, 720) + bytes(200_000)   # ~200 KB, > the 64 KiB mark
    payload = payload[:-2] + b"\xff\xd9"
    emitted: list[int] = []

    async def counting():
        async for part in routes_mod._camera_parts(slot):
            emitted.append(len(part))
            yield part

    async def endpoint(_request):
        return StreamingResponse(counting(),
                                 media_type=routes_mod.CAMERA_MEDIA_TYPE)

    published = {"n": 0}

    async def publisher():
        for seq in range(1, 401):
            slot.publish(_frame(seq, payload))
            published["n"] += 1
            await asyncio.sleep(0.005)

    app = Starlette(routes=[Route("/api/camera/stream.mjpg", endpoint)],
                    lifespan=_publisher_lifespan(publisher))
    server, thread = _serve(app, 8782)
    try:
        sock = socket.create_connection(("127.0.0.1", 8782), timeout=10)
        sock.settimeout(10)
        # Deliberately NOT shrinking SO_RCVBUF: a tiny receive window collapses
        # loopback throughput badly enough that a client which IS reading looks
        # identical to one that is not, which would leave this test measuring
        # the window rather than the backpressure it is named for.
        sock.sendall(b"GET /api/camera/stream.mjpg HTTP/1.1\r\n"
                     b"Host: 127.0.0.1\r\n\r\n")
        _read_until(sock, b"\r\n\r\n")
        time.sleep(2.5)                      # read nothing at all
        stalled_emitted = len(emitted)
        stalled_published = published["n"]

        # Now drain, so the generator wakes and CHARGES what it jumped over.
        # `skipped` is only counted at selection time (see LatestFrameSlot):
        # a stalled viewer is blocked inside its send, not inside wait_next, so
        # asserting on it mid-stall would be asserting on a counter that
        # structurally cannot have moved yet.
        deadline = time.monotonic() + 5
        while len(emitted) <= stalled_emitted and time.monotonic() < deadline:
            try:
                sock.recv(1 << 20)
            except OSError:
                break
        resumed_emitted = len(emitted)
        skipped = slot.skipped
        sock.close()
    finally:
        server.should_exit = True
        thread.join(timeout=10)

    assert stalled_published >= 200, (
        f"the publisher only managed {stalled_published} frames; the test never "
        f"reached the interesting regime")
    assert stalled_emitted < 60, (
        f"the proxy selected {stalled_emitted} frames for a client that read "
        f"nothing after the response head -- the ASGI send is not bounding it")
    assert resumed_emitted > stalled_emitted, "the viewer never resumed"
    assert skipped > 100, (
        f"on resuming, the subscriber should have SKIPPED most of the "
        f"{stalled_published} published frames; it counted {skipped}")
    assert slot.latest()[1] is not None
