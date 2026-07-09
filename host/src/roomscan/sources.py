"""Byte sources and the frame pump. All I/O lives here — decoder/deproject stay pure."""
from __future__ import annotations

import threading
from typing import Iterator, Optional

from .decoder import StreamDecoder
from .protocol import Frame

CDC_VID, CDC_PID = 0xCAFE, 0x4001   # milestone 1b TinyUSB descriptors (docs/protocol.md)
ST_VID = 0x0483                     # ST-Link VCOM fallback (milestone 1a)


class FileSource:
    def __init__(self, path, chunk: int = 4096):
        self._f = open(path, "rb")
        self._chunk = chunk

    def read(self) -> bytes:
        return self._f.read(self._chunk)

    def write(self, data: bytes) -> None:
        raise NotImplementedError("FileSource is replay-only; there is no device to write to")

    def close(self) -> None:
        self._f.close()


class SerialSource:
    def __init__(self, port: Optional[str] = None, baud: int = 921600, timeout: float = 0.05):
        import serial  # deferred: tests must not need pyserial hardware access
        if port is None:
            port = self.find_port()
        self._ser = serial.Serial(port, baud, timeout=timeout)
        self.port = port

    @staticmethod
    def find_port() -> str:
        from serial.tools import list_ports
        ports = list(list_ports.comports())
        for p in ports:
            if p.vid == CDC_VID and p.pid == CDC_PID:
                return p.device
        for p in ports:
            if p.vid == ST_VID:
                return p.device
        raise RuntimeError(f"no scanner serial port found among {[p.device for p in ports]}")

    def read(self) -> bytes:
        return self._ser.read(4096)

    def write(self, data: bytes) -> None:
        """Write bytes to the serial port (delegates to pyserial).

        CAUTION: this blocks until the OS accepts the write and, for anything
        beyond a small command frame, potentially until the device drains its
        RX buffer per its pacing policy (see docs/protocol.md and
        host/tests/bench_commands.py). NEVER call this from the thread that is
        draining reads (the loop calling `.read()` / `pump()`): starving that
        loop for >100 ms causes the device to abort an in-flight send by
        design (proven on hardware in Phase 3 Task 2). Call it from a
        different thread than the reader — see CommandClient, which is built
        around exactly this split.
        """
        self._ser.write(data)

    def close(self) -> None:
        self._ser.close()


class Recorder:
    """Thread-safe mid-stream recording handle for the GUI's Record button.

    The reader thread calls `write()` on every raw chunk unconditionally; it
    is a no-op while not recording. The UI thread calls `start()`/`stop()` to
    toggle recording at any point. All state transitions are guarded by a
    single lock so a `stop()` racing a `write()` can never write to (or
    close) a half-closed file.

    Design choice: `start()` while already recording does NOT raise — it
    closes the current file and switches to the new path. This is the
    friendlier behavior for a UI Record button (e.g. double-click, or
    starting a new take without an explicit Stop first) than forcing the
    caller to stop() before every start().
    """

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._f = None
        self._path = None

    @property
    def active(self) -> bool:
        with self._lock:
            return self._f is not None

    @property
    def path(self):
        with self._lock:
            return self._path

    def start(self, path) -> None:
        with self._lock:
            if self._f is not None:
                self._f.close()
            self._f = open(path, "wb")
            self._path = path

    def stop(self) -> None:
        with self._lock:
            if self._f is not None:
                self._f.close()
                self._f = None
                self._path = None

    def write(self, data: bytes) -> None:
        with self._lock:
            if self._f is not None:
                self._f.write(data)
                self._f.flush()   # keep on-disk bytes current while still "active" (readable mid-recording)

    def close(self) -> None:
        """Final teardown; safe to call multiple times (alias for stop())."""
        self.stop()


def pump(source, decoder: StreamDecoder, record_path=None, recorder: Optional[Recorder] = None) -> Iterator[Frame]:
    """Read raw chunks from `source`, tee them to recording sink(s), decode, yield frames.

    `record_path`, if given, opens a file at pump start and writes every raw
    chunk to it for the whole run (legacy all-or-nothing recording); pump
    owns that file's lifecycle and closes it in `finally`, exactly as before.

    `recorder`, if given, is a `Recorder` the caller starts/stops from
    another thread (e.g. a GUI Record button) to capture only part of the
    stream. Every raw chunk is teed to `recorder.write()`, which is a no-op
    while the recorder is inactive. Pump does NOT own `recorder`'s lifecycle:
    it never calls start/stop/close on it, so the recorder is left exactly
    as the caller last set it (active or not) when pump exits — the caller
    may keep using it across multiple pump() calls.

    Both may be passed at once (record_path captures everything, recorder
    captures a caller-controlled sub-range); normal panel usage passes only
    `recorder`.
    """
    rec = None
    try:
        if record_path:
            rec = open(record_path, "wb")
        while True:
            data = source.read()
            if not data:
                if isinstance(source, FileSource):
                    return          # EOF on replay; live sources just idle
                continue
            if rec:
                rec.write(data)
            if recorder is not None:
                recorder.write(data)
            yield from decoder.feed(data)
    finally:
        if rec:
            rec.close()
        source.close()
