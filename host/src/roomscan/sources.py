"""Byte sources and the frame pump. All I/O lives here — decoder/deproject stay pure."""
from __future__ import annotations

from pathlib import Path
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

    def close(self) -> None:
        self._ser.close()


def pump(source, decoder: StreamDecoder, record_path=None) -> Iterator[Frame]:
    rec = open(record_path, "wb") if record_path else None
    try:
        while True:
            data = source.read()
            if not data:
                if isinstance(source, FileSource):
                    return          # EOF on replay; live sources just idle
                continue
            if rec:
                rec.write(data)
            yield from decoder.feed(data)
    finally:
        if rec:
            rec.close()
        source.close()
