"""Unit tests for the non-GUI seams of the control panel.

The Open3D gui shell itself is supervised-run verified (like the classic
viewer); here we cover the two pieces that are testable headless: the
config-backed panel-field fill, and the reader-thread routing/pacing logic
(EVENT -> log bus, ACK -> CommandClient, DATA -> render slot).
"""
import argparse
import queue
import struct
import time

from roomscan.config import ViewerConfig
from roomscan.decoder import StreamDecoder
from roomscan.logbus import LogBus
from roomscan.panel import _Pacer, _fill_panel_fields, _run_reader
from roomscan.pipeline import TransformStage
from roomscan.protocol import (
    CommandCode,
    FrameHeader,
    FrameType,
    ResultCode,
    StreamId,
    pack_frame,
)
from roomscan.sources import Recorder
from roomscan.viewer import Stats


# --- _fill_panel_fields ------------------------------------------------------

def _bare_args(**over):
    ns = argparse.Namespace(point_size=None, ir_colormap=None,
                            ir_freeze_range=None, panel_width=None)
    for k, v in over.items():
        setattr(ns, k, v)
    return ns


def test_fill_panel_fields_uses_builtin_defaults_when_no_config(tmp_path, monkeypatch):
    monkeypatch.setenv("APPDATA", str(tmp_path))   # empty dir -> no roomscan.toml
    args = _bare_args()
    _fill_panel_fields(args)
    assert args.point_size == 3.0
    assert args.ir_colormap == "gray"
    assert args.ir_freeze_range is False
    assert args.panel_width == 340


def test_fill_panel_fields_pulls_from_config_file(tmp_path, monkeypatch):
    monkeypatch.setenv("APPDATA", str(tmp_path))
    ViewerConfig(ir_colormap="turbo", ir_freeze_range=True,
                 point_size=5.0, panel_width=400).save()
    args = _bare_args()
    _fill_panel_fields(args)
    assert args.ir_colormap == "turbo"
    assert args.ir_freeze_range is True
    assert args.point_size == 5.0
    assert args.panel_width == 400


def test_fill_panel_fields_leaves_already_set_values(tmp_path, monkeypatch):
    monkeypatch.setenv("APPDATA", str(tmp_path))
    ViewerConfig(ir_colormap="turbo").save()
    args = _bare_args(ir_colormap="gray")   # already resolved -> must win
    _fill_panel_fields(args)
    assert args.ir_colormap == "gray"


# --- _run_reader routing -----------------------------------------------------

class _OneShotSource:
    def __init__(self, data: bytes):
        self._data = data
        self._sent = False

    def read(self):
        if self._sent:
            raise StopIteration   # any exception ends the reader via the fault path
        self._sent = True
        return self._data

    def close(self):
        pass


def _depth_frame(seq):
    return pack_frame(FrameHeader(FrameType.DATA, StreamId.DEPTH_ZF32, 0, seq, 0, 2, 2, 16),
                      struct.pack("<4f", 1.0, 2.0, 3.0, 4.0))


def _run(source, client=None, pacer=None):
    stats = Stats()
    slot: queue.Queue = queue.Queue(maxsize=1)
    bus = LogBus()
    sub = bus.subscribe()
    fault: dict = {}
    _run_reader(source, StreamDecoder(), TransformStage(), stats, slot, fault, bus,
                client, Recorder(), pacer or _Pacer(0.0), lambda: False)
    return stats, slot, bus, sub, fault


def test_reader_routes_depth_data_to_slot():
    stats, slot, bus, sub, fault = _run(_OneShotSource(_depth_frame(5)))
    assert stats.frames == 1 and stats._last_seq == 5
    header, outputs = slot.get_nowait()
    assert "depth" in outputs and outputs["depth"].shape == (2, 2)


def test_reader_routes_event_to_logbus_not_slot():
    payload = struct.pack("<II", 2, 3) + b"trigger retries exhausted"
    frame = pack_frame(FrameHeader(FrameType.EVENT, 0, 0, 1, 0, 0, 0, len(payload)), payload)
    stats, slot, bus, sub, fault = _run(_OneShotSource(frame))
    msgs = bus.drain(sub)
    assert any("code=2" in m and "trigger retries exhausted" in m for m in msgs)
    assert stats.frames == 0 and slot.empty()


class _StubOfferClient:
    def __init__(self):
        self.offered = []

    def offer(self, frame):
        self.offered.append(frame)
        return True


def test_reader_routes_ack_to_client_not_slot():
    payload = struct.pack("<III", CommandCode.PING, ResultCode.OK, 1)
    frame = pack_frame(FrameHeader(FrameType.ACK, 0, 0, 42, 0, 0, 0, len(payload)), payload)
    client = _StubOfferClient()
    stats, slot, bus, sub, fault = _run(_OneShotSource(frame), client=client)
    assert len(client.offered) == 1
    assert client.offered[0].header.frame_type == FrameType.ACK
    assert slot.empty()


def test_reader_surfaces_fault():
    class Exploding:
        def read(self):
            raise OSError("device gone")

        def close(self):
            pass

    _, _, _, _, fault = _run(Exploding())
    assert isinstance(fault["error"], OSError)


def test_reader_paces_frames_with_interval():
    frames = b"".join(_depth_frame(i) for i in range(1, 4))
    t0 = time.monotonic()
    stats, slot, bus, sub, fault = _run(_OneShotSource(frames), pacer=_Pacer(0.05))
    elapsed = time.monotonic() - t0
    assert stats.frames == 3
    # frames 2 and 3 each wait ~50 ms; theoretical min 0.10 s, 0.08 keeps jitter margin
    assert elapsed >= 0.08
