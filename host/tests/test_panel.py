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
import numpy as np

from roomscan.panel import (
    _Pacer,
    _fill_panel_fields,
    _ir_freeze_range,
    _orbit_eye,
    _rot_xy,
    _run_reader,
)
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
    ns = argparse.Namespace(point_size=None, ir_colormap=None, ir_freeze_range=None,
                            panel_width=None, near_mode=None, near_cutoff_m=None,
                            near_emphasis=None)
    for k, v in over.items():
        setattr(ns, k, v)
    return ns


def test_fill_panel_fields_uses_builtin_defaults_when_no_config(tmp_path, monkeypatch):
    monkeypatch.setenv("APPDATA", str(tmp_path))   # empty dir -> no roomscan.toml
    args = _bare_args()
    _fill_panel_fields(args)
    assert args.point_size == 5.0
    assert args.ir_colormap == "gray"
    assert args.ir_freeze_range is False
    assert args.panel_width == 340
    assert args.near_mode == "window"
    assert args.near_cutoff_m == 1.5
    assert args.near_emphasis == 0.5


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


# --- _rot_xy (cloud rotation) ------------------------------------------------

def test_rot_xy_identity_and_wrap():
    pts = np.array([[1.0, 0.0, 5.0], [0.0, 2.0, 7.0]])
    assert np.allclose(_rot_xy(pts, 0), pts)
    assert np.allclose(_rot_xy(pts, 4), pts)      # 4x90 = full turn


def test_rot_xy_90_ccw_leaves_z():
    pts = np.array([[1.0, 0.0, 5.0]])
    out = _rot_xy(pts, 1)                          # (x,y)->(-y,x): (1,0)->(0,1)
    assert np.allclose(out, [[0.0, 1.0, 5.0]])
    assert out[0, 2] == 5.0                        # depth untouched


def test_rot_xy_180():
    pts = np.array([[1.0, 2.0, 9.0]])
    assert np.allclose(_rot_xy(pts, 2), [[-1.0, -2.0, 9.0]])


def test_rot_xy_empty():
    assert _rot_xy(np.zeros((0, 3)), 1).shape == (0, 3)


# --- _orbit_eye (roll-locked turntable camera) -------------------------------

def test_orbit_eye_radius_and_axis():
    target = np.array([1.0, 2.0, 3.0])
    # az=0, el=0 -> eye is +radius along z from target
    eye = _orbit_eye(target, 0.0, 0.0, 5.0)
    assert np.allclose(eye, [1.0, 2.0, 8.0])
    # distance from target is always the radius, at any angle
    for az, el in [(0.5, 0.3), (2.0, -0.4), (-1.0, 1.4)]:
        e = _orbit_eye(target, az, el, 4.2)
        assert abs(np.linalg.norm(e - target) - 4.2) < 1e-9


def test_orbit_eye_elevation_sign():
    target = np.zeros(3)
    up = _orbit_eye(target, 0.0, 0.5, 1.0)      # positive elevation -> eye above
    down = _orbit_eye(target, 0.0, -0.5, 1.0)
    assert up[1] > 0 and down[1] < 0


# --- _ir_freeze_range (IR pane freeze state machine) -------------------------

def test_ir_range_auto_when_not_frozen():
    vmin, vmax, frozen = _ir_freeze_range(False, None, (10.0, 20.0))
    assert (vmin, vmax) == (10.0, 20.0)
    assert frozen is None                      # nothing captured while auto


def test_ir_range_lazy_captures_when_frozen_but_none():
    # freeze set from config / before any frame: frozen starts None -> capture this frame
    vmin, vmax, frozen = _ir_freeze_range(True, None, (10.0, 20.0))
    assert (vmin, vmax) == (10.0, 20.0)
    assert frozen == (10.0, 20.0)


def test_ir_range_reuses_frozen_ignoring_new_auto():
    vmin, vmax, frozen = _ir_freeze_range(True, (10.0, 20.0), (99.0, 200.0))
    assert (vmin, vmax) == (10.0, 20.0)        # frozen wins over the new auto-range
    assert frozen == (10.0, 20.0)


def test_ir_range_frozen_from_config_persists_across_frames():
    # simulate the config-freeze path across two frames with drifting auto-ranges
    _, _, frozen = _ir_freeze_range(True, None, (10.0, 20.0))       # frame 1 captures
    vmin, vmax, frozen = _ir_freeze_range(True, frozen, (50.0, 90.0))  # frame 2 drifts
    assert (vmin, vmax) == (10.0, 20.0)        # still the captured range, not frame 2's
    assert frozen == (10.0, 20.0)


# --- run(): graceful failure when the scanner port is busy -------------------

def test_run_reports_busy_port_cleanly(monkeypatch, capsys, tmp_path):
    monkeypatch.setenv("APPDATA", str(tmp_path))
    import roomscan.panel as panel

    class _BusyPort:
        def __init__(self, *a, **k):
            raise Exception("could not open port 'COM99': PermissionError(13, 'Access is denied.')")

    monkeypatch.setattr(panel, "SerialSource", _BusyPort)
    args = panel._resolve([])          # no --replay -> live path -> SerialSource
    args.port = "COM99"
    rc = panel.run(args)               # pytest stdin is not a tty -> no interactive prompt
    assert rc == 1                     # clean exit, not an uncaught traceback
    err = capsys.readouterr().err
    assert "port is in use" in err
    assert "Close any other roomscan" in err   # busy-port hint shown (non-interactive)


def test_run_reports_missing_port_cleanly(monkeypatch, capsys, tmp_path):
    monkeypatch.setenv("APPDATA", str(tmp_path))
    import roomscan.panel as panel

    class _NoPort:
        def __init__(self, *a, **k):
            raise FileNotFoundError(2, "The system cannot find the file specified.")

    monkeypatch.setattr(panel, "SerialSource", _NoPort)
    args = panel._resolve([])
    args.port = "COM99"
    rc = panel.run(args)
    assert rc == 1
    err = capsys.readouterr().err
    assert "scanner not found" in err
    assert "RESET" in err              # missing-port hint, not the busy one


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
