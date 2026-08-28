"""SlamRunner with `[slam] backend = "remote"` in packet mode (#224): the web
loop only forwards MESH bytes, Save fetches the full mesh over the wire, a
dead service tears the worker down for a rebuild, and the spawned service
subprocess is supervised across the runner's lifecycle.

Fakes stand in for the socket-level worker (its own behaviour is pinned in
test_slam_ipc_v2.py); the supervisor test spawns the REAL service module on
CPU:0 -- a subprocess fake would omit exactly what matters (listen, exit
code, stdout pump)."""
from __future__ import annotations

import socket
import time

import numpy as np
import pytest

from roomscan import web
from roomscan.slam.mapper import FrameStep
from roomscan.slam.supervisor import SlamServiceSupervisor, is_loopback
from roomscan.web import LogBus

pytest.importorskip("open3d")


class _FakePacketWorker:
    """What `RemoteSlamWorker(mesh_format="packet")` looks like to the runner."""
    backend = "remote"
    mesh_format = "packet"
    device = "CUDA:0"
    frames_submitted = frames_processed = frames_overwritten = 0
    mesh_extract_ms = 3.0
    prep_ms, pack_ms, payload_bytes = 12.0, 2.0, 36

    def __init__(self):
        self.alive = True
        self.has_mesh = False
        self.stopped = False
        self.wall_modes = []
        self.fetched = 0
        self._packets = []
        self._traj = [np.eye(4) for _ in range(5)]
        self.tracking_lost_count = 1

    def start(self): pass
    def stop(self): self.stopped = True
    def submit(self, *a, **k): pass
    def offer_packet(self, seq):
        self.has_mesh = True
        self._packets.append((memoryview(b"\x05" + bytes(35)),
                              {"mesh_seq": seq, "source_vertex_count": 77 * seq,
                               "sent_vertex_count": 70, "prep_ms": 12.0, "pack_ms": 2.0}))
    def latest(self):
        step = FrameStep(pose=np.eye(4), fitness=0.5, rmse=0.02, tracking_lost=False, slam_ms=5.0)
        return (None, self._traj, step)
    def latest_packet(self):
        return self._packets.pop(0) if self._packets else None
    def set_wall_mode(self, m): self.wall_modes.append(m)
    def fetch_mesh(self, timeout=30.0):
        import open3d as o3d
        self.fetched += 1
        if not self.has_mesh:                 # the real service answers `empty`
            return None
        tm = o3d.t.geometry.TriangleMesh()
        tm.vertex.positions = o3d.core.Tensor([[0, 0, 0], [1, 0, 0], [0, 1, 0]], o3d.core.float32)
        tm.triangle.indices = o3d.core.Tensor([[0, 1, 2]], o3d.core.int32)
        return tm


def _packet_runner():
    r = web.SlamRunner(bus=LogBus())
    w = _FakePacketWorker()
    with r._lock:
        r._active = True
        r._worker, r._meshprep, r._packet_mode = w, None, True
        r._backend, r._device, r._wh = "remote", "CUDA:0", (54, 42)
    return r, w


def test_packet_mode_poll_forwards_service_bytes_and_counters():
    r, w = _packet_runner()
    msg, mesh = r.poll("split")
    assert msg is not None and mesh is None            # pose ticks, no mesh yet
    assert msg["backend"] == "remote" and msg["mesh_seq"] == 0
    w.offer_packet(3)
    msg, mesh = r.poll("solid")
    assert mesh is not None and len(mesh) == 36 and mesh[0] == 5
    assert msg["mesh_seq"] == 3 and msg["mesh_verts"] == 231
    assert msg["mesh_prep_ms"] == 12.0 and msg["mesh_pack_ms"] == 2.0
    assert msg["mesh_payload_bytes"] == 36
    assert w.wall_modes == ["split", "solid"]           # UI toggle reaches the service
    msg, mesh = r.poll("solid")
    assert mesh is None and msg["mesh_seq"] == 3         # nothing new: no re-send


def test_packet_mode_has_map_and_save_fetch_the_full_mesh_over_the_wire(tmp_path):
    r, w = _packet_runner()
    assert r.has_map is False
    with pytest.raises(ValueError):
        r.save(tmp_path / "a.ply", tmp_path / "a.tum")
    w.offer_packet(1)
    r.poll("split")
    assert r.has_map is True
    n = r.save(tmp_path / "m.ply", tmp_path / "m.tum")
    assert n == 3 and w.fetched == 2          # one empty probe earlier, one real
    assert (tmp_path / "m.ply").stat().st_size > 0
    assert len((tmp_path / "m.tum").read_text().splitlines()) == 5


def test_dead_remote_worker_is_torn_down_so_the_next_frame_rebuilds():
    r, w = _packet_runner()
    w.alive = False
    assert r.poll("split") == (None, None)
    assert w.stopped is True
    with r._lock:
        assert r._worker is None and r._packet_mode is False and r._active is True


def test_construct_builds_no_meshprep_in_packet_mode(monkeypatch):
    import roomscan.slam.backend as backend
    import roomscan.slam.meshprep as meshprep
    made = {}
    monkeypatch.setattr(backend, "make_slam_worker", lambda w, h, **k: made.setdefault("w", _FakePacketWorker()))
    monkeypatch.setattr(meshprep, "MeshPrep", lambda *a, **k: pytest.fail("MeshPrep built in packet mode"))
    r = web.SlamRunner(bus=LogBus())
    monkeypatch.setattr(r, "_ensure_service", lambda cfg, dev: made.setdefault("ensured", (cfg.backend,)))
    worker, mp, dev, backend_name = r._construct(54, 42)
    assert worker is made["w"] and mp is None and backend_name == "remote"
    assert made["ensured"] == ("local",)                # default config: no spawn attempted


def test_close_stops_the_supervisor():
    r, _w = _packet_runner()

    class _Sup:
        addr = "127.0.0.1:5555"
        stopped = False
        def stop(self): self.stopped = True
    sup = _Sup()
    r._supervisor = sup
    assert r.service_supervisor is sup
    r.close()
    assert sup.stopped is True and r.service_supervisor is None


# ---- supervisor: the real service module, CPU:0 -----------------------------------

def test_is_loopback():
    assert is_loopback("127.0.0.1:5555") and is_loopback("localhost:1") and is_loopback(":5555")
    assert not is_loopback("192.168.1.10:5555")


def test_supervisor_spawns_real_service_restarts_after_death_and_stops():
    s = socket.socket(); s.bind(("127.0.0.1", 0)); port = s.getsockname()[1]; s.close()
    lines = []
    sup = SlamServiceSupervisor(addr=f"127.0.0.1:{port}", device="CPU:0", publish=lines.append)
    assert sup.alive is False and sup.pid is None
    assert sup.ensure(timeout=120.0) is True
    assert sup.alive is True and sup.starts == 1
    pid1 = sup.pid
    socket.create_connection(("127.0.0.1", port), timeout=1.0).close()
    # A second ensure() while alive is a no-op.
    assert sup.ensure() is True and sup.starts == 1
    # Kill it the way a CUDA OOM would: the process dies, ensure() restarts it.
    import os, signal
    os.kill(pid1, signal.SIGKILL)
    t0 = time.monotonic()
    while sup.alive and time.monotonic() - t0 < 10:
        time.sleep(0.05)
    assert sup.alive is False
    assert sup.ensure(timeout=120.0) is True
    assert sup.starts == 2 and sup.pid != pid1 and sup.alive
    sup.stop()
    assert sup.alive is False
    assert any("spawned pid" in ln for ln in lines)
    assert any("listening on" in ln for ln in lines), lines   # the child's stdout was pumped
    assert any("exited" in ln for ln in lines)


def test_supervisor_uses_an_already_listening_port_without_spawning():
    lsock = socket.socket(); lsock.bind(("127.0.0.1", 0)); lsock.listen(1)
    port = lsock.getsockname()[1]
    lines = []
    sup = SlamServiceSupervisor(addr=f"127.0.0.1:{port}", device="CPU:0", publish=lines.append)
    assert sup.ensure() is True and sup.starts == 0 and sup.pid is None
    assert any("already in use" in ln for ln in lines)
    lsock.close()


_CAPTURE = "/home/sam/git/personal/lidar-roomscanner/captures/coffeeRoomCircuitNoMnt.bin"


@pytest.mark.skipif(not __import__("os").path.isfile(_CAPTURE), reason="capture not on this box")
def test_end_to_end_remote_packet_runner_saves_a_real_map(tmp_path, monkeypatch):
    """The productised path, no fakes: `[slam] backend = "remote"` config ->
    SlamRunner spawns the REAL service (CPU:0) via the supervisor, feeds real
    capture frames, forwards packet-mode MESH bytes from `poll()`, and
    `save()` fetches the full mesh over the wire and writes a PLY."""
    import roomscan.slam.config as slam_config
    from roomscan.slam.cli import _load_frames
    s = socket.socket(); s.bind(("127.0.0.1", 0)); port = s.getsockname()[1]; s.close()
    toml = tmp_path / "roomscan.toml"
    toml.write_text(f'[slam]\nbackend = "remote"\nremote_addr = "127.0.0.1:{port}"\n'
                    f'remote_mesh_format = "packet"\nremote_spawn = true\n'
                    f'remote_device = "CPU:0"\n')
    monkeypatch.setattr(slam_config, "config_path", lambda: toml)
    frames, _w, _h = _load_frames(_CAPTURE)
    frames = frames[:90]
    lines = []
    bus = LogBus()
    r = web.SlamRunner(bus=bus)
    bus.publish = lambda text: lines.append(text)          # capture the log lines
    r._bus = bus
    r.set_active(True)
    try:
        t0 = time.monotonic()
        mesh_bytes = None
        msgs = 0
        while time.monotonic() - t0 < 180:
            for depth, refl, conf, quat, pressure, _t in frames:
                r.submit(depth, quat, pressure, reflectance=refl, confidence=conf)
                msg, mb = r.poll("split")
                if msg is not None:
                    msgs += 1
                    assert msg["backend"] == "remote"
                if mb is not None:
                    mesh_bytes = mb
                time.sleep(0.01)
            if mesh_bytes is not None and msgs > 30:
                break
        assert mesh_bytes is not None, lines
        assert mesh_bytes[0] == web.TAG_MESH                # a finished MESH packet
        assert r.has_map is True
        assert r.service_supervisor is not None and r.service_supervisor.alive
        with r._lock:
            assert r._meshprep is None and r._packet_mode is True
        n = r.save(tmp_path / "m.ply", tmp_path / "m.tum")
        assert n > 0 and (tmp_path / "m.ply").stat().st_size > 0
        assert any("mesh_format=packet" in ln for ln in lines), lines
    finally:
        r.close()
    assert r.service_supervisor is None
    assert any("[slam-service]" in ln and "exited" in ln for ln in lines), lines


def test_supervisor_never_spawns_for_a_non_loopback_address():
    s = socket.socket(); s.bind(("127.0.0.1", 0)); port = s.getsockname()[1]; s.close()
    sup = SlamServiceSupervisor(addr=f"192.0.2.1:{port}", device="CPU:0")
    assert sup.ensure() is False and sup.starts == 0
