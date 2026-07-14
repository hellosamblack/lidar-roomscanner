import socket, threading, time
import numpy as np
import pytest
from roomscan.slam.remote import RemoteSlamWorker
from roomscan.slam.service import SlamService
from roomscan.slam import wire

pytest.importorskip("open3d")

H, W = 42, 54


def _serve_on_ephemeral(device="CPU:0"):
    srv = SlamService(device=device, fov_h=55.0, fov_v=42.0)
    lsock = socket.socket(); lsock.bind(("127.0.0.1", 0)); lsock.listen(1)
    port = lsock.getsockname()[1]

    def loop():
        conn, _ = lsock.accept()
        try:
            srv.serve_client(conn)
        except OSError:
            pass
        finally:
            conn.close()
    th = threading.Thread(target=loop, daemon=True); th.start()
    return port, lsock, th, srv


def test_remote_worker_publishes_results():
    port, lsock, th, srv = _serve_on_ephemeral()
    rw = RemoteSlamWorker(W, H, addr=f"127.0.0.1:{port}", fov_h=55.0, fov_v=42.0)
    assert rw.connect() is True
    rw.start()
    depth = np.full((H, W), 500.0, np.float32)
    quat = np.array([1.0, 0.0, 0.0, 0.0], np.float32)
    got = None
    for _ in range(200):                       # poll up to ~2 s for the first result
        rw.submit(depth, quat, None)
        time.sleep(0.01)
        got = rw.latest()
        if got is not None:
            break
    rw.stop(); lsock.close(); th.join(timeout=2)
    assert got is not None
    mesh, traj, step = got
    assert step.pose.shape == (4, 4)
    assert isinstance(traj, list)
    assert rw.tracking_lost_count >= 0


def test_connect_returns_false_when_no_server():
    rw = RemoteSlamWorker(W, H, addr="127.0.0.1:1", connect_timeout=0.3)
    assert rw.connect() is False
    assert rw.latest() is None       # never raises


def test_connect_returns_false_for_malformed_addr_missing_port():
    # Finding 2: a malformed remote_addr (no ":port") must not raise out of
    # __init__ or connect() -- address parsing is deferred to connect(), and
    # a parse failure there must behave like any other unreachable service.
    rw = RemoteSlamWorker(W, H, addr="127.0.0.1")   # no ":port" at all
    assert rw.connect() is False
    assert rw.latest() is None       # never raises


def test_remote_worker_forwards_client_mapper_cfg_to_service():
    # Finding 1: the client's mapper kwargs (fov_h/fov_v here) must reach the
    # server's lazily-created SlamWorker, overriding the service's own
    # defaults -- not be silently dropped.
    port, lsock, th, srv = _serve_on_ephemeral()   # server defaults: fov_h=55.0, fov_v=42.0
    rw = RemoteSlamWorker(W, H, addr=f"127.0.0.1:{port}", fov_h=70.0, fov_v=50.0)
    assert rw.connect() is True
    rw.start()
    depth = np.full((H, W), 500.0, np.float32)
    quat = np.array([1.0, 0.0, 0.0, 0.0], np.float32)
    got = None
    for _ in range(200):                       # poll up to ~2 s for the first result
        rw.submit(depth, quat, None)
        time.sleep(0.01)
        got = rw.latest()
        if got is not None:
            break
    rw.stop(); lsock.close(); th.join(timeout=2)
    assert got is not None

    # The server must have built its lazily-created worker with the client's
    # fov (70.0/50.0), not its own defaults (55.0/42.0).
    assert srv._last_effective_kwargs is not None
    assert srv._last_effective_kwargs["fov_h"] == 70.0
    assert srv._last_effective_kwargs["fov_v"] == 50.0


def test_start_is_idempotent_and_stop_start_cycle_is_clean():
    # Finding 2: a second start() while already running must be a clean no-op
    # (not spawn a second send/recv thread pair on top of the first).
    port, lsock, th, srv = _serve_on_ephemeral()
    rw = RemoteSlamWorker(W, H, addr=f"127.0.0.1:{port}", fov_h=55.0, fov_v=42.0)
    assert rw.connect() is True
    rw.start()
    rw.start()  # second call: must no-op, not leak a duplicate thread pair
    assert len(rw._threads) == 2

    rw.stop()
    assert rw._threads == []
    assert rw._sock is None

    # Finding 1: stop() must null self._sock only after the worker threads have
    # joined. Exercise a start/stop cycle again and confirm it completes cleanly
    # within the bounded join timeout (no hang, no thread pair leaked).
    assert rw.connect() is True
    rw.start()
    assert len(rw._threads) == 2
    rw.stop()
    assert rw._threads == []
    assert rw._sock is None

    lsock.close(); th.join(timeout=2)


def test_remote_worker_accumulates_trajectory_from_pose_deltas():
    port, lsock, th, srv = _serve_on_ephemeral()
    rw = RemoteSlamWorker(W, H, addr=f"127.0.0.1:{port}", fov_h=55.0, fov_v=42.0)
    assert rw.connect() is True
    rw.start()
    depth = np.full((H, W), 500.0, np.float32)
    quat = np.array([1.0, 0.0, 0.0, 0.0], np.float32)
    last_len = 0
    for _ in range(400):                       # drive enough frames to grow the traj
        rw.submit(depth, quat, None)
        time.sleep(0.01)
        got = rw.latest()
        if got is not None:
            _mesh, traj, _step = got
            last_len = len(traj)
            if last_len >= 3:                  # trajectory grew from >=3 pose deltas
                break
    rw.stop(); lsock.close(); th.join(timeout=2)
    assert last_len >= 3


def _serve_legacy_untagged_on_ephemeral():
    """A fake service that speaks the PRE-Component-B combined format: one
    message per frame with NO 'type' tag, the mesh arrays inline (only when a
    new mesh appears), and a full 'traj'. Emulates a GPU container image built
    before the pose/mesh split -- the exact skew that silently blanked the live
    view (client got pose/traj but never a mesh). The new client must stay
    backward-compatible and still recover the inline mesh."""
    import open3d as o3d
    v = np.array([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]], np.float32)
    t = np.array([[0, 1, 2]], np.int32)
    m = o3d.t.geometry.TriangleMesh()
    m.vertex["positions"] = o3d.core.Tensor(v)
    m.triangle["indices"] = o3d.core.Tensor(t)

    lsock = socket.socket(); lsock.bind(("127.0.0.1", 0)); lsock.listen(1)
    port = lsock.getsockname()[1]

    def loop():
        conn, _ = lsock.accept()
        fid = 0
        try:
            while True:
                req = wire.recv_message(conn)
                if req is None:
                    break
                out = {                                   # legacy combined message, no "type"
                    "fid": fid,
                    "pose": np.eye(4, dtype=np.float32),
                    "fitness": 0.9, "rmse": 0.01,
                    "tracking_lost": False, "slam_ms": 5.0,
                    "traj": np.zeros((0, 4, 4), np.float32),
                    "tracking_lost_count": 0,
                    "mesh_seq": 1 if fid >= 2 else 0,
                }
                if fid == 2:                              # a new mesh appears once, inline
                    out.update(wire.mesh_to_arrays(m))
                wire.send_message(conn, out)
                fid += 1
        except OSError:
            pass
        finally:
            conn.close()
    th = threading.Thread(target=loop, daemon=True); th.start()
    return port, lsock, th


def test_remote_worker_backward_compatible_with_legacy_untagged_service():
    import roomscan.slam.wire as _wire  # noqa: F401  (ensure wire imported)
    port, lsock, th = _serve_legacy_untagged_on_ephemeral()
    rw = RemoteSlamWorker(W, H, addr=f"127.0.0.1:{port}", fov_h=55.0, fov_v=42.0)
    assert rw.connect() is True
    rw.start()
    depth = np.full((H, W), 500.0, np.float32)
    quat = np.array([1.0, 0.0, 0.0, 0.0], np.float32)
    got_mesh = False
    for _ in range(200):
        rw.submit(depth, quat, None)
        time.sleep(0.01)
        got = rw.latest()
        if got is not None and got[0] is not None:   # a non-None mesh recovered
            got_mesh = True
            break
    rw.stop(); lsock.close(); th.join(timeout=2)
    assert got_mesh, ("client must recover the inline mesh from a legacy "
                      "untagged service (container built before the split)")
