import socket, threading, time
import numpy as np
import pytest
from roomscan.slam.remote import RemoteSlamWorker
from roomscan.slam.service import SlamService

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
    return port, lsock, th


def test_remote_worker_publishes_results():
    port, lsock, th = _serve_on_ephemeral()
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


def test_start_is_idempotent_and_stop_start_cycle_is_clean():
    # Finding 2: a second start() while already running must be a clean no-op
    # (not spawn a second send/recv thread pair on top of the first).
    port, lsock, th = _serve_on_ephemeral()
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
