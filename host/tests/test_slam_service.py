import socket, threading
import numpy as np
import pytest
from roomscan.slam import wire
from roomscan.slam.service import SlamService

pytest.importorskip("open3d")

H, W = 42, 54


def _synthetic_frame(fid):
    depth = np.full((H, W), 500.0, np.float32)     # 0.5 m plane, mm
    quat = np.array([1.0, 0.0, 0.0, 0.0], np.float32)
    return {"fid": fid, "depth": depth, "quat": quat, "pressure": None}


def test_service_returns_stepresult_per_frame():
    srv = SlamService(device="CPU:0", fov_h=55.0, fov_v=42.0)
    lsock = socket.socket(); lsock.bind(("127.0.0.1", 0)); lsock.listen(1)
    port = lsock.getsockname()[1]

    def accept_once():
        conn, _ = lsock.accept()
        srv.serve_client(conn)
        conn.close()
    th = threading.Thread(target=accept_once, daemon=True); th.start()

    cli = socket.create_connection(("127.0.0.1", port))
    results = []
    for fid in range(4):
        wire.send_message(cli, _synthetic_frame(fid))
        results.append(wire.recv_message(cli))
    cli.close(); lsock.close(); th.join(timeout=2)

    assert [r["fid"] for r in results] == [0, 1, 2, 3]
    for r in results:
        assert r["pose"].shape == (4, 4)
        assert isinstance(r["tracking_lost"], bool)
        assert r["traj"].shape[1:] == (4, 4)
    # mesh sent at least once, and mesh_seq is monotone non-decreasing
    seqs = [r["mesh_seq"] for r in results]
    assert seqs == sorted(seqs)
    assert any("mesh_v" in r for r in results)
