import socket, threading
import numpy as np
import pytest
from roomscan.slam import wire, service
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


def test_serve_survives_bad_client_and_keeps_serving():
    """A malformed frame (missing 'depth') raises inside serve_client; the
    real serve() accept loop must catch it, close that connection, and keep
    serving the next client rather than crashing the process."""
    lsock = socket.socket(); lsock.bind(("127.0.0.1", 0)); lsock.listen(1)
    port = lsock.getsockname()[1]

    th = threading.Thread(
        target=service.serve,
        kwargs=dict(device="CPU:0", fov_h=55.0, fov_v=42.0, _sock=lsock),
        daemon=True,
    )
    th.start()

    # First client: malformed frame missing the required "depth" key ->
    # KeyError inside serve_client. Connection should just end.
    bad_cli = socket.create_connection(("127.0.0.1", port))
    wire.send_message(bad_cli, {"fid": 0, "quat": np.array([1.0, 0.0, 0.0, 0.0], np.float32)})
    bad_cli.close()

    # Second client: valid synthetic frame. Server must still be alive.
    good_cli = socket.create_connection(("127.0.0.1", port))
    good_cli.settimeout(5)
    wire.send_message(good_cli, _synthetic_frame(0))
    result = wire.recv_message(good_cli)

    good_cli.close()
    lsock.close()
    th.join(timeout=2)

    assert result is not None
    assert result["fid"] == 0
    assert result["pose"].shape == (4, 4)
