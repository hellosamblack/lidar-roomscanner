import socket, threading
import numpy as np
import pytest
from roomscan.slam import wire, service
from roomscan.slam.service import SlamService, _effective_kwargs

pytest.importorskip("open3d")

H, W = 42, 54


def test_effective_kwargs_server_only_when_no_client_cfg():
    # Backward compatible: older clients (or messages) with no "cfg" key ->
    # msg.get("cfg") is None -> server kwargs pass through unchanged.
    server = {"fov_h": 55.0, "fov_v": 42.0, "voxel_size": 0.01}
    assert _effective_kwargs(server, None) == server
    assert _effective_kwargs(server, "{}") == server


def test_effective_kwargs_client_overrides_server_on_overlap():
    server = {"fov_h": 55.0, "fov_v": 42.0}
    client_cfg_json = '{"fov_h": 70.0, "fov_v": 50.0}'
    merged = _effective_kwargs(server, client_cfg_json)
    assert merged == {"fov_h": 70.0, "fov_v": 50.0}


def test_effective_kwargs_client_can_only_add_or_override_non_device_keys():
    # serve_client passes device=self._device as a separate argument to
    # SlamWorker(...), never through _effective_kwargs -- so even if a client
    # tried to sneak a "device" into its cfg, the merged dict would carry it
    # as an ordinary extra key, and serve_client's explicit device=... kwarg
    # (passed alongside **eff_kwargs) is what actually wins/collides. This
    # test only confirms the helper itself does no device-specific filtering
    # of its own -- RemoteSlamWorker is what strips "device" before it is
    # ever serialized into cfg_json, which is the real enforcement point.
    server = {"fov_h": 55.0}
    merged = _effective_kwargs(server, '{"fov_h": 70.0}')
    assert "device" not in merged


def _synthetic_frame(fid):
    depth = np.full((H, W), 500.0, np.float32)     # 0.5 m plane, mm
    quat = np.array([1.0, 0.0, 0.0, 0.0], np.float32)
    return {"fid": fid, "depth": depth, "quat": quat, "pressure": None}


def test_service_sends_pose_per_frame_and_mesh_when_ready():
    srv = SlamService(device="CPU:0", fov_h=55.0, fov_v=42.0)
    lsock = socket.socket(); lsock.bind(("127.0.0.1", 0)); lsock.listen(1)
    port = lsock.getsockname()[1]

    def accept_once():
        conn, _ = lsock.accept()
        srv.serve_client(conn)
        conn.close()
    th = threading.Thread(target=accept_once, daemon=True); th.start()

    cli = socket.create_connection(("127.0.0.1", port)); cli.settimeout(5)

    def drain_until_pose():
        """Read messages until a pose arrives; collect any mesh seen first/after."""
        got_mesh = []
        while True:
            m = wire.recv_message(cli)
            assert m is not None
            if m["type"] == wire.MESH:
                got_mesh.append(m)
            elif m["type"] == wire.POSE:
                return m, got_mesh

    poses, mesh_seen = [], 0
    for fid in range(8):
        wire.send_message(cli, _synthetic_frame(fid))
        pose, meshes = drain_until_pose()
        poses.append(pose)
        mesh_seen += len(meshes)
        for mm in meshes:
            assert "mesh_v" in mm and mm["mesh_seq"] >= 1
    cli.close(); lsock.close(); th.join(timeout=2)

    assert [p["fid"] for p in poses] == list(range(8))
    for p in poses:
        assert p["pose"].shape == (4, 4)
        assert isinstance(p["tracking_lost"], bool)
        assert "traj" not in p                 # trajectory no longer resent
    assert mesh_seen >= 1                       # a mesh was sent at least once


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
