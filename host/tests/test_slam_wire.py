import io, socket, threading
import numpy as np
import pytest
from roomscan.slam import wire


def test_encode_decode_roundtrip_mixed_scalars_and_arrays():
    depth = np.arange(54 * 42, dtype=np.float32).reshape(42, 54)
    fields = {
        "fid": 7,
        "pressure": None,
        "tracking_lost": True,
        "slam_ms": 12.5,
        "depth": depth,
        "quat": np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float32),
    }
    out = wire.decode_message(wire.encode_message(fields))
    assert out["fid"] == 7
    assert out["pressure"] is None
    assert out["tracking_lost"] is True
    assert out["slam_ms"] == pytest.approx(12.5)
    np.testing.assert_array_equal(out["depth"], depth)
    assert out["depth"].dtype == np.float32
    np.testing.assert_array_equal(out["quat"], fields["quat"])


def test_send_recv_over_socketpair():
    a, b = socket.socketpair()
    fields = {"fid": 3, "pose": np.eye(4, dtype=np.float32)}
    t = threading.Thread(target=wire.send_message, args=(a, fields))
    t.start()
    got = wire.recv_message(b)
    t.join()
    assert got["fid"] == 3
    np.testing.assert_array_equal(got["pose"], np.eye(4, dtype=np.float32))
    a.close(); b.close()


def test_recv_message_returns_none_on_eof():
    a, b = socket.socketpair()
    a.close()
    assert wire.recv_message(b) is None
    b.close()


def test_mesh_arrays_roundtrip():
    o3d = pytest.importorskip("open3d")
    v = np.array([[0, 0, 0], [1, 0, 0], [0, 1, 0]], np.float32)
    t = np.array([[0, 1, 2]], np.int32)
    src = o3d.t.geometry.TriangleMesh()
    src.vertex["positions"] = o3d.core.Tensor(v)
    src.triangle["indices"] = o3d.core.Tensor(t)
    d = wire.mesh_to_arrays(src)
    # survives an encode/decode round-trip too
    d = wire.decode_message(wire.encode_message(d))
    rebuilt = wire.arrays_to_mesh(d)
    np.testing.assert_array_equal(rebuilt.vertex["positions"].numpy(), v)
    np.testing.assert_array_equal(rebuilt.triangle["indices"].numpy(), t)


def test_pose_message_roundtrip():
    pose = np.eye(4, dtype=np.float32)
    msg = wire.pose_message(5, pose, fitness=0.8, rmse=0.02,
                            tracking_lost=False, slam_ms=9.1, tracking_lost_count=3)
    assert msg["type"] == wire.POSE
    out = wire.decode_message(wire.encode_message(msg))
    assert out["type"] == "pose"
    assert out["fid"] == 5
    assert out["fitness"] == pytest.approx(0.8)
    assert out["tracking_lost"] is False
    assert out["tracking_lost_count"] == 3
    np.testing.assert_array_equal(out["pose"], pose)
    assert "mesh_v" not in out            # a pose message carries no mesh


def test_mesh_message_roundtrip():
    o3d = pytest.importorskip("open3d")
    v = np.array([[0, 0, 0], [1, 0, 0], [0, 1, 0]], np.float32)
    t = np.array([[0, 1, 2]], np.int32)
    src = o3d.t.geometry.TriangleMesh()
    src.vertex["positions"] = o3d.core.Tensor(v)
    src.triangle["indices"] = o3d.core.Tensor(t)
    msg = wire.mesh_message(4, src)
    assert msg["type"] == wire.MESH and msg["mesh_seq"] == 4
    out = wire.decode_message(wire.encode_message(msg))
    assert out["type"] == "mesh" and out["mesh_seq"] == 4
    rebuilt = wire.arrays_to_mesh(out)
    np.testing.assert_array_equal(rebuilt.vertex["positions"].numpy(), v)
