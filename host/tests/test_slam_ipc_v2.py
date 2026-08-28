"""Wire v2 IPC behaviours (E2, #223): config sent once, mesh credits with
latest-wins, FrameStep timings forwarded, frame credit, packet mode.

The service-side tests drive `SlamService.serve_client` over a real socket with
a hand-rolled v2 client so each assertion is about bytes that crossed the
socket, not about a fake's bookkeeping. `_MeshCredit` has its own pure tests.
"""
from __future__ import annotations

import json
import socket
import threading
import time

import numpy as np
import pytest

from roomscan.slam import wire
from roomscan.slam.mapper import FrameStep
from roomscan.slam.service import SlamService, _MeshCredit

pytest.importorskip("open3d")

H, W = 42, 54


# ---- wire ----------------------------------------------------------------------

def test_step_fields_roundtrip_every_numeric_frame_step_field():
    step = FrameStep(pose=np.eye(4), fitness=0.9, rmse=0.01, tracking_lost=False,
                     slam_ms=12.5, blocks_used=10, blocks_capacity=100, blocks_configured=200,
                     raycast_ms=2.5, icp_ms=8.0, integrate_ms=1.2, inliers=1500,
                     source_points=2000, t_tracking_ms=0.3, t_icp_ms=8.0,
                     t_integrate_ms=1.2, t_raycast_ms=2.5, total_step_ms=12.5,
                     budget_breached=True, t_device_us=123456.0)
    msg = wire.pose_message(7, step.pose, step.fitness, step.rmse, step.tracking_lost,
                            step.slam_ms, 0, device="CPU:0", step=step,
                            t_client_submit=1.5, t_service_recv=None)
    out = wire.decode_message(wire.encode_message(msg))
    for name in wire.STEP_SCALAR_FIELDS:
        assert out[name] == getattr(step, name), name
    assert out["budget_breached"] is True
    assert out["t_client_submit"] == 1.5
    assert "t_service_recv" not in out          # None extras are dropped, not sent


def test_decoded_arrays_are_aligned_views_not_copies():
    depth = np.arange(H * W, dtype=np.float32).reshape(H, W)
    buf = wire.encode_message({"fid": 1, "depth": depth, "quat": np.ones(4, np.float32)})
    out = wire.decode_message(buf)
    assert out["depth"].flags.aligned and out["quat"].flags.aligned
    assert not out["depth"].flags.owndata            # a view over the buffer
    np.testing.assert_array_equal(out["depth"], depth)


def test_bytes_blob_travels_as_uint8_array_and_empty_arrays_survive():
    blob = b"\x01\x02\x03" * 1000
    buf = wire.encode_message({"packet": blob, "empty": np.zeros((0, 3), np.float32), "n": 3})
    out = wire.decode_message(buf)
    assert bytes(out["packet"]) == blob
    assert out["empty"].shape == (0, 3)
    assert out["n"] == 3


def test_hello_message_carries_cfg_once_and_frames_carry_none():
    from roomscan.slam.remote import RemoteSlamWorker
    rw = RemoteSlamWorker(W, H, addr="127.0.0.1:1", fov_h=70.0, device="CUDA:0")
    depth = np.full((H, W), 500.0, np.float32)
    rw.submit(depth, np.array([1, 0, 0, 0], np.float32), None)
    assert "cfg" not in rw._in_slot                   # config-once: not per frame
    assert rw._in_slot["type"] == wire.FRAME
    hello = wire.hello_message(cfg=rw._cfg, width=W, height=H)
    assert json.loads(hello["cfg"]) == {"fov_h": 70.0}  # device stripped, cfg kept


# ---- _MeshCredit (pure) --------------------------------------------------------

class _FakeConn:
    def __init__(self):
        self.sent = []

    def sendall(self, b):
        self.sent.append(bytes(b))


def test_mesh_credit_sends_one_then_holds_until_ack_latest_wins():
    conn = _FakeConn()
    c = _MeshCredit(gated=True)
    c.offer({"type": wire.MESH, "mesh_seq": 1})
    assert c.flush(conn) is True and c.sent == 1
    c.offer({"type": wire.MESH, "mesh_seq": 2})
    assert c.flush(conn) is False                     # credit outstanding: held
    c.offer({"type": wire.MESH, "mesh_seq": 3})       # replaces 2, never sent
    assert c.replaced == 1
    assert c.flush(conn) is False
    c.ack()
    assert c.flush(conn) is True
    seqs = [wire.decode_message(b[4:])["mesh_seq"] for b in conn.sent]
    assert seqs == [1, 3]                             # 2 was superseded


def test_mesh_credit_ungated_v1_client_never_blocks():
    conn = _FakeConn()
    c = _MeshCredit(gated=False)
    for seq in (1, 2, 3):
        c.offer({"type": wire.MESH, "mesh_seq": seq})
        assert c.flush(conn) is True
    assert c.sent == 3 and c.replaced == 0


# ---- service over a real socket ------------------------------------------------

def _serve(srv):
    lsock = socket.socket()
    lsock.bind(("127.0.0.1", 0))
    lsock.listen(1)
    port = lsock.getsockname()[1]

    def accept_once():
        conn, _ = lsock.accept()
        try:
            srv.serve_client(conn)
        except OSError:
            pass
        finally:
            conn.close()
    th = threading.Thread(target=accept_once, daemon=True)
    th.start()
    return port, lsock, th


def _frame(fid, t_client_submit=None):
    msg = {"type": wire.FRAME, "fid": fid,
           "depth": np.full((H, W), 500.0, np.float32),
           "quat": np.array([1.0, 0.0, 0.0, 0.0], np.float32), "pressure": None}
    if t_client_submit is not None:
        msg["t_client_submit"] = t_client_submit
    return msg


def _recv_until(cli, typ):
    others = []
    while True:
        m = wire.recv_message(cli)
        assert m is not None
        if wire.message_type(m) == typ:
            return m, others
        others.append(m)


def test_service_builds_worker_from_hello_and_frames_need_no_cfg():
    srv = SlamService(device="CPU:0", fov_h=55.0, fov_v=42.0)
    port, lsock, th = _serve(srv)
    cli = socket.create_connection(("127.0.0.1", port))
    cli.settimeout(10)
    wire.send_message(cli, wire.hello_message(cfg={"fov_h": 70.0, "fov_v": 50.0},
                                              width=W, height=H))
    reply, _ = _recv_until(cli, wire.HELLO)
    assert reply["version"] == wire.WIRE_VERSION
    assert reply["device"] == "CPU:0"
    assert srv._last_effective_kwargs["fov_h"] == 70.0      # client cfg applied once
    wire.send_message(cli, _frame(1, t_client_submit=42.0))   # no cfg key at all
    pose, _ = _recv_until(cli, wire.POSE)
    assert pose["fid"] == 1
    # E2 fix (c): stage timings + counters + IPC stamps on every pose.
    for name in ("raycast_ms", "icp_ms", "integrate_ms", "total_step_ms",
                 "inliers", "source_points", "budget_breached"):
        assert name in pose, name
    assert pose["t_client_submit"] == 42.0
    assert pose["t_service_recv"] <= pose["t_service_done"]
    assert "mesh_extract_ms" in pose and "frames_processed" in pose
    cli.close()
    lsock.close()
    th.join(timeout=5)


def test_service_holds_second_mesh_until_ack_and_sends_latest():
    """Frame 1 extracts (first integration) -> mesh 1 goes out on the initial
    credit. Frames 2..11 extract twice more (mesh_every=5) while no ack has been
    sent -> NOTHING more arrives. One ack -> exactly one mesh, the LATEST."""
    srv = SlamService(device="CPU:0", fov_h=55.0, fov_v=42.0)
    port, lsock, th = _serve(srv)
    cli = socket.create_connection(("127.0.0.1", port))
    cli.settimeout(10)
    wire.send_message(cli, wire.hello_message(cfg={}, width=W, height=H, mesh_every=5))
    _recv_until(cli, wire.HELLO)
    meshes = []
    for fid in range(1, 12):
        wire.send_message(cli, _frame(fid))
        _pose, others = _recv_until(cli, wire.POSE)
        meshes += [m for m in others if wire.message_type(m) == wire.MESH]
    # The first mesh may trail its pose; give it a moment, then assert count.
    cli.settimeout(0.5)
    try:
        while True:
            m = wire.recv_message(cli)
            if m is None:
                break
            if wire.message_type(m) == wire.MESH:
                meshes.append(m)
    except (socket.timeout, TimeoutError):
        pass
    assert [m["mesh_seq"] for m in meshes] == [1]            # credit held after 1
    cli.settimeout(10)
    wire.send_message(cli, wire.mesh_ack_message(1))
    m, _ = _recv_until(cli, wire.MESH)
    assert m["mesh_seq"] == 3                                  # latest-wins: 2 skipped
    assert m["format"] == wire.MESH_FORMAT_ARRAYS and "mesh_v" in m
    cli.close()
    lsock.close()
    th.join(timeout=5)
    sent, replaced, acks = srv.last_credit_stats
    assert (sent, replaced, acks) == (2, 1, 1)


def test_service_packet_mode_ships_packed_bytes_with_prep_counters():
    srv = SlamService(device="CPU:0", fov_h=55.0, fov_v=42.0)
    port, lsock, th = _serve(srv)
    cli = socket.create_connection(("127.0.0.1", port))
    cli.settimeout(20)
    wire.send_message(cli, wire.hello_message(cfg={}, width=W, height=H,
                                              mesh_format=wire.MESH_FORMAT_PACKET))
    reply, _ = _recv_until(cli, wire.HELLO)
    assert reply["mesh_format"] == wire.MESH_FORMAT_PACKET
    got = None
    for fid in range(1, 40):
        wire.send_message(cli, _frame(fid))
        _pose, others = _recv_until(cli, wire.POSE)
        ms = [m for m in others if wire.message_type(m) == wire.MESH]
        if ms:
            got = ms[0]
            break
        time.sleep(0.02)
    assert got is not None, "packet-mode mesh never arrived"
    assert got["format"] == wire.MESH_FORMAT_PACKET
    assert got["packet"].dtype == np.uint8 and len(got["packet"]) > 0
    assert "mesh_v" not in got
    for name in ("source_vertex_count", "sent_vertex_count", "prep_ms", "pack_ms", "t_pub"):
        assert name in got, name
    cli.close()
    lsock.close()
    th.join(timeout=5)


# ---- client: frame credit + ack-on-consume -----------------------------------------

def _fake_v2_service(pose_delay_s=0.0, mesh_on_fid=None):
    """A v2 service double: answers hello, one pose per frame (optionally
    delayed), and a tiny mesh once on `mesh_on_fid`. Records what it received."""
    lsock = socket.socket()
    lsock.bind(("127.0.0.1", 0))
    lsock.listen(1)
    port = lsock.getsockname()[1]
    seen = {"frames": [], "acks": [], "hello": None}

    def loop():
        conn, _ = lsock.accept()
        try:
            while True:
                req = wire.recv_message(conn)
                if req is None:
                    break
                typ = wire.message_type(req)
                if typ == wire.HELLO:
                    seen["hello"] = req
                    wire.send_message(conn, wire.hello_reply(device="CPU:0"))
                elif typ == wire.MESH_ACK:
                    seen["acks"].append(req["mesh_seq"])
                else:
                    seen["frames"].append(req["fid"])
                    if pose_delay_s:
                        time.sleep(pose_delay_s)
                    wire.send_message(conn, wire.pose_message(
                        req["fid"], np.eye(4, dtype=np.float32), 0.9, 0.01, False, 1.0, 0,
                        device="CPU:0", t_client_submit=req.get("t_client_submit"),
                        t_client_send=req.get("t_client_send"),
                        t_service_recv=time.monotonic(), t_service_done=time.monotonic()))
                    if mesh_on_fid is not None and req["fid"] == mesh_on_fid:
                        v = np.array([[0, 0, 0], [1, 0, 0], [0, 1, 0]], np.float32)
                        t = np.array([[0, 1, 2]], np.int32)
                        wire.send_message(conn, {"type": wire.MESH, "mesh_seq": 1,
                                                 "format": wire.MESH_FORMAT_ARRAYS,
                                                 "mesh_v": v, "mesh_t": t})
        except OSError:
            pass
        finally:
            conn.close()
    th = threading.Thread(target=loop, daemon=True)
    th.start()
    return port, lsock, th, seen


def test_client_sends_hello_once_and_holds_frames_to_one_in_flight():
    from roomscan.slam.remote import RemoteSlamWorker
    port, lsock, th, seen = _fake_v2_service(pose_delay_s=0.15)
    rw = RemoteSlamWorker(W, H, addr=f"127.0.0.1:{port}", fov_h=61.0)
    assert rw.connect()
    rw.start()
    depth = np.full((H, W), 500.0, np.float32)
    quat = np.array([1.0, 0.0, 0.0, 0.0], np.float32)
    t0 = time.monotonic()
    while time.monotonic() - t0 < 0.25:      # ~25 submits while the first pose is pending
        rw.submit(depth, quat, None)
        time.sleep(0.01)
    time.sleep(0.4)
    rw.stop()
    lsock.close()
    th.join(timeout=2)
    assert seen["hello"] is not None and json.loads(seen["hello"]["cfg"]) == {"fov_h": 61.0}
    assert rw.service_version == wire.WIRE_VERSION and rw.device == "CPU:0"
    # Credit: at most ONE frame between send and pose; the rest overwrote the
    # slot rather than queueing in the socket.
    assert len(seen["frames"]) <= 3
    assert rw.frames_overwritten >= 15
    assert rw.frames_processed == len(seen["frames"])
    assert rw.last_pose_timing["t_client_send"] >= rw.last_pose_timing["t_client_submit"]


def test_client_acks_mesh_only_when_consumer_takes_it():
    from roomscan.slam.remote import RemoteSlamWorker
    port, lsock, th, seen = _fake_v2_service(mesh_on_fid=1)
    rw = RemoteSlamWorker(W, H, addr=f"127.0.0.1:{port}")
    assert rw.connect()
    rw.start()
    depth = np.full((H, W), 500.0, np.float32)
    quat = np.array([1.0, 0.0, 0.0, 0.0], np.float32)
    rw.submit(depth, quat, None)
    t0 = time.monotonic()
    while rw.meshes_received == 0 and time.monotonic() - t0 < 3:
        time.sleep(0.01)
    assert rw.meshes_received == 1
    time.sleep(0.05)
    assert seen["acks"] == []                    # received, but nobody consumed it
    mesh, _traj, _step = rw.latest()
    assert mesh is not None
    t0 = time.monotonic()
    while not seen["acks"] and time.monotonic() - t0 < 2:
        time.sleep(0.01)
    assert seen["acks"] == [1]                   # consumed -> credit returned once
    rw.latest()
    time.sleep(0.05)
    assert seen["acks"] == [1]                   # re-reading does not re-ack
    rw.stop()
    lsock.close()
    th.join(timeout=2)
