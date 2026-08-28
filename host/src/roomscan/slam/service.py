"""Container-side SLAM compute service. Owns one Mapper (via SlamWorker for its
mesh throttle + publish logic) on a chosen Open3D device (CUDA:0 in the GPU
container, or a plain subprocess on this host -- E2, #223). One client at a
time; localhost-only; no auth. See
docs/superpowers/specs/deprecated/2026-07-13-slam-gpu-container-service-design.md
and docs/superpowers/specs/2026-08-27-rust-migration-feasibility.md (E2).

Wire v2 per-connection protocol (see wire.py):
  client: hello(cfg, w, h, mesh_format, wall_mode)  -> service: hello(device)
  client: frame ...                                  -> service: pose (every frame)
                                                        service: mesh (only while
                                                        the client holds the credit)
  client: mesh_ack                                   -> credit returned; a pending
                                                        (latest) mesh flushes at once
  client: mesh_request(req_id)                       -> service: mesh_full(req_id,
                                                        arrays) -- the newest full-res
                                                        mesh, for Save (#224)
  client: set_view(wall_mode)                        -> packet mode preps with it
A v1 client (no hello, cfg on every frame) is still served: the worker is
built from the first frame's cfg, and meshes flow with an implicit credit that
is never consumed, i.e. the old unbounded behaviour.
"""
from __future__ import annotations

import argparse
import json
import socket
import sys
import time

import numpy as np

from . import wire
from .config import SlamConfig
from .worker import SlamWorker


def _effective_kwargs(server_kwargs: dict, client_cfg_json: str | None) -> dict:
    """Merge the service's own mapper kwargs with the client's requested cfg,
    client overriding server on overlap. Robust to a missing/None/"{}" cfg
    (older clients, or a client with nothing to override) -- server kwargs
    pass through unchanged. `device` is never part of either dict here: the
    client strips it before sending, and callers keep it as a separate,
    server-owned argument to SlamWorker(...)."""
    client_cfg = json.loads(client_cfg_json or "{}")
    return {**server_kwargs, **client_cfg}


def _imu_raw_from_msg(msg: dict):
    if "imu_raw_gyro_dps" not in msg or "imu_raw_accel_g" not in msg:
        return None
    from ..protocol import ImuRawBatch
    gyro_dps = np.asarray(msg["imu_raw_gyro_dps"], np.float64)
    accel_g = np.asarray(msg["imu_raw_accel_g"], np.float64)
    gravity_g = np.asarray(msg.get("imu_raw_gravity_g", np.zeros((0, 3), dtype=np.float64)), np.float64)
    gbias_dps = np.asarray(msg.get("imu_raw_gbias_dps", np.zeros((0, 3), dtype=np.float64)), np.float64)
    timestamp_ticks = np.asarray(msg.get("imu_raw_timestamp_ticks", np.zeros(0, dtype=np.uint32)), np.uint32)
    return ImuRawBatch(
        gyro_dps=gyro_dps,
        gyro_cnt=np.zeros(len(gyro_dps), dtype=np.uint8),
        accel_g=accel_g,
        accel_cnt=np.zeros(len(accel_g), dtype=np.uint8),
        gravity_g=gravity_g,
        gravity_cnt=np.zeros(len(gravity_g), dtype=np.uint8),
        gbias_dps=gbias_dps,
        gbias_cnt=np.zeros(len(gbias_dps), dtype=np.uint8),
        timestamp_ticks=timestamp_ticks,
        timestamp_cnt=np.zeros(len(timestamp_ticks), dtype=np.uint8),
        n_records=int(msg.get("imu_raw_n_records", 0)),
        tick_us=float(msg.get("imu_raw_tick_us", 21.7)),
    )


class _MeshCredit:
    """One mesh in flight, latest-wins (E2 fix b). `offer()` replaces any
    pending mesh; `flush()` sends the pending one iff the client holds no
    unacknowledged mesh. Counters are what the service reports about itself:
    `sent`, `replaced` (a pending mesh superseded before it could go), and
    `acks`."""

    def __init__(self, gated: bool = True):
        self.gated = gated          # False = v1 client, no acks ever come
        self.credit = True
        self.pending = None         # (mesh_seq, payload_message)
        self.sent = 0
        self.replaced = 0
        self.acks = 0

    def offer(self, msg: dict) -> None:
        if self.pending is not None:
            self.replaced += 1
        self.pending = msg

    def ack(self) -> None:
        self.acks += 1
        self.credit = True

    def flush(self, conn) -> bool:
        if self.pending is None or not self.credit:
            return False
        msg, self.pending = self.pending, None
        if self.gated:
            self.credit = False
        wire.send_message(conn, msg)
        self.sent += 1
        return True


class SlamService:
    def __init__(self, device="CUDA:0", mesh_every=5, **mapper_kwargs):
        self._device = device
        self._mesh_every = mesh_every
        self._mapper_kwargs = mapper_kwargs
        self._last_effective_kwargs = None
        self._last_hello = None            # the most recent client's hello (tests)
        self._last_worker = None           # the most recent worker built (tests)
        self.last_credit_stats = None       # (sent, replaced, acks) of the last client

    def _build_worker(self, w: int, h: int, cfg_json: str | None, mesh_every=None):
        eff_kwargs = _effective_kwargs(self._mapper_kwargs, cfg_json)
        self._last_effective_kwargs = eff_kwargs
        self._last_worker = SlamWorker(
            w, h, mesh_every=self._mesh_every if mesh_every is None else mesh_every,
            device=self._device, **eff_kwargs)
        return self._last_worker

    def serve_client(self, conn) -> None:
        try:
            conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            # 16 MB each way: a whole mesh can sit in the kernel so this
            # single-threaded loop is not blocked in sendall() while the
            # client drains it, and a burst of frames never fills the inbound
            # side. The client sets the same on its end.
            conn.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 16 << 20)
            conn.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 16 << 20)
        except OSError:
            pass
        worker = None
        meshprep = None                  # packet mode only
        mesh_format = wire.MESH_FORMAT_ARRAYS
        wall_mode = "split"
        last_mesh = object()             # sentinel; never equal to a real mesh
        mesh_seq = 0
        # Packet mode: the same byte-rate governor `SlamRunner.poll` applies
        # (BUG-060) -- a 36 MB map is prepped at most every ~3 s, not on every
        # extraction. Without it the in-service MeshPrep ran 170 preps per
        # 1800 frames instead of ~105 and its CPU/GIL load pushed the mapper
        # step's p95 from 53 to 77 ms (E2 run 1).
        mesh_rate = None
        next_prep_at = 0.0
        credit = _MeshCredit(gated=False)   # v1 default until a hello arrives
        try:
            while True:
                msg = wire.recv_message(conn)
                if msg is None:
                    break
                t_recv = time.monotonic()
                typ = wire.message_type(msg)

                if typ == wire.HELLO:
                    self._last_hello = msg
                    mesh_format = str(msg.get("mesh_format") or wire.MESH_FORMAT_ARRAYS)
                    wall_mode = str(msg.get("wall_mode") or "split")
                    credit = _MeshCredit(gated=True)
                    if worker is None:
                        worker = self._build_worker(int(msg["width"]), int(msg["height"]),
                                                    msg.get("cfg"), msg.get("mesh_every"))
                    if mesh_format == wire.MESH_FORMAT_PACKET and meshprep is None:
                        meshprep, mesh_rate = self._make_meshprep()
                        meshprep.start()
                    wire.send_message(conn, wire.hello_reply(
                        device=worker.device, icp_device=getattr(worker, "icp_device", None),
                        mesh_format=mesh_format))
                    continue

                if typ == wire.MESH_ACK:
                    credit.ack()
                    credit.flush(conn)
                    continue

                if typ == wire.MESH_REQUEST:
                    # One-shot, not credit-gated: the newest mesh the worker
                    # has published (full-res -- MeshPrep never touches it),
                    # or an `empty` reply when there is no map yet. Sent from
                    # this loop, so a Save costs the frame loop one 36 MB
                    # sendall (tens of ms) -- a deliberate, rare trade.
                    latest = worker.latest() if worker is not None else None
                    full = latest[0] if latest is not None else None
                    wire.send_message(conn, wire.mesh_full_message(
                        int(msg.get("req_id", 0)), mesh_seq, full))
                    continue

                if typ == wire.SET_VIEW:
                    wall_mode = str(msg.get("wall_mode") or wall_mode)
                    continue

                # A frame (v2 `type: frame`, or v1 untagged with inline cfg).
                depth = np.asarray(msg["depth"], np.float32)
                if worker is None:
                    h, w = depth.shape
                    worker = self._build_worker(w, h, msg.get("cfg"))
                quat = np.asarray(msg["quat"], np.float32)
                refl = msg.get("reflectance")
                conf = msg.get("confidence")
                # Task 8 step 2 + #181: forward rate, phase offset, timing, and
                # imu_raw through to the SAME SlamWorker.submit()/run_once()
                # path the local backend uses.
                worker.submit(depth, quat, msg.get("pressure"),
                              reflectance=None if refl is None else np.asarray(refl, np.float32),
                              confidence=None if conf is None else np.asarray(conf, np.float32),
                              imu_rate_hz=msg.get("imu_rate_hz"),
                              imu_raw=_imu_raw_from_msg(msg),
                              quat_offset_us=msg.get("quat_offset_us"),
                              t_device_us=msg.get("t_device_us"))
                worker.run_once()
                mesh, _traj, step = worker.latest()

                # Pose first: small, sent immediately, never delayed behind a
                # mesh. Carries every FrameStep scalar (E2 fix c) plus the IPC
                # timestamps a client can turn into a pose-age measurement
                # (same CLOCK_MONOTONIC on one host; meaningless across hosts).
                wire.send_message(conn, wire.pose_message(
                    msg["fid"], step.pose, step.fitness, step.rmse,
                    step.tracking_lost, step.slam_ms, worker.tracking_lost_count,
                    device=worker.device, step=step,
                    t_client_submit=msg.get("t_client_submit"),
                    t_client_send=msg.get("t_client_send"),
                    t_service_recv=t_recv, t_service_done=time.monotonic(),
                    mesh_extract_ms=worker.mesh_extract_ms,
                    frames_processed=worker.frames_processed))

                # Mesh: only when the worker published a new one (identity
                # check), and only as far as the credit allows.
                if mesh is not None and mesh is not last_mesh:
                    if meshprep is None:
                        last_mesh = mesh
                        mesh_seq += 1
                        credit.offer(wire.mesh_message(mesh_seq, mesh))
                    elif time.monotonic() >= next_prep_at:
                        last_mesh = mesh
                        mesh_seq += 1
                        meshprep.submit(mesh, mesh_seq=mesh_seq, glow_origin=None,
                                        wall_mode=wall_mode)
                if meshprep is not None:
                    pkt = meshprep.latest()
                    if pkt is not None:
                        nbytes = len(pkt.packed)
                        next_prep_at = time.monotonic() + (nbytes / mesh_rate if mesh_rate else 0.0)
                        credit.offer(wire.mesh_packet_message(
                            pkt.mesh_seq, pkt.packed,
                            source_vertex_count=pkt.source_vertex_count,
                            sent_vertex_count=len(pkt.non_wall_verts) + len(pkt.wall_verts),
                            prep_ms=meshprep.prep_ms, pack_ms=meshprep.pack_ms,
                            extract_ms=worker.mesh_extract_ms))
                credit.flush(conn)
        finally:
            self.last_credit_stats = (credit.sent, credit.replaced, credit.acks)
            if meshprep is not None:
                meshprep.stop()

    def _make_meshprep(self):
        """`(MeshPrep, bytes_per_s)`: the live packer built the way
        `SlamRunner._construct` builds its own, plus its byte-rate governor --
        so "packet" mode ships exactly the bytes, at exactly the cadence, the
        web loop would otherwise have produced in-process."""
        from ..server.session import pack_mesh
        from .meshprep import MeshPrep
        cfg = SlamConfig.load()
        return (MeshPrep(vertex_budget=cfg.live_vertex_budget,
                         fps_budget_ms=cfg.fps_budget_ms, packer=pack_mesh),
                float(cfg.live_mesh_bytes_per_s))


def serve(host="0.0.0.0", port=5555, device="CUDA:0", *, _sock=None, **mapper_kwargs) -> None:
    srv = SlamService(device=device, **mapper_kwargs)
    if _sock is not None:
        lsock = _sock
    else:
        lsock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        lsock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        lsock.bind((host, port))
        lsock.listen(1)
    print(f"[slam-service] listening on {host}:{port} device={device}", flush=True)
    while True:
        try:
            conn, addr = lsock.accept()
        except OSError:
            break
        print(f"[slam-service] client {addr} connected", flush=True)
        try:
            srv.serve_client(conn)
        except Exception as e:
            print(f"[slam-service] client dropped: {e}", flush=True)
        finally:
            conn.close()
            print(f"[slam-service] client disconnected; credit stats "
                  f"sent/replaced/acks={srv.last_credit_stats}; awaiting next", flush=True)


def main(argv=None) -> int:
    cfg = SlamConfig.load()
    ap = argparse.ArgumentParser(prog="roomscan-slam-service")
    ap.add_argument("--host", default="0.0.0.0")
    ap.add_argument("--port", type=int, default=5555)
    ap.add_argument("--device", default="CUDA:0")
    args = ap.parse_args(argv)

    if args.device.upper().startswith("CUDA"):
        import open3d as o3d
        if not o3d.core.cuda.is_available():
            print("[slam-service] CUDA requested but not available in this container",
                  file=sys.stderr)
            return 2

    # The service's own defaults are the full `[slam]` knob set (BUG-062's
    # single source); a v2 client's hello cfg overrides them key by key.
    kw = {k: v for k, v in cfg.mapper_kwargs().items() if k != "device"}
    serve(host=args.host, port=args.port, device=args.device, **kw)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
