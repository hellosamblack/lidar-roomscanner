"""Client-side drop-in for SlamWorker that ships frames to a SlamService over a
localhost socket and republishes the returned (mesh, trajectory, FrameStep).
Same interface as slam.worker.SlamWorker, so the web SlamRunner is agnostic to
which one it holds. On any socket failure the caller falls back to the local
worker.

Wire v2 (E2, #223): `start()` sends one `hello` carrying the mapper cfg and
frame geometry (cfg used to ride on EVERY frame), frames are tagged and cfg-
free, and meshes are credit-gated: the service holds at most one unacked mesh
in flight and this client returns the credit (`mesh_ack`) when the CONSUMER
first takes the mesh out of `latest()` (arrays) / `latest_packet()` (packet)
-- so a consumer that falls behind sees only the newest mesh, and neither the
socket nor this process ever holds a backlog of 36 MB copies.
"""
from __future__ import annotations

import logging
import socket
import threading
import time

import numpy as np

from . import wire
from .mapper import FrameStep

_IDLE_SLEEP_S = 0.005
_INFLIGHT_TIMEOUT_S = 1.0     # credit safety valve (v1 service never replies per frame)
_SOCK_BUF = 16 << 20           # let a whole mesh drain into the kernel, not block the peer

logger = logging.getLogger(__name__)


class RemoteSlamWorker:
    def __init__(self, width, height, addr="127.0.0.1:5555", mesh_every=5,
                 connect_timeout=1.0, mesh_format=wire.MESH_FORMAT_ARRAYS,
                 wall_mode="split", **mapper_kwargs):
        self._w, self._h = width, height
        self._addr = addr
        self._connect_timeout = connect_timeout
        self._mesh_every = int(mesh_every)
        self._mesh_format = str(mesh_format)
        self._wall_mode = str(wall_mode)
        self._sock = None
        self._fid = 0
        # The client's mapper params, forwarded ONCE in the hello so the remote
        # Mapper is built with the same effective config (except `device`,
        # which the service owns).
        self._cfg = {k: v for k, v in mapper_kwargs.items() if k != "device"}

        self._in_lock = threading.Lock()
        self._in_cv = threading.Condition(self._in_lock)
        self._in_slot = None
        # Frame credit (E2): at most `_max_inflight` frames between send and
        # its pose reply. Without it every submitted frame went straight into
        # the socket, and whenever the service paused (a 36 MB mesh send) the
        # queue in the kernel buffer aged every pose behind it by hundreds of
        # ms -- measured p95 submit->service-recv 170-207 ms. With it, a stall
        # turns into overwrites of the client-side slot (fresh frames), exactly
        # what the local SlamWorker's latest-wins slot does. `_inflight_since`
        # bounds the wait so a v1 service that never answers cannot deadlock it.
        self._max_inflight = 1
        self._inflight = 0
        self._inflight_since = 0.0
        self._out_lock = threading.Lock()
        self._out_slot = None
        self._send_lock = threading.Lock()
        self._tracking_lost_count = 0

        self._last_mesh_seq = -1
        self._last_mesh = None
        self._last_packet = None           # (bytes, meta) in packet mode
        self._unacked_seq = None           # mesh_seq the consumer has not yet taken
        self._acked_seq = -1
        # Accumulated from the per-frame poses (the service never resends a
        # full trajectory). Published to `latest()` AS-IS, not copied: the copy
        # was O(n) per pose under the GIL (1800 poses = 1800 x 128 B every
        # frame, one of E2's named residuals). The list is append-only and
        # only this recv thread appends, so a reader that only takes `len()`,
        # indexes, or iterates (build_slam_message, write_tum) sees a
        # consistent prefix -- never a torn entry.
        self._trajectory = []
        self._warned_legacy = False        # one-time warn on a pre-split (untagged) service

        # Plan item 2 (2026-08-02): the same submit/overwrite/processed shape
        # as SlamWorker, but measured on THIS side of the socket -- i.e. the
        # transport slot, not the service's own mapper.
        self._frames_submitted = 0
        self._frames_overwritten = 0
        self._frames_processed = 0
        self._device: str | None = None
        self._icp_device: str | None = None
        self._service_version: int | None = None
        # E2 instrumentation: latest pose's IPC timestamps, mesh counters, and
        # the prep-side numbers a packet-mode service reports.
        self.last_pose_timing: dict | None = None
        self._meshes_received = 0
        self._mesh_bytes_received = 0
        self._mesh_extract_ms = 0.0
        self._prep_ms = 0.0
        self._pack_ms = 0.0
        self._payload_bytes = 0
        self._last_source_verts = 0

        # Full-mesh request/reply (#224): `fetch_mesh()` blocks the caller
        # (Save runs off the event loop, via to_thread) until the recv loop
        # lands the `mesh_full` with a matching req_id.
        self._req_lock = threading.Lock()
        self._req_cv = threading.Condition(self._req_lock)
        self._req_id = 0
        self._req_reply: dict | None = None
        self._sent_wall_mode: str = self._wall_mode
        # True once the recv loop exits for any reason other than `stop()`:
        # the service died or dropped us. The runner polls this and rebuilds.
        self._dead = False

        self._threads = []
        self._stop_evt = threading.Event()

    @property
    def alive(self) -> bool:
        """False once the socket to the service has closed without `stop()`
        being called -- the service process died (CUDA OOM, crash) or dropped
        the connection. The runner treats that as "rebuild me"."""
        return self._sock is not None and not self._dead

    @property
    def has_mesh(self) -> bool:
        """A mesh has arrived on this connection (either format). In packet
        mode `latest()` never carries one, so this is what "the map exists"
        means for a remote worker."""
        return self._meshes_received > 0

    def set_wall_mode(self, wall_mode: str) -> None:
        """Tell a packet-mode service the wall_mode the UI wants; a no-op
        when unchanged (called every poll tick)."""
        wall_mode = str(wall_mode)
        if wall_mode == self._sent_wall_mode or self._sock is None:
            return
        self._sent_wall_mode = wall_mode
        try:
            with self._send_lock:
                wire.send_message(self._sock, wire.set_view_message(wall_mode))
        except (OSError, AttributeError):
            pass

    def fetch_mesh(self, timeout: float = 30.0):
        """Ask the service for its newest full-res mesh and wait for it (#224).
        Returns an Open3D tensor mesh, or None when the service has no map yet
        or does not answer within `timeout` (an older service ignores the
        request). Serialised: one request in flight at a time."""
        if self._sock is None:
            return None
        with self._req_cv:
            self._req_id += 1
            req_id = self._req_id
            self._req_reply = None
            try:
                with self._send_lock:
                    wire.send_message(self._sock, wire.mesh_request_message(req_id))
            except (OSError, AttributeError):
                return None
            deadline = time.monotonic() + float(timeout)
            while (self._req_reply is None or self._req_reply.get("req_id") != req_id):
                remaining = deadline - time.monotonic()
                if remaining <= 0 or self._dead:
                    return None
                self._req_cv.wait(remaining)
            reply, self._req_reply = self._req_reply, None
        if reply.get("empty") or "mesh_v" not in reply:
            return None
        return wire.arrays_to_mesh(reply)

    def connect(self) -> bool:
        try:
            host, _, port = self._addr.partition(":")
            port = int(port)
            self._sock = socket.create_connection(
                (host, port), timeout=self._connect_timeout)
            self._sock.settimeout(None)
            try:
                self._sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, _SOCK_BUF)
                self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, _SOCK_BUF)
            except OSError:
                pass
            return True
        except (OSError, ValueError):
            self._sock = None
            return False

    # ---- producer side ---------------------------------------------------------
    def submit(self, depth, quat, pressure, reflectance=None, confidence=None,
               imu_rate_hz=None, imu_raw=None, quat_offset_us=None,
               t_device_us=None) -> None:
        """`imu_rate_hz` (Task 8), `t_device_us`/`quat_offset_us` (#181) and
        `imu_raw` travel in the message when given, omitted otherwise (an older
        service never looks for the keys). `t_client_submit` stamps this
        process's monotonic clock; the service echoes it on the pose so
        `last_pose_timing` can measure submit -> pose-visible age."""
        with self._in_lock:
            self._fid += 1
            msg = {"type": wire.FRAME, "fid": self._fid,
                   "depth": np.asarray(depth, np.float32),
                   "quat": np.asarray(quat, np.float32),
                   "pressure": None if pressure is None else float(pressure),
                   "t_client_submit": time.monotonic()}
            if reflectance is not None:
                msg["reflectance"] = np.asarray(reflectance, np.float32)
            if confidence is not None:
                msg["confidence"] = np.asarray(confidence, np.float32)
            if imu_rate_hz is not None:
                msg["imu_rate_hz"] = float(imu_rate_hz)
            if quat_offset_us is not None:
                msg["quat_offset_us"] = float(quat_offset_us)
            if t_device_us is not None:
                msg["t_device_us"] = float(t_device_us)
            if imu_raw is not None:
                if hasattr(imu_raw, "gyro_dps") and hasattr(imu_raw, "accel_g"):
                    msg["imu_raw_gyro_dps"] = np.asarray(imu_raw.gyro_dps, np.float64)
                    msg["imu_raw_accel_g"] = np.asarray(imu_raw.accel_g, np.float64)
                    if getattr(imu_raw, "gravity_g", None) is not None:
                        msg["imu_raw_gravity_g"] = np.asarray(imu_raw.gravity_g, np.float64)
                    if getattr(imu_raw, "gbias_dps", None) is not None:
                        msg["imu_raw_gbias_dps"] = np.asarray(imu_raw.gbias_dps, np.float64)
                    if getattr(imu_raw, "timestamp_ticks", None) is not None:
                        msg["imu_raw_timestamp_ticks"] = np.asarray(imu_raw.timestamp_ticks, np.uint32)
                    if getattr(imu_raw, "tick_us", None) is not None:
                        msg["imu_raw_tick_us"] = float(imu_raw.tick_us)
                    if getattr(imu_raw, "n_records", None) is not None:
                        msg["imu_raw_n_records"] = int(imu_raw.n_records)
            self._frames_submitted += 1
            if self._in_slot is not None:
                self._frames_overwritten += 1
            self._in_slot = msg
            self._in_cv.notify()

    # ---- consumer side -----------------------------------------------------------
    def latest(self):
        """Latest `(mesh, trajectory, FrameStep)`. Taking a NEW mesh out of
        here returns the mesh credit to the service (arrays mode)."""
        with self._out_lock:
            res = self._out_slot
        if res is not None and self._mesh_format == wire.MESH_FORMAT_ARRAYS:
            self._maybe_ack()
        return res

    def latest_packet(self):
        """Packet mode: pop the newest finished MESH bytes as `(packed, meta)`
        or None; popping returns the mesh credit. `meta` carries `mesh_seq`,
        `source_vertex_count`, `sent_vertex_count`, `prep_ms`, `pack_ms`,
        `extract_ms`, `t_pub`, `t_recv`."""
        with self._out_lock:
            pkt, self._last_packet = self._last_packet, None
        if pkt is not None:
            self._maybe_ack()
        return pkt

    def _maybe_ack(self) -> None:
        seq = self._unacked_seq
        if seq is None or seq == self._acked_seq:
            return
        self._acked_seq = seq
        self._unacked_seq = None
        try:
            with self._send_lock:
                wire.send_message(self._sock, wire.mesh_ack_message(seq))
        except (OSError, AttributeError):
            pass

    @property
    def tracking_lost_count(self) -> int:
        return self._tracking_lost_count

    # ---- instrumentation --------------------------------------------------------
    @property
    def frames_submitted(self) -> int:
        return self._frames_submitted

    @property
    def frames_processed(self) -> int:
        """POSE responses received -- one per frame the SERVICE actually ran
        through its Mapper (transport-confirmed)."""
        return self._frames_processed

    @property
    def frames_overwritten(self) -> int:
        """Submits that replaced a still-pending, not-yet-SENT message in
        this client's own transport slot."""
        return self._frames_overwritten

    @property
    def device(self) -> str | None:
        """The remote SlamService's own resolved compute device (from its
        hello reply or the most recent POSE). None until the first arrives."""
        return self._device

    @property
    def icp_device(self) -> str | None:
        return self._icp_device

    @property
    def service_version(self) -> int | None:
        """Wire version the service announced in its hello; None for a v1
        service (no hello ever comes back)."""
        return self._service_version

    @property
    def backend(self) -> str:
        return "remote"

    @property
    def mesh_format(self) -> str:
        return self._mesh_format

    @property
    def mesh_extract_ms(self) -> float:
        return self._mesh_extract_ms

    @property
    def prep_ms(self) -> float:
        return self._prep_ms

    @property
    def pack_ms(self) -> float:
        return self._pack_ms

    @property
    def payload_bytes(self) -> int:
        return self._payload_bytes

    @property
    def meshes_received(self) -> int:
        return self._meshes_received

    @property
    def mesh_bytes_received(self) -> int:
        return self._mesh_bytes_received

    # ---- lifecycle -----------------------------------------------------------------
    def start(self) -> None:
        if self._threads:
            return
        if self._sock is None and not self.connect():
            raise ConnectionError(f"slam-service unreachable at {self._addr}")
        self._stop_evt.clear()
        with self._send_lock:
            wire.send_message(self._sock, wire.hello_message(
                cfg=self._cfg, width=self._w, height=self._h,
                mesh_format=self._mesh_format, wall_mode=self._wall_mode,
                mesh_every=self._mesh_every))
        self._threads = [threading.Thread(target=self._send_loop, daemon=True),
                         threading.Thread(target=self._recv_loop, daemon=True)]
        for t in self._threads:
            t.start()

    def _credit_ok(self) -> bool:
        return (self._inflight < self._max_inflight
                or time.monotonic() - self._inflight_since > _INFLIGHT_TIMEOUT_S)

    def _send_loop(self) -> None:
        while not self._stop_evt.is_set():
            with self._in_cv:
                if self._in_slot is None or not self._credit_ok():
                    self._in_cv.wait(_IDLE_SLEEP_S)
                    continue
                msg, self._in_slot = self._in_slot, None
                self._inflight += 1
                self._inflight_since = time.monotonic()
            # Re-stamp at the moment it actually leaves, so the age a pose
            # reports is transport + compute, not time spent waiting for credit
            # (that wait is what `frames_overwritten` counts).
            msg["t_client_send"] = time.monotonic()
            try:
                with self._send_lock:
                    wire.send_message(self._sock, msg)
            except OSError:
                break

    def _on_mesh(self, res: dict) -> None:
        seq = int(res["mesh_seq"])
        if seq == self._last_mesh_seq:
            return
        t_recv = time.monotonic()
        fmt = res.get("format", wire.MESH_FORMAT_ARRAYS)
        if fmt == wire.MESH_FORMAT_PACKET and "packet" in res:
            # Zero-copy: the packet is a uint8 view over this message's own
            # receive buffer, which nothing reuses, so hand out a memoryview
            # (bytes-like: `len()`, websocket `send_bytes`, file writes all
            # accept it) rather than paying a 36 MB `bytes()` copy under the GIL.
            packed = memoryview(res["packet"]).cast("B")
            meta = {"mesh_seq": seq, "t_pub": res.get("t_pub"), "t_recv": t_recv,
                    "source_vertex_count": int(res.get("source_vertex_count", 0)),
                    "sent_vertex_count": int(res.get("sent_vertex_count", 0)),
                    "prep_ms": float(res.get("prep_ms", 0.0)),
                    "pack_ms": float(res.get("pack_ms", 0.0)),
                    "extract_ms": float(res.get("extract_ms", 0.0))}
            self._prep_ms, self._pack_ms = meta["prep_ms"], meta["pack_ms"]
            self._payload_bytes = len(packed)
            self._last_source_verts = meta["source_vertex_count"]
            self._mesh_bytes_received += len(packed)
            with self._out_lock:
                self._last_packet = (packed, meta)
        elif "mesh_v" in res:
            self._last_mesh = wire.arrays_to_mesh(res)
            nbytes = sum(int(np.asarray(res[k]).nbytes)
                         for k in ("mesh_v", "mesh_t", "mesh_c") if k in res)
            self._mesh_bytes_received += nbytes
        else:
            return
        self._last_mesh_seq = seq
        self._meshes_received += 1
        self._unacked_seq = seq
        # Republish so the new mesh is visible NOW, not on the next pose: a
        # mesh always trails the pose of the frame that extracted it, and
        # before this a consumer polling between the two saw the stale mesh.
        if fmt != wire.MESH_FORMAT_PACKET:
            with self._out_lock:
                if self._out_slot is not None:
                    _m, traj, step = self._out_slot
                    self._out_slot = (self._last_mesh, traj, step)

    def _recv_loop(self) -> None:
        while not self._stop_evt.is_set():
            try:
                res = wire.recv_message(self._sock)
            except OSError:
                break
            if res is None:
                break
            typ = wire.message_type(res)
            if typ == wire.HELLO:
                self._service_version = int(res.get("version") or 1)
                self._device = res.get("device") or self._device
                self._icp_device = res.get("icp_device")
                continue
            if typ == wire.MESH:
                self._on_mesh(res)
                continue
            if typ == wire.MESH_FULL:
                with self._req_cv:
                    self._req_reply = res
                    self._req_cv.notify_all()
                continue
            # A pose message (tagged) OR a legacy untagged combined message
            # from a service built before the pose/mesh split. The tagged POSE
            # never carries mesh arrays, so an inline "mesh_v" means a legacy
            # service; recover its mesh and warn once.
            if "mesh_v" in res and res.get("mesh_seq") != self._last_mesh_seq:
                self._last_mesh = wire.arrays_to_mesh(res)
                self._last_mesh_seq = res.get("mesh_seq", self._last_mesh_seq)
                if not self._warned_legacy:
                    self._warned_legacy = True
                    logger.warning(
                        "remote SLAM service speaks the legacy pre-split wire "
                        "format (untagged combined message with inline mesh); "
                        "recovering meshes in compatibility mode. Rebuild the "
                        "SLAM container image to get the pose/mesh-split "
                        "bandwidth win (tools/slam-container/build.ps1).")
            t_recv = time.monotonic()
            pose = np.asarray(res["pose"], np.float64)
            step_kwargs = {}
            for name in wire.STEP_SCALAR_FIELDS:
                if name in res:
                    step_kwargs[name] = res[name]
            step = FrameStep(pose=pose, fitness=res["fitness"], rmse=res["rmse"],
                             tracking_lost=res["tracking_lost"], slam_ms=res["slam_ms"],
                             **step_kwargs)
            self._tracking_lost_count = res["tracking_lost_count"]
            self._frames_processed += 1
            with self._in_cv:
                self._inflight = max(0, self._inflight - 1)
                self._in_cv.notify()
            dev = res.get("device")
            if dev is not None:
                self._device = dev
            if "mesh_extract_ms" in res:
                self._mesh_extract_ms = float(res["mesh_extract_ms"])
            self.last_pose_timing = {
                "fid": res.get("fid"),
                "t_client_submit": res.get("t_client_submit"),
                "t_client_send": res.get("t_client_send"),
                "t_service_recv": res.get("t_service_recv"),
                "t_service_done": res.get("t_service_done"),
                "t_client_recv": t_recv,
                "t_device_us": step.t_device_us,
            }
            self._trajectory.append(pose)
            with self._out_lock:
                self._out_slot = (self._last_mesh, self._trajectory, step)
        # Fell out of the loop: EOF or socket error. Unless we are stopping,
        # that is the service going away -- flag it and wake any fetch_mesh.
        if not self._stop_evt.is_set():
            self._dead = True
            with self._req_cv:
                self._req_cv.notify_all()

    def stop(self) -> None:
        self._stop_evt.set()
        if self._sock is not None:
            try:
                self._sock.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            self._sock.close()
        for t in self._threads:
            t.join(timeout=1.5)
        self._threads = []
        self._sock = None
        self._dead = False
