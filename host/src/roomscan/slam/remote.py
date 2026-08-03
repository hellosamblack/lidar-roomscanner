"""Client-side drop-in for SlamWorker that ships frames to a SlamService over a
localhost socket and republishes the returned (mesh, trajectory, FrameStep).
Same interface as slam.worker.SlamWorker, so panel.py is agnostic to which one
it holds. On any socket failure the caller falls back to the local worker."""
from __future__ import annotations

import json
import logging
import socket
import threading
import time

import numpy as np

from . import wire
from .mapper import FrameStep

_IDLE_SLEEP_S = 0.005

logger = logging.getLogger(__name__)


class RemoteSlamWorker:
    def __init__(self, width, height, addr="127.0.0.1:5555", mesh_every=5,
                 connect_timeout=1.0, **mapper_kwargs):
        self._w, self._h = width, height
        self._addr = addr
        self._connect_timeout = connect_timeout
        self._sock = None
        self._fid = 0
        # Forward the client's mapper params to the service so the remote
        # Mapper is built with the same effective config (except `device`,
        # which the container owns). Sent on every submitted frame; the
        # server only needs to read it once, on lazy worker creation.
        self._cfg_json = json.dumps(
            {k: v for k, v in mapper_kwargs.items() if k != "device"})

        self._in_lock = threading.Lock()
        self._in_slot = None
        self._out_lock = threading.Lock()
        self._out_slot = None
        self._tracking_lost_count = 0

        self._last_mesh_seq = -1
        self._last_mesh = None
        self._trajectory = []          # accumulated from pose deltas (no full-traj resend)
        self._warned_legacy = False    # one-time warn on a pre-split (untagged) service

        # Plan item 2 (2026-08-02): the same submit/overwrite/processed shape
        # as SlamWorker, but measured on THIS side of the socket -- i.e. the
        # transport slot, not the container's own mapper. `frames_submitted`/
        # `frames_overwritten` count `submit()` calls into `_in_slot` exactly
        # like SlamWorker's; `frames_processed` counts POSE messages received
        # back (each one frame the service actually ran through its Mapper).
        # `_device` is populated from the service's own report on that same
        # POSE message (an optional field -- see wire.pose_message/service.py)
        # and stays None until the first one arrives, rather than assuming
        # whatever the LOCAL host would infer for itself (the whole point of
        # this counter set: the remote compute device lives in a different
        # process and must report itself).
        self._frames_submitted = 0
        self._frames_overwritten = 0
        self._frames_processed = 0
        self._device: str | None = None

        self._threads = []
        self._stop_evt = threading.Event()

    def connect(self) -> bool:
        try:
            host, _, port = self._addr.partition(":")
            port = int(port)
            self._sock = socket.create_connection(
                (host, port), timeout=self._connect_timeout)
            self._sock.settimeout(None)
            return True
        except (OSError, ValueError):
            self._sock = None
            return False

    def submit(self, depth, quat, pressure, reflectance=None, confidence=None) -> None:
        with self._in_lock:
            self._fid += 1
            msg = {"fid": self._fid,
                   "depth": np.asarray(depth, np.float32),
                   "quat": np.asarray(quat, np.float32),
                   "pressure": None if pressure is None else float(pressure),
                   "cfg": self._cfg_json}
            if reflectance is not None:
                msg["reflectance"] = np.asarray(reflectance, np.float32)
            if confidence is not None:
                msg["confidence"] = np.asarray(confidence, np.float32)
            self._frames_submitted += 1
            if self._in_slot is not None:
                self._frames_overwritten += 1
            self._in_slot = msg

    def latest(self):
        with self._out_lock:
            return self._out_slot

    @property
    def tracking_lost_count(self) -> int:
        return self._tracking_lost_count

    # ---- instrumentation (plan item 2, 2026-08-02) ---------------------------
    @property
    def frames_submitted(self) -> int:
        """Total `submit()` calls into this client's send slot."""
        return self._frames_submitted

    @property
    def frames_processed(self) -> int:
        """POSE responses received -- one per frame the SERVICE actually ran
        through its Mapper. NOTE this is transport-confirmed, not the
        service's own processed count (which is not on the wire); a frame
        that was sent but whose response hasn't arrived yet is neither
        processed nor overwritten here."""
        return self._frames_processed

    @property
    def frames_overwritten(self) -> int:
        """Submits that replaced a still-pending, not-yet-SENT message in
        this client's own transport slot (measured on the CLIENT side of the
        socket -- see SlamWorker.frames_overwritten's docstring for the
        general shape). Does not see drops or overwrites inside the service
        itself, since the service processes one frame per received message."""
        return self._frames_overwritten

    @property
    def device(self) -> str | None:
        """The remote SlamService's own resolved compute device, from the
        most recent POSE response's optional "device" field. None until the
        first response arrives, or when talking to a service built before
        this field existed."""
        return self._device

    @property
    def backend(self) -> str:
        return "remote"

    def start(self) -> None:
        if self._threads:
            return
        if self._sock is None and not self.connect():
            raise ConnectionError(f"slam-service unreachable at {self._addr}")
        self._stop_evt.clear()
        self._threads = [threading.Thread(target=self._send_loop, daemon=True),
                         threading.Thread(target=self._recv_loop, daemon=True)]
        for t in self._threads:
            t.start()

    def _send_loop(self) -> None:
        while not self._stop_evt.is_set():
            with self._in_lock:
                msg, self._in_slot = self._in_slot, None
            if msg is None:
                time.sleep(_IDLE_SLEEP_S); continue
            try:
                wire.send_message(self._sock, msg)
            except OSError:
                break

    def _recv_loop(self) -> None:
        while not self._stop_evt.is_set():
            try:
                res = wire.recv_message(self._sock)
            except OSError:
                break
            if res is None:
                break
            if res.get("type") == wire.MESH:
                if res["mesh_seq"] != self._last_mesh_seq and "mesh_v" in res:
                    self._last_mesh = wire.arrays_to_mesh(res)
                    self._last_mesh_seq = res["mesh_seq"]
                continue
            # A pose message (new tagged format) OR a legacy untagged combined
            # message from a service built before the pose/mesh split -- e.g. a
            # GPU container image that predates Component B. The new POSE message
            # never carries mesh arrays, so an inline "mesh_v" here means we're
            # talking to a legacy service; recover its mesh (otherwise the live
            # view is silently starved of surfaces -- pose/traj fine, mesh None)
            # and warn once to rebuild the container for the pose/mesh-split win.
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
            step = FrameStep(pose=np.asarray(res["pose"], np.float64),
                             fitness=res["fitness"], rmse=res["rmse"],
                             tracking_lost=res["tracking_lost"], slam_ms=res["slam_ms"],
                             # Optional: a container built before the block
                             # gauge (BUG-035) simply omits them -> None, which
                             # the UI renders as "unknown", not as 0 blocks.
                             blocks_used=res.get("blocks_used"),
                             blocks_capacity=res.get("blocks_capacity"),
                             blocks_configured=res.get("blocks_configured"))
            self._tracking_lost_count = res["tracking_lost_count"]
            self._frames_processed += 1
            # The service's own resolved device (optional -- older services
            # simply omit it, and this stays None until the first response).
            # See Mapper.device's docstring: this is what makes device
            # reporting for a remote backend correct rather than the host
            # guessing its own preferred_device() for a process it isn't
            # running compute in.
            dev = res.get("device")
            if dev is not None:
                self._device = dev
            self._trajectory.append(np.asarray(res["pose"], np.float64))
            with self._out_lock:
                self._out_slot = (self._last_mesh, list(self._trajectory), step)

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
