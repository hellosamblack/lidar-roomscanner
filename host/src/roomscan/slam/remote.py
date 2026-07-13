"""Client-side drop-in for SlamWorker that ships frames to a SlamService over a
localhost socket and republishes the returned (mesh, trajectory, FrameStep).
Same interface as slam.worker.SlamWorker, so panel.py is agnostic to which one
it holds. On any socket failure the caller falls back to the local worker."""
from __future__ import annotations

import socket
import threading
import time

import numpy as np

from . import wire
from .mapper import FrameStep

_IDLE_SLEEP_S = 0.005


class RemoteSlamWorker:
    def __init__(self, width, height, addr="127.0.0.1:5555", mesh_every=5,
                 connect_timeout=1.0, **mapper_kwargs):
        self._w, self._h = width, height
        host, _, port = addr.partition(":")
        self._host, self._port = host, int(port)
        self._connect_timeout = connect_timeout
        self._sock = None
        self._fid = 0

        self._in_lock = threading.Lock()
        self._in_slot = None
        self._out_lock = threading.Lock()
        self._out_slot = None
        self._tracking_lost_count = 0

        self._last_mesh_seq = -1
        self._last_mesh = None

        self._threads = []
        self._stop_evt = threading.Event()

    def connect(self) -> bool:
        try:
            self._sock = socket.create_connection(
                (self._host, self._port), timeout=self._connect_timeout)
            self._sock.settimeout(None)
            return True
        except OSError:
            self._sock = None
            return False

    def submit(self, depth, quat, pressure, reflectance=None, confidence=None) -> None:
        with self._in_lock:
            self._fid += 1
            msg = {"fid": self._fid,
                   "depth": np.asarray(depth, np.float32),
                   "quat": np.asarray(quat, np.float32),
                   "pressure": None if pressure is None else float(pressure)}
            if reflectance is not None:
                msg["reflectance"] = np.asarray(reflectance, np.float32)
            if confidence is not None:
                msg["confidence"] = np.asarray(confidence, np.float32)
            self._in_slot = msg

    def latest(self):
        with self._out_lock:
            return self._out_slot

    @property
    def tracking_lost_count(self) -> int:
        return self._tracking_lost_count

    def start(self) -> None:
        if self._sock is None and not self.connect():
            raise ConnectionError(f"slam-service unreachable at {self._host}:{self._port}")
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
            step = FrameStep(pose=np.asarray(res["pose"], np.float64),
                             fitness=res["fitness"], rmse=res["rmse"],
                             tracking_lost=res["tracking_lost"], slam_ms=res["slam_ms"])
            traj = [np.asarray(p, np.float64) for p in res["traj"]]
            self._tracking_lost_count = res["tracking_lost_count"]
            if res["mesh_seq"] != self._last_mesh_seq and "mesh_v" in res:
                self._last_mesh = wire.arrays_to_mesh(res)
                self._last_mesh_seq = res["mesh_seq"]
            with self._out_lock:
                self._out_slot = (self._last_mesh, traj, step)

    def stop(self) -> None:
        self._stop_evt.set()
        if self._sock is not None:
            try:
                self._sock.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            self._sock.close()
            self._sock = None
        for t in self._threads:
            t.join(timeout=1.5)
        self._threads = []
