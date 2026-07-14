"""Container-side SLAM compute service. Owns one Mapper (via SlamWorker for its
mesh throttle + publish logic) on a chosen Open3D device (CUDA:0 in the GPU
container). One client at a time; localhost-only; no auth. See
docs/superpowers/specs/2026-07-13-slam-gpu-container-service-design.md."""
from __future__ import annotations

import argparse
import json
import socket
import sys

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


class SlamService:
    def __init__(self, device="CUDA:0", mesh_every=5, **mapper_kwargs):
        self._device = device
        self._mesh_every = mesh_every
        self._mapper_kwargs = mapper_kwargs
        self._last_effective_kwargs = None

    def serve_client(self, conn) -> None:
        worker = None
        last_mesh = object()          # sentinel; never equal to a real mesh
        mesh_seq = 0
        while True:
            msg = wire.recv_message(conn)
            if msg is None:
                break
            depth = np.asarray(msg["depth"], np.float32)
            if worker is None:
                h, w = depth.shape
                eff_kwargs = _effective_kwargs(self._mapper_kwargs, msg.get("cfg"))
                self._last_effective_kwargs = eff_kwargs
                worker = SlamWorker(w, h, mesh_every=self._mesh_every,
                                    device=self._device, **eff_kwargs)
            quat = np.asarray(msg["quat"], np.float32)
            pressure = msg.get("pressure")
            refl = msg.get("reflectance")
            conf = msg.get("confidence")
            worker.submit(depth, quat, pressure,
                          reflectance=None if refl is None else np.asarray(refl, np.float32),
                          confidence=None if conf is None else np.asarray(conf, np.float32))
            worker.run_once()
            mesh, _traj, step = worker.latest()

            # Pose first: tiny, sent immediately, never delayed behind a mesh
            # transfer or the (no-longer-sent) full trajectory.
            wire.send_message(conn, wire.pose_message(
                msg["fid"], step.pose, step.fitness, step.rmse,
                step.tracking_lost, step.slam_ms, worker.tracking_lost_count))

            # Mesh only when the worker published a new one (identity check).
            if mesh is not None and mesh is not last_mesh:
                mesh_seq += 1
                wire.send_message(conn, wire.mesh_message(mesh_seq, mesh))
                last_mesh = mesh


def serve(host="0.0.0.0", port=5555, device="CUDA:0", *, _sock=None, **mapper_kwargs) -> None:
    srv = SlamService(device=device, **mapper_kwargs)
    if _sock is not None:
        lsock = _sock
    else:
        lsock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        lsock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        lsock.bind((host, port)); lsock.listen(1)
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
            print("[slam-service] client disconnected; awaiting next", flush=True)


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

    serve(host=args.host, port=args.port, device=args.device,
          fov_h=cfg.fov_h, fov_v=cfg.fov_v, voxel_size=cfg.voxel_size,
          icp_mode=cfg.icp_mode, baro_weight=cfg.baro_weight, max_dist=cfg.max_dist,
          min_fitness=cfg.min_fitness, max_rmse=cfg.max_rmse,
          min_confidence=cfg.min_confidence, weight_threshold=cfg.weight_threshold,
          stationary_hold=cfg.stationary_hold, stationary_window=cfg.stationary_window,
          stationary_coherence=cfg.stationary_coherence,
          stationary_step_ceiling=cfg.stationary_step_ceiling,
          stationary_rot_ceiling=cfg.stationary_rot_ceiling)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
