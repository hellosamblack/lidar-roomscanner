"""Select the live SLAM worker backend from config: in-process SlamWorker
(default) or a RemoteSlamWorker talking to a SlamService (a subprocess on this
host -- see `supervisor.py` -- or the GPU container), with automatic fallback
to local when the service is unreachable.

`mesh_format` / `wall_mode` (#224) are RemoteSlamWorker's own kwargs -- where
`prepare_packet`/`pack_mesh` run -- and are stripped before a local
`SlamWorker`, whose `Mapper` would reject them. `mesh_format=None` means
"whatever `[slam] remote_mesh_format` says"."""
from __future__ import annotations


from .config import SlamConfig
from .worker import SlamWorker


def make_slam_worker(width, height, cfg=None, mesh_format=None, wall_mode="split",
                     **mapper_kwargs):
    cfg = cfg if cfg is not None else SlamConfig.load()
    if cfg.backend == "remote":
        from .remote import RemoteSlamWorker
        fmt = mesh_format or cfg.remote_mesh_format
        rw = RemoteSlamWorker(width, height, addr=cfg.remote_addr, mesh_format=fmt,
                              wall_mode=wall_mode, **mapper_kwargs)
        if rw.connect():
            return rw
        import logging
        logging.getLogger().warning(f"[slam] remote backend at {cfg.remote_addr} unreachable; falling back to local CPU worker")
    return SlamWorker(width, height, **mapper_kwargs)
