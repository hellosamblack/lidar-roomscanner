"""Select the live SLAM worker backend from config: in-process CPU SlamWorker
(default) or a RemoteSlamWorker talking to the GPU container's SlamService,
with automatic fallback to local when the service is unreachable."""
from __future__ import annotations

import sys

from .config import SlamConfig
from .worker import SlamWorker


def make_slam_worker(width, height, cfg=None, **mapper_kwargs):
    cfg = cfg if cfg is not None else SlamConfig.load()
    if cfg.backend == "remote":
        from .remote import RemoteSlamWorker
        rw = RemoteSlamWorker(width, height, addr=cfg.remote_addr, **mapper_kwargs)
        if rw.connect():
            return rw
        import logging
        logging.getLogger().warning(f"[slam] remote backend at {cfg.remote_addr} unreachable; falling back to local CPU worker")
    return SlamWorker(width, height, **mapper_kwargs)
