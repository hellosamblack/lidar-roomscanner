import numpy as np
import pytest
from roomscan.slam.config import SlamConfig
from roomscan.slam.backend import make_slam_worker
from roomscan.slam.worker import SlamWorker

pytest.importorskip("open3d")


def test_config_has_backend_defaults():
    cfg = SlamConfig()
    assert cfg.backend == "local"
    assert cfg.remote_addr == "127.0.0.1:5555"


def test_local_backend_returns_local_worker():
    cfg = SlamConfig(backend="local")
    w = make_slam_worker(54, 42, cfg=cfg, fov_h=55.0, fov_v=42.0, device="CPU:0")
    assert isinstance(w, SlamWorker)


def test_remote_backend_falls_back_to_local_when_unreachable():
    cfg = SlamConfig(backend="remote", remote_addr="127.0.0.1:1")
    w = make_slam_worker(54, 42, cfg=cfg, fov_h=55.0, fov_v=42.0, device="CPU:0")
    # unreachable service -> silent fall back to the in-process CPU worker
    assert isinstance(w, SlamWorker)


def test_remote_backend_falls_back_to_local_on_malformed_addr():
    # Finding 2: a malformed remote_addr (no ":port") must not raise out of
    # RemoteSlamWorker's constructor and bypass the fallback -- make_slam_worker
    # must still land on the local CPU worker.
    cfg = SlamConfig(backend="remote", remote_addr="127.0.0.1")
    w = make_slam_worker(54, 42, cfg=cfg, fov_h=55.0, fov_v=42.0, device="CPU:0")
    assert isinstance(w, SlamWorker)
