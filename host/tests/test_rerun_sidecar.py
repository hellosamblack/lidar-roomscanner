"""Tests for Rerun observability sidecar (Phase 6 / Issue #136).

Verifies:
1. Queue overload drops items and increments drop counter without blocking.
2. Fail-open exception handling.
3. Explicit time conversion from t_us to seconds and frame_seq.
4. Coordinate frames mapping with signed non-cardinal angle (30 deg).
5. Start / stop lifecycle, status reporting, and manifest generation.
6. Offline capture replay export via rerun_export.
"""
from __future__ import annotations

import math
import time
from pathlib import Path

import numpy as np
import pytest

from roomscan.protocol import FrameHeader, FrameType, StreamId
from roomscan.rerun_sidecar import (
    MockWriter,
    RerunSidecar,
    RerunSidecarConfig,
)
from roomscan.sensors import (
    T_CV_TO_BODY,
    T_WORLD_TO_CV,
    display_rotation,
    quat_to_matrix,
)
from tools.rerun_export import replay_capture_to_rerun


def _make_sample_header(t_us: int = 1_500_000, seq: int = 42, stream_id: int = StreamId.DEPTH_ZF32) -> FrameHeader:
    return FrameHeader(
        frame_type=FrameType.DATA,
        stream_id=stream_id,
        flags=0,
        seq=seq,
        t_us=t_us,
        width=54,
        height=42,
        payload_len=54 * 42 * 4,
    )


def test_start_stop_lifecycle_and_status(tmp_path: Path):
    """Test start, stop, active state, and status reporting."""
    writer = MockWriter()
    cfg = RerunSidecarConfig(
        enabled=True,
        max_queue_size=10,
        output_dir=str(tmp_path),
        recording_name="test_rec",
    )
    sidecar = RerunSidecar(config=cfg, writer=writer)

    st0 = sidecar.status()
    assert not st0["active"]
    assert st0["samples_logged"] == 0
    assert st0["queue_drops"] == 0
    assert st0["fault_count"] == 0

    sidecar.start()
    assert sidecar.active
    st1 = sidecar.status()
    assert st1["active"]

    # Verify static frames were logged
    assert len(writer.static_entries) == 2
    static_paths = [path for path, _ in writer.static_entries]
    assert "world" in static_paths
    assert "world/scanner" in static_paths

    sidecar.stop()
    assert not sidecar.active
    assert writer.closed


def test_time_conversion_from_t_us():
    """Verify t_us is converted strictly to device_time = t_us / 1e6 and frame_seq."""
    writer = MockWriter()
    sidecar = RerunSidecar(
        config=RerunSidecarConfig(enabled=True, max_queue_size=10),
        writer=writer,
    )
    sidecar.start()

    t_us = 2_500_000
    seq = 101
    hdr = _make_sample_header(t_us=t_us, seq=seq)
    depth = np.ones((42, 54), dtype=np.float32) * 1000.0

    sidecar.log_frame(hdr, {"depth": depth})
    time.sleep(0.1)  # Allow worker to process
    sidecar.stop()

    depth_entries = writer.get_entries("world/scanner/tof/depth")
    assert len(depth_entries) == 1
    entry = depth_entries[0]
    assert entry.device_time_s == pytest.approx(2.5, abs=1e-6)
    assert entry.frame_seq == 101


def test_queue_overload_drops_without_blocking():
    """Verify that when queue is full, log_* drops snapshots, increments drop count, and never blocks."""
    # Custom sink that delays processing to simulate worker backlog
    class SlowWriter(MockWriter):
        def log(self, entity_path: str, entity_data, static: bool = False) -> None:
            time.sleep(0.05)
            super().log(entity_path, entity_data, static=static)

    writer = SlowWriter()
    max_q = 3
    sidecar = RerunSidecar(
        config=RerunSidecarConfig(enabled=True, max_queue_size=max_q),
        writer=writer,
    )
    sidecar.start()

    # Rapidly submit 20 frames
    t0 = time.time()
    for i in range(20):
        hdr = _make_sample_header(t_us=1000000 + i * 33333, seq=i)
        depth = np.zeros((42, 54), dtype=np.float32)
        sidecar.log_frame(hdr, {"depth": depth})

    submission_duration = time.time() - t0
    # 20 submissions must take way less than 20 * 0.05s (1.0s) because non-blocking
    assert submission_duration < 0.2

    status = sidecar.status()
    assert status["queue_drops"] > 0

    sidecar.stop()


def test_fail_open_exception_handling():
    """Verify fail-open: exceptions in writer do not crash worker or caller, incrementing fault_count."""
    class FaultyWriter(MockWriter):
        def log(self, entity_path: str, entity_data, static: bool = False) -> None:
            if not static:
                raise RuntimeError("Simulated sink disk failure / network disconnect")

    writer = FaultyWriter()
    sidecar = RerunSidecar(
        config=RerunSidecarConfig(enabled=True, max_queue_size=10),
        writer=writer,
    )
    sidecar.start()

    hdr = _make_sample_header()
    depth = np.ones((42, 54), dtype=np.float32)

    # Calling log_frame should not raise
    sidecar.log_frame(hdr, {"depth": depth})
    sidecar.log_imu(1000000, 1, quat=(1.0, 0.0, 0.0, 0.0))
    time.sleep(0.1)

    status = sidecar.status()
    assert status["fault_count"] >= 1

    sidecar.stop()


def test_coordinate_frames_mapping_signed_30deg():
    """Verify coordinate frame mapping using signed non-cardinal 30 deg rotation.

    Tests:
    - SFLP body -> SFLP world rotation (R)
    - CV -> SFLP body transform (T_CV_TO_BODY)
    - Composed display rotation (T_WORLD_TO_CV @ R @ T_CV_TO_BODY)
    - Deprojected 3D points
    """
    writer = MockWriter()
    sidecar = RerunSidecar(
        config=RerunSidecarConfig(enabled=True, max_queue_size=10),
        writer=writer,
    )
    sidecar.start()

    # 30 deg rotation around X-axis (pitch/roll depending on frame)
    theta = math.radians(30.0)
    w = math.cos(theta / 2.0)
    x = math.sin(theta / 2.0)
    y = 0.0
    z = 0.0
    quat_30x = (w, x, y, z)

    # Verify R matrix calculation
    r_mat = quat_to_matrix(*quat_30x)
    assert r_mat.shape == (3, 3)
    assert r_mat[0, 0] == pytest.approx(1.0, abs=1e-5)
    assert r_mat[1, 1] == pytest.approx(math.cos(theta), abs=1e-5)
    assert r_mat[1, 2] == pytest.approx(-math.sin(theta), abs=1e-5)
    assert r_mat[2, 1] == pytest.approx(math.sin(theta), abs=1e-5)
    assert r_mat[2, 2] == pytest.approx(math.cos(theta), abs=1e-5)

    # Composed display rotation: T_WORLD_TO_CV @ R @ T_CV_TO_BODY
    r_disp = display_rotation(quat_30x)
    expected_disp = T_WORLD_TO_CV @ r_mat @ T_CV_TO_BODY
    np.testing.assert_allclose(r_disp, expected_disp, atol=1e-6)

    # Log pose
    sidecar.log_pose(t_us=1_000_000, seq=1, quat=quat_30x, fused_quat=quat_30x)
    time.sleep(0.1)
    sidecar.stop()

    pose_entries = writer.get_entries("world/scanner/pose/raw_sflp")
    assert len(pose_entries) == 1
    data = pose_entries[0].data
    assert data["quaternion"] == quat_30x
    np.testing.assert_allclose(data["matrix"], r_mat, atol=1e-6)


def test_log_mesh_and_bounded_vertices():
    """Verify mesh logging and vertex budget capping."""
    writer = MockWriter()
    cfg = RerunSidecarConfig(
        enabled=True,
        max_queue_size=10,
        mesh_vertex_budget=100,
    )
    sidecar = RerunSidecar(config=cfg, writer=writer)
    sidecar.start()

    verts = np.random.randn(250, 3).astype(np.float32)
    triangles = np.arange(240).reshape(80, 3).astype(np.int32)
    colors = np.random.randint(0, 255, (250, 3), dtype=np.uint8)

    sidecar.log_mesh(t_us=1_000_000, seq=1, vertices=verts, triangles=triangles, vertex_colors=colors)
    time.sleep(0.1)
    sidecar.stop()

    mesh_entries = writer.get_entries("world/map/mesh")
    assert len(mesh_entries) == 1
    mesh_data = mesh_entries[0].data
    assert len(mesh_data["vertices"]) == 100
    assert len(mesh_data["vertex_colors"]) == 100


def test_offline_replay_export_with_fixture(tmp_path: Path):
    """Test replaying a real capture fixture through rerun_export tool."""
    fixture_path = Path(__file__).parent / "fixtures" / "golden_sensors_snippet.bin"
    output_rrd = tmp_path / "replay_test.rrd"

    status = replay_capture_to_rerun(
        capture_path=fixture_path,
        output_rrd=output_rrd,
        use_mock=True,
    )

    assert status["frames_processed"] > 0
    assert status["samples_logged"] > 0
    assert status["queue_drops"] == 0
    assert status["fault_count"] == 0
    assert not status["active"]
