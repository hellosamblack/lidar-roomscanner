from __future__ import annotations

from roomscan.slam.config import DetailedSlamPreset
from roomscan.slam.detailed import build_manifest, estimate_seconds, sidecar_status, write_manifest_atomic
from roomscan.slam.validation import paired_loop_gate
from roomscan import web
from roomscan.protocol import FrameHeader, FrameType, StreamId, pack_frame


def test_detailed_preset_fingerprint_changes_with_effective_params():
    a = DetailedSlamPreset()
    b = DetailedSlamPreset(voxel_size=0.006)
    assert a.fingerprint() != b.fingerprint()
    assert a.mapper_kwargs()["icp_retry_dist"] == a.retry_dist


def test_fingerprint_ignores_timing_calibration():
    """Calibrating the estimate must not invalidate every existing sidecar.

    `per_frame_ms`/`global_opt_ms`/`benchmark_note` describe how long a build
    TAKES, not what it PRODUCES -- and populating them is the explicitly planned
    next step. Hashing them would mark every reconstruction stale the moment the
    benchmark lands, and a staleness flag that fires on unrelated changes is one
    the user learns to ignore, so it can no longer warn about a real one.
    """
    base = DetailedSlamPreset()
    calibrated = DetailedSlamPreset(per_frame_ms=9.4, global_opt_ms=420.0,
                                    benchmark_note="measured on CUDA:0 2026-07-31")
    assert calibrated.fingerprint() == base.fingerprint()
    # ...but a real reconstruction parameter still moves it.
    assert DetailedSlamPreset(max_iter=8).fingerprint() != base.fingerprint()


def test_sidecar_is_current_only_when_manifest_matches(tmp_path):
    capture = tmp_path / "take.bin"
    capture.write_bytes(b"raw wire bytes")
    preset = DetailedSlamPreset()
    paths = {"ply": tmp_path / "take.ply", "tum": tmp_path / "take.tum", "manifest": tmp_path / "take.slam.json"}
    paths["ply"].write_bytes(b"ply")
    paths["tum"].write_bytes(b"tum")
    write_manifest_atomic(paths["manifest"], build_manifest(capture, preset, stats={}, estimate={}))
    assert sidecar_status(capture, tmp_path, preset)["current"]
    assert sidecar_status(capture, tmp_path, DetailedSlamPreset(max_iter=8))["stale"]


def test_estimate_reports_uncalibrated_and_cpu_warning():
    out = estimate_seconds(10, DetailedSlamPreset(), cuda=False)
    assert not out["calibrated"] and out["cpu_warning"] and out["note"]


def test_paired_gate_requires_positive_interval_and_no_loss_regression():
    baseline = [{"horizontal_closure_m": 1.0, "lost": 0} for _ in range(10)]
    closed = [{"horizontal_closure_m": 0.5, "lost": 0} for _ in range(10)]
    assert paired_loop_gate(baseline, closed)["accepted"]
    closed[0]["lost"] = 1
    assert not paired_loop_gate(baseline, closed)["accepted"]


def test_capture_metadata_uses_header_time_and_detects_stream_9(tmp_path):
    p = tmp_path / "timed.bin"
    wires = []
    for seq, sid, t_us in ((1, StreamId.IMU_QUAT, 1_000_000),
                            (2, StreamId.DEPTH_ZF32, 1_000_000),
                            (3, StreamId.DEPTH_ZF32, 3_000_000)):
        payload = b"\0" * (16 if sid == StreamId.IMU_QUAT else 4)
        wires.append(pack_frame(FrameHeader(FrameType.DATA, sid, 0, seq, t_us, 1, 1, len(payload)), payload))
    p.write_bytes(b"".join(wires))
    info = web.scan_capture_metadata(p)
    assert info["has_stream_9"] and info["timestamped"]
    assert info["duration_s"] == 2.0
