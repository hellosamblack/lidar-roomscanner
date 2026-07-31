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
