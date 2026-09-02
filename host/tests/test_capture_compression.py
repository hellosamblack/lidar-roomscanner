"""Tests for tools/capture_compression.py -- the Phase 5.5 compression go/no-go.

`docs/roadmap-history.md` quotes two numbers from a one-off 2026-07-31
investigation (54.3% pooled in-transit LZ4-on-de-interleaved-RAW_3DMD, zstd-9
1.868x vs zlib-6 1.808x at rest) that existed only as prose. These tests pin the
tool that makes them re-runnable, and prove the de-interleave measurement is
real by constructing data where it demonstrably helps and checking the gap is
large, not just present.
"""
from __future__ import annotations

import json
import random
import struct
import subprocess
import sys
import zlib
from pathlib import Path

import pytest

from tools.capture_compression import (deinterleave16, lz4_compress_bound,
                                       measure_captures, _codec_report, _timed,
                                       LZ4_STREAM_STATE_BYTES)

MAGIC = b"RSCN"
REPO = Path(__file__).resolve().parents[2]


def _frame(seq: int, stream: int, payload: bytes, t_us: int | None = None) -> bytes:
    header = struct.pack("<4sBBBBIQHHII", MAGIC, 1, 1, stream, 0, seq,
                         t_us if t_us is not None else seq * 33_000, 1, 1,
                         len(payload), 0)
    body = header + payload
    return body + struct.pack("<I", zlib.crc32(body))


def _write(tmp_path, frames: list[bytes], name: str = "cap.bin") -> str:
    p = tmp_path / name
    p.write_bytes(b"".join(frames))
    return str(p)


def _correlated_high_noisy_low(n_frames: int, samples_per_frame: int, seed: int) -> list[bytes]:
    """RAW_3DMD-like payloads: 16-bit LE, low byte = noise, high byte = a slow
    run (95% chance of repeating the previous value). Interleaved, LZ4 can only
    match a repeated (low, high) PAIR, which the noisy low byte makes nearly
    never happen; de-interleaved, the high-byte plane collapses into long runs
    LZ4 crushes.
    """
    rng = random.Random(seed)
    frames = []
    high = 0
    for i in range(n_frames):
        payload = bytearray()
        for _ in range(samples_per_frame):
            if rng.random() > 0.95:
                high = (high + 1) % 256
            payload.append(rng.randrange(256))  # low byte: noise
            payload.append(high)                # high byte: slow run
        frames.append(_frame(i, 7, bytes(payload)))  # stream 7 = RAW_3DMD
    return frames


# --- deinterleave16 itself -------------------------------------------------

def test_deinterleave16_splits_even_and_odd_offsets():
    payload = bytes([0xAA, 0x01, 0xBB, 0x02, 0xCC, 0x03])
    out = deinterleave16(payload)
    assert out == bytes([0xAA, 0xBB, 0xCC, 0x01, 0x02, 0x03])


def test_deinterleave16_preserves_length_on_odd_input():
    payload = bytes([1, 2, 3, 4, 5])
    out = deinterleave16(payload)
    assert len(out) == len(payload)
    assert out == bytes([1, 3, 5, 2, 4])


def test_lz4_compress_bound_matches_documented_formula():
    # LZ4_COMPRESSBOUND(inputSize) = inputSize + inputSize/255 + 16 (lz4.h)
    assert lz4_compress_bound(14842) == 14842 + (14842 // 255) + 16
    assert lz4_compress_bound(0) == 16


# --- the de-interleave gain must be real and large --------------------------

def test_deinterleave_measurably_beats_plain_lz4(tmp_path):
    frames = _correlated_high_noisy_low(n_frames=60, samples_per_frame=200, seed=7)
    path = _write(tmp_path, frames)

    report = measure_captures([path], max_frames_per_stream=200)
    s = report["pooled"]["RAW_3DMD"]

    raw_ratio = s["lz4"]["pct_of_original"]
    deint_ratio = s["lz4_deinterleaved"]["pct_of_original"]

    # Both are real percentages, not degenerate 0/None.
    assert 0.0 < deint_ratio < 120.0
    assert 0.0 < raw_ratio < 120.0
    # The gap constructed above is large (interleaved pairs almost never repeat;
    # the de-interleaved high-byte plane is mostly one long run) -- assert an
    # interval, not just "some" improvement, so a broken transform that only
    # nudges the ratio still fails this.
    assert raw_ratio - deint_ratio > 15.0, (
        f"expected de-interleave to buy >15pp on correlated-high/noisy-low data, "
        f"got raw={raw_ratio}% deinterleaved={deint_ratio}%")


def test_deinterleave_gain_disappears_if_the_transform_is_a_no_op(tmp_path, monkeypatch):
    """Prove the previous test's assertion is discriminating (engineering-practices:
    prove a regression test by reintroducing the defect). If `deinterleave16` were
    accidentally wired as an identity function, the measured gap must collapse to
    ~0 -- confirming the test above would have caught that defect.
    """
    frames = _correlated_high_noisy_low(n_frames=60, samples_per_frame=200, seed=7)
    path = _write(tmp_path, frames)

    import tools.capture_compression as cc
    monkeypatch.setattr(cc, "deinterleave16", lambda payload: payload)

    report = cc.measure_captures([path], max_frames_per_stream=200)
    s = report["pooled"]["RAW_3DMD"]
    gap = s["lz4"]["pct_of_original"] - s["lz4_deinterleaved"]["pct_of_original"]
    assert abs(gap) < 1.0, f"identity de-interleave should show ~0 gap, got {gap}pp"


# --- zstd vs zlib at-rest comparison ----------------------------------------

def test_zstd_beats_or_matches_zlib_on_redundant_data(tmp_path):
    # Redundancy WITHIN each frame (codecs here compress one frame at a time,
    # matching per-frame in-transit compression, so cross-frame repetition
    # buys nothing): a short random tile repeated many times, lightly mutated.
    rng = random.Random(3)
    tile = bytes(rng.randrange(256) for _ in range(20))
    frames = []
    for i in range(40):
        noisy = bytearray(tile * 20)  # 400 bytes, 20 verbatim repeats
        for _ in range(5):
            noisy[rng.randrange(len(noisy))] = rng.randrange(256)
        frames.append(_frame(i, 7, bytes(noisy)))
    path = _write(tmp_path, frames)

    report = measure_captures([path], max_frames_per_stream=100)
    s = report["pooled"]["RAW_3DMD"]

    assert s["zstd"]["factor"] > 1.5, "template data should compress well under zstd"
    assert s["zlib"]["factor"] > 1.5, "template data should compress well under zlib"
    # zstd-9 dominating zlib-6 is the roadmap's claim -- assert it holds (allow
    # exact ties since both may fully collapse the template).
    assert s["zstd"]["compressed_bytes"] <= s["zlib"]["compressed_bytes"]


# --- round-trip guard ---------------------------------------------------

def test_codec_report_raises_on_round_trip_mismatch():
    def bad_decompress(_c, _n):
        return b"WRONG"

    with pytest.raises(RuntimeError, match="round-trip mismatch"):
        _codec_report("broken", [b"hello world" * 10], lambda d: d, bad_decompress, b"hello world")


# --- throughput units: decompress must rate by UNCOMPRESSED bytes ----------
#
# `_timed` rates by `len(args[0])` unless told otherwise. A decompress call's
# `args[0]` is the COMPRESSED bytes it consumes, not the uncompressed bytes it
# produces -- rating by that silently reports decompress throughput in a
# different unit from compress throughput ("compressed MB/s consumed" vs
# "uncompressed MB/s consumed"), under-reporting decompress by exactly the
# compression factor. `_codec_report` must pass `size_bytes=len(sample)` to
# fix the unit.

def test_timed_size_bytes_overrides_args0_length():
    # Direct formula check: with size_bytes given, the rate is computed from
    # size_bytes, not from len(args[0]) -- construct them to disagree by 100x
    # so a wrong wiring is unmissable.
    compressed_stub = b"c" * 10          # what the timed call is invoked WITH
    uncompressed_size = 1000             # what it should be RATED by

    _, mb_s = _timed(lambda c: b"x", compressed_stub, size_bytes=uncompressed_size,
                     min_s=0.01)
    _, mb_s_default = _timed(lambda c: b"x", compressed_stub, min_s=0.01)

    # Same call, same wall time budget, ~100x different rating input -> the
    # size_bytes-driven rate must be far larger than the args0-driven one.
    assert mb_s > mb_s_default * 20


def test_codec_report_times_decompress_by_the_uncompressed_sample_size(monkeypatch):
    """Pins the actual fix site: `_codec_report`'s decompress `_timed` call
    must pass `size_bytes=len(sample)`, not leave it defaulting to the
    compressed bytes decompress is invoked with.

    Proved discriminating by reintroducing the defect (manually, not shipped):
    deleting `size_bytes=len(sample)` from `_codec_report`'s decompress
    `_timed` call makes `decompress_call["size_bytes"]` read `None` here,
    failing this assertion immediately.
    """
    import tools.capture_compression as cc

    calls = []

    def fake_timed(fn, *args, min_s=None, max_iters=None, size_bytes=None):
        calls.append({"args0_len": len(args[0]), "size_bytes": size_bytes})
        return fn(*args), 42.0

    monkeypatch.setattr(cc, "_timed", fake_timed)

    sample = b"hello world" * 50

    def compress(_d):
        return b"C" * 5  # deliberately much shorter than `sample`

    def decompress(_c, _n):
        return sample

    report = _codec_report("x", [sample], compress, decompress, sample)

    assert len(calls) == 2, "expected one _timed call for compress, one for decompress"
    compress_call, decompress_call = calls
    assert compress_call["size_bytes"] is None      # rides the default: len(args[0]) == len(sample)
    assert compress_call["args0_len"] == len(sample)
    assert decompress_call["args0_len"] == 5, "decompress is called WITH the compressed bytes"
    assert decompress_call["size_bytes"] == len(sample), (
        "decompress throughput must be rated by the UNCOMPRESSED sample size "
        "(what it produces), not the compressed bytes it consumes -- otherwise "
        "decompress_mb_s under-reports by exactly the compression factor")
    assert report["decompress_mb_s"] == 42.0


# --- shape / plumbing -------------------------------------------------------

def test_json_shape_and_plausible_ranges(tmp_path):
    frames = _correlated_high_noisy_low(n_frames=10, samples_per_frame=100, seed=1)
    path = _write(tmp_path, frames)

    report = measure_captures([path])

    assert set(report.keys()) >= {"zstd_level", "zlib_level", "captures", "pooled"}
    assert path in report["captures"]
    s = report["pooled"]["RAW_3DMD"]
    assert s["n_frames"] == 10
    assert s["raw_bytes_total"] == 10 * 200  # 100 samples * 2 bytes each
    for codec in ("lz4", "lz4_deinterleaved", "zstd", "zlib"):
        c = s[codec]
        assert 0.0 < c["pct_of_original"] < 200.0
        assert c["factor"] > 0.0
        assert c["compress_mb_s"] > 0.0
        assert c["decompress_mb_s"] > 0.0
    m = s["m33_footprint_estimate"]
    assert m["state_bytes"] == LZ4_STREAM_STATE_BYTES
    assert m["total_bytes"] > LZ4_STREAM_STATE_BYTES


def test_streams_filter_restricts_to_named_streams(tmp_path):
    raw_frames = _correlated_high_noisy_low(n_frames=5, samples_per_frame=40, seed=2)
    env_frames = [_frame(i, 10, bytes(range(20)), t_us=i * 33_000) for i in range(5)]
    interleaved = []
    for r, e in zip(raw_frames, env_frames):
        interleaved.append(r)
        interleaved.append(e)
    path = _write(tmp_path, interleaved)

    report = measure_captures([path], streams=["RAW_3DMD"])

    assert list(report["pooled"].keys()) == ["RAW_3DMD"]


def test_max_frames_per_stream_caps_sampling(tmp_path):
    frames = _correlated_high_noisy_low(n_frames=30, samples_per_frame=20, seed=4)
    path = _write(tmp_path, frames)

    report = measure_captures([path], max_frames_per_stream=7)

    assert report["pooled"]["RAW_3DMD"]["n_frames"] == 7


def test_pooled_combines_multiple_captures(tmp_path):
    a = _write(tmp_path, _correlated_high_noisy_low(8, 30, seed=1), name="a.bin")
    b = _write(tmp_path, _correlated_high_noisy_low(5, 30, seed=2), name="b.bin")

    report = measure_captures([a, b], max_frames_per_stream=100)

    assert report["pooled"]["RAW_3DMD"]["n_frames"] == 13
    assert (report["captures"][a]["RAW_3DMD"]["n_frames"]
           + report["captures"][b]["RAW_3DMD"]["n_frames"]) == 13


# --- CLI smoke ---------------------------------------------------------

def test_cli_json_smoke(tmp_path):
    frames = _correlated_high_noisy_low(n_frames=6, samples_per_frame=30, seed=5)
    path = _write(tmp_path, frames)

    out = subprocess.run(
        [sys.executable, str(REPO / "host" / "tools" / "capture_compression.py"),
         path, "--json"],
        cwd=REPO / "host", capture_output=True, text=True, timeout=60)

    assert out.returncode == 0, out.stderr
    data = json.loads(out.stdout)
    assert data["pooled"]["RAW_3DMD"]["n_frames"] == 6


def test_cli_table_smoke(tmp_path):
    frames = _correlated_high_noisy_low(n_frames=6, samples_per_frame=30, seed=5)
    path = _write(tmp_path, frames)

    out = subprocess.run(
        [sys.executable, str(REPO / "host" / "tools" / "capture_compression.py"), path],
        cwd=REPO / "host", capture_output=True, text=True, timeout=60)

    assert out.returncode == 0, out.stderr
    assert "RAW_3DMD" in out.stdout
    assert "lz4_deinterleaved" in out.stdout
    assert "pooled" in out.stdout
