import math
import struct
import zlib

import numpy as np
import pytest

from roomscan.protocol import IMU_RAW_TICK_US
from roomscan.slam.cli import main, _load_frames
from roomscan.slam import cli as slamcli
from roomscan.slam.mapper import Mapper

def test_cli_runs_on_synthetic_capture(tmp_path, monkeypatch):
    # Build a 3-frame synthetic (depth, reflectance, confidence, quat, pressure, t_s)
    # stream and monkeypatch the loader so the CLI logic is exercised without a real
    # .bin. See _load_frames. reflectance/confidence are None here (old/depth-only
    # shape) -- test_reflectance_and_confidence_are_forwarded_to_mapper below covers
    # the non-None path.
    frames = [(np.full((42, 54), 1000.0 + 5 * i, np.float32), None, None,
               (1.0, 0.0, 0.0, 0.0), 101325.0, float(i) * 0.03) for i in range(3)]
    monkeypatch.setattr(slamcli, "_load_frames", lambda path, max_frames=None, with_imu=False: (frames, 54, 42))
    rc = main([str(tmp_path / "dummy.bin"), "--out-mesh", str(tmp_path / "m.ply"),
               "--out-traj", str(tmp_path / "t.tum")])
    assert rc == 0
    assert (tmp_path / "t.tum").exists()


def test_reflectance_and_confidence_are_forwarded_to_mapper(monkeypatch):
    # _run() must pass each frame's reflectance/confidence through to
    # Mapper.step (not silently drop them) -- proven by capturing step()'s
    # actual call arguments rather than re-deriving SLAM behavior.
    from roomscan.slam.config import SlamConfig

    reflectance = np.full((42, 54), 42.0, dtype=np.float32)
    confidence = np.full((42, 54), 200.0, dtype=np.float32)
    frames = [(np.full((42, 54), 1000.0, np.float32), reflectance, confidence,
               (1.0, 0.0, 0.0, 0.0), 101325.0, 0.0)]

    seen = {}
    orig_step = Mapper.step

    def spy_step(self, depth, quat, pressure_pa=None, reflectance=None, confidence=None,
                 imu_raw=None, quat_offset_us=None):
        seen["reflectance"] = reflectance
        seen["confidence"] = confidence
        return orig_step(self, depth, quat, pressure_pa, reflectance=reflectance,
                         confidence=confidence, imu_raw=imu_raw, quat_offset_us=quat_offset_us)

    monkeypatch.setattr(Mapper, "step", spy_step)
    slamcli._run(frames, 54, 42, SlamConfig(), "translation")
    assert seen["reflectance"] is reflectance
    assert seen["confidence"] is confidence


def test_run_forwards_device_to_mapper(monkeypatch):
    # _run()'s new `device` argument (backing --device) must reach Mapper's
    # constructor -- proven by capturing the actual kwarg Mapper was built
    # with, not by re-deriving SLAM device behavior. CUDA:0 isn't testable
    # without a CUDA build, but the string plumbing itself is device-agnostic.
    from roomscan.slam.config import SlamConfig

    frames = [(np.full((42, 54), 1000.0, np.float32), None, None,
               (1.0, 0.0, 0.0, 0.0), 101325.0, 0.0)]
    seen = {}
    orig_init = Mapper.__init__

    def spy_init(self, *args, **kwargs):
        seen["device"] = kwargs.get("device")
        return orig_init(self, *args, **kwargs)

    monkeypatch.setattr(Mapper, "__init__", spy_init)
    slamcli._run(frames, 54, 42, SlamConfig(), "translation", device="CPU:0")
    assert seen["device"] == "CPU:0"


def test_run_defaults_device_from_config_when_not_given(monkeypatch):
    from roomscan.slam.config import SlamConfig

    frames = [(np.full((42, 54), 1000.0, np.float32), None, None,
               (1.0, 0.0, 0.0, 0.0), 101325.0, 0.0)]
    seen = {}
    orig_init = Mapper.__init__

    def spy_init(self, *args, **kwargs):
        seen["device"] = kwargs.get("device")
        return orig_init(self, *args, **kwargs)

    monkeypatch.setattr(Mapper, "__init__", spy_init)
    slamcli._run(frames, 54, 42, SlamConfig(), "translation")   # no device kwarg
    assert seen["device"] == SlamConfig().device == "CPU:0"


def test_cli_device_flag_parses_and_reaches_run(tmp_path, monkeypatch):
    # End-to-end through main(): --device is parsed and threaded through to
    # _run (and thus Mapper), without breaking the no-flag default path
    # exercised by test_cli_runs_on_synthetic_capture above.
    frames = [(np.full((42, 54), 1000.0 + 5 * i, np.float32), None, None,
               (1.0, 0.0, 0.0, 0.0), 101325.0, float(i) * 0.03) for i in range(3)]
    monkeypatch.setattr(slamcli, "_load_frames", lambda path, max_frames=None, with_imu=False: (frames, 54, 42))

    seen_devices = []
    orig_run = slamcli._run

    def spy_run(frames, width, height, cfg, mode, device=None, imu_aux=None):
        seen_devices.append(device)
        return orig_run(frames, width, height, cfg, mode, device=device, imu_aux=imu_aux)

    monkeypatch.setattr(slamcli, "_run", spy_run)
    rc = main([str(tmp_path / "dummy.bin"), "--device", "CPU:0",
               "--out-mesh", str(tmp_path / "m.ply"), "--out-traj", str(tmp_path / "t.tum")])
    assert rc == 0
    assert seen_devices == ["CPU:0"]


def test_cli_help_expands(capsys):
    """argparse %-expands help strings, so a bare "% o" in one of them (as in
    "~97% of its capacity") is read as an octal conversion and `--help`
    crashes with a TypeError. That shipped and went unnoticed because nothing
    exercised --help; this pins it."""
    import pytest
    with pytest.raises(SystemExit):
        main(["--help"])
    assert "--baro-authority" in capsys.readouterr().out


def test_cli_baro_authority_flag_reaches_the_mapper(tmp_path, monkeypatch):
    """BUG-037: the authority knob exists so the default can be RE-measured, so
    it has to arrive at Mapper -- a flag that only lands on `cfg` would sweep
    nothing. Also pins the no-flag path to the config default."""
    frames = [(np.full((42, 54), 1000.0 + 5 * i, np.float32), None, None,
               (1.0, 0.0, 0.0, 0.0), 101325.0, float(i) * 0.03) for i in range(3)]
    monkeypatch.setattr(slamcli, "_load_frames", lambda path, max_frames=None, with_imu=False: (frames, 54, 42))
    seen = []
    orig_init = Mapper.__init__

    def spy_init(self, *args, **kwargs):
        seen.append((kwargs.get("baro_authority"), kwargs.get("baro_tau_frames")))
        return orig_init(self, *args, **kwargs)

    monkeypatch.setattr(Mapper, "__init__", spy_init)
    common = ["--out-mesh", str(tmp_path / "m.ply"), "--out-traj", str(tmp_path / "t.tum")]
    assert main([str(tmp_path / "d.bin"), "--baro-authority", "0"] + common) == 0
    assert seen[-1] == (0.0, 900)
    assert main([str(tmp_path / "d.bin")] + common) == 0
    assert seen[-1] == (0.05, 900)


def test_cli_icp_device_flag_reaches_the_mapper_and_the_report(tmp_path, monkeypatch):
    """Item 5 (2026-08-02). Two things a device selector can silently fail at,
    both of which look exactly like "the change had no effect": never reaching
    the constructor, and never being written down. `--icp-device` must land on
    `Mapper` AND on the JSON report, because a run with a different ICP index
    device is not comparable with one without it.

    Also pins the no-flag path to the `[slam]` default, which is the value
    every existing install will get."""
    import json
    frames = [(np.full((42, 54), 1000.0 + 5 * i, np.float32), None, None,
               (1.0, 0.0, 0.0, 0.0), 101325.0, float(i) * 0.03) for i in range(3)]
    monkeypatch.setattr(slamcli, "_load_frames", lambda path, max_frames=None, with_imu=False: (frames, 54, 42))
    seen = []
    orig_init = Mapper.__init__

    def spy_init(self, *args, **kwargs):
        seen.append(kwargs.get("icp_device"))
        return orig_init(self, *args, **kwargs)

    monkeypatch.setattr(Mapper, "__init__", spy_init)
    out = tmp_path / "r.json"
    common = ["--out-mesh", str(tmp_path / "m.ply"), "--out-traj", str(tmp_path / "t.tum"),
              "--json", str(out)]
    assert main([str(tmp_path / "d.bin"), "--icp-device", "CPU:0"] + common) == 0
    assert seen[-1] == "CPU:0"
    assert json.loads(out.read_text())["icp_device"] == "CPU:0"
    assert main([str(tmp_path / "d.bin")] + common) == 0
    from roomscan.slam.config import SlamConfig
    assert seen[-1] == SlamConfig.load().icp_device


def test_run_forwards_every_configured_mapper_knob(monkeypatch):
    """`_run` used to re-list all eighteen `Mapper` knobs by hand -- the same
    second-construction-site shape as BUG-062, and item 5's `icp_device` would
    have had to be remembered in it. Set every shared field off its default and
    prove each one arrives."""
    from roomscan.slam.config import SlamConfig

    overrides = dict(
        voxel_size=0.02, max_dist=0.07, icp_retry_dist=0.19, max_iter=11,
        min_fitness=0.44, max_rmse=0.066, min_confidence=33.0,
        weight_threshold=4.5, baro_authority=0.11, baro_tau_frames=450,
        stationary_hold=False, stationary_window=17, stationary_coherence=0.71,
        stationary_step_ceiling=0.041, stationary_rot_ceiling=0.55,
        release_cache_every=3, block_count=222_000, fov_h=51.0, fov_v=39.0,
        # A device that does not exist here: `o3d.core.Device` resolves the
        # string without touching the driver, and this frame is a bootstrap
        # (no ICP call), so nothing ever allocates on it. Proves the value is
        # forwarded verbatim rather than re-derived from `device`.
        icp_device="CUDA:3",
    )
    stock = SlamConfig()
    assert all(getattr(stock, k) != v for k, v in overrides.items())

    frames = [(np.full((42, 54), 1000.0, np.float32), None, None,
               (1.0, 0.0, 0.0, 0.0), 101325.0, 0.0)]
    seen = {}
    orig_init = Mapper.__init__

    def spy_init(self, *args, **kwargs):
        seen.update(kwargs)
        # `fov_h`/`fov_v` were positional before; a regression that reverted to
        # positional args would leave them out of `kwargs` and be caught below.
        return orig_init(self, *args, **kwargs)

    monkeypatch.setattr(Mapper, "__init__", spy_init)
    slamcli._run(frames, 54, 42, SlamConfig(**overrides), "translation")
    missing = {k: (v, seen.get(k)) for k, v in overrides.items() if seen.get(k) != v}
    assert not missing, f"[slam] keys the offline CLI ignores: {missing}"
    # `icp_mode` is the one deliberate override: --compare-modes calls _run per mode.
    assert seen["icp_mode"] == "translation"


# --- #155: timestamp interpolation in _load_frames -------------------------------------
#
# Synthetic framed captures prove the ASSOCIATION, not just the math (the math is
# tests/test_sensor_time.py's job): the group's stream 9/13 arrive AFTER its depth
# payload on the wire, so at depth time the carried-forward quat belongs to the
# PREVIOUS group — pairing must be by exact (seq, t_us), and the interpolation must
# land each frame's quat at that frame's OWN frame-ready instant.

_PERIOD_TICKS = 1536          # ~33.3 ms at the nominal 21.7 us tick
_LEAD_TICKS = 358             # ~7.77 ms quat-midpoint lead (BUG-031's shape)
_BASE_TICK = 100_000
_DEG_PER_FRAME = 10.0


def _wire_frame(stream_id, payload, seq, t_us, w=0, h=0):
    header = struct.pack("<4sBBBBIQHHII", b"RSCN", 1, 1, stream_id, 0,
                         seq, t_us, w, h, len(payload), 0)
    return header + payload + zlib.crc32(header + payload).to_bytes(4, "little")


def _qz(deg):
    half = math.radians(deg) / 2.0
    return (math.cos(half), 0.0, 0.0, math.sin(half))


def _qz_deg(q):
    return math.degrees(2.0 * math.atan2(q[3], q[0]))


def _angle_at(tick):
    return (tick - _BASE_TICK) / _PERIOD_TICKS * _DEG_PER_FRAME


def _group(k, *, with_quat=True, with_sync=True, quat_deg=None):
    """One coincident ToF group: DEPTH_ZF32 first, then its stream 9 and 13 —
    the real firmware wire order (#155's whole association problem)."""
    seq, t_us = 42 + k, 1_000_000 + 33_333 * k
    fr = _BASE_TICK + _PERIOD_TICKS * k          # frame-ready instant, LSM ticks
    depth = np.full((4, 4), 1000.0, dtype="<f4").tobytes()
    out = _wire_frame(0, depth, seq, t_us, w=4, h=4)  # stream 0 = DEPTH_ZF32
    if with_quat:
        deg = _angle_at(fr + _LEAD_TICKS) if quat_deg is None else quat_deg
        out += _wire_frame(9, struct.pack("<4f", *_qz(deg)), seq, t_us)
    if with_sync:
        # latch 217 us after the edge = exactly 10 nominal ticks
        payload = struct.pack("<IIIIHHBB", fr + 10, 217, 24_109,
                              fr + _LEAD_TICKS, 44, 15, 1, 0)
        out += _wire_frame(13, payload, seq, t_us)
    return out


def _capture(tmp_path, blob, name="cap.bin"):
    p = tmp_path / name
    p.write_bytes(blob)
    return str(p)


def test_interp_aligns_each_frame_to_its_own_frame_ready_instant(tmp_path):
    """Rotation is linear in LSM time, so the interpolated quat at frame k's
    frame-ready instant must read exactly 10k deg — while the legacy carry-forward
    hands frame k the PREVIOUS group's midpoint (10(k-1)+2.33 deg). A hard-coded
    7.76 ms would also pass here; the reflected/no-13 tests below break that tie."""
    path = _capture(tmp_path, b"".join(_group(k) for k in range(4)))
    frames, w, h, aux = _load_frames(path, with_imu=True, quat_interp=True)
    assert (w, h) == (4, 4) and len(frames) == 4
    stats = aux.interp_stats
    assert stats == {"mode": "on", "frames": 4, "timed_samples": 4,
                     "eligible": 4, "applied": 3}
    for k in (1, 2, 3):
        assert _qz_deg(frames[k][3]) == pytest.approx(_DEG_PER_FRAME * k, abs=1e-6)
        assert aux[k][1] is None, "interpolated frame must not also get the fixed rollback"
    # frame 0 has no left bracket: keeps the carried-forward quat (identity here)
    assert _qz_deg(frames[0][3]) == pytest.approx(0.0, abs=1e-9)


def test_interp_off_keeps_legacy_carry_forward(tmp_path):
    """The default path must stay byte-identical: frame k carries the previous
    group's midpoint quat, and the legacy fixed-rollback offset stays populated."""
    path = _capture(tmp_path, b"".join(_group(k) for k in range(4)))
    frames, _, _, aux = _load_frames(path, with_imu=True)
    assert getattr(aux, "interp_stats", None) is None
    for k in (1, 2, 3):
        expected = _angle_at(_BASE_TICK + _PERIOD_TICKS * (k - 1) + _LEAD_TICKS)
        assert _qz_deg(frames[k][3]) == pytest.approx(expected, abs=1e-6)
        # (mid - frame_ready) x tick: the 217 us latch back-out is already inside
        # frame_ready_ticks, so the offset is exactly the 358-tick lead
        assert aux[k][1] == pytest.approx(_LEAD_TICKS * IMU_RAW_TICK_US, abs=1e-6)


def test_reflected_mode_mirrors_the_query_direction(tmp_path):
    """The validation-only null arm must shift the SAME magnitude the WRONG way:
    frame k reads the orientation at mid + (mid - frame_ready), i.e. 10k + 4.66 deg
    here. If an implementation ignored timestamps and rolled back a constant, 'on'
    and 'reflected' could not differ like this."""
    path = _capture(tmp_path, b"".join(_group(k) for k in range(4)))
    frames, _, _, aux = _load_frames(path, with_imu=True, quat_interp="reflected")
    stats = aux.interp_stats
    assert stats["mode"] == "reflected"
    mirror = 2.0 * _LEAD_TICKS / _PERIOD_TICKS * _DEG_PER_FRAME  # 4.6615 deg
    # coverage shifts one frame vs 'on': frame 0's mirrored target sits AFTER its
    # own midpoint (bracketed), frame 3's sits beyond the newest sample (refused)
    for k in (0, 1, 2):
        assert _qz_deg(frames[k][3]) == pytest.approx(_DEG_PER_FRAME * k + mirror, abs=1e-6)
    assert stats["applied"] == 3
    assert _qz_deg(frames[3][3]) == pytest.approx(
        _angle_at(_BASE_TICK + _PERIOD_TICKS * 2 + _LEAD_TICKS), abs=1e-6)


def test_no_stream13_capture_is_untouched(tmp_path):
    """Legacy captures (pre-2026-07-30) must decode identically with the lever on:
    zero eligible frames, zero applied, quats equal to the interp-off load."""
    blob = b"".join(_group(k, with_sync=False) for k in range(3))
    path = _capture(tmp_path, blob)
    on = _load_frames(path, with_imu=True, quat_interp=True)
    off = _load_frames(path, with_imu=True)
    assert on[3].interp_stats == {"mode": "on", "frames": 3, "timed_samples": 0,
                                  "eligible": 0, "applied": 0}
    for fon, foff in zip(on[0], off[0]):
        assert fon[3] == foff[3]
    assert [a[1] for a in on[3]] == [a[1] for a in off[3]] == [None, None, None]


def test_offcycle_stream9_without_sync_is_not_a_timed_sample(tmp_path):
    """Decoupled-mode regression: an off-cycle stream 9 reusing the group's seq
    (different t_us, no stream 13) must not poison the timed samples — even a
    180-deg outlier. It may only affect the legacy carry-forward, exactly as
    before #155."""
    groups = [_group(k) for k in range(4)]
    rogue = _wire_frame(9, struct.pack("<4f", *_qz(180.0)), 42 + 1, 999_999_999)
    blob = b"".join(groups[:2]) + rogue + b"".join(groups[2:])
    path = _capture(tmp_path, blob)
    frames, _, _, aux = _load_frames(path, with_imu=True, quat_interp=True)
    assert aux.interp_stats["applied"] == 3
    for k in (1, 2, 3):
        assert _qz_deg(frames[k][3]) == pytest.approx(_DEG_PER_FRAME * k, abs=1e-6)


def test_missing_sync_frame_falls_back_per_frame(tmp_path):
    """One group with no stream 13: that frame keeps the legacy quat AND the legacy
    fixed-rollback offset (the old mechanism stays its fallback); its neighbours
    still interpolate — group 3 bridges the missing sample (2 frame periods is
    within the span guard)."""
    blob = b"".join(_group(k, with_sync=(k != 2)) for k in range(4))
    path = _capture(tmp_path, blob)
    frames, _, _, aux = _load_frames(path, with_imu=True, quat_interp=True)
    stats = aux.interp_stats
    assert stats["eligible"] == 3 and stats["applied"] == 2 and stats["timed_samples"] == 3
    assert _qz_deg(frames[1][3]) == pytest.approx(_DEG_PER_FRAME, abs=1e-6)
    assert _qz_deg(frames[3][3]) == pytest.approx(_DEG_PER_FRAME * 3, abs=1e-6)
    # frame 2: not eligible (no own-group 13) -> legacy quat (group 1's midpoint,
    # because its own group's stream 9 arrives after the depth payload) + legacy offset
    assert _qz_deg(frames[2][3]) == pytest.approx(
        _angle_at(_BASE_TICK + _PERIOD_TICKS * 1 + _LEAD_TICKS), abs=1e-6)
    assert aux[2][1] is not None and aux[2][1] > 0


def test_malformed_sync_degrades_that_frame_only(tmp_path):
    """A truncated stream-13 payload must not abort depth decoding (the standing
    malformed-aux contract) and must not become a timed sample."""
    good = b"".join(_group(k) for k in (0, 1))
    seq, t_us = 42 + 2, 1_000_000 + 33_333 * 2
    bad_group = _group(2, with_sync=False) + _wire_frame(13, b"\x00" * 10, seq, t_us)
    blob = good + bad_group + _group(3)
    frames, _, _, aux = _load_frames(_capture(tmp_path, blob),
                                     with_imu=True, quat_interp=True)
    assert len(frames) == 4
    assert aux.interp_stats["eligible"] == 3
    assert _qz_deg(frames[3][3]) == pytest.approx(_DEG_PER_FRAME * 3, abs=1e-6)


def test_max_frames_still_sees_the_last_frames_trailing_group(tmp_path):
    """The pre-#155 loader broke out of the parse the moment the Nth depth frame
    landed — before that frame's own stream 9/13 (which follow it on the wire)
    were seen, so the Nth frame could never be aligned. The drain must let the
    final group complete without decoding a 5th frame."""
    path = _capture(tmp_path, b"".join(_group(k) for k in range(4)))
    frames, _, _, aux = _load_frames(path, max_frames=2, with_imu=True, quat_interp=True)
    assert len(frames) == 2
    assert aux.interp_stats["applied"] == 1
    assert _qz_deg(frames[1][3]) == pytest.approx(_DEG_PER_FRAME, abs=1e-6)
    # and the legacy path's early break is preserved verbatim
    frames_off, _, _, _ = _load_frames(path, max_frames=2, with_imu=True)
    assert len(frames_off) == 2
