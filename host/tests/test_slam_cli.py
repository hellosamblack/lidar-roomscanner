import numpy as np
from roomscan.slam.cli import main
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
    monkeypatch.setattr(slamcli, "_load_frames", lambda path, max_frames=None: (frames, 54, 42))
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

    def spy_step(self, depth, quat, pressure_pa=None, reflectance=None, confidence=None):
        seen["reflectance"] = reflectance
        seen["confidence"] = confidence
        return orig_step(self, depth, quat, pressure_pa, reflectance=reflectance, confidence=confidence)

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
    monkeypatch.setattr(slamcli, "_load_frames", lambda path, max_frames=None: (frames, 54, 42))

    seen_devices = []
    orig_run = slamcli._run

    def spy_run(frames, width, height, cfg, mode, device=None):
        seen_devices.append(device)
        return orig_run(frames, width, height, cfg, mode, device=device)

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
    monkeypatch.setattr(slamcli, "_load_frames", lambda path, max_frames=None: (frames, 54, 42))
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
    monkeypatch.setattr(slamcli, "_load_frames", lambda path, max_frames=None: (frames, 54, 42))
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
