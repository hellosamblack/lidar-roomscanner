import numpy as np
from pathlib import Path
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
