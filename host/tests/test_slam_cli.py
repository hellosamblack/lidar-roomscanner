import numpy as np
from pathlib import Path
from roomscan.slam.cli import main
from roomscan.slam import cli as slamcli

def test_cli_runs_on_synthetic_capture(tmp_path, monkeypatch):
    # Build a 3-frame synthetic (depth, quat, pressure) stream and monkeypatch the
    # loader so the CLI logic is exercised without a real .bin. See _load_frames.
    frames = [(np.full((42, 54), 1000.0 + 5 * i, np.float32),
               (1.0, 0.0, 0.0, 0.0), 101325.0, float(i) * 0.03) for i in range(3)]
    monkeypatch.setattr(slamcli, "_load_frames", lambda path, max_frames=None: (frames, 54, 42))
    rc = main([str(tmp_path / "dummy.bin"), "--out-mesh", str(tmp_path / "m.ply"),
               "--out-traj", str(tmp_path / "t.tum")])
    assert rc == 0
    assert (tmp_path / "t.tum").exists()
