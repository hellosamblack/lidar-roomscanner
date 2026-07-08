"""PC-vs-MCU transform equivalence gate.

The go/no-go decision point for removing the on-MCU transform: does the PC-side
Transform (roomscan.native, wrapping the same vl53l9-transform-c pipeline) produce
depth output that matches what the MCU actually emitted for the same raw 3DMD
frames, within the plan's 0.01 mm tolerance?

Uses the committed golden-pair fixture (3 pairs, hardware-captured, from the
stream's true frame 1 so the TNR temporal-noise-reduction filter's internal
state is aligned start-to-start). See host/tests/sweep_golden_capture.py for
the full 731-pair capture sweep (not a pytest test -- needs the gitignored
full capture file).
"""
import numpy as np
import pytest

from roomscan.native import Transform
from tests.golden import load_golden_pairs

pytestmark = pytest.mark.skipif(not Transform.available(),
                                reason="native transform DLL not built")


def test_pc_transform_matches_mcu_output():
    calib, pairs = load_golden_pairs()
    assert len(pairs) >= 3
    t = Transform(calib)
    exact = 0
    for i, (raw, depth_mcu) in enumerate(pairs):   # capture order — TNR is stateful
        depth_pc = t.process(raw)["depth"]
        mcu = np.frombuffer(depth_mcu, dtype="<f4").reshape(42, 54)
        if np.array_equal(depth_pc, mcu):
            exact += 1
        assert np.allclose(depth_pc, mcu, atol=0.01, equal_nan=True), \
            f"frame {i}: max abs diff {np.nanmax(np.abs(depth_pc - mcu))} mm"
    print(f"\nexact-match frames: {exact}/{len(pairs)}")
