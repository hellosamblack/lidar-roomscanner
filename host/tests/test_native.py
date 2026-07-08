import numpy as np
import pytest

from roomscan.native import Transform
from tests.golden import load_golden_pairs

pytestmark = pytest.mark.skipif(not Transform.available(),
                                reason="native transform DLL not built")


def test_create_process_destroy_smoke():
    calib, pairs = load_golden_pairs()
    t = Transform(calib)
    depth = t.process(pairs[0][0])["depth"]
    assert depth.shape == (42, 54) and depth.dtype == np.float32
    assert np.isfinite(depth).all()
