"""Motion primitives shared by the SLAM pipeline and the web display path.

`coherence` started life in `slam/motion.py` (the SLAM translation stationarity
gate) but is a general discriminator between real motion and zero-mean jitter,
and the web server needs it to de-jitter the gravity-aligned display. It lives
here rather than in the `slam` subpackage because importing `roomscan.slam`
executes its `__init__`, which pulls in Open3D — and `web.py` must start on a
box without it (all of web.py's other slam imports are deliberately local, for
exactly this reason). Same hoist that gave `reader.py` its neutral home in
Web Phase 5.

`slam.motion` re-exports this, so SLAM code and its tests are unaffected.
"""
from __future__ import annotations

import numpy as np


def coherence(increments) -> float:
    """Directional coherence of a sequence of 3D increments:
    ``||sum(inc)|| / sum(||inc||)``.

    - ~1.0 for consistent straight-line motion (increments reinforce),
    - ~1/sqrt(N) for N zero-mean random jitter steps (increments cancel),
    - 0.0 for no motion at all (all increments zero),
    - 1.0 for an empty set (no evidence of jitter -> treat as "moving", i.e.
      never suppress on no data).

    The increments can be displacements (SLAM translation gate) or rotation
    vectors (the web display's orientation smoother) — the statistic only cares
    that they are 3-vectors that add.
    """
    inc = np.asarray(increments, dtype=np.float64).reshape(-1, 3)
    if inc.shape[0] == 0:
        return 1.0
    path = float(np.linalg.norm(inc, axis=1).sum())
    if path < 1e-9:
        return 0.0
    net = float(np.linalg.norm(inc.sum(axis=0)))
    return net / path
