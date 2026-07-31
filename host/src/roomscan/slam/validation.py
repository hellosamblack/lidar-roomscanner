"""Evidence gate for optional Detailed-SLAM loop closure.

The gate deliberately accepts recorded ensemble summaries rather than a single
run: frame-to-model tracking is chaotic at the centimetre scale. Keeping this
math separate makes the acceptance rule testable and lets the expensive GPU
runner report structured evidence without reimplementing statistics.
"""
from __future__ import annotations

import numpy as np


def paired_loop_gate(baseline: list[dict], closed: list[dict], *, samples: int = 10000) -> dict:
    """Assess matched no-loop/loop-closure runs.

    Each item supplies ``horizontal_closure_m``, ``lost`` and optional ``died``.
    A positive delta means the global pass reduced closure error. The returned
    95% percentile interval is intentionally paired, preserving the same small
    perturbation in each arm.
    """
    if len(baseline) != len(closed) or not baseline:
        return {"accepted": False, "reason": "need equally sized non-empty matched ensembles"}
    try:
        delta = np.asarray([float(a["horizontal_closure_m"]) - float(b["horizontal_closure_m"])
                            for a, b in zip(baseline, closed)], dtype=np.float64)
    except (KeyError, TypeError, ValueError):
        return {"accepted": False, "reason": "runs need numeric horizontal_closure_m"}
    tracking_ok = all(not bool(b.get("died")) and int(b.get("lost", 0)) <= int(a.get("lost", 0))
                      for a, b in zip(baseline, closed))
    rng = np.random.default_rng(20260731)
    draws = delta[rng.integers(0, len(delta), size=(max(1000, int(samples)), len(delta)))].mean(axis=1)
    ci = np.percentile(draws, [2.5, 97.5])
    accepted = bool(ci[0] > 0.0 and tracking_ok)
    return {"accepted": accepted, "n": int(len(delta)),
            "mean_improvement_m": float(delta.mean()),
            "ci95_m": [float(ci[0]), float(ci[1])], "tracking_ok": tracking_ok,
            "reason": ("positive paired 95% CI with no tracking regression" if accepted
                       else "CI must be positive and loop closure must not die or add loss")}
