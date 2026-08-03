"""Guards on the ICP study tool (`host/tools/slam_icp_bench.py`).

Two things here can quietly lie, and both would produce the SAME output as a
correct result -- "no difference":

1. the candidate solver could be wrong in a way that only shows up as drift, so
   it is pinned against the shipped solver on identical inputs;
2. the monkey-patch could silently not apply, so the shim's call counter is
   pinned too.

Everything runs on CPU:0. The tool's conclusions are CUDA-specific, but its
arithmetic is not, and the suite must not require a GPU.
"""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import pytest

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "host" / "tools"))

o3d = pytest.importorskip("open3d")
import slam_icp_bench as bench                                    # noqa: E402
from roomscan.slam import odometry                                # noqa: E402

CPU = o3d.core.Device("CPU:0")


def _scene(seed=7, n=400):
    """A source cloud and a target plane-ish cloud with normals, offset by a
    known translation, so both solvers have something real to converge on."""
    rng = np.random.default_rng(seed)
    xy = rng.uniform(-0.5, 0.5, size=(n, 2))
    z = 1.0 + 0.05 * np.sin(6.0 * xy[:, 0]) + 0.05 * np.cos(5.0 * xy[:, 1])
    tgt = np.column_stack([xy, z]).astype(np.float64)
    nrm = np.tile(np.array([0.0, 0.0, -1.0]), (n, 1))
    nrm = (nrm + 0.15 * rng.standard_normal((n, 3)))
    nrm /= np.linalg.norm(nrm, axis=1, keepdims=True)
    src = tgt - np.array([0.004, -0.003, 0.006])          # known offset
    return src, tgt, nrm


def _scene_with_misses(seed=11, n=600, miss_frac=0.4):
    """Same, but with `miss_frac` of the SOURCE displaced far past `max_dist`.

    The full-match scene above cannot separate a weighting bug from a correct
    implementation: with every point matched, the zero-weight path never runs.
    Demonstrated -- dropping the weight from the normal-equations left factor
    moves the answer 0.0 m on `_scene()` and 3.6 mm here. This is the scene the
    weighting exists for, so it is the one the equivalence test must use."""
    src, tgt, nrm = _scene(seed=seed, n=n)
    rng = np.random.default_rng(seed + 1)
    miss = rng.random(n) < miss_frac
    src = src.copy()
    src[miss] += np.array([0.0, 0.0, 3.0])
    return src, tgt, nrm


def _clouds(src, tgt, nrm):
    s = o3d.t.geometry.PointCloud(CPU)
    s.point.positions = o3d.core.Tensor(src, device=CPU)
    t = o3d.t.geometry.PointCloud(CPU)
    t.point.positions = o3d.core.Tensor(tgt, device=CPU)
    t.point.normals = o3d.core.Tensor(nrm, device=CPU)
    return s, t


@pytest.mark.parametrize("scene", [_scene, _scene_with_misses])
def test_gpu_variant_matches_the_shipped_translation_solve(scene):
    """Identical inputs -> identical answer, to float64 round-off.

    This is the check that can SEPARATE a sign / column-mapping / weighting
    error from round-off: an algorithmic difference lands at 1e-3..1e0 m,
    round-off at ~1e-15 m. A trajectory comparison cannot, because SLAM chaos is
    larger than both.

    Both scenes are run because the partial-match one is the only one with any
    power over the zero-weighting: measured on the full-match scene, dropping
    the weight from the left factor changes the answer by exactly 0.0 m."""
    src, tgt, nrm = scene()
    s, t = _clouds(src, tgt, nrm)
    init = np.eye(4)
    gate = dict(max_dist=0.05, min_fitness=0.3, max_rmse=0.05, max_iter=6)
    a = odometry.register(s, t, init, mode="translation", device=CPU, **gate)
    b = bench.register_gpu(s, t, init, device=CPU, **gate)
    assert np.linalg.norm(a.pose[:3, 3] - b.pose[:3, 3]) < 1e-12
    assert a.fitness == pytest.approx(b.fitness, abs=1e-12)
    assert a.rmse == pytest.approx(b.rmse, abs=1e-12)
    assert a.ok == b.ok
    # ...and it actually solved something, rather than both returning the init.
    assert np.linalg.norm(a.pose[:3, 3]) > 1e-4


def test_the_equivalence_scene_actually_contains_misses():
    """Guard the guard: if `_scene_with_misses` ever stopped producing
    unmatched points, the parametrised test above would silently lose the only
    coverage it has of the zero-weight path and still pass."""
    src, tgt, nrm = _scene_with_misses()
    s, t = _clouds(src, tgt, nrm)
    r = odometry.register(s, t, np.eye(4), mode="translation", device=CPU,
                          max_dist=0.05, min_fitness=0.3, max_rmse=0.05, max_iter=6)
    assert 0.3 < r.fitness < 0.9, f"expected genuine misses, got fitness {r.fitness}"


def test_gpu_variant_reports_no_match_like_the_shipped_one():
    """A target far outside `max_dist` must degrade the same way in both."""
    src, tgt, nrm = _scene()
    s, t = _clouds(src + 10.0, tgt, nrm)
    gate = dict(max_dist=0.05, min_fitness=0.3, max_rmse=0.05, max_iter=6)
    a = odometry.register(s, t, np.eye(4), mode="translation", device=CPU, **gate)
    b = bench.register_gpu(s, t, np.eye(4), device=CPU, **gate)
    assert a.ok is False and b.ok is False
    assert a.fitness == pytest.approx(b.fitness)


def test_shim_installs_routes_and_counts():
    """The patch must be provably live: a silent no-op would read as 'no
    difference', which is what a correct variant also reads as."""
    bench.SHIM_CALLS.clear()
    bench.install_variant()
    try:
        assert odometry.register is not bench._ORIGINAL_REGISTER
        src, tgt, nrm = _scene()
        s, t = _clouds(src, tgt, nrm)
        odometry.register(s, t, np.eye(4), mode="gpu_translation", device=CPU)
        odometry.register(s, t, np.eye(4), mode="translation_cpu_nns", device=CPU)
        odometry.register(s, t, np.eye(4), mode="translation", device=CPU)
        assert bench.SHIM_CALLS == {"gpu_translation": 1, "translation_cpu_nns": 1,
                                    "translation": 1}
    finally:
        bench.uninstall_variant()
    assert odometry.register is bench._ORIGINAL_REGISTER


def test_paired_equivalence_is_non_inferiority_not_improvement():
    """A candidate that is merely *different* by less than the baseline's own
    spread passes; one that shifts closure well beyond it fails -- in EITHER
    direction, unlike `paired_loop_gate`, which only rewards improvement."""
    rng = np.random.default_rng(3)
    base = [{"horizontal_closure_m": float(x), "lost": 0, "died": False}
            for x in 0.7 + 0.15 * rng.standard_normal(10)]
    same = [{"horizontal_closure_m": r["horizontal_closure_m"] + 1e-9,
             "lost": 0, "died": False} for r in base]
    worse = [{"horizontal_closure_m": r["horizontal_closure_m"] + 1.0,
              "lost": 0, "died": False} for r in base]
    better = [{"horizontal_closure_m": r["horizontal_closure_m"] - 1.0,
               "lost": 0, "died": False} for r in base]

    ok = bench.paired_equivalence(base, same)
    assert ok["accepted"] is True
    # The tolerance is not a constant in the source -- it is read off the data.
    assert ok["tolerance_m"] == pytest.approx(float(np.std(
        [r["horizontal_closure_m"] for r in base], ddof=1)))
    assert bench.paired_equivalence(base, worse)["accepted"] is False
    assert bench.paired_equivalence(base, better)["accepted"] is False


def test_paired_equivalence_rejects_a_run_that_died():
    base = [{"horizontal_closure_m": 0.7, "lost": 0, "died": False} for _ in range(5)]
    dead = [{"horizontal_closure_m": 0.7, "lost": 0, "died": i == 0} for i in range(5)]
    r = bench.paired_equivalence(base, dead)
    assert r["accepted"] is False and r["tracking_ok"] is False


# ---- item 5: the interleaved paired A/B pass (`--what ab`) ------------------

def test_ab_arm_asserts_the_knob_took_effect(monkeypatch):
    """`_ab_arm` must refuse to time a `Mapper` whose `icp_device` is not the
    one it asked for.

    This is the whole reason the assertion exists: the change under test is
    bit-identical by design, so a knob that silently failed to apply and a knob
    that worked perfectly produce the SAME output. Without this the benchmark
    would happily report "+0.00 ms, no difference" for a run that never
    switched anything."""
    from roomscan.slam.config import SlamConfig
    from roomscan.slam import mapper as mapper_mod

    class _DeafMapper(mapper_mod.Mapper):
        @property
        def icp_device(self):        # ignores what it was constructed with
            return "CUDA:9"

    monkeypatch.setattr(bench, "_odometry", odometry)
    monkeypatch.setattr(mapper_mod, "Mapper", _DeafMapper)
    wd = bench._Watchdog()
    with pytest.raises(AssertionError, match="icp_device did not take"):
        bench._ab_arm([], 54, 42, SlamConfig(), "CPU:0", "CPU:0", wd, "x")


def test_ab_pair_delta_is_candidate_minus_baseline():
    """Sign convention: negative means the candidate is faster. Getting this
    backwards would invert the recommendation while looking entirely plausible."""
    base = {"step": {"p50_ms": 5.0}}
    cand = {"step": {"p50_ms": 4.5}}
    assert bench._pair_delta(base, cand, "step") == pytest.approx(-0.5)
    assert bench._pair_delta(cand, base, "step") == pytest.approx(0.5)
    assert bench._pair_delta({"step": {}}, cand, "step") is None


def test_env_sample_reports_cpu_load_not_just_the_gpu():
    """SS F of the study: an 8-pair run was discarded because a sibling
    session's headless Chrome appeared at 1270% CPU and slowed both arms 2.3x
    with `nvidia-smi` showing nothing. A GPU-only environment check cannot see
    the contamination that matters for a CPU-bound variant."""
    env = bench._env_sample()
    assert "loadavg_1m" in env and isinstance(env["loadavg_1m"], float)
    assert env.get("cpu_count", 0) > 0
