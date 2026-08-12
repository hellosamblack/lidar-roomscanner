"""Matched device benchmark for the SLAM ICP solve and the raycast round-trips.

    host/.venv/bin/python host/tools/slam_icp_bench.py captures/roomSweepFull20260730.bin
    host/.venv/bin/python host/tools/slam_icp_bench.py <capture> --frames 400 --json out.json

ROADMAP 6.I / plan item 4. Answers three questions on ONE device, over the SAME
recorded ICP inputs, so the comparison is matched rather than three separate runs:

1. What does the shipped `translation` ICP cost, and how much of that is the
   host round-trip it does per iteration (`odometry.py:41-89` downloads
   neighbour counts and indices every iteration and solves the 3x3 in numpy)?
2. What does Open3D's tensor `6dof` ICP cost on the same inputs?
3. What would a GPU-RESIDENT translation solve cost -- matching AND the normal
   equations on device, with one small download per iteration?

...and, added for item 5 (`--what ab`), the question the study's own conclusion
turned on:

4. What does moving ICP's NN index OFF the GPU actually buy end to end? That is
   measured by an interleaved, paired, whole-pipeline A/B on the SHIPPED
   `Mapper` (via its `icp_device` knob), not by (1)-(3)'s isolated
   microbenchmark -- which is not stable enough to size a change: the identical
   measurement on identical inputs gave p50 2.519 ms in one session and
   3.594 ms in another. Arms alternate order within each pair, the whole
   trajectory is compared for equivalence, and CPU load is sampled around every
   arm because the winning variant is the CPU-bound one.

`gpu_translation` is implemented HERE, not in `roomscan.slam.odometry`, on
purpose: this is a study, and the shipped ICP path must stay untouched so a
later before/after stays interpretable. `install_variant()` monkey-patches
`odometry.register` for the accuracy ensemble; nothing on disk changes.

WHY THE VARIANT IS SHAPED THE WAY IT IS (measured on the installed Open3D
0.19 CUDA build -- re-run `--api` to re-derive):

  ASYNC (no host sync): elementwise ops, broadcasting, `matmul`, `T()`,
    integer/boolean tensor ops, `concatenate`, slicing, host->device upload.
  SYNC (blocks until the CUDA queue drains): `sum(dim=)`, boolean-mask
    indexing `t[mask]`, `nonzero()`, `.item()`, ANY `.cpu()`, and every
    linalg entry point (`solve`/`inv`/`lstsq`/`svd`) -- AND
    `nns.hybrid_search` itself.

So the obvious device-side formulation (boolean-mask the matches, `sum` the
normal equations, `solve` on device) would add THREE syncs per iteration
rather than removing any. The variant below therefore uses no mask, no
reduction and no device solve: invalid correspondences are weighted to zero
(algebraically identical to dropping them, since a zero row contributes
nothing to either normal-equation term), every reduction is expressed as a
matmul, and all four scalars the host needs -- A, b, the match count and the
squared-residual sum -- come back in ONE 4x6 download. One sync per iteration,
24 floats, on top of the `hybrid_search` sync the shipped path already pays.

Never binds the device; only reads a capture file. It does allocate a TSDF on
the compute device, so it competes with a live `roomscan-web` for the GPU --
check `nvidia-smi` around a timed run.
"""
from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

import numpy as np
import open3d as o3d

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "host" / "src"))

from roomscan.slam import odometry as _odometry            # noqa: E402
from roomscan.slam.odometry import (                       # noqa: E402
    RegistrationResult, _solve_translation_step,
)

_reg = o3d.t.pipelines.registration

DEFAULT_FRAMES = 400
# Mirror the SHIPPED cap, by reference. This rig exists to profile the shipped
# pipeline; a rig that quietly keeps the old solver stops doing that (the exact
# failure recorded on 2026-08-02, when two rigs had drifted off the real path).
_COND_CAP = _odometry._COND_CAP


def _dev(device) -> o3d.core.Device:
    return device if isinstance(device, o3d.core.Device) else o3d.core.Device(device)


# --------------------------------------------------------------------------
# Variant 3: GPU-resident translation-only point-to-plane ICP.
# --------------------------------------------------------------------------
def gpu_translation_icp(src_t: o3d.core.Tensor, tgt_t: o3d.core.Tensor,
                        nrm_t: o3d.core.Tensor, R: np.ndarray, t0: np.ndarray,
                        max_dist: float, max_iter: int, tol: float = 1e-7):
    """Same algorithm as `odometry._translation_icp`, with matching and both
    normal-equation terms kept on the compute device.

    `src_t`/`tgt_t`/`nrm_t` are (N,3) Float64 tensors already on the device.
    Returns (t, fitness, rmse, singular) with the same semantics.

    Per iteration the host learns exactly one 4x6 matrix::

        L = [ n*w | 1 ]            (N,4)
        Rt = [ n | r*w | w | r*r*w ]   (N,6)
        M = L^T @ Rt               (4,6)

    from which  A = M[:3,:3] = sum w n n^T,  -b = M[:3,3] = sum w n r,
    n_valid = M[3,2] = sum w,  and sum w r^2 = M[3,3+... ] -- see below. The
    3x3 solve and the condition check stay in numpy because Open3D's device
    `solve()` costs 0.079 ms against numpy's ~0.01 ms for a 3x3 AND syncs.
    """
    dev = src_t.device
    f64 = o3d.core.float64
    n_source = int(src_t.shape[0])
    R_t = o3d.core.Tensor(np.ascontiguousarray(R, dtype=np.float64), device=dev)
    rotated = src_t @ R_t.T()                       # (N,3), device matmul

    nns = o3d.core.nns.NearestNeighborSearch(tgt_t)
    nns.hybrid_index(max_dist)

    ones_col = o3d.core.Tensor(np.ones((n_source, 1), np.float64), device=dev)
    sum3 = o3d.core.Tensor(np.ones((3, 1), np.float64), device=dev)

    t = np.asarray(t0, dtype=np.float64).copy()
    fitness, rmse = 0.0, float("inf")
    for _ in range(max_iter):
        t_t = o3d.core.Tensor(t.reshape(1, 3), device=dev)      # upload, async
        query = rotated + t_t
        idx, _d2, counts = nns.hybrid_search(query, max_dist, 1)   # SYNC (as shipped)
        cnt = counts.reshape((n_source, 1))
        # Misses come back as idx = -1 / counts = 0. Clamp the index with the
        # count so the gather is always in range; the weight zeroes the row's
        # contribution anyway, so this is algebraically the same as masking.
        safe = (idx.reshape((n_source,)) * counts.reshape((n_source,)))
        q = tgt_t[safe]
        n = nrm_t[safe]
        w = cnt.to(f64)                                          # (N,1) 0.0/1.0
        r = ((n * (query - q)) @ sum3)                           # (N,1) row dot
        rw = r * w
        left = o3d.core.concatenate([n * w, ones_col], axis=1)   # (N,4)
        right = o3d.core.concatenate([n, rw, w, r * rw], axis=1)  # (N,6)
        M = (left.T() @ right).cpu().numpy()                     # ONE sync, 24 floats

        n_valid = float(M[3, 3 + 1])          # sum(w)          -> column 4
        if n_valid < 1.0:
            return t, 0.0, float("inf"), False
        sq = float(M[3, 3 + 2])               # sum(w r^2)      -> column 5
        fitness = n_valid / n_source
        rmse = float(np.sqrt(sq / n_valid))

        a = M[:3, :3]                          # sum w n n^T
        b = -M[:3, 3]                          # -sum w n r
        # Same conditioning cap as odometry._translation_icp (BUG-068), so the
        # A/B stays an apples-to-apples comparison of WHERE the work runs.
        dt, _cond = _solve_translation_step(a, b, _COND_CAP)
        if dt is None:
            return t, fitness, rmse, True
        t = t + dt
        if np.linalg.norm(dt) < tol:
            break
    return t, fitness, rmse, False


def register_gpu(source, target, init_pose, max_dist=0.05, min_fitness=0.3,
                 max_rmse=0.05, max_iter=6, device="CUDA:0") -> RegistrationResult:
    """`odometry.register(mode="translation")`'s contract, GPU-resident."""
    dev = _dev(device)
    init_pose = np.asarray(init_pose, dtype=np.float64)
    R = init_pose[:3, :3]
    f64 = o3d.core.float64
    src_t = source.point.positions.to(dev).to(f64)
    tgt_t = target.point.positions.to(dev).to(f64)
    nrm_t = target.point.normals.to(dev).to(f64)
    t, fitness, rmse, singular = gpu_translation_icp(
        src_t, tgt_t, nrm_t, R, init_pose[:3, 3], max_dist, max_iter)
    if singular:
        return RegistrationResult(pose=init_pose.copy(), fitness=0.0,
                                  rmse=float("inf"), ok=False)
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t
    ok = bool(fitness >= min_fitness and rmse <= max_rmse)
    return RegistrationResult(pose=T, fitness=float(fitness), rmse=float(rmse), ok=ok)


_ORIGINAL_REGISTER = _odometry.register

#: Proof the monkey-patch actually took effect. "Verify the knob took effect"
#: is a standing lesson here -- a patch that silently does not apply reads as
#: "no effect", and byte-identical results would be exactly what we expect from
#: a variant that never ran. Every ensemble asserts these are non-zero.
SHIM_CALLS: dict[str, int] = {}


def install_variant() -> None:
    """Teach `odometry.register` the extra mode `"gpu_translation"`, in memory
    only, so `Mapper`/`slam_ensemble` can score it without the shipped file
    changing. Idempotent."""
    if getattr(_odometry.register, "_is_bench_shim", False):
        return

    def _shim(source, target, init_pose, mode="translation", **kw):
        SHIM_CALLS[mode] = SHIM_CALLS.get(mode, 0) + 1
        if mode == "gpu_translation":
            return register_gpu(source, target, init_pose, **kw)
        if mode == "translation_cpu_nns":
            kw = dict(kw)
            kw["device"] = o3d.core.Device("CPU:0")
            return _ORIGINAL_REGISTER(source, target, init_pose, mode="translation", **kw)
        return _ORIGINAL_REGISTER(source, target, init_pose, mode=mode, **kw)

    _shim._is_bench_shim = True
    _odometry.register = _shim


def uninstall_variant() -> None:
    _odometry.register = _ORIGINAL_REGISTER


# --------------------------------------------------------------------------
# Recording matched ICP inputs from a real run
# --------------------------------------------------------------------------
def _record_inputs(capture, frames, device, cfg):
    """Replay `frames` frames of `capture` through the real pipeline and capture
    every (source, model, init) triple the mapper actually handed to ICP.

    Recorded as HOST numpy so each variant rebuilds its own device tensors --
    otherwise the first variant benchmarked would donate warm device buffers to
    the others."""
    from roomscan.slam.cli import _load_frames
    from roomscan.slam.mapper import Mapper

    all_frames, width, height = _load_frames(str(capture), frames)
    if not all_frames:
        raise SystemExit(f"no depth frames decoded from {capture}")

    captured: list[tuple] = []
    orig = _odometry.register

    def _tap(source, target, init_pose, mode="translation", **kw):
        captured.append((
            source.point.positions.cpu().numpy().copy(),
            target.point.positions.cpu().numpy().copy(),
            target.point.normals.cpu().numpy().copy(),
            np.asarray(init_pose, dtype=np.float64).copy(),
        ))
        return orig(source, target, init_pose, mode=mode, **kw)

    _odometry.register = _tap
    try:
        mapper = Mapper(width, height, cfg.fov_h, cfg.fov_v, icp_mode="translation",
                        voxel_size=cfg.voxel_size, device=device,
                        # Pinned to the pre-item-5 behaviour (one device for
                        # everything) on purpose: these are the study's own
                        # numbers, and they must not silently change footing
                        # when the shipped default moves. `--what ab` is the
                        # pass that measures the new default.
                        icp_device=None,
                        block_count=cfg.block_count,
                        release_cache_every=cfg.release_cache_every,
                        max_dist=cfg.max_dist, icp_retry_dist=cfg.icp_retry_dist,
                        max_iter=cfg.max_iter, min_fitness=cfg.min_fitness,
                        max_rmse=cfg.max_rmse, min_confidence=cfg.min_confidence,
                        weight_threshold=cfg.weight_threshold,
                        baro_authority=cfg.baro_authority,
                        baro_tau_frames=cfg.baro_tau_frames)
        for depth, refl, conf, quat, pa, _t in all_frames:
            mapper.step(depth, quat, pa, reflectance=refl, confidence=conf)
    finally:
        _odometry.register = orig
    return captured, mapper, width, height


def _clouds(rec, dev):
    src = o3d.t.geometry.PointCloud(dev)
    src.point.positions = o3d.core.Tensor(rec[0], device=dev)
    tgt = o3d.t.geometry.PointCloud(dev)
    tgt.point.positions = o3d.core.Tensor(rec[1], device=dev)
    tgt.point.normals = o3d.core.Tensor(rec[2], device=dev)
    return src, tgt


def _stats(vals):
    a = np.asarray(sorted(vals), dtype=np.float64) * 1000.0
    if a.size == 0:
        return {}
    return {"n": int(a.size), "p50_ms": round(float(np.median(a)), 3),
            "p90_ms": round(float(a[min(a.size - 1, int(0.9 * a.size))]), 3),
            "max_ms": round(float(a[-1]), 3), "total_s": round(float(a.sum()) / 1000.0, 3)}


class _Watchdog:
    """GIL-starvation watchdog, same principle as slam_stall_profile's."""

    def __init__(self, period_s=0.005):
        import threading
        self.period_s = period_s
        self.stage = "idle"
        self.late: dict[str, list[float]] = {}
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)

    def _run(self):
        nxt = time.monotonic() + self.period_s
        while not self._stop.is_set():
            now = time.monotonic()
            if now < nxt:
                time.sleep(nxt - now)
                now = time.monotonic()
            self.late.setdefault(self.stage, []).append(max(0.0, now - nxt))
            nxt = now + self.period_s

    def start(self):
        self._thread.start()

    def stop(self):
        self._stop.set()
        self._thread.join(timeout=1.0)

    def report(self, wall_by_stage: dict) -> dict:
        """`tick_share` is the number to read, NOT `starved_pct_of_stage`.

        Summing tick lateness silently UNDER-reports the worst case: if the
        measured code holds the GIL so completely that the watchdog thread
        barely runs at all, there are almost no samples to sum. Measured here:
        `gpu_translation` produced **1 tick in 10.93 s** where ~2186 were due,
        and reported a *lower* starvation percentage than a stage that ran
        freely. So report how many of the expected ticks actually landed --
        `tick_share` near 1.0 means other Python threads (the asyncio loop, the
        reader) got to run; near 0 means they did not, whatever the percentage
        says. `host/tools/slam_stall_profile.py::GilWatchdog.report()` carries
        the same `ticks`/`expected_ticks`/`tick_share` fields (issue #74)."""
        out = {}
        for stage, samples in self.late.items():
            if stage == "idle":
                continue
            total = sum(samples)
            wall = wall_by_stage.get(stage, 0.0)
            expected = wall / self.period_s if wall else 0.0
            out[stage] = {"ticks": len(samples),
                          "expected_ticks": int(expected),
                          "tick_share": round(len(samples) / expected, 4) if expected else None,
                          "starved_s": round(total, 3),
                          "starved_pct_of_stage": round(100.0 * total / wall, 1) if wall else None,
                          "worst_stall_ms": round(max(samples) * 1000.0, 1) if samples else 0.0}
        return out


# --------------------------------------------------------------------------
# The three benchmarks
# --------------------------------------------------------------------------
def bench_icp(capture, *, frames=DEFAULT_FRAMES, device=None, max_frames_icp=None):
    """Time the three ICP variants over the SAME recorded (source, model, init)
    triples, with GIL starvation measured alongside wall time."""
    from roomscan.slam.config import SlamConfig, preferred_device
    cfg = SlamConfig.load()
    device = device or preferred_device()
    dev = _dev(device)

    recs, _mapper, width, height = _record_inputs(capture, frames, device, cfg)
    if not recs:
        return {"error": "no ICP calls recorded (every frame was bootstrap or lost)"}
    if max_frames_icp:
        recs = recs[:max_frames_icp]

    gate = dict(max_dist=cfg.max_dist, min_fitness=cfg.min_fitness,
                max_rmse=cfg.max_rmse, max_iter=cfg.max_iter)
    cpu = o3d.core.Device("CPU:0")
    variants = {
        "translation_shipped": lambda s, t, i: _ORIGINAL_REGISTER(
            s, t, i, mode="translation", device=dev, **gate),
        "sixdof_open3d": lambda s, t, i: _ORIGINAL_REGISTER(
            s, t, i, mode="6dof", device=dev, **gate),
        "gpu_translation": lambda s, t, i: register_gpu(s, t, i, device=dev, **gate),
        # The shipped translation solve with its NN search on the HOST. Same
        # code, same numbers -- the only difference is which device runs the
        # hybrid index. Included because the shipped path already downloads
        # every array it needs, so a host index removes syncs rather than
        # adding transfers, and the clouds here are only ~2268 points.
        "translation_cpu_nns": lambda s, t, i: _ORIGINAL_REGISTER(
            s, t, i, mode="translation", device=cpu, **gate),
    }

    # Prebuild the clouds once per variant pass so cloud construction is not
    # timed (it is identical for all three and is not what is under test).
    prebuilt = [_clouds(r, dev) for r in recs]
    inits = [r[3] for r in recs]

    wd = _Watchdog()
    wd.start()
    out, wall_by_stage, results = {}, {}, {}
    try:
        for name, fn in variants.items():
            # warm-up
            for k in range(min(5, len(recs))):
                fn(prebuilt[k][0], prebuilt[k][1], inits[k])
            o3d.core.cuda.synchronize() if "CUDA" in str(dev).upper() else None
            wd.stage = name
            ts, res = [], []
            t_stage = time.monotonic()
            for (s, t), i in zip(prebuilt, inits):
                t0 = time.perf_counter()
                r = fn(s, t, i)
                ts.append(time.perf_counter() - t0)
                res.append(r)
            wall_by_stage[name] = time.monotonic() - t_stage
            wd.stage = "idle"
            out[name] = _stats(ts)
            results[name] = res
    finally:
        wd.stage = "idle"
        wd.stop()

    gil = wd.report(wall_by_stage)
    for name in out:
        out[name]["gil"] = gil.get(name, {"note": "no watchdog ticks attributed"})
        out[name]["stage_wall_s"] = round(wall_by_stage.get(name, 0.0), 3)
        rs = results[name]
        out[name]["fitness_mean"] = round(float(np.mean([r.fitness for r in rs])), 5)
        finite = [r.rmse for r in rs if np.isfinite(r.rmse)]
        out[name]["rmse_mean"] = round(float(np.mean(finite)), 6) if finite else None
        out[name]["not_ok"] = int(sum(1 for r in rs if not r.ok))

    # Equivalence, per call, against the shipped translation solve. This is the
    # check that can SEPARATE a sign/index/mask error from float round-off: it
    # feeds identical inputs to both solvers, so any algorithmic difference
    # shows up at 1e-3..1e0 m while round-off shows up at 1e-16 m. A
    # trajectory-only comparison cannot do that (chaos swamps it).
    base = results["translation_shipped"]
    out["equivalence"] = {"note": (
        "per-call, identical inputs, vs translation_shipped. Round-off is ~1e-16 m; "
        "any real algorithmic difference lands many orders above that.")}
    for name in ("gpu_translation", "translation_cpu_nns"):
        other = results[name]
        dt = np.array([np.linalg.norm(a.pose[:3, 3] - b.pose[:3, 3])
                       for a, b in zip(base, other)])
        df = np.array([abs(a.fitness - b.fitness) for a, b in zip(base, other)])
        dr = np.array([abs(a.rmse - b.rmse) for a, b in zip(base, other)
                       if np.isfinite(a.rmse) and np.isfinite(b.rmse)])
        out["equivalence"][name] = {
            "n": len(base),
            "translation_max_abs_m": float(dt.max()) if dt.size else None,
            "translation_p99_m": float(np.percentile(dt, 99)) if dt.size else None,
            "fitness_max_abs": float(df.max()) if df.size else None,
            "rmse_max_abs_m": float(dr.max()) if dr.size else None,
            "ok_flag_disagreements": int(sum(1 for a, b in zip(base, other) if a.ok != b.ok)),
        }
    out["_meta"] = {"capture": str(capture), "device": str(dev), "frames_replayed": frames,
                    "icp_calls": len(recs), "width": width, "height": height,
                    "max_dist": cfg.max_dist, "max_iter": cfg.max_iter}
    return out


def bench_raycast(capture, *, frames=200, device=None, repeats=3):
    """Cost of `TsdfMap.raycast()`'s host round-trip, and of `Mapper.step()`'s
    second download of the same positions just to count them (mapper.py:283-287).

    Measured on a REAL map built from `frames` frames of the capture, at the real
    poses, because raycast cost scales with the map and the frustum."""
    from roomscan.slam.cli import _load_frames
    from roomscan.slam.config import SlamConfig, preferred_device
    from roomscan.slam.mapper import Mapper
    cfg = SlamConfig.load()
    device = device or preferred_device()
    dev = _dev(device)
    is_cuda = "CUDA" in str(dev).upper()

    all_frames, width, height = _load_frames(str(capture), frames)
    mapper = Mapper(width, height, cfg.fov_h, cfg.fov_v, voxel_size=cfg.voxel_size,
                    device=device, block_count=cfg.block_count,
                    icp_device=None,        # see _record_inputs: study footing
                    release_cache_every=cfg.release_cache_every)
    poses = []
    for depth, refl, conf, quat, pa, _t in all_frames:
        step = mapper.step(depth, quat, pa, reflectance=refl, confidence=conf)
        poses.append((step.pose.copy(), depth))

    tsdf = mapper._tsdf
    intr = mapper._intr
    tot = {k: [] for k in ("ray_cast", "download_3x", "numpy_mask", "rebuild_device",
                           "shipped_total", "device_resident_total", "mapper_recount")}
    valid_counts: list[int] = []
    shipped_counts: list[int] = []
    sync = o3d.core.cuda.synchronize if is_cuda else (lambda: None)
    sample = poses[len(poses) // 3::max(1, len(poses) // 60)]
    for _ in range(repeats):
        for pose, depth in sample:
            ext = np.linalg.inv(pose)
            coords = tsdf.frustum_block_coords(depth, intr, ext)
            if coords.shape[0] == 0:
                continue
            ext_t = o3d.core.Tensor(np.asarray(ext, np.float64), device=o3d.core.Device("CPU:0"))
            intr_c = intr.to(o3d.core.Device("CPU:0"))

            sync()
            t_all = time.perf_counter()
            t0 = time.perf_counter()
            res = tsdf._vbg.ray_cast(coords, intr_c, ext_t, width, height,
                                     render_attributes=["vertex", "normal", "depth"],
                                     depth_scale=tsdf.depth_scale, depth_min=0.1,
                                     depth_max=tsdf.depth_max, weight_threshold=1.0,
                                     trunc_voxel_multiplier=tsdf.trunc_multiplier,
                                     range_map_down_factor=1)
            sync()
            tot["ray_cast"].append(time.perf_counter() - t0)

            t0 = time.perf_counter()
            vertex = res["vertex"].cpu().numpy().reshape(-1, 3)
            normal = -res["normal"].cpu().numpy().reshape(-1, 3)
            dep = res["depth"].cpu().numpy().reshape(-1)
            tot["download_3x"].append(time.perf_counter() - t0)

            t0 = time.perf_counter()
            keep = dep > 0.0
            v, n = vertex[keep], normal[keep]
            tot["numpy_mask"].append(time.perf_counter() - t0)

            t0 = time.perf_counter()
            pc = o3d.t.geometry.PointCloud(dev)
            pc.point.positions = o3d.core.Tensor(v.astype(np.float32), device=dev)
            pc.point.normals = o3d.core.Tensor(n.astype(np.float32), device=dev)
            sync()
            tot["rebuild_device"].append(time.perf_counter() - t0)
            tot["shipped_total"].append(time.perf_counter() - t_all)

            # What Mapper.step() then does: download the positions AGAIN to
            # learn how many there are (mapper.py:285).
            t0 = time.perf_counter()
            shipped_counts.append(pc.point.positions.cpu().numpy().shape[0])
            tot["mapper_recount"].append(time.perf_counter() - t0)

            # Device-resident alternative: same ray_cast, then mask+gather on
            # device and return the valid count as metadata so the mapper never
            # transfers. Boolean masking SYNCS on this build, so the count comes
            # from a matmul reduction and the gather from an index tensor.
            sync()
            t0 = time.perf_counter()
            res2 = tsdf._vbg.ray_cast(coords, intr_c, ext_t, width, height,
                                      render_attributes=["vertex", "normal", "depth"],
                                      depth_scale=tsdf.depth_scale, depth_min=0.1,
                                      depth_max=tsdf.depth_max, weight_threshold=1.0,
                                      trunc_voxel_multiplier=tsdf.trunc_multiplier,
                                      range_map_down_factor=1)
            npix = width * height
            d_t = res2["depth"].reshape((npix,))
            keep_t = d_t.gt(o3d.core.Tensor([0.0], dtype=d_t.dtype, device=dev))
            # nonzero() returns a LIST of index tensors (one per dim); it also
            # SYNCS on this build (measured) -- there is no non-syncing way to
            # produce a variable-length selection, so this is the floor.
            sel = keep_t.nonzero()[0].reshape((-1,))
            pc2 = o3d.t.geometry.PointCloud(dev)
            pc2.point.positions = res2["vertex"].reshape((npix, 3))[sel]
            pc2.point.normals = res2["normal"].reshape((npix, 3))[sel] * (-1.0)
            # The valid count is already on the host (nonzero() had to sync to
            # produce it), so the mapper would need no second transfer at all.
            valid_counts.append(int(sel.shape[0]))
            sync()
            tot["device_resident_total"].append(time.perf_counter() - t0)

    out = {k: _stats(v) for k, v in tot.items() if v}
    used, cap = tsdf.block_usage()
    # The two paths must select the SAME points, or the timing comparison is
    # between two different computations. Cheap, and it would catch a mask
    # polarity or reshape-order mistake in the device-resident version.
    out["selection_agrees"] = bool(valid_counts == shipped_counts)
    out["mean_valid_points"] = (round(sum(valid_counts) / len(valid_counts), 1)
                                if valid_counts else None)
    out["_meta"] = {"capture": str(capture), "device": str(dev), "frames_built": frames,
                    "blocks": used, "live_capacity": cap, "samples": len(sample),
                    "repeats": repeats, "width": width, "height": height}
    return out


def bench_api(device=None):
    """Which Open3D 0.19 tensor ops force a host synchronization.

    Deep-queue discrimination: enqueue a long chain of large matmuls, then time
    the candidate op. An op that returns while the queue is still draining is
    asynchronous; one that costs ~the whole drain synchronized. Reported next to
    the op's cost on an EMPTY queue, so "slow" and "synchronizing" cannot be
    confused."""
    from roomscan.slam.config import preferred_device
    device = device or preferred_device()
    dev = _dev(device)
    if "CUDA" not in str(dev).upper():
        return {"error": f"{dev} is not a CUDA device; sync probing is meaningless on CPU"}
    sync = o3d.core.cuda.synchronize
    N = 2048
    A = o3d.core.Tensor(np.random.rand(N, N).astype(np.float32), device=dev)
    B = o3d.core.Tensor(np.random.rand(N, N).astype(np.float32), device=dev)
    for _ in range(3):
        _ = A @ B
    sync()

    def enqueue():
        for _ in range(30):
            globals()["_sink"] = A @ B

    t0 = time.perf_counter()
    enqueue()
    e = time.perf_counter() - t0
    t0 = time.perf_counter()
    sync()
    d = time.perf_counter() - t0
    D = e + d

    M = 2268
    pos = o3d.core.Tensor(np.random.rand(M, 3).astype(np.float64), device=dev)
    nrm = o3d.core.Tensor(np.random.rand(M, 3).astype(np.float64), device=dev)
    cnt = o3d.core.Tensor(np.random.randint(0, 2, size=M), device=dev)
    idx = o3d.core.Tensor(np.random.randint(0, M, size=M), device=dev)
    ones = o3d.core.Tensor(np.ones((3, 1), np.float64), device=dev)
    nns = o3d.core.nns.NearestNeighborSearch(pos)
    nns.hybrid_index(0.05)

    ops = {
        "elementwise_mul": lambda: nrm * pos,
        "broadcast_mul": lambda: nrm * nrm[:, 0:1],
        "matmul_rowsum": lambda: (nrm * pos) @ ones,
        "matmul_normal_eq": lambda: nrm.T() @ pos,
        "concatenate_axis1": lambda: o3d.core.concatenate([nrm, nrm[:, 0:1]], axis=1),
        "int_gather": lambda: pos[idx],
        "compare_gt": lambda: cnt.gt(o3d.core.Tensor([0], device=dev)),
        "upload_host_to_device": lambda: o3d.core.Tensor(np.zeros((1, 3)), device=dev),
        "sum_dim0": lambda: pos.sum(dim=0),
        "bool_mask_index": lambda: pos[cnt.gt(o3d.core.Tensor([0], device=dev))],
        "nonzero": lambda: cnt.nonzero(),
        "item": lambda: cnt.sum(dim=0).item(),
        "download_small_4x6": lambda: (nrm.T() @ pos).cpu().numpy(),
        "download_full_Nx3": lambda: pos.cpu().numpy(),
        "linalg_solve_3x3": lambda: (nrm.T() @ pos).solve(ones),
        "linalg_inv_3x3": lambda: (nrm.T() @ pos).inv(),
        "nns_hybrid_search": lambda: nns.hybrid_search(pos, 0.05, 1),
    }
    out = {"_meta": {"device": str(dev), "queue_drain_ms": round(D * 1000, 1),
                     "open3d": o3d.__version__}}
    for name, fn in ops.items():
        try:
            for _ in range(3):
                fn()
            sync()
        except Exception as exc:
            out[name] = {"error": f"{type(exc).__name__}: {' '.join(str(exc).split())[:160]}"}
            continue
        ts = []
        for _ in range(20):
            sync()
            t0 = time.perf_counter()
            fn()
            ts.append(time.perf_counter() - t0)
        sync()
        ds = []
        for _ in range(5):
            sync()
            enqueue()
            t0 = time.perf_counter()
            fn()
            ds.append(time.perf_counter() - t0)
            sync()
        out[name] = {"empty_queue_ms": round(float(np.median(ts)) * 1000, 4),
                     "deep_queue_ms": round(float(np.median(ds)) * 1000, 2),
                     "syncs": bool(np.median(ds) > 0.5 * D)}
    return out


MODES = ("translation", "gpu_translation", "translation_cpu_nns", "6dof")


def paired_equivalence(baseline_runs, candidate_runs, *, samples: int = 10000) -> dict:
    """Non-inferiority, not improvement.

    `slam.validation.paired_loop_gate` asks whether a change makes closure
    BETTER (one-sided, CI strictly above zero). An ICP optimisation is not
    supposed to make anything better -- it has to be indistinguishable. So this
    is a paired bootstrap CI on (candidate - baseline) `horizontal_closure_m`,
    and the verdict is that the whole CI sits inside a tolerance band.

    The band is not chosen here: it is one standard deviation of the BASELINE
    ensemble's own closure, i.e. the size of the run-to-run chaos already
    present in the metric. Anything smaller than that cannot be distinguished
    from a re-run of the unchanged code, and quoting a tighter tolerance would
    be inventing precision the instrument does not have."""
    if len(baseline_runs) != len(candidate_runs) or not baseline_runs:
        return {"accepted": False, "reason": "need equally sized non-empty matched ensembles"}
    base = np.asarray([float(r["horizontal_closure_m"]) for r in baseline_runs])
    cand = np.asarray([float(r["horizontal_closure_m"]) for r in candidate_runs])
    delta = cand - base
    tol = float(base.std(ddof=1)) if base.size > 1 else 0.0
    rng = np.random.default_rng(20260802)
    draws = delta[rng.integers(0, delta.size, size=(max(1000, samples), delta.size))].mean(axis=1)
    ci = [float(np.percentile(draws, 2.5)), float(np.percentile(draws, 97.5))]
    tracking_ok = all(not bool(c.get("died")) for c in candidate_runs) and \
        max(int(c.get("lost", 0)) for c in candidate_runs) <= \
        max(int(b.get("lost", 0)) for b in baseline_runs)
    inside = bool(abs(ci[0]) <= tol and abs(ci[1]) <= tol)
    return {"accepted": bool(inside and tracking_ok), "n": int(delta.size),
            "tolerance_m": tol,
            "tolerance_provenance": "1 sd of the BASELINE ensemble's horizontal_closure_m",
            "mean_delta_m": float(delta.mean()), "ci95_m": ci,
            "tracking_ok": tracking_ok,
            "reason": ("paired CI inside +/-1 baseline sd with no tracking regression"
                       if inside and tracking_ok else
                       "CI must lie inside +/-1 baseline sd and tracking must not regress")}


def bench_ensemble(capture, *, modes=MODES, n=10, device=None, max_frames=None) -> dict:
    """Score each ICP variant with a matched perturbation ENSEMBLE (BUG-037:
    one run is not a measurement) and apply the non-inferiority gate above.

    Reuses `host/tools/slam_ensemble.py::run_ensemble`, so the perturbations,
    the metrics and the `died`/`trailing_lost` caveats are exactly the ones the
    repo already validated against -- runs are paired by perturbation index
    across modes."""
    sys.path.insert(0, str(REPO / "host" / "tools"))
    from slam_ensemble import run_ensemble
    from roomscan.slam.config import preferred_device
    device = device or preferred_device()
    install_variant()
    out = {"capture": str(capture), "device": device, "n": n, "modes": {}}
    try:
        for mode in modes:
            SHIM_CALLS.clear()
            t0 = time.monotonic()
            r = run_ensemble(capture, n=n, device=device, icp_mode=mode,
                             max_frames=max_frames)
            r["wall_s"] = round(time.monotonic() - t0, 1)
            # The knob must be proven, not assumed: a patch that silently did
            # not apply would report "no difference", which is also what a
            # working equivalent variant reports.
            r["shim_calls"] = dict(SHIM_CALLS)
            assert SHIM_CALLS.get(mode, 0) > 0, (
                f"the register() shim was never called with mode={mode!r} -- the "
                f"variant did not run, so any 'no change' result is meaningless")
            out["modes"][mode] = r
            print(f"[ensemble] {mode}: closure "
                  f"{r['summary']['horizontal_closure_m']['mean']:.3f} +/- "
                  f"{r['summary']['horizontal_closure_m']['sd']:.3f} m, "
                  f"median step {r['summary']['median_ms']['mean']:.2f} ms, "
                  f"{r['wall_s']} s", flush=True)
    finally:
        uninstall_variant()
    base = out["modes"].get("translation", {}).get("runs")
    if base:
        out["gate"] = {m: paired_equivalence(base, out["modes"][m]["runs"])
                       for m in out["modes"] if m != "translation"}
    return out


# --------------------------------------------------------------------------
# Interleaved, paired, whole-pipeline A/B on the SHIPPED code path
# --------------------------------------------------------------------------
def _env_sample() -> dict:
    """CPU load AND GPU state. `nvidia-smi` alone is not enough of an
    environment check on this box: the study (SS F) discarded an 8-pair run
    because a sibling session's headless Chrome appeared at 1270% CPU and
    slowed BOTH arms 2.3x with the GPU untouched -- and the variant under test
    is the CPU-bound one. Sampled around every arm so a contaminated pair can
    be reported or discarded rather than averaged in."""
    import subprocess
    out = {}
    try:
        out["loadavg_1m"] = float(Path("/proc/loadavg").read_text().split()[0])
    except Exception:
        out["loadavg_1m"] = None
    try:
        import os
        out["cpu_count"] = os.cpu_count()
    except Exception:
        pass
    try:
        q = subprocess.run(
            ["nvidia-smi",
             "--query-gpu=utilization.gpu,memory.used,clocks.sm,temperature.gpu",
             "--format=csv,noheader,nounits"],
            capture_output=True, text=True, timeout=10)
        util, mem, sm, temp = (x.strip() for x in q.stdout.strip().splitlines()[0].split(","))
        out.update(gpu_util_pct=int(util), gpu_mem_used_mib=int(mem),
                   gpu_sm_mhz=int(sm), gpu_temp_c=int(temp))
    except Exception:
        pass
    return out


def _ab_arm(all_frames, width, height, cfg, device, icp_device, wd, stage):
    """One replay of the whole capture through the SHIPPED `Mapper`, with
    `icp_device` set. Returns per-frame stage timings plus the proof that the
    knob took effect.

    Nothing here is monkey-patched except a pass-through tap on
    `odometry.register` that records which device it was actually handed. That
    tap is the point: a device selector that silently fails to apply produces
    output identical to one that applied, because the change is bit-identical
    by design -- so 'no difference' cannot be read as evidence either way
    without it (the repo's `verify-the-knob-took-effect` lesson)."""
    from roomscan.slam.mapper import Mapper

    kwargs = cfg.mapper_kwargs()
    kwargs.update(device=device, icp_device=icp_device)
    mapper = Mapper(width, height, **kwargs)
    # Assert AFTER construction, not before: this is the exact shape of the
    # dataclass class-attribute patch that once silently did not apply.
    expect = str(_dev(device if icp_device is None else icp_device))
    assert mapper.icp_device == expect, (
        f"icp_device did not take: asked {icp_device!r}, got {mapper.icp_device!r}")
    assert mapper.device == str(_dev(device))

    seen_devices, calls = set(), [0]
    orig = _odometry.register

    def _tap(source, target, init_pose, **kw):
        calls[0] += 1
        seen_devices.add(str(kw.get("device")))
        return orig(source, target, init_pose, **kw)

    _odometry.register = _tap
    step_ms, ray_ms, icp_ms, integ_ms, fitness = [], [], [], [], []
    wd.stage = stage
    t_stage = time.monotonic()
    try:
        for depth, refl, conf, quat, pa, _t in all_frames:
            s = mapper.step(depth, quat, pa, reflectance=refl, confidence=conf)
            step_ms.append(s.slam_ms / 1000.0)
            ray_ms.append(s.raycast_ms / 1000.0)
            icp_ms.append(s.icp_ms / 1000.0)
            integ_ms.append(s.integrate_ms / 1000.0)
            fitness.append(s.fitness)
    finally:
        _odometry.register = orig
        wall = time.monotonic() - t_stage
        wd.stage = "idle"

    used, cap = mapper._tsdf.block_usage()
    fit = np.asarray([f for f in fitness if f > 0.0])
    result = {
        "icp_device_reported": mapper.icp_device,
        "device_reported": mapper.device,
        "register_calls": calls[0],
        "register_devices_seen": sorted(seen_devices),
        "wall_s": round(wall, 3),
        "step": _stats(step_ms), "raycast": _stats(ray_ms),
        "register": _stats([v for v in icp_ms if v > 0.0]),
        "integrate": _stats([v for v in integ_ms if v > 0.0]),
        "frames": len(step_ms),
        "tracking_lost": mapper.tracking_lost_count,
        "icp_escalations": mapper.icp_escalations,
        "blocks_used": int(used), "blocks_capacity": int(cap),
        # Equivalence payload. The trajectory is the whole run's answer, so an
        # arithmetic difference anywhere in ICP lands here; `mean_fitness` is
        # reported because a check run on a FULL-MATCH scene has no power over
        # the correspondence handling at all (study SS B).
        "trajectory": np.asarray(mapper.trajectory, dtype=np.float64),
        "mean_fitness": round(float(fit.mean()), 5) if fit.size else None,
        "frac_frames_with_misses": round(float((fit < 0.999).mean()), 4) if fit.size else None,
        "path_length_m": float(np.sum(np.linalg.norm(
            np.diff(np.asarray([T[:3, 3] for T in mapper.trajectory]), axis=0), axis=1)))
        if len(mapper.trajectory) > 1 else 0.0,
    }
    # Each arm allocates a whole VoxelBlockGrid (2.3 GiB of device memory at
    # the default block_count) and this box shares its 8 GiB card with the
    # owner's live server. Drop it and release Open3D's cached device blocks
    # BETWEEN arms -- outside every timed region, and it also means each arm
    # starts from the same allocator state, which is the fair comparison.
    del mapper
    try:
        o3d.core.cuda.release_cache()
    except Exception:
        pass
    return result


def _pair_delta(base, cand, key):
    b, c = base[key].get("p50_ms"), cand[key].get("p50_ms")
    if b is None or c is None:
        return None
    return c - b


def bench_ab(capture, *, pairs=4, frames=None, device=None, baseline_icp_device=None,
             candidate_icp_device="CPU:0") -> dict:
    """Interleaved, paired, whole-pipeline A/B of `Mapper.icp_device`.

    The isolated ICP microbenchmark (`--what icp`) is NOT stable enough to size
    this change: the same code on the same inputs measured p50 2.519 ms and
    3.594 ms in two sessions (a 43% swing). So this replays the entire capture
    through the shipped `Mapper` once per arm, back to back, `pairs` times,
    **alternating which arm goes first** so a warm-up or a drifting box biases
    the two arms equally, and reports the per-pair paired difference.

    `baseline_icp_device=None` means "follow the compute device", i.e. exactly
    the pre-item-5 behaviour. The default candidate is the host.

    Report the SPREAD, not a single number: the effect is small enough that run
    ordering, GPU clock state and the box's other tenants each move it by more
    than the effect itself. Environment is sampled around every arm for the same
    reason.
    """
    from roomscan.slam.cli import _load_frames
    from roomscan.slam.config import SlamConfig, preferred_device

    cfg = SlamConfig.load()
    device = device or preferred_device()
    all_frames, width, height = _load_frames(str(capture), frames or None)
    if not all_frames:
        raise SystemExit(f"no depth frames decoded from {capture}")

    arms = {"baseline": baseline_icp_device, "candidate": candidate_icp_device}
    out = {"capture": str(capture), "device": device, "frames": len(all_frames),
           "pairs": pairs, "width": width, "height": height,
           "arms": {"baseline_icp_device": str(baseline_icp_device),
                    "candidate_icp_device": str(candidate_icp_device)},
           "env_start": _env_sample(), "runs": []}

    wd = _Watchdog()
    wd.start()
    wall_by_stage = {}
    try:
        for i in range(pairs):
            # Alternate the within-pair order. Whichever arm runs first pays for
            # a cold allocator and a cold GPU clock; fixing the order would bake
            # that into the delta.
            order = ["baseline", "candidate"] if i % 2 == 0 else ["candidate", "baseline"]
            rec = {"pair": i, "order": order}
            for name in order:
                env_before = _env_sample()
                stage = f"{name}"
                t0 = time.monotonic()
                r = _ab_arm(all_frames, width, height, cfg, device, arms[name], wd, stage)
                wall_by_stage[stage] = wall_by_stage.get(stage, 0.0) + (time.monotonic() - t0)
                r["env_before"] = env_before
                r["env_after"] = _env_sample()
                rec[name] = r
                print(f"[ab] pair {i} {name}: step p50 {r['step']['p50_ms']} ms "
                      f"register p50 {r['register'].get('p50_ms')} ms "
                      f"(load {env_before.get('loadavg_1m')} -> "
                      f"{r['env_after'].get('loadavg_1m')}, "
                      f"sm {r['env_after'].get('gpu_sm_mhz')} MHz)", flush=True)
            out["runs"].append(rec)
    finally:
        wd.stage = "idle"
        wd.stop()

    gil = wd.report(wall_by_stage)
    out["gil"] = {k: gil.get(k) for k in ("baseline", "candidate")}

    # ---- paired deltas -----------------------------------------------------
    deltas = {k: [] for k in ("step", "register", "raycast", "integrate")}
    rel_step = []
    for rec in out["runs"]:
        for k in deltas:
            d = _pair_delta(rec["baseline"], rec["candidate"], k)
            if d is not None:
                deltas[k].append(d)
        b = rec["baseline"]["step"]["p50_ms"]
        rel_step.append(100.0 * (rec["candidate"]["step"]["p50_ms"] - b) / b)
    out["paired_delta_ms"] = {
        k: {"per_pair": [round(x, 4) for x in v],
            "mean": round(float(np.mean(v)), 4) if v else None,
            "sd": round(float(np.std(v, ddof=1)), 4) if len(v) > 1 else None,
            "n": len(v)}
        for k, v in deltas.items()}
    out["paired_delta_step_pct"] = {
        "per_pair": [round(x, 3) for x in rel_step],
        "mean": round(float(np.mean(rel_step)), 3) if rel_step else None,
        "sd": round(float(np.std(rel_step, ddof=1)), 3) if len(rel_step) > 1 else None}

    # ---- equivalence -------------------------------------------------------
    # Whole-trajectory, on a REAL capture. Both arms are supposed to be
    # bit-identical, so the bar is exact equality, not a tolerance -- and the
    # capture must actually contain unmatched points or the check has no power
    # over the correspondence handling (study SS B: on a full-match scene a
    # dropped-weight defect reads 0.0 m).
    eq = {"note": ("max |Δ| of the world position over every frame of every pair, "
                   "between the two arms. Expected exactly 0.0: the shipped "
                   "translation solve does all its arithmetic in numpy and the "
                   "device only selects which hybrid index runs the search.")}
    worst, worst_path, per_pair = 0.0, 0.0, []
    for rec in out["runs"]:
        a, b = rec["baseline"]["trajectory"], rec["candidate"]["trajectory"]
        if a.shape != b.shape:
            per_pair.append(None)
            eq["shape_mismatch"] = True
            continue
        d = float(np.abs(a[:, :3, 3] - b[:, :3, 3]).max())
        per_pair.append(d)
        worst = max(worst, d)
        worst_path = max(worst_path, abs(rec["candidate"]["path_length_m"]
                                         - rec["baseline"]["path_length_m"]))
    eq.update(max_abs_position_delta_m=worst, per_pair=per_pair,
              max_abs_path_length_delta_m=worst_path,
              bit_identical=bool(worst == 0.0 and worst_path == 0.0),
              mean_fitness=out["runs"][0]["baseline"]["mean_fitness"],
              frac_frames_with_misses=out["runs"][0]["baseline"]["frac_frames_with_misses"],
              has_discriminating_power=bool(
                  (out["runs"][0]["baseline"]["frac_frames_with_misses"] or 0.0) > 0.5),
              tracking_lost_delta=(out["runs"][0]["candidate"]["tracking_lost"]
                                   - out["runs"][0]["baseline"]["tracking_lost"]),
              blocks_delta=(out["runs"][0]["candidate"]["blocks_used"]
                            - out["runs"][0]["baseline"]["blocks_used"]))
    out["equivalence"] = eq

    # Trajectories are only carried for the comparison above; they are large
    # and are not part of the report.
    for rec in out["runs"]:
        for name in ("baseline", "candidate"):
            rec[name].pop("trajectory", None)
    out["env_end"] = _env_sample()
    return out


def run(capture=None, *, what="all", frames=DEFAULT_FRAMES, device=None,
        raycast_frames=200, ensemble_n=10, ab_pairs=4, ab_frames=None,
        baseline_icp_device=None, candidate_icp_device="CPU:0") -> dict:
    """Pure entry point: returns a dict, writes nothing, binds no device."""
    from roomscan.slam.config import preferred_device
    result = {"device_requested": device or preferred_device(),
              "cuda_available": bool(o3d.core.cuda.is_available()),
              "open3d": o3d.__version__}
    if what in ("all", "api"):
        result["api"] = bench_api(device)
    if what in ("all", "icp"):
        if not capture:
            result["icp"] = {"error": "a capture path is required for the ICP benchmark"}
        else:
            result["icp"] = bench_icp(capture, frames=frames, device=device)
    if what in ("all", "raycast"):
        if not capture:
            result["raycast"] = {"error": "a capture path is required"}
        else:
            result["raycast"] = bench_raycast(capture, frames=raycast_frames, device=device)
    if what == "ensemble":
        if not capture:
            result["ensemble"] = {"error": "a capture path is required"}
        else:
            result["ensemble"] = bench_ensemble(capture, n=ensemble_n, device=device,
                                                max_frames=frames or None)
    if what == "ab":
        if not capture:
            result["ab"] = {"error": "a capture path is required"}
        else:
            result["ab"] = bench_ab(capture, pairs=ab_pairs, frames=ab_frames,
                                    device=device,
                                    baseline_icp_device=baseline_icp_device,
                                    candidate_icp_device=candidate_icp_device)
    return result


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(
        prog="slam_icp_bench",
        description="Matched ICP-variant and raycast round-trip benchmark on one device.")
    ap.add_argument("capture", nargs="?", default=None)
    ap.add_argument("--what", choices=["all", "api", "icp", "raycast", "ensemble", "ab"],
                    default="all")
    ap.add_argument("--frames", type=int, default=DEFAULT_FRAMES,
                    help="frames replayed for the ICP benchmark; also caps the "
                         "ensemble's frame count (0 = whole capture)")
    ap.add_argument("--raycast-frames", type=int, default=200)
    ap.add_argument("-n", "--ensemble-n", type=int, default=10)
    ap.add_argument("--device", default=None)
    ap.add_argument("--ab-pairs", type=int, default=4,
                    help="interleaved A/B pairs (--what ab). Each pair replays the whole "
                         "capture twice, alternating which arm runs first.")
    ap.add_argument("--ab-frames", type=int, default=None,
                    help="cap the A/B's frame count (default: the whole capture)")
    ap.add_argument("--baseline-icp-device", default=None,
                    help='A/B baseline arm\'s Mapper.icp_device; omit for None = "follow '
                         'the compute device", i.e. the pre-2026-08-02 behaviour')
    ap.add_argument("--candidate-icp-device", default="CPU:0",
                    help="A/B candidate arm's Mapper.icp_device (default CPU:0)")
    ap.add_argument("--json", default=None, metavar="PATH")
    args = ap.parse_args(argv)
    r = run(args.capture, what=args.what, frames=args.frames, device=args.device,
            raycast_frames=args.raycast_frames, ensemble_n=args.ensemble_n,
            ab_pairs=args.ab_pairs, ab_frames=args.ab_frames,
            baseline_icp_device=args.baseline_icp_device,
            candidate_icp_device=args.candidate_icp_device)
    print(json.dumps(r, indent=2, default=float))
    if args.json:
        Path(args.json).parent.mkdir(parents=True, exist_ok=True)
        Path(args.json).write_text(json.dumps(r, indent=2, default=float), encoding="utf-8")
    return 0


if __name__ == "__main__":
    sys.exit(main())
