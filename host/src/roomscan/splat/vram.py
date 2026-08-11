"""Per-scene VRAM sweep: the largest MCMC ``cap_max`` that fits on this 8 GB card.

The question this answers -- "how dense a splat can we get away with on THIS
capture?" -- cannot be extrapolated. The discredited synthetic ``/tmp/vram_probe.py``
built a uniform-cube scene and reported 2M gaussians peaking at ~2.6 GiB; the real
Sam Office build OOM'd at n=2M near step 12000 because a stochastic worst-case
frame's *backward* needed ~6.7 GiB. A uniform cube misses the rasterizer's real
tile-intersection cost, which depends on the actual camera geometry and how the
gaussians cluster on real surfaces. So we measure on the REAL COLMAP model + REAL
frames, at forced gaussian counts, cycling every view to catch the worst frame.

Two deliberate biases keep the recommendation on the SAFE side:

* **Count is forced to exactly N, never grown by MCMC.** ``gsplat.MCMCStrategy``
  densifies geometrically over thousands of iterations, so a short warmup lands
  well below ``cap_max`` and would under-measure the very buffer that OOMs. We
  clone the real cloud up to N instead -- the count is what the real build
  eventually reaches.
* **Untrained knn scales are >= trained scales** (training shrinks them via
  ``scale_reg`` and the floater cull), so the cloned gaussians project to larger
  radii, touch more tiles, and over-estimate the peak. Over-estimation yields a
  conservative cap; a guard cross-checks this against a real short-train datum
  and warns if the preset defeats it.

Heavy imports (torch/gsplat) are function-local so the package stays importable
without the training stack, exactly like ``train.py``.
"""
from __future__ import annotations

import time
from pathlib import Path

# The reused seams live in train.py -- module-level despite the leading underscore,
# so importable. We mirror train.py's iteration shape rather than editing it, so a
# measured peak is the real trainer's peak, not an approximation of it.
from .train import (_gaussian_window, _init_gaussians, _load_views, _ssim)

_DEFAULT_LADDER = (250_000, 500_000, 750_000, 1_000_000, 1_250_000,
                   1_500_000, 1_750_000, 2_000_000, 2_500_000, 3_000_000)

# The per-param Adam learning rates train.py uses (train.py:191). Copied, not
# imported, because they are a local in train_splat; the VALUES do not affect the
# peak (only the moment-buffer SHAPES do), but building the same optimizers is what
# forces those buffers to allocate, so we reproduce them faithfully.
_LRS = {"means": 1.6e-4, "scales": 5e-3, "quats": 1e-3,
        "opacities": 5e-2, "sh0": 2.5e-3, "shN": 2.5e-3 / 20}


def _make_gaussians_at_count(rec, n_target, sh_degree, device, jitter_frac):
    """A ParameterDict at EXACTLY ``n_target`` gaussians, seeded from the real cloud.

    N <= seed -> random subsample; N > seed -> duplicate real points and jitter the
    copies by a fraction of each point's own knn scale, so duplicates stay on the
    local surface (preserves the real per-tile clustering; a uniform smear would
    degenerate footprints and under-count). Scales/colors come straight from
    ``_init_gaussians``, i.e. the untrained knn init -- deliberately >= trained
    scale, the conservative direction.
    """
    import torch

    params = _init_gaussians(rec, sh_degree, device)
    n0 = int(params["means"].shape[0])
    if n_target <= n0:
        idx = torch.randperm(n0, device=device)[:n_target]
    else:
        pad = torch.randint(n0, (n_target - n0,), device=device)
        idx = torch.cat([torch.arange(n0, device=device), pad])
    for k in list(params.keys()):
        params[k] = torch.nn.Parameter(params[k][idx])
    if n_target > n0:
        with torch.no_grad():
            sigma = torch.exp(params["scales"]).mean(dim=1, keepdim=True) * jitter_frac
            params["means"] += torch.randn_like(params["means"]) * sigma
    return params


def _one_iter(params, optimizers, v, gt, ssim_window, sh_degree,
              render_mode, depth_lambda, device):
    """One train.py-shaped forward+backward+step on a single view. Returns nothing;
    it exists purely to allocate the same tensors the real trainer allocates.

    Deliberately NO ``strategy.step_post_backward`` -- MCMC densification would
    change the gaussian count, and the whole point is to hold it at the forced N.
    ``sh_degree`` is passed at full bands (the real run ramps to it; full bands is
    the max-memory state we want to size for)."""
    import torch
    import gsplat

    viewmat = torch.from_numpy(v["viewmat"]).to(device)[None]
    K = torch.from_numpy(v["K"]).to(device)[None]
    colors = torch.cat([params["sh0"], params["shN"]], dim=1)
    render, alpha, info = gsplat.rasterization(
        params["means"], params["quats"], torch.exp(params["scales"]),
        torch.sigmoid(params["opacities"]), colors, viewmat, K,
        v["w"], v["h"], sh_degree=sh_degree, packed=True, render_mode=render_mode)
    pred = render[0][..., :3]
    l1 = (pred - gt).abs().mean()
    ssim = _ssim(pred.permute(2, 0, 1)[None], gt.permute(2, 0, 1)[None], ssim_window)
    loss = 0.8 * l1 + 0.2 * (1.0 - ssim)
    if depth_lambda > 0:
        # No monocular target here (that model is freed off-GPU in the real build),
        # but the RGB+ED render + its backward are the depth build's real VRAM cost;
        # a self-referential term makes the ED channel carry gradients so the
        # measured peak includes the extra-channel backward buffers.
        d = render[0][..., 3]
        loss = loss + depth_lambda * (d - d.detach().mean()).abs().mean()
    loss = loss + 0.01 * torch.sigmoid(params["opacities"]).mean() \
                + 0.02 * torch.exp(params["scales"]).mean()
    loss.backward()
    for opt in optimizers.values():
        opt.step()
        opt.zero_grad(set_to_none=True)
    del render, alpha, info, loss


def measure_peak_at_count(rec, views, scene_scale, n_target, *,
                          sh_degree=3, render_mode="RGB", depth_lambda=0.0,
                          jitter_frac=0.5, views_subset=None, device="cuda",
                          nvml=None, nvml_baseline_bytes=0,
                          log=lambda m: None) -> dict:
    """Peak VRAM of a real forward+backward at EXACTLY ``n_target`` gaussians.

    Mirrors ``train_splat``'s per-iteration allocations (same rasterization call,
    same loss, same per-param Adam optimizers -- whose moment buffers double
    per-param VRAM and are load-bearing). Cycles ``views_subset or views`` and
    keeps the worst frame's peak, because the OOM in the real build is one
    stochastic worst-case frame's backward.

    Returns a dict with allocated / reserved / device-wide-nvml peaks (all GiB),
    the worst view index, ``oom``, and wall seconds.
    """
    import torch
    import gsplat

    t0 = _mono()
    torch.cuda.reset_peak_memory_stats()
    torch.cuda.empty_cache()

    result = {"n": int(n_target), "n_actual": int(n_target),
              "peak_alloc_gib": None, "peak_reserved_gib": None,
              "peak_nvml_gib": None, "worst_view": None, "oom": False}
    try:
        params = _make_gaussians_at_count(rec, n_target, sh_degree, device, jitter_frac)
        result["n_actual"] = int(params["means"].shape[0])
        means_lr = _LRS["means"] * scene_scale
        optimizers = {k: torch.optim.Adam(
            [{"params": params[k], "lr": (means_lr if k == "means" else _LRS[k])}], eps=1e-15)
            for k in params}
        strategy = gsplat.MCMCStrategy(cap_max=n_target, verbose=False)
        strategy.check_sanity(params, optimizers)
        ssim_window = _gaussian_window(3, device)

        use = list(views_subset) if views_subset else list(views)
        # Warm iter: forces Adam moment buffers + gsplat's first-call JIT so the
        # measured views below see steady-state allocation, not one-time setup.
        gt0 = _load_gt(use[0], device)
        _one_iter(params, optimizers, use[0], gt0, ssim_window,
                  sh_degree, render_mode, depth_lambda, device)
        del gt0

        worst_alloc = -1.0
        peak_reserved = 0.0
        peak_nvml = 0.0
        for i, v in enumerate(use):
            torch.cuda.reset_peak_memory_stats()   # per-view: peak counter restarts
            gt = _load_gt(v, device)               # from current live allocation, so
            _one_iter(params, optimizers, v, gt, ssim_window,                       # the
                      sh_degree, render_mode, depth_lambda, device)                  # reported
            alloc = torch.cuda.max_memory_allocated() / 2**30      # peak is the TOTAL
            reserved = torch.cuda.max_memory_reserved() / 2**30    # (params+moments+
            peak_reserved = max(peak_reserved, reserved)           # activations), not a
            if nvml is not None and nvml.ok:                       # delta.
                nv = max(0, nvml.used_bytes() - nvml_baseline_bytes) / 2**30
                peak_nvml = max(peak_nvml, nv)
            if alloc > worst_alloc:
                worst_alloc = alloc
                result["worst_view"] = i
            del gt
        result["peak_alloc_gib"] = round(worst_alloc, 3)
        result["peak_reserved_gib"] = round(peak_reserved, 3)
        result["peak_nvml_gib"] = round(peak_nvml, 3) if peak_nvml > 0 else None
    except torch.cuda.OutOfMemoryError:
        result["oom"] = True
        torch.cuda.empty_cache()
        log(f"[vram] n={n_target}: OOM")
    result["seconds"] = round(_mono() - t0, 1)
    return result


def _fit_metric_gib(row) -> float | None:
    """The number a fit decision rides on: the most conservative available.

    ``max_memory_allocated`` undercounts -- it misses reserved fragmentation, the
    CUDA context, and gsplat's own non-torch allocations, all of which are what
    actually OOMs. Prefer the larger of process-local reserved and the (baseline-
    subtracted) device-wide NVML peak, which adds the CUDA context reserved cannot
    see. Never the raw allocated number.
    """
    if row["oom"]:
        return None
    cands = [c for c in (row.get("peak_reserved_gib"), row.get("peak_nvml_gib")) if c]
    return max(cands) if cands else row.get("peak_alloc_gib")


def _derive_budget(budget_gib, total_gib, margin_gib, reserve_gib, nvml_ok):
    """(budget_gib, source). Pure -- the VRAM ceiling minus the context margin and
    any concurrency reserve, or the explicit override. Split out so the arithmetic
    is testable without a GPU."""
    if budget_gib is not None:
        return float(budget_gib), "cli"
    source = "nvml-total-margin" if nvml_ok else "torch-total-margin"
    return round(total_gib - margin_gib - reserve_gib, 2), source


# Calibrated clone-vs-trained correction. The clone-at-N measurement is a LOWER
# BOUND on the real training peak: it skips the thousands of MCMC densify/relocate
# steps that fragment the CUDA allocator and spread gaussians into a higher-overlap
# trained distribution. Measured on the new Sam Office 2 depth build (2026-08-08): a
# real 1.3M-cap build peaked at 4.86 GiB where this sweep measured 2.38 -- a 2.04x
# gap (and the same "worst-case ~= 2x logged" the splat-vram-ceiling memory noted).
# The recommendation multiplies the measured peak by this before comparing to budget.
_DEFAULT_SAFETY_FACTOR = 2.0


def _estimated_real_peak(row, safety_factor):
    """The measured clone peak scaled to an ESTIMATE of the real training peak."""
    m = row.get("fit_metric_gib")
    return None if (row["oom"] or m is None) else round(m * safety_factor, 3)


def _recommend(rows, budget, safety_factor=1.0):
    """(recommended_cap, its_estimated_real_peak) = largest count whose ESTIMATED
    real peak (measured x safety_factor) fits the budget.

    Pure over the ladder rows. Recomputes fit here rather than trusting a possibly-
    stale row['fit'], so a caller can score any synthetic ladder. None when nothing
    fits.
    """
    fit = [r for r in rows if _estimated_real_peak(r, safety_factor) is not None
           and _estimated_real_peak(r, safety_factor) <= budget]
    if not fit:
        return None, None
    best = max(fit, key=lambda r: r["n"])
    return best["n"], _estimated_real_peak(best, safety_factor)


def _density_flags(seed_pts, recommended_cap, registered_ratio):
    """(capture_limited, effective_ceiling). MCMC densifies the COLMAP seed by a
    bounded factor, so a sparse seed may never REACH the cap; and a low
    registered_ratio means the cap was never the binding constraint (the new-video
    case). Pure."""
    effective_ceiling = None
    if recommended_cap is not None:
        effective_ceiling = int(min(recommended_cap, seed_pts * 20))
    capture_limited = registered_ratio is not None and registered_ratio < 0.5
    return capture_limited, effective_ceiling


def sweep_vram(model_dir, image_dir, *, budget_gib=None, margin_gib=0.8,
               reserve_gib=0.0, ladder=None, refine=True, sh_degree=3,
               depth_lambda=0.0, jitter_frac=0.5, worst_k=0, nvml_index=0,
               safety_factor=_DEFAULT_SAFETY_FACTOR, registered_ratio=None,
               log=lambda m: None, progress=lambda f: None) -> dict:
    """Find the max MCMC ``cap_max`` that fits VRAM, measured on a real COLMAP scene.

    Ascends a geometric ladder of gaussian counts, measuring the real per-view
    peak at each; stops at the first count that OOMs or exceeds the budget; then
    (``refine``) bisects the boundary for a tighter integer cap. ``budget_gib``
    defaults to ``NVML total - margin - reserve``; ``reserve_gib`` is the
    concurrency headroom to leave for a co-resident process (0 = the build runs
    isolated, which it does). Returns the full ladder table + a recommended cap +
    honest caveats.
    """
    import torch
    from ..slam.gpumem import Nvml

    if not torch.cuda.is_available():
        return {"ok": False, "error": "no CUDA GPU (gsplat/torch has no CPU path)"}

    nvml = Nvml(nvml_index)
    nvml_baseline = nvml.used_bytes() if nvml.ok else 0   # other processes, before we touch CUDA
    total_gib = round(nvml.total_bytes() / 2**30, 2) if nvml.ok else None
    device_name = nvml.name() if nvml.ok else torch.cuda.get_device_name(0)

    if not nvml.ok:
        # No NVML: fall back to torch's reported total for the budget derivation.
        total_gib = round(torch.cuda.get_device_properties(0).total_memory / 2**30, 2)
    budget, budget_source = _derive_budget(budget_gib, total_gib, margin_gib,
                                           reserve_gib, nvml.ok)

    rec, views, scene_scale = _load_views(model_dir, image_dir)
    long_edge = max(int(views[0]["w"]), int(views[0]["h"]))
    render_mode = "RGB+ED" if depth_lambda > 0 else "RGB"
    use_views = _worst_view_subset(views, worst_k) if worst_k else views

    ladder = list(ladder) if ladder else list(_DEFAULT_LADDER)
    ladder.sort()
    log(f"[vram] {len(views)} views, long_edge={long_edge}, seed "
        f"{rec.num_points3D()} pts, budget {budget:.2f} GiB ({budget_source})")

    rows = []
    for i, n in enumerate(ladder):
        row = measure_peak_at_count(
            rec, views, scene_scale, n, sh_degree=sh_degree, render_mode=render_mode,
            depth_lambda=depth_lambda, jitter_frac=jitter_frac,
            views_subset=use_views, nvml=nvml, nvml_baseline_bytes=nvml_baseline, log=log)
        fit_val = _fit_metric_gib(row)
        row["fit_metric_gib"] = round(fit_val, 3) if fit_val is not None else None
        est = _estimated_real_peak(row, safety_factor)
        row["estimated_real_peak_gib"] = est
        row["fit"] = (not row["oom"]) and est is not None and est <= budget
        rows.append(row)
        progress((i + 1) / len(ladder))
        log(f"[vram] n={n:>9} alloc={row['peak_alloc_gib']} reserved={row['peak_reserved_gib']} "
            f"nvml={row['peak_nvml_gib']} est_real={est} fit={row['fit']} oom={row['oom']} ({row['seconds']}s)")
        if row["oom"] or (est is not None and est > budget):
            break   # peak is monotone in N; nothing above this fits either

    recommended_cap, recommended_peak = _recommend(rows, budget, safety_factor)
    if recommended_cap is not None and refine:
        fail = next((r["n"] for r in rows if not r["fit"]), None)
        if fail is not None and fail > recommended_cap:
            recommended_cap, recommended_peak = _bisect_boundary(
                rec, views, scene_scale, recommended_cap, fail, budget,
                sh_degree, render_mode, depth_lambda, jitter_frac, use_views,
                nvml, nvml_baseline, safety_factor, log)

    seed_pts = rec.num_points3D()
    capture_limited, effective_ceiling = _density_flags(
        seed_pts, recommended_cap, registered_ratio)

    caveats = _caveats(capture_limited, safety_factor)
    warnings = []
    if capture_limited:
        warnings.append(
            f"registered_ratio={registered_ratio}: density is SfM-bound, not "
            f"VRAM-bound -- raising the cap adds no gaussians for this capture.")
    if effective_ceiling is not None and recommended_cap is not None \
            and effective_ceiling < recommended_cap:
        warnings.append(
            f"effective_ceiling {effective_ceiling} < recommended_cap {recommended_cap}: "
            f"the {seed_pts}-point seed cannot densify past ~{effective_ceiling}, so the "
            f"cap is not the binding constraint -- build at the ceiling, not the cap.")

    nvml.close()
    return {
        "ok": True, "device": device_name,
        "budget_gib": budget, "budget_source": budget_source, "total_gib": total_gib,
        "margin_gib": margin_gib, "reserve_gib": reserve_gib, "safety_factor": safety_factor,
        "scene": {"model_dir": str(model_dir), "n_colmap_points": int(seed_pts),
                  "n_views": len(views), "long_edge": long_edge, "sh_degree": sh_degree,
                  "scene_scale": round(scene_scale, 3), "render_mode": render_mode,
                  "registered_ratio": registered_ratio, "worst_k": worst_k},
        "ladder": rows,
        "recommended_cap": recommended_cap,
        "recommended_est_real_peak_gib": recommended_peak,   # measured x safety_factor
        "capture_limited": capture_limited,
        "effective_ceiling": effective_ceiling,
        "caveats": caveats, "warnings": warnings,
    }


def _bisect_boundary(rec, views, scene_scale, lo, hi, budget, sh_degree, render_mode,
                     depth_lambda, jitter_frac, use_views, nvml, nvml_baseline,
                     safety_factor, log, rounds=3):
    """Narrow the fit boundary between a known-fit ``lo`` and known-fail ``hi``.

    Returns (best_fit_n, its_estimated_real_peak). A few extra measurements buy a
    tighter integer cap than the coarse ladder gives. Fit is on the estimated real
    peak (measured x safety_factor), same as the ladder.
    """
    best_n, best_peak = lo, None
    for _ in range(rounds):
        mid = (lo + hi) // 2
        if mid <= lo or mid >= hi:
            break
        row = measure_peak_at_count(
            rec, views, scene_scale, mid, sh_degree=sh_degree, render_mode=render_mode,
            depth_lambda=depth_lambda, jitter_frac=jitter_frac, views_subset=use_views,
            nvml=nvml, nvml_baseline_bytes=nvml_baseline, log=log)
        row["fit_metric_gib"] = _fit_metric_gib(row)
        est = _estimated_real_peak(row, safety_factor)
        if est is not None and est <= budget:
            best_n, best_peak, lo = mid, est, mid
        else:
            hi = mid
        log(f"[vram] bisect n={mid} fit={best_n == mid}")
    return best_n, best_peak


def _worst_view_subset(views, k):
    """The k views most likely to peak: nearest median scene depth + fullest frustum.

    A proxy only (cheap, from poses/intrinsics) -- and using ANY subset risks
    missing the true worst frame, so this is an optimization the caller opts into,
    never the default. Ranks by inverse camera-to-scene-centroid distance.
    """
    import numpy as np
    centers = np.stack([np.linalg.inv(v["viewmat"])[:3, 3] for v in views])
    centroid = centers.mean(0)
    dist = np.linalg.norm(centers - centroid, axis=1) + 1e-6
    order = np.argsort(1.0 / dist)[::-1]   # nearest first
    return [views[i] for i in order[:k]]


def _load_gt(v, device):
    import numpy as np
    import torch
    from PIL import Image
    im = np.asarray(Image.open(v["image_path"]).convert("RGB"), np.float32) / 255.0
    return torch.from_numpy(im).to(device)


def _caveats(capture_limited, safety_factor):
    c = [
        f"The clone-at-N measurement is a LOWER BOUND on the real training peak: it "
        f"skips the MCMC densify/relocate steps that fragment the allocator and spread "
        f"gaussians into a higher-overlap trained distribution. Calibrated ~2x low (a "
        f"real 1.3M build hit 4.86 GiB vs 2.38 measured), so the recommendation scales "
        f"the measured peak by safety_factor={safety_factor}. Ground truth is a real "
        f"build's `vram=` log; re-derive the factor if the preset changes materially.",
        "Peak is the max over the MEASURED views; the real OOM is one worst-case "
        "frame's backward. Measuring all views (worst_k=0) is the only way to be sure.",
        "Fit is decided on reserved / device-wide NVML, never max_memory_allocated "
        "(which misses fragmentation + the CUDA context + gsplat's non-torch allocs).",
        "The measurement must match the target build's long_edge / sh_degree / "
        "depth_lambda -- all are VRAM levers -- or the cap is meaningless.",
        "Budget assumes the build runs ISOLATED; pass reserve_gib if it will share "
        "the card with roomscan-web (~1.7 GiB).",
    ]
    if capture_limited:
        c.append("This scene is CAPTURE-LIMITED (few frames registered): the VRAM cap "
                 "is not the binding constraint; improve SfM registration instead.")
    return c


def sweep_vram_from_video(video, *, work_dir=None, keep_work=False, preset=None,
                          log=lambda m: None, progress=lambda f: None, **sweep_kwargs) -> dict:
    """Run frames+SfM ONCE into a temp, then sweep on that model. The primary entry.

    SfM is CPU-minutes and identical across every ladder rung, so it runs once and
    the whole sweep reuses the model. Persist the work dir with ``keep_work`` to
    skip SfM on a re-run (or call ``sweep_vram`` directly with an existing
    ``model_dir``/``image_dir``).
    """
    import shutil
    import tempfile

    from .config import SplatPreset
    from .frames import extract_frames
    from .sfm import run_sfm

    video = Path(video)
    if not video.is_file():
        return {"ok": False, "error": f"video not found: {video}"}
    preset = preset or SplatPreset.load()
    owns_work = work_dir is None
    work_dir = Path(work_dir) if work_dir else Path(tempfile.mkdtemp(prefix="splat-vram-"))
    frames_dir = work_dir / "frames"
    try:
        extract_frames(video, frames_dir, fps=preset.fps, long_edge=preset.long_edge,
                       max_frames=preset.max_frames, log=log)
        sfm_stats = run_sfm(frames_dir, work_dir, matcher=preset.matcher,
                            sequential_overlap=preset.sequential_overlap, log=log)
        rep = sweep_vram(sfm_stats["model_dir"], frames_dir, sh_degree=preset.sh_degree,
                         depth_lambda=preset.depth_lambda,
                         registered_ratio=sfm_stats.get("registered_ratio"),
                         log=log, progress=progress, **sweep_kwargs)
        if rep.get("ok"):
            rep["sfm"] = sfm_stats
        return rep
    finally:
        if owns_work and not keep_work:
            shutil.rmtree(work_dir, ignore_errors=True)


def _mono() -> float:
    return time.monotonic()
