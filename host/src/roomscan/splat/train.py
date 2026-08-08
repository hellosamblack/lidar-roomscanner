"""Train a 3D Gaussian Splat from a COLMAP model + the source frames.

Self-contained gsplat MCMC trainer -- deliberately NOT nerfstudio/gsplat's
``simple_trainer.py`` (which drags in nerfview/tyro/viser and a fixed dataset
layout).  It reads the ``pycolmap`` reconstruction directly, initialises
gaussians from the sparse cloud, optimises means/scales/quats/opacities/SH
against the frames, and exports the standard INRIA ``.ply`` (``f_dc_*``/
``f_rest_*``/``opacity``/``scale_*``/``rot_*``) that ``@mkkellogg/gaussian-splats-3d``
loads.  The MCMC ``cap_max`` is the VRAM governor on this 8 GB box.

Heavy imports (torch/gsplat) are function-local so the rest of the package stays
importable without the training stack.
"""
from __future__ import annotations

import math
from pathlib import Path

import numpy as np

SH_C0 = 0.28209479177387814   # Y_0^0, the constant SH basis -- rgb<->dc conversion


def _rgb_to_sh_dc(rgb01: np.ndarray) -> np.ndarray:
    return (rgb01 - 0.5) / SH_C0


def _knn_mean_dist(xyz, k: int = 3, chunk: int = 4096):
    """Mean distance to the ``k`` nearest neighbours, chunked to bound VRAM.

    Used to size initial gaussian scales so each ellipsoid roughly fills the gap
    to its neighbours -- the standard 3DGS scale init.
    """
    import torch
    n = xyz.shape[0]
    out = torch.empty(n, device=xyz.device)
    for i in range(0, n, chunk):
        d = torch.cdist(xyz[i:i + chunk], xyz)            # (chunk, n)
        knn = d.topk(min(k + 1, n), largest=False).values  # includes self (0)
        out[i:i + chunk] = knn[:, 1:].mean(dim=1)
    return out.clamp_min(1e-7)


def _load_views(model_dir, image_dir):
    """Return (views, scene_scale). Each view: dict(K, w, h, viewmat, image_path)."""
    import pycolmap
    rec = pycolmap.Reconstruction(str(model_dir))
    views = []
    centers = []
    for img in rec.images.values():
        if not img.has_pose:
            continue
        cam = rec.cameras[img.camera_id]
        w2c = np.eye(4, dtype=np.float32)
        w2c[:3, :4] = img.cam_from_world().matrix().astype(np.float32)
        views.append({"K": cam.calibration_matrix().astype(np.float32),
                      "w": int(cam.width), "h": int(cam.height),
                      "viewmat": w2c, "image_path": str(Path(image_dir) / img.name)})
        centers.append(np.linalg.inv(w2c)[:3, 3])
    if not views:
        raise RuntimeError("reconstruction has no posed images")
    centers = np.stack(centers)
    scene_scale = float(np.linalg.norm(centers - centers.mean(0), axis=1).mean()) or 1.0
    return rec, views, scene_scale


def _init_gaussians(rec, sh_degree, device):
    import torch
    pts = rec.points3D
    xyz = np.stack([p.xyz for p in pts.values()]).astype(np.float32)
    rgb = np.stack([np.asarray(p.color, np.float32) for p in pts.values()]) / 255.0
    means = torch.from_numpy(xyz).to(device)
    dist = _knn_mean_dist(means)
    K = (sh_degree + 1) ** 2
    sh0 = torch.from_numpy(_rgb_to_sh_dc(rgb)).float().to(device).unsqueeze(1)  # (N,1,3)
    shN = torch.zeros(means.shape[0], K - 1, 3, device=device)
    params = torch.nn.ParameterDict({
        "means": torch.nn.Parameter(means),
        "scales": torch.nn.Parameter(torch.log(dist).unsqueeze(-1).repeat(1, 3)),
        "quats": torch.nn.Parameter(torch.tensor([1., 0, 0, 0], device=device).repeat(means.shape[0], 1)),
        "opacities": torch.nn.Parameter(torch.logit(torch.full((means.shape[0],), 0.1, device=device))),
        "sh0": torch.nn.Parameter(sh0),
        "shN": torch.nn.Parameter(shN),
    }).to(device)
    return params


def _gaussian_window(channels: int, device):
    import torch
    coords = torch.arange(11, device=device) - 5
    g = torch.exp(-(coords ** 2) / (2 * 1.5 ** 2))
    g = g / g.sum()
    return (g[:, None] * g[None, :]).expand(channels, 1, 11, 11).contiguous()


def _ssim(a, b, window):
    """Standard SSIM over an 11x11 Gaussian window, per channel.

    The variances are **clamped >= 0**: the naive ``E[x^2] - E[x]^2`` form dips a
    hair negative from float rounding, which makes the SSIM ratio (and its
    gradient) blow up and silently destroys training -- an earlier box-filter
    version without the clamp drove the loss UP and produced a floaters-only
    "galaxy" (BUG-094). Inputs are ``(1, 3, H, W)``.
    """
    import torch
    import torch.nn.functional as F
    mu_a = F.conv2d(a, window, padding=5, groups=3)
    mu_b = F.conv2d(b, window, padding=5, groups=3)
    va = (F.conv2d(a * a, window, padding=5, groups=3) - mu_a ** 2).clamp_min(0)
    vb = (F.conv2d(b * b, window, padding=5, groups=3) - mu_b ** 2).clamp_min(0)
    vab = F.conv2d(a * b, window, padding=5, groups=3) - mu_a * mu_b
    c1, c2 = 0.01 ** 2, 0.03 ** 2
    return (((2 * mu_a * mu_b + c1) * (2 * vab + c2)) /
            ((mu_a ** 2 + mu_b ** 2 + c1) * (va + vb + c2))).mean()


def _ssi_normalize(d):
    """Median-center, MAD-scale -> scale/shift invariant. `d` is a 1-D tensor."""
    m = d.median()
    return (d - m) / ((d - m).abs().mean() + 1e-6)


def _load_mono_depths(views, model_name, device, log):
    """Run a monocular depth model once per frame; cache inverse-depth on CPU.

    Depth-Anything-V2 predicts *inverse* depth (larger = nearer), affine-ambiguous.
    We store it at each frame's resolution and align it scale/shift-invariantly at
    loss time, so absolute scale never matters.
    """
    import torch
    import torch.nn.functional as F
    from transformers import pipeline
    from PIL import Image
    log(f"[train] loading monocular depth model {model_name}")
    pipe = pipeline("depth-estimation", model=model_name, device=0 if device == "cuda" else -1)
    out = {}
    for v in views:
        pred = pipe(Image.open(v["image_path"]).convert("RGB"))["predicted_depth"].float()
        if pred.dim() == 2:
            pred = pred[None, None]
        pred = F.interpolate(pred, size=(v["h"], v["w"]), mode="bilinear", align_corners=False)
        out[v["image_path"]] = pred[0, 0].cpu()   # (H,W) inverse depth
    # Free the depth model off the GPU before training -- depths are cached on CPU,
    # and the 8 GB card needs every MiB for the gaussians (a depth build already
    # carries the extra RGB+ED render channel).
    del pipe
    torch.cuda.empty_cache()
    return out


def _depth_loss(render_depth, mono_inv, alpha):
    """Scale/shift-invariant L1 between rendered and monocular depth.

    Only where a surface was actually rendered (alpha > 0.5). Rendered forward
    depth is inverted to match the monocular inverse-depth convention, then both
    masked regions are normalised and compared -- this supervises depth *shape*,
    not absolute scale (which the video cannot provide).
    """
    mask = alpha > 0.5
    if int(mask.sum()) < 100:
        return render_depth.sum() * 0.0
    inv = 1.0 / render_depth[mask].clamp_min(1e-3)
    return (_ssi_normalize(inv) - _ssi_normalize(mono_inv[mask])).abs().mean()


def train_splat(model_dir: str | Path, image_dir: str | Path, out_ply: str | Path, *,
                sh_degree: int = 3, max_gaussians: int = 2_000_000, iters: int = 15_000,
                min_opacity: float = 0.05, opacity_reg: float = 0.01, scale_reg: float = 0.02,
                cull_opacity: float = 0.12, cull_radius_factor: float = 3.0,
                depth_lambda: float = 0.0, depth_model: str = "depth-anything/Depth-Anything-V2-Small-hf",
                log=lambda m: None, progress=lambda f: None) -> dict:
    """Optimise a splat and write ``out_ply``. Returns a stats dict."""
    import torch
    import gsplat
    from PIL import Image

    device = "cuda" if torch.cuda.is_available() else "cpu"
    if device == "cpu":
        raise RuntimeError("splat training needs a CUDA GPU (gsplat has no CPU path)")
    torch.manual_seed(0)
    # Reset the CUDA peak counter so `max_memory_allocated` below reports THIS run's
    # peak VRAM -- the headroom number on the 8 GB card, surfaced in stats + logs.
    torch.cuda.reset_peak_memory_stats()

    rec, views, scene_scale = _load_views(model_dir, image_dir)
    params = _init_gaussians(rec, sh_degree, device)
    log(f"[train] {params['means'].shape[0]} init gaussians, {len(views)} views, "
        f"scene_scale={scene_scale:.2f}, cap={max_gaussians}")

    means_lr = 1.6e-4 * scene_scale
    lrs = {"means": means_lr, "scales": 5e-3, "quats": 1e-3,
           "opacities": 5e-2, "sh0": 2.5e-3, "shN": 2.5e-3 / 20}
    optimizers = {k: torch.optim.Adam([{"params": params[k], "lr": lr, "name": k}], eps=1e-15)
                  for k, lr in lrs.items()}

    strategy = gsplat.MCMCStrategy(cap_max=max_gaussians, refine_stop_iter=int(iters * 0.9),
                                   min_opacity=min_opacity, verbose=False)
    strategy.check_sanity(params, optimizers)
    state = strategy.initialize_state()
    ssim_window = _gaussian_window(3, device)

    # Cache frames on CPU as float tensors; move each to GPU per step.
    gts = {}
    for v in views:
        im = np.asarray(Image.open(v["image_path"]).convert("RGB"), np.float32) / 255.0
        gts[v["image_path"]] = torch.from_numpy(im)   # (H,W,3)

    mono = _load_mono_depths(views, depth_model, device, log) if depth_lambda > 0 else None
    render_mode = "RGB+ED" if depth_lambda > 0 else "RGB"

    for step in range(iters):
        v = views[torch.randint(len(views), (1,)).item()]
        gt = gts[v["image_path"]].to(device)          # (H,W,3)
        viewmat = torch.from_numpy(v["viewmat"]).to(device)[None]
        K = torch.from_numpy(v["K"]).to(device)[None]
        active_sh = min(sh_degree, step // 1000)
        colors = torch.cat([params["sh0"], params["shN"]], dim=1)

        render, alpha, info = gsplat.rasterization(
            params["means"], params["quats"], torch.exp(params["scales"]),
            torch.sigmoid(params["opacities"]), colors, viewmat, K,
            v["w"], v["h"], sh_degree=active_sh, packed=True, render_mode=render_mode)
        # NO clamp before the loss: clamping to [0,1] zeros the gradient on every
        # saturated pixel (a bright ceiling / window fills much of an indoor frame),
        # which stalls convergence -- half of BUG-094.
        pred = render[0][..., :3]

        l1 = (pred - gt).abs().mean()
        ssim = _ssim(pred.permute(2, 0, 1)[None], gt.permute(2, 0, 1)[None], ssim_window)
        loss = 0.8 * l1 + 0.2 * (1.0 - ssim)
        if mono is not None:
            loss = loss + depth_lambda * _depth_loss(
                render[0][..., 3], mono[v["image_path"]].to(device), alpha[0][..., 0])
        # MCMC opacity + scale regularisation (the paper's terms): without them the
        # gaussians grow into blurry anisotropic needles that never resolve into
        # surfaces. Strengths are preset knobs (anti-snowglobe tuning).
        loss = loss + opacity_reg * torch.sigmoid(params["opacities"]).mean() \
                    + scale_reg * torch.exp(params["scales"]).mean()
        loss.backward()

        # Exponentially decay the means LR init->1% over the run (standard 3DGS).
        cur_means_lr = means_lr * (0.01 ** (step / max(1, iters)))
        for g in optimizers["means"].param_groups:
            g["lr"] = cur_means_lr
        for opt in optimizers.values():
            opt.step()
            opt.zero_grad(set_to_none=True)
        # gsplat's canonical order: densify/relocate + inject MCMC noise AFTER the
        # optimizer step (its own example omits step_pre_backward for MCMC).
        strategy.step_post_backward(params, optimizers, state, step, info, lr=cur_means_lr)

        if step % 200 == 0 or step == iters - 1:
            progress((step + 1) / iters)
            if step % 1000 == 0:
                log(f"[train] step {step}/{iters} loss={loss.item():.4f} "
                    f"n={params['means'].shape[0]} "
                    f"vram={torch.cuda.max_memory_allocated() / 2**30:.2f}GiB")

    # Post-train floater cull. Photometry-only 3DGS scatters faint, oversized
    # gaussians through empty space (the "snowglobe", BUG-094 follow-up). Drop the
    # near-transparent ones and anything far outside the camera-observed volume.
    n_trained = params["means"].shape[0]
    with torch.no_grad():
        cam_centers = np.stack([np.linalg.inv(v["viewmat"])[:3, 3] for v in views])
        center = torch.from_numpy(np.median(cam_centers, axis=0)).float().to(device)
        cam_radius = float(np.percentile(
            np.linalg.norm(cam_centers - np.median(cam_centers, axis=0), axis=1), 90)) or 1.0
        keep = (torch.sigmoid(params["opacities"]) >= cull_opacity) & \
               ((params["means"] - center).norm(dim=1) <= cull_radius_factor * cam_radius)
        if keep.any():
            for k in list(params.keys()):
                params[k] = torch.nn.Parameter(params[k][keep])
    n_final = params["means"].shape[0]
    log(f"[train] culled {n_trained - n_final} floaters "
        f"({100 * (n_trained - n_final) / max(1, n_trained):.0f}%), kept {n_final}")
    _export_ply(params, out_ply)
    # Peak memory this run: VRAM (the 8 GB ceiling) from CUDA, and host RSS from
    # getrusage (ru_maxrss is KiB on Linux). Reported so every build shows its real
    # headroom -- the OOM-confidence number, and what the resources panel echoes.
    import resource
    peak_vram_gib = round(torch.cuda.max_memory_allocated() / 2**30, 2)
    peak_rss_gib = round(resource.getrusage(resource.RUSAGE_SELF).ru_maxrss / 2**20, 2)
    log(f"[train] done: {n_final} gaussians -> {Path(out_ply).name} "
        f"(peak vram {peak_vram_gib} GiB, peak rss {peak_rss_gib} GiB)")
    return {"gaussians": int(n_final), "gaussians_trained": int(n_trained),
            "iters": int(iters), "views": len(views),
            "sh_degree": int(sh_degree), "scene_scale": round(scene_scale, 3),
            "final_loss": round(float(loss.item()), 4),
            "peak_vram_gib": peak_vram_gib, "peak_rss_gib": peak_rss_gib}


def _export_ply(params, out_ply: str | Path) -> None:
    """Write the standard INRIA 3DGS ply (the layout GaussianSplats3D expects)."""
    import torch
    from plyfile import PlyData, PlyElement

    with torch.no_grad():
        xyz = params["means"].detach().cpu().numpy()
        f_dc = params["sh0"].detach().transpose(1, 2).flatten(1).cpu().numpy()   # (N,3)
        f_rest = params["shN"].detach().transpose(1, 2).flatten(1).cpu().numpy()  # (N,3*(K-1))
        opac = params["opacities"].detach().cpu().numpy().reshape(-1, 1)
        scale = params["scales"].detach().cpu().numpy()
        rot = torch.nn.functional.normalize(params["quats"].detach()).cpu().numpy()

    n = xyz.shape[0]
    fields = ["x", "y", "z", "nx", "ny", "nz"]
    fields += [f"f_dc_{i}" for i in range(f_dc.shape[1])]
    fields += [f"f_rest_{i}" for i in range(f_rest.shape[1])]
    fields += ["opacity"]
    fields += [f"scale_{i}" for i in range(scale.shape[1])]
    fields += [f"rot_{i}" for i in range(rot.shape[1])]

    data = np.concatenate([xyz, np.zeros((n, 3), np.float32), f_dc, f_rest, opac, scale, rot], axis=1)
    dtype = [(f, "f4") for f in fields]
    elem = np.empty(n, dtype=dtype)
    for i, f in enumerate(fields):
        elem[f] = data[:, i]
    out_ply = Path(out_ply)
    out_ply.parent.mkdir(parents=True, exist_ok=True)
    PlyData([PlyElement.describe(elem, "vertex")]).write(str(out_ply))
