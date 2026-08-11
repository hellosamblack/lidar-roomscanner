#!/usr/bin/env python3
"""Render an INRIA 3DGS .ply to a PNG on the GPU (gsplat), headless.

    host/.venv/bin/python host/tools/splat_render.py results/splats/<slug>/point_cloud.ply out.png

This box has no display and llvmpipe can't drive the browser splat viewer at these
counts, so we rasterize directly with gsplat on CUDA -- the same rasterizer the
trainer uses. Colors are the view-independent DC term (SH band 0), which is plenty
for a coverage/quality comparison and sidesteps the higher-SH/scale/quat decode. The
camera auto-frames the cloud: `--transform` (a splat manifest's 4x4) supplies "up",
else up is estimated from the cloud's thinnest principal axis. `--azimuth/--elevation`
orbit; `--views` renders several angles into `out_<i>.png`.
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

SH_C0 = 0.28209479177387814


def load_ply(path):
    """Return (means, scales, quats, opacities, rgb) as numpy from an INRIA 3DGS ply."""
    from plyfile import PlyData
    v = PlyData.read(str(path))["vertex"]
    means = np.stack([v["x"], v["y"], v["z"]], axis=1).astype(np.float32)
    scales = np.exp(np.stack([v[f"scale_{i}"] for i in range(3)], axis=1)).astype(np.float32)
    quats = np.stack([v[f"rot_{i}"] for i in range(4)], axis=1).astype(np.float32)
    quats /= (np.linalg.norm(quats, axis=1, keepdims=True) + 1e-9)
    opac = 1.0 / (1.0 + np.exp(-v["opacity"].astype(np.float32)))          # sigmoid
    dc = np.stack([v[f"f_dc_{i}"] for i in range(3)], axis=1).astype(np.float32)
    rgb = np.clip(dc * SH_C0 + 0.5, 0.0, 1.0)                              # inverse of _rgb_to_sh_dc
    return means, scales, quats, opac, rgb


def _up_from_transform(transform):
    """World-frame up axis from a splat manifest's upright transform (Y-up display)."""
    R = np.asarray(transform, np.float32)[:3, :3]
    R = R / (np.linalg.norm(R[:, 0]) + 1e-9)      # strip the uniform scale
    return R.T @ np.array([0, 1, 0], np.float32)  # display-up mapped back to world


def _up_from_cloud(means):
    """Fallback: the thinnest principal axis of the cloud (a room's up = its flattest spread)."""
    c = means - means.mean(0)
    _, _, vt = np.linalg.svd(c[:: max(1, len(c) // 20000)], full_matrices=False)
    return vt[2] / (np.linalg.norm(vt[2]) + 1e-9)


def _look_at(eye, target, up):
    """World->camera matrix, OpenCV convention (x right, y down, z forward)."""
    f = target - eye
    f /= np.linalg.norm(f) + 1e-9
    r = np.cross(f, up)
    r /= np.linalg.norm(r) + 1e-9
    d = np.cross(f, r)                       # camera-down = f x right
    R = np.stack([r, d, f], axis=0)          # rows = cam axes in world
    vm = np.eye(4, dtype=np.float32)
    vm[:3, :3] = R
    vm[:3, 3] = -R @ eye
    return vm


def _auto_camera(means, up, azimuth_deg, elevation_deg, w, h, dist_factor=2.2):
    center = np.median(means, axis=0).astype(np.float32)
    radius = float(np.percentile(np.linalg.norm(means - center, axis=1), 90)) or 1.0
    up = up / (np.linalg.norm(up) + 1e-9)
    # a horizontal basis perpendicular to up
    ref = np.array([1, 0, 0], np.float32)
    if abs(np.dot(ref, up)) > 0.9:
        ref = np.array([0, 0, 1], np.float32)
    x = np.cross(up, ref); x /= np.linalg.norm(x) + 1e-9
    y = np.cross(up, x)
    az, el = np.radians(azimuth_deg), np.radians(elevation_deg)
    horiz = np.cos(az) * x + np.sin(az) * y
    view_dir = np.cos(el) * horiz + np.sin(el) * up
    eye = center + view_dir * (radius * dist_factor)
    vm = _look_at(eye, center, up)
    focal = 0.9 * max(w, h)
    K = np.array([[focal, 0, w / 2], [0, focal, h / 2], [0, 0, 1]], np.float32)
    return vm, K


def render(ply, out_png, *, transform=None, azimuth=35.0, elevation=25.0,
           width=1280, height=960, bg=1.0, opacity_min=0.0, core_pct=100.0,
           iso_scale=0.0, log=lambda m: None):
    import torch
    import gsplat

    means, scales, quats, opac, rgb = load_ply(ply)
    n0 = len(means)
    keep = opac >= opacity_min
    if core_pct < 100.0:   # drop the floater halo: keep gaussians within core_pct
        c = np.median(means[keep] if keep.any() else means, axis=0)
        d = np.linalg.norm(means - c, axis=1)
        keep &= d <= np.percentile(d[opac >= opacity_min] if (opac >= opacity_min).any() else d, core_pct)
    means, scales, quats, opac, rgb = means[keep], scales[keep], quats[keep], opac[keep], rgb[keep]
    if iso_scale > 0:   # clamp to round blobs: kills the oversized-anisotropic "needles"
        scales = np.full_like(scales, iso_scale)
        quats = np.tile(np.array([1, 0, 0, 0], np.float32), (len(means), 1))
        opac = np.clip(opac + 0.5, 0, 1)          # firm up so points read as solid coverage
    log(f"[render] {len(means)}/{n0} gaussians from {Path(ply).name} "
        f"(opacity>={opacity_min}, core {core_pct}%, iso_scale={iso_scale})")
    # Frame on the kept (confident) gaussians so floaters don't push the camera out.
    up = _up_from_transform(transform) if transform is not None else _up_from_cloud(means)
    vm, K = _auto_camera(means, up, azimuth, elevation, width, height)

    dev = "cuda"
    t = lambda a: torch.from_numpy(np.ascontiguousarray(a)).to(dev)
    img, alpha, _ = gsplat.rasterization(
        t(means), t(quats), t(scales), t(opac), t(rgb),
        t(vm)[None], t(K)[None], width, height, packed=True, render_mode="RGB")
    # Composite over a flat background using the coverage alpha (version-robust:
    # avoids gsplat's shape-picky `backgrounds` argument).
    comp = img[0] * alpha[0] + bg * (1.0 - alpha[0])
    arr = (comp.clamp(0, 1).cpu().numpy() * 255).astype(np.uint8)
    _save_png(arr, out_png)
    log(f"[render] wrote {out_png}")
    return {"ok": True, "gaussians": int(len(means)), "out": str(out_png),
            "azimuth": azimuth, "elevation": elevation}


def _save_png(arr, path):
    from PIL import Image
    Image.fromarray(arr).save(str(path))


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("ply")
    ap.add_argument("out", help="output PNG (or prefix when --views > 1)")
    ap.add_argument("--transform", help="splat manifest.json to read the upright transform from")
    ap.add_argument("--azimuth", type=float, default=35.0)
    ap.add_argument("--elevation", type=float, default=25.0)
    ap.add_argument("--width", type=int, default=1280)
    ap.add_argument("--height", type=int, default=960)
    ap.add_argument("--opacity-min", type=float, default=0.0,
                    help="render only gaussians at/above this opacity (hides floaters)")
    ap.add_argument("--core-pct", type=float, default=100.0,
                    help="keep only gaussians within this distance-percentile of the core")
    ap.add_argument("--iso-scale", type=float, default=0.0,
                    help="render every gaussian as a round blob this size (m); kills needles, "
                         "turns the splat into a clean colored coverage cloud")
    ap.add_argument("--views", type=int, default=1, help="render N orbit angles into out_<i>.png")
    args = ap.parse_args(argv)

    transform = None
    if args.transform:
        transform = json.loads(Path(args.transform).read_text()).get("transform")

    def log(m):
        print(m, file=sys.stderr, flush=True)

    kw = dict(transform=transform, elevation=args.elevation, width=args.width,
              height=args.height, opacity_min=args.opacity_min, core_pct=args.core_pct,
              iso_scale=args.iso_scale, log=log)
    if args.views > 1:
        stem = Path(args.out)
        for i in range(args.views):
            az = args.azimuth + i * (360.0 / args.views)
            render(args.ply, stem.with_name(f"{stem.stem}_{i}{stem.suffix}"), azimuth=az, **kw)
    else:
        render(args.ply, args.out, azimuth=args.azimuth, **kw)
    return 0


if __name__ == "__main__":
    sys.exit(main())
