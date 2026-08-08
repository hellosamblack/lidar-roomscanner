# Splat quality improvements — design

**Date:** 2026-08-07 (corrected 2026-08-08)
**Status:** in progress (A shipped, B–E specced)
**Owner ask:** close the gap between our video-only Gaussian splat and a Scaniverse
capture of the same room. The owner will re-capture; this spec is the "in the
meantime" program plus the longer-term path.

> **Correction (2026-08-08):** the Scaniverse scan was taken with the **same Pixel
> 10 Pro XL** that shot our videos — **not** an iPhone-LiDAR device. The Pixel has
> **no ToF/LiDAR** sensor, so Scaniverse hit its clean 2.46 M-gaussian result from
> the *same camera* using photogrammetry + **ARCore pose tracking + ML/monocular
> depth**. The original framing below ("they have LiDAR, we don't") is therefore
> **wrong**: the gap is **capture quality (Scaniverse's real-time coverage UI) +
> method (ARCore poses + ML depth + a well-tuned trainer)**, all reachable without
> new hardware. Consequences: the **monocular depth prior (§D)** is promoted to the
> top method lever (it approximates what Scaniverse actually does), and the real
> "match Scaniverse" path is the **ARCore capture app (ROADMAP OFFLINE-4)**, not the
> ToF sensor (§E). §A/§B/§D/§E are corrected inline below.

## Problem — quantified

BUG-094 fixed the trainer (broken SSIM / clamp) so the reconstruction is a room and
not a random cloud. But it is still a **"galactic snowglobe"**: the room sits inside
a halo of faint, oversized floaters. A statistical comparison of our 1 M-gaussian
build against the owner's Scaniverse capture (`results/splats/scaniverse-splat/SamOffice.ply`,
2.46 M gaussians) of the same office makes the cause unambiguous:

| metric | Scaniverse | Ours (pre-tuning) |
|---|---|---|
| gaussians | 2.46 M | 1.0 M |
| median gaussian size | **3 mm** | **49 mm** (16×) |
| median opacity | **0.78** | **0.03** |
| fraction opacity < 0.1 | 11 % | **74 %** |
| 95 % within radius | 3.7 | **15.3** |
| solid + small + near | — | **0.9 %** |

Only **0.9 %** of our gaussians are opaque, small, and near the room. The other 99 %
is fog (74 % near-transparent) plus a far-floater halo (24 % beyond 8 units, extremes
to ~10⁶). This is **under-constraint**, not un-pruned junk: photometry-only 3DGS from
a single video has nothing pinning gaussians to real surfaces, so the optimiser is
free to explain the images with translucent haze. Scaniverse avoids this **without
LiDAR** — with **ML/monocular depth + ARCore poses** and aggressive floater pruning
(same Pixel camera as ours; see the correction above).

**Priority order (measured, honest):** depth supervision (ML/monocular now, ToF later)
≫ better capture ≫ pipeline tuning. Tuning cleans up; it cannot manufacture the
geometry depth would. Because Scaniverse reaches its result from the *same camera*,
depth + capture are proven sufficient — no new sensor is required to close most of it.

## A. Tuning + floater cull — SHIPPED (this change)

Cheap wins on the *existing* video, all as `SplatPreset` knobs (fingerprinted):

- **`max_gaussians` 1 M → 2 M** (Scaniverse has 2.46 M; 2 M fits the 8 GB box).
- **`min_opacity` 0.005 → 0.05** — MCMC prunes faint gaussians every refine step, so
  the fog is relocated to useful surfaces during training instead of surviving.
- **`scale_reg` 0.01 → 0.02**, `opacity_reg` 0.01 — smaller, more decisive splats.
- **Post-train floater cull** (`cull_opacity` 0.12, `cull_radius_factor` 3.0): drop
  gaussians fainter than 0.12 or beyond 3× the camera-path radius from the scene
  centre — kills the transparent haze and the galactic halo deterministically.
- **Raise `long_edge` 1600 → ~2000** to use more of a 4K frame's detail (open item,
  measure COLMAP/VRAM cost first).

Acceptance: median opacity ↑ toward ~0.3+, fraction opacity<0.1 ↓ well below 74 %,
95 %-within-radius ↓ toward single digits, and the rendered interior visibly loses the
surrounding snow. This will improve but **not match** Scaniverse — that needs B/D/E.

## B. Capture guidance (owner re-capturing)

The single biggest lever after depth. Recommendations, to be written into a capture
doc / in-app guidance:

- **Main 1× camera, not 0.5× ultrawide.** Ultrawide's barrel distortion is not handled
  by the current pinhole pipeline (see C), and its sensor is softer/lower-res.
- **4K60, not 8K30.** 60 fps halves motion blur (the dominant defect) and gives more
  frames to sample; 8K's resolution is discarded by the `long_edge` downscale anyway.
- **Move slowly, translate (don't rotate in place** — COLMAP needs parallax), ~70–80 %
  overlap, each surface seen from 2–3 directions, even lighting, locked exposure/focus,
  one fixed zoom for the whole take. Last video registered only 161/287 frames, largely
  from blur + fast panning.

## C. COLMAP undistortion stage (pipeline)

`sfm.py` currently runs extract → match → mapper and feeds `Camera.calibration_matrix()`
to gsplat as a **pinhole** model, silently ignoring COLMAP's estimated distortion
coefficients. Add `pycolmap.undistort_images` (or `image_undistorter`) after mapping and
train on the undistorted images + pinhole intrinsics. Benefits: (1) removes residual
main-camera distortion, sharpening planes; (2) makes wider lenses (incl. 0.5×) usable.
Low risk, self-contained.

## D. Monocular depth prior (Depth-Anything-V2) — top method lever

Anchor gaussians to surfaces without new hardware, using an off-the-shelf monocular depth
model as a soft geometric prior. **This is essentially what Scaniverse itself relies on**
(ML depth, no LiDAR), so it is the most direct way to close the method gap — not a
stopgap. Implemented (`depth_lambda`, default-off); being A/B'd on the Sam Office capture
(build 1 photometry vs build 2 depth) 2026-08-08.

- **Model:** Depth-Anything-V2 (small/base) run per training frame once, cached. New
  optional dep in the `splat` extra; runs on the same GPU.
- **Loss:** render depth via gsplat `render_mode="RGB+ED"` (expected depth). Monocular
  depth is only correct up to per-frame scale+shift, so align it to the rendered depth
  by a least-squares (scale, shift) fit each step (standard depth-regularised 3DGS), then
  add `depth_lambda * L1(aligned_mono_depth, render_depth)`. Weight annealed so photometry
  dominates late.
- **Effect:** pins gaussians to a plausible surface early, collapsing the translucent
  volume — the same ML-depth mechanism Scaniverse uses (no LiDAR), applied here.
- **Risk:** monocular depth is biased/warped; keep `depth_lambda` modest and always
  scale-shift-align. Gate acceptance on the same metric table as A.

## E. ToF depth fusion (Phase 7) — a metric upgrade, NOT the Scaniverse gap

Note the corrected framing: Scaniverse has **no** LiDAR, so ToF is **not** what makes it
better — the "match Scaniverse" path is **ARCore posed frames + ML depth (ROADMAP
OFFLINE-4)**, which uses the same handset. ToF fusion remains valuable for *this project*
as a **metric** upgrade over ML depth: this device has a depth sensor, and Roadmap Phase 7
/ OFFLINE-1 is "COLMAP + ToF depth priors + depth-regularised 3DGS" — a **time-synced
ToF-depth + RGB-video capture**, ToF depth projected into each camera frame as D's prior,
replacing the monocular estimate with a *metric* one. Still blocked on the unbuilt pieces
in ROADMAP DC-I: a rigid phone/camera mount, a hand-eye extrinsic `T_lidar_camera`, and
video↔ToF temporal sync. Once D exists, E (or the ARCore-depth variant) is "swap the depth
source" — so D is the right next step and de-risks both.

## Metrics to track (all builds)

Report on every splat build (extend the manifest `stats`): median opacity, fraction
opacity < 0.1, median/p99 gaussian scale, radius percentiles, and the "solid + small +
near" fraction. These are the numbers that separate "room in a snowglobe" from "clean
room", and they let A–E be compared objectively rather than by eyeballing renders.
