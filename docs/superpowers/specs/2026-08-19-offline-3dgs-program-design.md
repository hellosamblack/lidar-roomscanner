# Offline 3DGS program — posed captures, depth regularization, CAD extraction

**Status:** Specced (priority/later)
**Date:** 2026-08-19
**Issues:** #132 (COLMAP pose priors + depth-regularized 3DGS), #133 (SuGaR), #134
(Nerfstudio/Splatfacto)
**Companion specs:** Android capture app (`2026-08-19-android-capture-design.md` — the
posed-capture source), splat quality improvements
(`2026-08-07-splat-quality-improvements-design.md` — items B–E; this spec supersedes
nothing there, it sequences the framework decisions around it)

## Reframe (binding, from the 2026-08-08 correction)

The Scaniverse comparison scan was taken with the **same Pixel 10 Pro XL** — no
LiDAR/ToF hardware advantage. Its 2.46 M-gaussian, 3 mm-median result comes from capture
UX + ARCore poses + ML depth + a tuned trainer. Our measured gap (median gaussian 49 mm
vs 3 mm, median opacity 0.03 vs 0.78, 74% of gaussians under 0.1 opacity) is
**under-constraint**, not missing hardware. Priority order, already measured and stated
in the splat-quality spec: **depth supervision ≫ better capture ≫ pipeline tuning.**
Every framework decision below serves that order.

## The three issues are one program with a shared substrate

All three want the same input: a **posed image set** (`transforms.json`-shaped: per-frame
pose + intrinsics (+ optional depth/confidence)) that does not depend on COLMAP SfM
succeeding on featureless painted walls (measured ceiling: 206/287 frames = 72%
registered on Sam Office; 161/287 at earlier settings — same video, different pipeline
config; say which when quoting). The posed set comes from RTAB-Map exports (#158 ingest —
shipped) or the ARCore capture app (#135). Owner decision 2026-08-11: Phase 7 captures
are taken with RTAB-Map; plan A (RTAB-Map-only frames) is measured **before** plan B
(RTAB-Map + separate 4K video) — do not skip A and lose the comparison.

## Decisions

1. **#132 splits in two.** The *pose-prior* half is unblocked and reframed: poses come
   from RTAB-Map/ARCore, and COLMAP's remaining role is refinement/undistortion — run
   `splat_sfm_probe` arms with posed-seeded COLMAP vs. poses-only vs. COLMAP-only, and
   let the registration ratio decide whether COLMAP stays in the pipeline at all. The
   *ToF-fusion* half (metric seed cloud, hand-eye `T_world_camera = T_world_lidar ×
   T_lidar_camera`, clock alignment) remains **blocked on DC-I (#146)** — rigid mount +
   extrinsic, undesigned. Nothing here changes that; the spec just stops the blocked
   half from blocking the unblocked half.
2. **Depth regularization lands in the existing `roomscan.splat` trainer first, not in a
   new framework.** The `depth_lambda` / Depth-Anything-V2 path is already implemented
   (default-off) with the right mechanism (per-frame scale+shift alignment of inverse
   depth, annealed). The A/B on Sam Office is the next action — it predates any
   framework migration and its result calibrates whether frameworks are needed at all.
3. **#134 (Splatfacto) is a benchmark arm, never an adoption.** Its role: an external
   reference implementation of depth-regularized 3DGS (via dn-splatter-lineage flags) to
   sanity-check our trainer's ceiling on the same posed set. It was already rejected for
   CAD output (volumetric focus; mesh via rendered-depth TSDF fusion degrades geometry).
   One benchmark round, numbers recorded, issue closes.
4. **#133 (SuGaR) is the CAD-mesh arm, entered only when a watertight mesh is actually
   wanted** and only after the posed-set + depth-regularized substrate exists — SuGaR on
   under-constrained photometry-only gaussians would measure the substrate's absence,
   not SuGaR. Known risks go in the protocol up front: Poisson floaters (mitigate with
   `--project_mesh_on_surface_points`), VRAM spikes at high octree depth (the source
   report sized these against 16 GB; our card is 8 GiB — chunked projection is
   mandatory, and a remote build via the distributed-GPU spec is the fallback), and no
   `transforms.json` extrinsics field (hand-eye applied before writing the JSON).

## Benchmark plan

- **Harness:** `splat_sfm_probe` (registration ratio + sub-model fragmentation per
  config — the tool that turns "COLMAP fails on featureless walls" into a per-config
  number), `splat_vram_sweep` (is VRAM or capture the binding ceiling), `splat_compare`
  (metric diff vs. the Scaniverse ground-truth splat and vs. the Detailed-SLAM mesh),
  and the manifest stats from the splat-quality spec (median opacity, fraction < 0.1,
  scale percentiles, solid+small+near fraction) on **every** build — they are what
  separates "room in a snowglobe" from "clean room" without eyeballing.
- **The pivotal A/B (gate for everything downstream):** same footage, (i) COLMAP-only,
  (ii) ARCore/RTAB-Map-posed, (iii) posed + `depth_lambda` on. Frames registered/used,
  manifest stats, `splat_compare` geometry error.
- **SuGaR gate:** its extracted mesh must beat the Detailed-SLAM TSDF mesh on
  `splat_compare` geometric error against the ground-truth splat on the same room —
  otherwise the rig already makes a better mesh and SuGaR's product is redundant.
- **Needs operator:** every arm needs captures only the owner can record
  (`needs/capture`); the capture protocol rides the existing splat-quality §B guidance
  plus the capture-app spec.

## Kill criteria

- **COLMAP-in-the-loop (#132 pose-prior half):** killed if posed-only (ii) registers ≥
  the COLMAP arms within noise on two different rooms — COLMAP then retires from the
  pipeline and #132 narrows to the DC-I-blocked ToF-fusion half only.
- **ToF-fusion half (#132):** stays blocked on DC-I; killed outright if (iii) already
  reaches Scaniverse-class manifest stats (3 mm-class median scale, >0.5 median opacity)
  without metric seeding — ML depth + poses would have closed the gap the fusion was
  meant to close, and the mount/extrinsic work is unjustified.
- **#134:** closes after one benchmark round by design; killed early if the posed-set
  substrate never materializes (it has nothing to benchmark).
- **#133:** killed if the SuGaR gate fails (TSDF mesh wins), if 8 GiB + chunking cannot
  complete Poisson on a room-sized scene and the remote-GPU fallback isn't available, or
  if no consumer for a watertight CAD mesh emerges by the time the substrate exists
  (product-pull test — do not build the exporter before something needs the export).
