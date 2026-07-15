---
name: cuda-at-scale-validation
description: "GPU SLAM validation results — 2.1x compute speedup, 4 latent CUDA bugs, live-view is architecture-bound not compute-bound"
metadata: 
  node_type: memory
  type: project
  originSessionId: b8045cc7-1c49-425d-a8b0-6915761e2502
---

Running the device-parameterized Phase-6 SLAM on a REAL GPU (via the WSL container, [[gpu-cuda-build-blocker]]) for the first time surfaced that it had **never actually executed on CUDA** — four latent bugs came out one at a time (2026-07-13):

1. `InverseTransformation` — VoxelBlockGrid integrate/raycast/compute_unique_block_coordinates require intrinsic/extrinsic on **CPU:0** even when the grid is on CUDA (tsdf.py `_CPU`). Fixed 8258f2d.
2. `NearestNeighborSearch.hybrid_index()` — needs a **radius** arg on GPU that CPU treats as optional (odometry.py `_translation_icp`). Fixed 8258f2d.
3. **CUDA marching-cubes mesh/point-cloud extraction OOMs** at scale (~25k blocks → 11 GB): must `.cpu()` the VBG for extraction (Open3D's own recommendation). Per-frame integrate/raycast stay on GPU; only the throttled display extraction moves to host. Fixed d229a58 + `tools/slam-container/cuda_smoke.py` regression guard.
4. **GPU memory exhaustion over a long scan** — ParallelFor OOM, GPU creeps to ~11.7 GB over the 68 m walk. Raycast is already frustum-bounded (mapper.py:164) and the 40k-block grid is only ~410 MB, so the growth is Open3D's **CUDA caching allocator + per-frame temporaries** never released. Suspected fix: periodic `o3d.core.cuda.release_cache()`. **STILL OPEN** — deferred to a GPU-hardening sub-project.

**Numbers (RTX 4080, 600-frame capped run, `verify_e2e.py --max-frames`):** CUDA median **8.85 ms** vs CPU **18.94 ms** = **2.1× per-step speedup**. Degradation over the run: **GPU 1.03× (flat)** vs **CPU 1.24× (climbs 24%)** — GPU flattens the compute-degradation curve.

**Key strategic insight — the live-view fps goal is architecture-bound, not compute-bound.** Wall-clock: GPU pass 35 s vs CPU 20 s — the remote path was *slower end-to-end despite faster compute*, dominated by the 5 ms poll, sending the **full trajectory every frame**, and growing **mesh extraction**. New scan data is sensor-limited to **~28 fps** (I3C ceiling), which CPU (18 ms) already meets; the **120 fps target is a viewport-render goal** (decouple render loop + mesh upload from data arrival, stream pose every frame + mesh async), independent of CPU vs GPU. GPU's real value: freeing host CPU cores + faster offline/full-scan passes. Owner directive (2026-07-13): live view ≥30 fps, ideally 120+, snappy/smooth, no final-quality or preview-quality loss. Plan is "both, sequenced": get numbers ✅ → rendering-first for live view → GPU hardening for offline. **UPDATE 2026-07-14: the rendering-first live-view step is now IMPLEMENTED — see [[live-view-fps-rendering]]** (off-thread mesh + pose/mesh transport split; code-complete + reviewed, runtime fps numbers still UNVERIFIED). GPU-memory OOM (#4 above) remains the open GPU-hardening item.
