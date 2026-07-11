# Phase 6 — Real-time SLAM (PC): design spec

**Status:** approved design (2026-07-10). **Next:** implementation plan (`writing-plans`).
**Prereq reading:** `docs/coordinate-frames.md` (every pose/prior/constraint below lives in those frames),
`ROADMAP.md` Phase 6 (locked algorithm decisions), `docs/deprojector-validation.md` (FoV model).

## 1. Goal & scope

Turn the live depth + orientation + baro stream into a drift-suppressed camera trajectory and a fused 3D
map, in real time on CPU. Deliver it both **offline** (a CLI over recorded captures, the validation gate)
and **live** (a new map view in `roomscan-panel`).

**Locked by the roadmap (not re-litigated here):** SFLP quaternion as rotation prior → 3-DoF-constrained
**point-to-plane ICP, frame-to-model** against a TSDF raycast (Open3D tensor `t.pipelines.registration` +
`VoxelBlockGrid`), IR as an intensity channel, barometer as a **soft** 1-DoF Z constraint, CPU-first
(Windows wheel is CPU-only), KISS-ICP kept only as an offline odometry benchmark.

**In scope:** offline SLAM CLI + metrics; live panel map view; adopt `captures/phase6_motion_ref.bin`
(102 s, 3187 RAW + full SFLP/ENV, 0 CRC, 3332° angular travel) as the canonical validation dataset;
synthetic deterministic fixtures for CI.

**Out of scope (YAGNI):** loop closure / global pose-graph optimization (this is frame-to-model
*odometry* + TSDF fusion, KinectFusion-style — the roadmap specs no loop closure); multi-session mapping;
CUDA (revisit only if profiling shows VoxelBlockGrid blowing the ~35 ms budget — roadmap §Phase 6);
translation from accelerometer double-integration (drift — translation comes only from ICP).

## 2. Module structure — new `host/src/roomscan/slam/` subpackage

File-disjoint, independently testable units (mirrors the existing package style). No changes to the wire
protocol, `TransformStage`, `Deprojector`, `sources`, `decoder`, or `config` beyond additive hooks.

| Module | Responsibility | Key API (sketch) | Depends on |
|---|---|---|---|
| `intrinsics.py` | Pinhole intrinsic from Deprojector FoV | `pinhole(w, h, fov_h, fov_v) -> o3c.Tensor(3x3)` | — |
| `frames.py` | Poses/priors/constraints per `coordinate-frames.md` | `prior_pose(quat, t_prev) -> 4x4`; `baro_to_world_z(pa, ref_pa) -> float`; `world_axis_up() -> vec3` | `sensors.py` |
| `tsdf.py` | `VoxelBlockGrid` wrapper | `integrate(depth, intr, pose)`; `raycast(intr, pose) -> (points, normals)`; `extract_mesh()`; `extract_pcd()` | Open3D tensor |
| `odometry.py` | Point-to-plane ICP, frame-to-model, prior-initialized, fitness/RMSE gated, **rotation held at prior (3-DoF translation-only)** | `register(src_pcd, model_pcd, init_pose) -> Result{pose, fitness, rmse, ok}` | Open3D tensor |
| `mapper.py` | Per-frame orchestration + SLAM state + trajectory; tracking-lost fallback; baro-Z soft correction | `Mapper.step(depth, quat, pressure) -> FrameResult`; `.trajectory`, `.map_mesh()` | all above |
| `metrics.py` | Trajectory export (TUM/npy), per-frame timing, drift stats, KISS-ICP comparison | `summarize(traj, timings) -> report`; `compare_kiss(...)` | (optional) `kiss-icp` |

**CLI:** `roomscan-slam` (new console-script) — runs `Mapper` over a capture via the existing
`FileSource`/`pump`/`TransformStage` path, writes trajectory + fused mesh (`.ply`) + a timing/drift report;
`--benchmark` adds the KISS-ICP comparison.

## 3. Per-frame data flow

Reuses the existing decode→transform→deproject front end unchanged:

1. `RAW+CALIB → TransformStage → depth (h×w, perpendicular Z mm)` *(existing)*
2. `Deprojector.grid(depth) → organized pts (h,w,3) m + valid mask` *(existing)*; compute per-vertex
   **normals** from the organized grid (structured neighbor cross-products — cheap, no KDTree).
3. `quat (stream 9) → R_prior` body→world via the `coordinate-frames.md` sandwich.
4. **Predict:** `T_pred = [R_prior | t_prev]` (rotation from SFLP each frame; translation carried from the
   previous estimate — no velocity model in v1).
5. **Raycast** model cloud (+normals) from the `VoxelBlockGrid` at `T_pred` (frame-to-**model**).
6. **Point-to-plane ICP (3-DoF, translation-only):** source = current frame cloud, target = model
   raycast, init = `T_pred`. Per the roadmap's "3-DoF constrained" decision, rotation is **held** at the
   SFLP prior and ICP estimates only the 3-DoF translation. Implementation: run point-to-plane ICP from
   `T_pred` and re-impose `R_prior` on the rotation block after convergence (equivalently, a
   translation-only point-to-plane solve); a test asserts the rotation block stays equal to `R_prior`.
   Rationale: SFLP orientation is hardware-fused and trustworthy; letting ICP move rotation on thin
   2,268-pt frames is where drift/divergence enters.
7. **Baro soft-Z:** nudge `T_est` translation toward the barometric height along **world-up**
   (`world_axis_up()` — Open3D CV world −Y) with a low weight; never override ICP (indoor pressure drifts
   several Pa — `12 Pa/m`, HVAC/doors are noise).
8. **Integrate** the depth into the `VoxelBlockGrid` at `T_est` (skip if tracking lost, step 5 below).
9. Append `T_est` to the trajectory; periodically `extract_mesh()`/`extract_pcd()` for rendering.

**First frame:** no model yet → identity translation, `R_prior` rotation, integrate directly (bootstraps
the TSDF). Subsequent frames register against the growing model.

## 4. Live execution model (decided: worker thread)

SLAM per-frame (raycast + ICP + integrate) can occasionally exceed the ~35 ms budget. **A dedicated SLAM
worker thread** consumes depth frames from a **latest-wins slot** (drops frames under load, never blocks
acquisition or render), exactly like the existing reader-thread/GUI-thread contract in `panel.py`. The
panel renders the latest TSDF **mesh snapshot** + camera-trajectory polyline on the GUI thread via the
existing `set_on_tick_event`. Serial writes stay off both threads per the standing contract.

**Panel surface:** a new **"SLAM"** view mode (alongside the existing point-cloud/surface modes) with a
toggle to start/stop mapping, a "Clear map" control (reuses the existing clear-scan affordance), and HUD
fields for tracking state + per-frame SLAM ms. Presentation-layer only; no wire change.

## 5. Error handling

- **Tracking lost** (ICP `fitness` below / `rmse` above thresholds, or too few valid points): keep the
  `R_prior` rotation, **hold** translation (no extrapolation), mark the frame `tracking_lost`, and **do
  not integrate** into the TSDF (never corrupt the map with an unregistered frame). Bounded — the next
  good frame resumes. A run of N consecutive losses surfaces in the HUD/report, not a crash.
- **Empty/degenerate frame** (all-invalid depth): skip cleanly, carry pose.
- **No Open3D CUDA**: expected on Windows; the pipeline is CPU-only by design — assert nothing about CUDA.

## 6. Testing (TDD)

**Synthetic unit tests (CI, deterministic, no hardware, no 48 MB file):**
- `frames`: prior-pose composition round-trips a known quat; baro→world-Z sign/scale; world-up axis.
- `intrinsics`: FoV→pinhole matches the Deprojector's zone-center directions at center; documents the
  known corner divergence (linear vs pinhole).
- `tsdf`: integrate a known planar/box depth image, raycast it back → geometry matches within voxel size;
  mesh/pcd extraction non-empty.
- `odometry`: apply a **known** SE(3) transform to a synthetic cloud → ICP recovers it within tolerance;
  low-overlap input trips the fitness gate (`ok=False`).
- `mapper`: two-frame synthetic sequence with a known between-pose → trajectory recovers it; a lost-track
  frame holds pose and doesn't integrate.
- Small synthetic fixtures live in `host/tests/fixtures/` (tracked, per `.gitignore` exception).

**Offline validation on `captures/phase6_motion_ref.bin` (the gate):**
- Trajectory is smooth and closed-ish (start/end near each other for a room loop) — qualitative + a
  drift-magnitude number.
- **Real-time budget:** median per-frame SLAM time **< 35 ms** on CPU (roadmap requirement); report the
  full distribution and the integrate/raycast/ICP split.
- **Benchmark:** trajectory vs KISS-ICP on the same deprojected frames — same ballpark, no gross
  divergence (sanity, not ground truth).
- Fused mesh visually reconstructs the room (rendered PNG via the headless snapshotter pattern).

**Live smoke (HW):** `roomscan-panel` SLAM mode against the board builds a map in real time, HUD shows
per-frame ms in budget, tracking-lost recovers — one supervised soak.

## 7. Risks

- **Pinhole vs zone-center FoV:** the pinhole intrinsic approximates the Deprojector's per-zone tan model;
  they diverge up to ~6% of z at extreme corners (`docs/deprojector-validation.md`). Acceptable for TSDF
  fusion; if corner artifacts appear, the per-zone tan-table path already exists as an escape hatch.
- **Thin frames (2,268 pts):** low overlap can weaken ICP. Mitigations: frame-to-**model** (denser than
  frame-to-frame), the SFLP prior anchoring rotation, and normals from the organized grid.
- **TNR state discontinuity across dropped frames** (documented Phase 2): a one-time depth-noise transient
  after a gap — expected, not a defect; the tracking-lost gate absorbs a bad frame if ICP rejects it.
- **Baro over-trust:** kept a soft, low-weight constraint — never ground truth.
- **VoxelBlockGrid CPU budget:** if integrate/raycast blows 35 ms, the roadmap's documented escape is a
  CUDA source build — out of scope unless profiling forces it.

## 8. Success criteria

1. `roomscan-slam captures/phase6_motion_ref.bin` produces a trajectory + fused room mesh, median SLAM
   time < 35 ms/frame on CPU, drift reported, KISS-ICP benchmark in the same ballpark.
2. Full synthetic test suite green; no regression in the existing host suite.
3. `roomscan-panel` SLAM mode maps live on hardware within budget (supervised soak).
4. No wire-protocol change; `docs/protocol.md` untouched.
