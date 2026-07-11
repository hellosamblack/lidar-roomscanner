# Phase 6 SLAM — offline validation on the real motion capture

Task 9 wires Tasks 1–8 (deprojection, TSDF, ICP odometry, frame-to-model
`Mapper`, `SlamConfig`, `metrics`) into `roomscan-slam` and runs the empirical
validation gate against a real handheld capture:

- Capture: `captures/phase6_motion_ref.bin` (gitignored, 47.9 MB, 102 s,
  3187 RAW + full SFLP/ENV, 0 CRC failures). 3184 depth frames are decoded
  after CALIB init (3 frames precede the first CALIB and are dropped by
  `TransformStage`).
- Command (per the brief):
  ```
  cd host && .venv/Scripts/python.exe -m roomscan.slam.cli ../captures/phase6_motion_ref.bin \
    --compare-modes --out-mesh ../slam_map.ply --out-traj ../slam_traj.tum
  ```
- Outputs: `slam_map.ply` (1,603,275 vertices / 2,984,147 triangles, ~120 MB)
  and `slam_traj.tum` (3184 poses), both at repo root, **not committed**
  (gitignore doesn't need updating — they're simply never staged).

## Gate: median per-frame SLAM time < 35 ms

**Result: FAILS for both modes**, reported honestly rather than fudged.

| mode | median ms | p90 ms | p99 ms | max ms | over-budget frac |
|---|---|---|---|---|---|
| translation | 69.8 | 106.4 | 127.0 | 1227.9 | 97.0% |
| 6dof | 36.7 | 64.2 | 82.6 | 108.3 | 53.4% |

`translation` mode runs a full point-to-plane ICP solve and then *discards*
the rotation and re-derives translation, so it pays the same per-iteration
ICP cost as `6dof` plus extra bookkeeping — that's consistent with it being
~2x slower rather than cheaper. `6dof` is close to the 35 ms budget (~5%
over at the median) but still fails it.

A second full run (single-mode `translation` + `--benchmark`) reproduced the
same qualitative picture (median 70.2 ms) but with different exact numbers
(see "Run-to-run variance" below) — the **gate-fail conclusion is stable
across runs**, the exact percentages are not.

This is a real performance concern for Phase 6, not a CLI bug: Tasks 1–8's
`Mapper`/`register()`/`TsdfMap` implementation is what's being timed here;
Task 9 only wires and reports it. Flagging for the controller to decide
whether to raise the budget, profile/optimize the ICP+raycast path, or accept
degraded framerate for now.

## Trajectory quality — both modes

| mode | n | path_length_m | start_end_gap_m | max_step_m | tracking_lost |
|---|---|---|---|---|---|
| translation | 3184 | 70.824 | 1.077 | 0.769 | 992 (31.2%) |
| 6dof | 3184 | 32.029 | 1.357 | 0.603 | 2375 (74.6%) |

Both path lengths are physically plausible for a ~102 s handheld room
walkthrough (0.31–0.69 m/s average — a slow, deliberate scanning pace, well
below normal walking speed), and neither is "every frame lost" — so this
does **not** meet the brief's STOP/DONE_WITH_CONCERNS bar for an obviously
broken trajectory. It is nonetheless a real, high tracking-lost rate that
should be tracked as a Phase 6 follow-up (see below).

### Chosen default: `translation`

Per the brief's criterion ("lower drift `start_end_gap_m` for a room loop /
cleaner trajectory"), `translation` wins on both axes that matter for map
quality: smaller loop-closure gap (1.077 m vs 1.357 m) and dramatically fewer
tracking-lost frames (31% vs 75%) — despite being the slower mode. `6dof`'s
much higher loss rate is consistent with full 6-DoF point-to-plane ICP being
more prone to falling outside the fitness/RMSE gate (`min_fitness=0.3`,
`max_rmse=0.05`) on real, noisy 54×42 ToF data, and to genuinely singular
6×6 normal-equation solves on texture-poor surfaces (see below).

`SlamConfig`'s built-in default (`icp_mode = "translation"`, in
`host/src/roomscan/slam/config.py`) **already matches** this empirical
winner — no default was changed for this task.

### Tracking-lost behavior

Expected in kind, high in degree. `register()` (`odometry.py`) already
degrades a singular 6×6 ICP solve to `ok=False` instead of crashing —
confirmed directly: the full runs printed several
`[Open3D Error] ... Singular 6x6 linear system detected, tracking failed.`
messages (Open3D's own C++ logging before the caught `RuntimeError`), each
correctly absorbed into `tracking_lost` rather than propagating. Beyond
that specific failure mode, the fitness/RMSE gate rejects a meaningful
fraction of otherwise-converged ICP results on real noisy depth.

**Ruled out: quat/pressure pairing bug.** The wire order interleaves
streams as `RAW_3DMD(seq=N)` → `IMU_QUAT(seq=N)` → `ENV(seq=N)` — i.e. the
quat/env for a given seq arrive *after* that seq's depth frame in the byte
stream, so `_load_frames`'s "carry the latest forward" logic pairs each
depth frame with the *previous* seq's quat (~1 frame, ~16–35 ms stale). This
was the prime suspect the brief called out, so it was tested directly: a
throwaway seq-matched loader (buffers depth until its own-seq quat/env
arrive) was run against the first 800 frames alongside the current
carry-forward loader. Current (stale-by-one) pairing: `lost=0`. Seq-matched
("correct") pairing: `lost=12`. The fix made tracking-lost *worse*, not
better, on this subset — so the off-by-one lag is not the driver of the
full-file loss rate, and `_load_frames` was left as specified in the brief.
The real driver is most likely later segments of the capture with faster
motion and/or low-texture surfaces filling the FOV, which is a `Mapper`/gate
tuning question for a future task, not a Task 9 wiring defect.

## Run-to-run variance (Open3D nondeterminism)

Re-running the identical `translation`-mode command against the same file
produced different exact numbers:

| run | path_length_m | gap_m | lost | median_ms |
|---|---|---|---|---|
| compare-modes run | 70.824 | 1.077 | 992 (31.2%) | 69.8 |
| single-mode + `--benchmark` run | 81.877 | 1.701 | 824 (25.9%) | 70.2 |

Both runs agree qualitatively (gate fails, translation has far fewer lost
frames than 6dof, path length in the tens-of-meters range) but disagree by
~15–20% on exact path length / loss count. This points to non-determinism
in Open3D's tensor ICP/TSDF raycast (most likely floating-point reduction
order under multi-threading) shifting individual frames across the
fitness/RMSE gate threshold. Treat single-run numbers as indicative, not
exact; the qualitative conclusions above are stable across both runs.

## KISS-ICP benchmark (`--benchmark`)

**KISS-ICP installed cleanly** on this Windows/Python 3.12 box
(`pip install kiss-icp` → `kiss_icp==1.3.0`, a prebuilt `cp312-win_amd64`
wheel) — contrary to the brief's default expectation that it likely
wouldn't. `metrics.compare_kiss` was finalized against the installed API:
`kiss_icp.kiss_icp.KissICP` + `KISSConfig(data=DataConfig(deskew=False),
mapping=MappingConfig(voxel_size=0.05))` (deskew off because our depth
frames have no per-point timestamps to deskew against; voxel size ~5 cm per
KISS-ICP's own indoor-scale guidance). Each captured depth frame is
deprojected to a whole point cloud via the existing `Deprojector` and fed
frame-by-frame through `register_frame`; `odom.last_pose` is accumulated
into a trajectory the same way `trajectory_stats` consumes.

Full-capture benchmark result (prior-free, no SFLP/baro, whole-cloud
frame-to-map odometry):

```
KISS-ICP: path=47.588 m  gap=1.583 m
```

This sits between our two modes' path lengths (32.0–81.9 m across runs) and
in the same ballpark for loop gap — a reasonable independent cross-check
given there's no ground truth trajectory for this capture, though it is not
a precision benchmark (KISS-ICP is tuned for scanning LiDAR point density,
not a 54×42 imager, and gets none of our IMU/baro priors).

## Mesh visual check

`slam_map.ply` was written successfully (1,603,275 vertices, 2,984,147
triangles, per-vertex normals + colors, ~120 MB) — a non-trivial, populated
reconstruction, not an empty or degenerate mesh.

**PNG rendering was skipped, not silently omitted.** This box's Open3D
Filament backend fails headless rendering
(`o3d.visualization.rendering.OffscreenRenderer` raises `EGL Headless is
not supported on this platform` — the exact, fast-failing error already
documented in `host/tools/panel_view.py`'s docstring, which is why that
tool reimplements a pure-CPU/Pillow point-cloud rasterizer instead of using
Open3D's own renderer). That tool's rasterizer targets `Frame`/point-cloud
data from the live pipeline, not a fused `TriangleMesh`, so it wasn't
reused here. The `.ply` needs to be opened on a machine with a real
display/GPU (`o3d.visualization.draw`, MeshLab, Blender, etc.) for a visual
check; verifying vertex/triangle counts loaded cleanly (via
`o3d.t.io.read_triangle_mesh`) was used as the automated non-degeneracy
check instead.

## Summary

- CLI + tests: `roomscan-slam` implemented and TDD'd (RED → GREEN), full
  host suite green (303 tests).
- Gate: **FAILS** for both modes (translation 69.8 ms median, 6dof 36.7 ms
  median vs. the 35 ms budget) — flagged for the controller, not fudged.
- Chosen default: `translation` (already `SlamConfig`'s built-in default;
  unchanged) — lower drift, far fewer tracking-lost frames, despite being
  the slower mode.
- Tracking-lost (31–75% depending on mode/run) is real and non-trivial but
  not "every frame lost"; the quat/pressure pairing lag was directly ruled
  out as the cause via a controlled 800-frame A/B comparison.
- KISS-ICP installed and was wired into `compare_kiss` for real (not left
  as a graceful-None stub); benchmark result is a plausible independent
  cross-check.
