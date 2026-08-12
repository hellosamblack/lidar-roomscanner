# What RTAB-Map does differently — study notes

Written 2026-08-11 after the owner ran a room capture with RTAB-Map on a Pixel 10 Pro XL and it
outperformed our own pipeline. Source read: `introlab/rtabmap` @ `2e193ee1` (v0.23.10), local
checkout `~/git/personal/rtabmap`.

This note records *what they do and where*, so the exploration issues (#149–#161) have a common
reference and nobody has to re-read 5100 lines of `RTABMapApp.cpp`. Capture configuration lives
separately in `docs/rtabmap-pixel10-capture.md`.

---

## 0. There is no Gaussian splatting in RTAB-Map

`grep -ri "splat|3dgs|gaussian splat"` over `corelib`, `app`, `guilib`, `tools` returns nothing.
What reads as splatting is `app/android/jni/point_cloud_drawable.cpp`: either `GL_POINTS` with
`gl_PointSize` sprites, or texture-mapped triangles from `util3d::organizedFastMesh`, with per-node
RGB gain compensation and a soft-occlusion blending pass. It is a well-tuned textured-mesh renderer.

Worth stating plainly because it reframes the whole comparison: none of the quality we were
impressed by comes from splat technology. It is pose-graph architecture plus rendering craft, all of
which is borrowable.

## 1. The structural difference: N node clouds + a graph, vs one TSDF

Every keyframe owns its own cloud/mesh **in its own frame** (`createdMeshes_`, keyed by node id) plus
a drawable with a settable pose. After the graph re-optimizes, the entire map update is
(`RTABMapApp.cpp:2036-2039`):

```cpp
if(main_scene_.hasCloud(id)) {
    main_scene_.setCloudPose(id, iter->second);   // just update pose
```

One mat4 write per node. No re-integration, no re-meshing, no GPU work.

Ours bakes poses into a `VoxelBlockGrid` at integrate time, so correcting a pose means
de-integrating and re-integrating. **That, not the size of the drift, is why loop closure has stayed
behind an evidence gate.** Same choice explains why BUG-053 (marching cubes segfaults above ~260k
blocks) and BUG-035 (block_count) have no analog for them: those are properties of *one global grid*.

→ #150 (SLAM-10), and it is the fork in the road for #110 and #111.

## 2. They do no tracking

`postOdometryEvent` (`RTABMapApp.cpp:4580`) takes the pose straight from ARCore VIO. RTAB-Map
contributes loop closure, graph optimization and memory management — not odometry. Their ICP is
clamped so it *cannot* become the estimator: `Icp/MaxTranslation = 0.05`, `Icp/MaxRotation = 0.17`
(10°), 10 iterations (`RTABMapApp.cpp:190-196`). It refines links; it never proposes motion.

We ask ICP against the TSDF model to produce translation from a 54×42 imager with no photometric
texture. BUG-067 — 18–20 m of fabricated path on a tripod capture at mean fitness 0.88 — is the
predictable outcome. Part of their advantage is plain observability: a wide-FOV RGB camera with
hardware-synced IMU beats 2268 depth points on a blank wall. That is the same conclusion as the
`lidar-primary-rotation-loses` finding, now extended from rotation to translation.

## 3. Mechanisms, with locations

| Mechanism | Where | Our issue |
| --- | --- | --- |
| Keyframe gate: `RGBD/LinearUpdate = 0.05` m, `AngularUpdate = 0.05` rad — a stationary device adds **zero** nodes | `RTABMapApp.cpp:159-160` | #149 |
| Per-node clouds updated by pose only | `RTABMapApp.cpp:2036-2039` | #150 |
| Gravity in the pose graph: `Mem/UseOdomGravity`, `Optimizer/GravitySigma = 0.2` | `RTABMapApp.cpp:165, 171-179` | #151 |
| `RGBD/OptimizeFromGraphEnd = true` — pin the last pose so the live camera doesn't jump on a loop | `RTABMapApp.cpp:154` | #151 |
| Time-budgeted working memory: `Rtabmap/TimeThr = 800` ms → `Memory::forget()` demotes to SQLite LTM | `Rtabmap.cpp:4560-4593` | #152 |
| Depth filtering before deprojection: per-pixel confidence, `depthBleedingFiltering`, `fastBilateralFiltering` | `RTABMapApp.cpp:2071-2085`, `4663-4680` | #153 |
| Loop closure: BoW (GFTT, 200 feats) → Bayes filter → PnP with `Vis/MinInliers = 25` → `RGBD/OptimizeMaxError = 3.0` wrong-loop reject | `Rtabmap.cpp:1960-2168`, `Parameters.h:409` | #154 |
| Pose buffer (1000 entries) + SLERP at each sensor's own stamp; rgb/depth stamp mismatch resolved by interpolation, not a constant offset | `CameraMobile.cpp:113-169`, `RTABMapApp.cpp:4696-4740` | #155 |
| Index-buffer LOD: one VBO, six index buffers, picked by squared camera distance (50 / 150 / 600), zero re-upload | `point_cloud_drawable.cpp:44-45, 548-590, 1194-1240` | #156 |
| Per-node world AABB maintained on `setPose()`, explicit frustum-plane culling | `point_cloud_drawable.h:104-119`, `scene.cpp:316-420, 484-508` | #156 |
| Depth pre-pass into an FBO with RGBA-packed depth, then blend pass discarding fragments >5 cm behind it | `scene.cpp:537-568`, `point_cloud_drawable.cpp` shaders | #157 |
| Live textures decimated 4×, full-res only at export; `mesh.texture` released right after GPU upload | `RTABMapApp.cpp:2189-2213` | #157 |
| `GainCompensator` — per-node RGB gains from link overlap, applied in-shader as `uGainR/G/B` | `RTABMapApp.cpp:1300-1352` | #160 |
| Export: posed images + per-frame calibration + depth + confidence | `tools/Export/main.cpp:116-126, 1621-1682` | #158, #159 |

## 4. Two things worth flagging as traps

**The pose interpolation is the right shape, and our fixed-offset plan was not.** BUG-031 measured a
+7.76 ms quat-phase lead and put `quat_mid_ticks` on the wire; #126 applied it as a constant
(default-off, measured worse and bistable on `imuTranslationError`). A constant is correct only if
the phase is constant, and it is not — beyond BUG-031's own load-dependence (CALIB-carrying frames
drain 655 µs later, Welch t = −5.8), the 2026-08-12 survey of stream-13 distributions found today's
captures at **+5.1 ± 0.7 ms** (not 7.76), breathing over a ~4 ms range within a capture, and
`officeFullScanAug6.bin` at **−3.9 ms** — the opposite *sign*, where any positive constant pushes
orientation the wrong way by twice the skew. **Shipped (#155, 2026-08-12):** `roomscan.sensor_time`'s
`TimestampedQuaternionBuffer` + the `_load_frames(quat_interp=)` collect-then-align pass SLERP each
depth frame's orientation at its own frame-ready instant from exact-group stream-9/13 pairs, wrap-safe
on the LSM clock, never extrapolating; still gated by `[slam] apply_quat_phase` (default off), with
the fixed rollback demoted to per-frame fallback. See
`docs/superpowers/plans/2026-08-12-155-pose-buffer-session.md` for the validation campaign.

**Their culling is the version ours should have been.** We set `frustumCulled = false` on all six SLAM
objects after BUG-033 (stale zero-radius bounding sphere) and BUG-065 (fractional
`BufferAttribute.count` → NaN bounding volumes). Those were correct emergency fixes; the end state is
that we draw everything, always. RTAB-Map maintains the AABB explicitly *where the pose is set*,
rather than relying on a cached volume the engine computes once and never invalidates. → #156.

## 5. Honest limits of the comparison

- **ARCore depth is not metric.** Depth-from-motion + ML at ~160×120 unless the handset has hardware
  ToF; `postOdometryEvent` unpacks a per-pixel confidence out of DEPTH16's high bits precisely because
  it does not trust it uniformly. The VL53L9CX stays our metric source. Copy their pose and
  map-structure architecture, not their depth.
- **The RTAB-Map Android app does not capture high-resolution RGB.** Driver 1 hard-selects the lowest
  CPU camera config (`CameraARCore.cpp:220`); driver 3 finds the highest and then does not use it, with
  an in-code `//FIXME` about `CaptureRequest contains unconfigured Input/Output Surface!`
  (`ARCoreSharedCamera.java:396-400`). "HD Mode" only skips the app's own extra 2× decimation. This
  materially shapes the Phase 7 plan — see `docs/rtabmap-pixel10-capture.md` §0.
- **We have not run their pipeline against ours on the same room.** Everything above is a source read
  plus one owner impression. #161 (DC-K) is the paired capture that would make any of it a
  measurement.
