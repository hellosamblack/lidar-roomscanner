# RTAB-Map capture on the Pixel 10 Pro XL — settings for Phase 7

**Purpose.** Phase 7 (offline 3D Gaussian Splatting) needs a *posed* image set, because COLMAP SfM
keeps failing on the featureless painted walls this scanner is aimed at — measured: 206 of 287 frames
registered (72%) on Sam Office. Running RTAB-Map on the phone gives us camera poses, per-frame
intrinsics, depth, and a loop-closure-corrected pose graph from a shipping app, with no build risk.

Owner decision, 2026-08-11: **Phase 7 captures are taken with RTAB-Map**, so the offline pass has
geometry as well as imagery. This document is the exact configuration.

Derived from `introlab/rtabmap` @ `2e193ee1` (v0.23.10), local checkout `~/git/personal/rtabmap`.
Every setting below is quoted with the label as it appears in the app's Settings screen. Background
on *why* these mechanisms matter to our own pipeline: `docs/rtabmap-study.md` and GitHub issues
#149–#161.

---

## 0. Read this first — the RGB resolution ceiling

**The RTAB-Map Android app does not capture high-resolution RGB, and the "HD Mode" setting does not
change that.** This is a property of the app, not of the phone, and it is visible in the source:

- **Camera Driver = ARCore NDK (1)** explicitly selects the **lowest** CPU camera config offered:
  `ArSession_setCameraConfig(arSession_, cpu_low_resolution_camera_config_ptr->config)`
  — `app/android/jni/CameraARCore.cpp:220`.
- **Camera Driver = ARCore Java (3)** enumerates configs, finds the highest, and then **does not use
  it** — the `setCameraConfig` call is commented out with an in-code note:
  `//FIXME: Can we make it work to use HD rgb images? To avoid this error "CaptureRequest contains
  unconfigured Input/Output Surface!"` — `app/android/src/com/introlab/rtabmap/ARCoreSharedCamera.java:396-400`.
- **"HD Mode"** (`pref_key_resolution` → `RTABMapLib.setFullResolution`) only controls whether the app
  applies its own extra 2× decimation to that already-small image
  (`RTABMapApp.cpp:4750-4754`, `postOdometryEvent`). ON = no extra decimation. It is not 1080p, and
  it is certainly not 4K.

**Consequence for Phase 7.** Treat RTAB-Map as the source of **poses and geometry**, not as the source
of splat training imagery. Two workable plans, and the choice is a measurement, not a preference:

| Plan | How | Trade |
| --- | --- | --- |
| **A — RTAB-Map only** | Train the splat on RTAB-Map's own exported frames | Simplest; one clock, one coordinate frame, poses already attached. Image resolution is the ceiling on splat detail. |
| **B — RTAB-Map + separate 4K video** | Second pass walking the same path with the stock camera app at 4K; register the video frames to the RTAB-Map pose set | Best detail, but re-introduces a registration problem — though an easier one, since RTAB-Map's poses give a strong initialization and the *geometry* is known. |

Start with **A** for the first capture. It is the honest baseline: if a low-res posed set already
beats COLMAP-on-4K, that is the finding, and it is the gate written into #159 (OFFLINE-6). Do not
skip straight to B and lose the comparison.

**First thing to record from a real session:** the actual image resolution in the export. It settles
this section with a number instead of an inference. Note it here when known.

---

## 1. Install and first launch

- Install **RTAB-Map** by IntRoLab from the Play Store (package `com.introlab.rtabmap`). The Play
  build tracks the same source as the checkout; if the version differs materially from 0.23.10,
  re-check §0 against the newer source before trusting it.
- Grant camera and storage permissions. Location is only needed if you want GPS tagging (we don't).
- Working directory on-device:
  `Android/data/com.introlab.rtabmap/files/RTAB-Map/` — the app shows it as `/Internal storage/RTAB-Map/`.
  Databases land there as `<name>.db`; on-device exports land in `RTAB-Map/Export/`.

**Verify the depth path before your first real capture.** Point the phone at a wall ~1.5 m away with
`Point Cloud` rendering selected and Debug on. You should see a *dense* cloud filling the view. If
you see only a sparse scatter of feature points, depth is off — see §2.

---

## 2. Camera Driver — the one setting that is not obvious

`Settings → Camera Driver`

| Value | What it does | Verdict for us |
| --- | --- | --- |
| Auto (−1) | Falls through to **ARCore Java (3)** on a Pixel (no Tango, no AREngine) — `RTABMapActivity.java:756-778` | Do not use; it lands on 3 |
| ARCore NDK (1) | ARCore's ML **depth-from-motion** via `ArSession_isDepthModeSupported(AR_DEPTH_MODE_AUTOMATIC)` + `ArFrame_acquireDepthImage` (`CameraARCore.cpp:160-169, 379-420`). Works on any ARCore phone | **Use this** |
| ARCore Java (3) | Camera2 `SharedCamera` path. Its depth comes **only from a hardware DEPTH16 stream** (`mTOFAvailable`, `ARCoreSharedCamera.java:421-536`). No hardware ToF ⇒ **no depth at all**, just ARCore feature points | Only if the handset really has a ToF sensor |

**Set `Camera Driver = ARCore NDK`.**

Pixel phones have historically shipped without a hardware ToF, and whether the Pixel 10 Pro XL has one
is **unverified** — do not assume either way. The empirical test is the wall check in §1: driver 3 with
no ToF gives a sparse feature cloud, driver 1 gives a dense one. If driver 3 *does* produce dense
depth, the handset has a ToF and that is a finding worth recording on #135 (OFFLINE-4), because it
changes the depth-quality argument for the whole phase.

Caveat carried from the study: ARCore depth is **depth-from-motion + ML**, ~160×120 raw, best between
about 0.5–5 m, and explicitly imprecise on featureless white walls. It is a **regularizer and a
cross-check**, not metric truth. The VL53L9CX stays our metric source.

---

## 3. Settings — exact values

Labels are as shown in the app. "Default" is the app's shipped default; change only where the
**Set** column differs.

### Rendering
These affect what is stored in the map and exported, not just what you look at.

| Setting | Default | **Set** | Why |
| --- | --- | --- | --- |
| Point Cloud Density | High (1) | **Maximum (0)** | Density level drives `meshDecimation` at cloud construction (`RTABMapApp.cpp:863-928`). Maximum = no decimation. We want every depth sample; decimate later, on the host, where it is reversible. |
| Max Depth | 2.5 m | **4 m** | 2.5 m truncates a room. ARCore depth degrades past ~5 m, so 4 m keeps the useful band without inviting garbage. Raise to 5 m for a large room; do not use "No Limit". |
| Min Depth | 0 m | **0.3 m** | Excludes the near-field junk ARCore produces at arm's length. |
| Point Size | 10 | 10 | Display only. |
| Mesh Angle Tolerance | 20 deg | 20 deg | `organizedFastMesh` grazing-angle cutoff. Fine. |
| Mesh Triangle Size | 2 pix | 2 pix | Finest available. |
| Texture Resolution | Low (4) | **Maximum (1)** | This is `renderingTextureDecimation_`, the 4× downscale applied to *live rendering* textures. At Maximum the stored texture is not pre-shrunk. Costs memory; worth it when the source image is already small. |
| Background Color | 0.2 | any | Display only. |
| Blending | true | true | The depth pre-pass soft-occlusion pass (see #157). Display only, but leave it on — it is how you *see* whether the map is consistent. |
| Nodes Filtering | false | **false** | ON drops nodes that have non-neighbour links, i.e. throws away exactly the frames a loop closure touched. Never enable this for a capture we intend to export. |

### Mapping

| Setting | Default | **Set** | Why |
| --- | --- | --- | --- |
| Append Mode | true | **false** | ON continues into the existing database on a new scan. We want one room = one clean database. |
| HD Mode | false | **true** | Skips the app's own extra 2× RGB decimation (`RTABMapApp.cpp:4750`). Read §0 — this is not HD, it is "not further reduced". |
| Depth From Motion | false | false | Only relevant to drivers without their own depth. Driver 1 has depth. |
| ARCore Re-Localization Acceleration Threshold | 58.8399 | 58.8399 | Detects a jolt that likely broke ARCore tracking. Leave alone unless you see spurious map splits. |
| Smoothing | false | **true** | Enables `util2d::fastBilateralFiltering` on depth before deprojection (`RTABMapApp.cpp:2078-2081`). ARCore depth is noisy; this is the cheap, standard fix. |
| Depth Bleeding Filtering Error | 0.0 | **0.02** | Enables `util2d::depthBleedingFiltering` — kills flying pixels at depth discontinuities. Depth-from-motion bleeds badly at object edges, and flying pixels poison both the mesh and any depth supervision we feed the splat. Start at 0.02 and raise if edges still smear. |
| Fish Eye Camera | false | false | Not applicable. |

### Mapping → Core

| Setting | Default | **Set** | Why |
| --- | --- | --- | --- |
| Update Rate | 1 Hz | **Max (0)** | `Rtabmap/DetectionRate`. At the 1 Hz default a 3-minute sweep yields ~180 keyframes — far too few for a splat. Max removes the rate cap; node creation is then governed by displacement (`RGBD/LinearUpdate` = 0.05 m / 0.05 rad, set in code at `RTABMapApp.cpp:159-160`), which is the gate we actually want. |
| Maximum Motion Speed | No Limit (0) | **No Limit (0)** | Sets `RGBD/LinearSpeedUpdate`, which *rejects* frames captured while moving faster than the limit. We would rather have the frame and judge it later. |
| Time Limit | No Limit (0) | **No Limit (0)** | `Rtabmap/TimeThr`. The memory-management budget (#152). Excellent for a long robot deployment, wrong for a 3-minute room where we want every node kept in Working Memory. |
| Memory Limit | No Limit (0) | **No Limit (0)** | Same reasoning. |
| Loop Closure Threshold | 0.11 | 0.11 | Bayes posterior threshold. Leave at the tuned default. |
| Similarity Threshold | 0.3 | 0.3 | `Mem/RehearsalSimilarity`. Leave. |
| Min Inliers | 25 | 25 | PnP inlier count for accepting a loop closure. Leave — lowering it is how you get a wrong loop. |
| Max Optimization Error | 2 | 2 | `RGBD/OptimizeMaxError`: rejects a loop whose post-optimization residual/σ ratio exceeds this. The cheap wrong-loop detector. Leave. |
| Max Features Extracted (Vocabulary) | 200 | 200 | Leave. |
| Max Features Extracted (Loop Closure) | 400 | 400 | Leave. |
| Feature Type | BRIEF (6) | BRIEF (6) | Leave. |
| Graph Optimizer | GTSAM (2) | GTSAM (2) | GTSAM also brings the gravity constraint (`Optimizer/GravitySigma = 0.2`, `RTABMapApp.cpp:171-175`) — the mechanism we want for ourselves in #151. |
| Optimization from Graph End | true | true | Pins the *last* pose so the live camera doesn't jump when a loop fires. Leave on. |
| ArUco Marker Detection | Disabled | **Disabled** — but see below | Markers give metrically-known control points. If you ever want a hard scale/registration check between the phone map and the rig scan, printing one ArUco board and enabling this is the cheapest way to get it. Not needed for the first capture. |

### Mapping → Database

| Setting | Default | **Set** | Why |
| --- | --- | --- | --- |
| Save All Frames in Database | true | **true** | `Mem/NotLinkedNodesKept`. **Load-bearing.** OFF and the intermediate frames never reach the `.db`, so `rtabmap-export --images` has nothing to export. |
| Save Raw Scan | false | false | Only meaningful with an external lidar. |
| Save GPS | false | false | Indoors. |
| Save Environmental Sensors | false | **true** | Free. Stores ambient temperature/pressure/light/humidity + Wi-Fi RSSI per node. The barometer trace is a useful independent cross-check against our own (BUG-037 territory), and it costs nothing. |
| Database in Memory | false | **false** | ON risks losing the whole session if the app is killed. A room scan is unrepeatable in the same sense a live rig scan is (#43). Write to disk. |

### Export (used by the on-device "Assemble" menu, not by `rtabmap-export`)
Only relevant if you also want an on-device mesh to eyeball. Defaults are fine; `Voxel Size = 0.01`,
`Texture Size = 4096`, `Max Texture Distance = 3 m`. The authoritative export is the host-side one in §6.

---

## 4. Capture protocol

1. **New Scan** from the menu. Confirm the driver in the status overlay (turn on `Visibility → Status`
   and `Debug`).
2. Stand still for ~3 s before moving, so ARCore's VIO initializes on a static scene.
3. **Walk slowly and smoothly.** ARCore's translation estimate is what we are buying; motion blur and
   jerks are what degrade it. Continuous slow motion beats stop-and-go.
4. **Keep the camera roughly level and pointed at surfaces 1–4 m away.** Depth-from-motion needs
   parallax, so pure rotation on the spot produces nothing useful — the phone must translate.
5. **Cover each surface from at least two viewpoints.** This is a splat requirement, not a SLAM one:
   a wall seen from one angle cannot be reconstructed with correct view-dependent appearance.
6. **Close the loop** — return to the exact start pose and dwell there for a couple of seconds. That
   is what makes the capture usable for the loop-closure comparison in #110 and #161.
7. Watch for the loop-closure indicator. If the map visibly snaps and looks *worse*, that is a bad
   loop; note the time and keep going rather than restarting.
8. **Save** with a descriptive name. Keep the `.db` — it is the reusable artifact. Every export can be
   regenerated from it with different parameters, and the pose graph exists nowhere else.

**Lighting.** Fix it before you start: lights on, blinds in a consistent state. Phone AE/AWB will
drift across the sweep regardless, which is what #160 (OFFLINE-7, gain compensation) is about — but
a room with a bright window in half the frames is a much harder version of that problem. Note the
lighting conditions with the capture.

**Pairing with a rig scan.** If this capture is half of a paired set (#161, DC-K): same room, same
path, no scene change between takes, and record which one was first. Rig-side, disable auto-idle if
any part of the sweep is static — the laser parks and you record IMU only.

---

## 5. On-device post-processing (before export)

From the menu: **Optimize → Advanced…**, in this order:

1. **Detect More Loop Closures** — a second pass that finds closures the online pass missed. This is
   the single most valuable button on the phone; the online detector is deliberately conservative
   because it runs in real time, and the offline pass is not.
2. **Global Graph Optimization** — re-optimize with everything found.
3. **Adjust Colors (Full)** — full-pairwise `GainCompensator` (`RTABMapApp.cpp:1300-1352`) rather than
   the link-only fast version. Equalizes per-node exposure. Whether these gains survive into the
   export is unverified — check, and if they do not, #160 is the host-side answer.
4. Skip **Bundle Adjustment** on the first run. Note whether it changes anything and by how much;
   it is worth adopting only if it does.
5. Skip **Mesh Smoothing** and **Noise Filtering** — both discard data we would rather filter
   ourselves, reversibly, on the host.

Save again after optimizing.

---

## 6. Export for our pipeline

Pull the database off the phone (MTP, `adb pull`, or share to Drive):

```
Android/data/com.introlab.rtabmap/files/RTAB-Map/<name>.db
```

The on-device Assemble menu is for eyeballing. The export that feeds `roomscan.splat` is the desktop
tool `rtabmap-export` (`tools/Export/main.cpp`). Easiest route on our GPU-less Linux box is the
published image rather than a source build:

```sh
docker run --rm -v "$PWD:/data" introlab3it/rtabmap:noble \
  rtabmap-export \
    --images \
    --poses_camera \
    --poses_format 1 \
    --output_dir /data/out \
    /data/<name>.db
```

What that produces (`tools/Export/main.cpp:116-126, 1621-1682`):

- RGB frames, plus registered depth and the per-pixel **depth confidence** image;
- **per-image calibration** — written per frame on purpose, because autofocus changes intrinsics
  during a session. Do not collapse these to one shared intrinsic without checking they are actually
  constant;
- graph-optimized **camera poses in the optical frame**, in the selected `--poses_format`
  (`1` = RGBD-SLAM, `2` = KITTI, `3` = TORO, `4` = g2o; `--poses` gives the base-frame variant).

Add `--cloud` or the mesh/texture options if you also want assembled geometry for the ground-truth
comparison in #161.

**Before trusting any of it, verify the frame conventions.** RTAB-Map's optical frame is not ours, and
the app additionally carries an OpenGL↔RTAB-Map world pair (`rtabmap_world_T_opengl_world`,
`RTABMapApp.cpp:4690`). Getting this wrong produces output that is plausibly, silently wrong — the
BUG-051 / BUG-058 failure mode, twice burned. Check with a capture whose true geometry you know
(a corridor, a right-angled corner) before running it on anything that matters. The ingest work is
tracked in #158 (OFFLINE-5).

---

## 7. Checklist

```
[ ] Camera Driver = ARCore NDK                 [ ] Update Rate = Max
[ ] Point Cloud Density = Maximum              [ ] Time Limit = No Limit
[ ] Max Depth = 4 m, Min Depth = 0.3 m         [ ] Memory Limit = No Limit
[ ] Texture Resolution = Maximum               [ ] Graph Optimizer = GTSAM
[ ] Nodes Filtering = OFF                      [ ] Save All Frames in Database = ON
[ ] Append Mode = OFF                          [ ] Save Environmental Sensors = ON
[ ] HD Mode = ON                               [ ] Database in Memory = OFF
[ ] Smoothing = ON
[ ] Depth Bleeding Filtering Error = 0.02      Capture: dense cloud on the wall check?
                                               Loop closed at the start pose?
```

## 8. Open questions to settle on the handset

Each of these is a measurement, not a guess. Record the answer here when you have it.

1. **Actual exported RGB resolution** — settles §0 and decides Plan A vs Plan B.
2. **Does the Pixel 10 Pro XL have a hardware ToF?** Driver 3 wall check (§2). Feeds #135.
3. **Do the "Adjust Colors" gains survive into the export?** Decides how much of #160 we still need.
4. **How many nodes does a typical room produce at Update Rate = Max?** Sets expectations for splat
   training set size and for the ingest work in #158.
5. **Does Bundle Adjustment change the poses measurably?** If yes, it belongs in the standard §5
   sequence.
