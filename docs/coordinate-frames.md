# Coordinate frames

Canonical reference for every coordinate frame and transform in the `roomscan` host stack. **Read this
before touching any pose, quaternion, deprojection, world-accumulation, or SLAM code** — the conventions
below were hard-won (BUG-004, the IMU-axis-mapping commits `14f6a4b`/`cb2b01c`/`55108ec`, the
gizmo yaw-as-roll fix) and are re-derived incorrectly every time they aren't written down. Phase 6 SLAM
(ICP + TSDF) operates entirely in these frames; get them wrong and registration diverges silently.

All matrices below are defined in `host/src/roomscan/sensors.py`; the deprojection math is in
`host/src/roomscan/deproject.py`; world accumulation is in `host/src/roomscan/panel.py`.

## The four frames

| Frame | Axes | Where it comes from |
|---|---|---|
| **ToF (CV)** | X = Right, Y = Down, Z = Forward | The deprojected point cloud (`Deprojector`). Standard computer-vision camera frame. |
| **SFLP body** | X = Up, Y = Right, Z = Forward | The LSM6DSV16X orientation output, board held vertically with USB down. |
| **SFLP world** | X = North, Y = West, Z = **Up** | The fixed world the SFLP quaternion rotates *into* (Z-up, ENU-like but N/W/U). |
| **Open3D CV world** | X = Right (East), Y = **Down** (−Up), Z = Forward (North) | The renderer's world. Open3D's world-up is **Y**, not Z — the source of the yaw-as-roll bug. |

The SFLP quaternion (stream 9) is a **body → SFLP-world** rotation `R = quat_to_matrix(*quat)`.

## The two structural transforms

Defined once in `sensors.py:168-181`, reused everywhere (never redefine them locally):

```
T_CV_TO_BODY   (sensors.py:168)   # CV camera axes → SFLP body axes
  X_body = -Y_cv ; Y_body = X_cv ; Z_body = Z_cv

T_WORLD_TO_CV  (sensors.py:177)   # SFLP world axes → Open3D CV world axes
  X_cv = -Y_world ; Y_cv = -Z_world ; Z_cv = X_world
```

## The composed mapping (the one that matters)

To render a body-frame point cloud into the fixed Open3D world using the SFLP orientation, both
`gizmo_pose` (`sensors.py:188`) and the panel's world accumulation (`panel.py:734`) use the **same**
composition:

```python
r_mapped = T_WORLD_TO_CV @ R @ T_CV_TO_BODY      # R = body→world from the quat
world_pts = (r_mapped @ cv_points.T).T
```

Read right-to-left: CV points → body axes → rotate to SFLP world by the live orientation → re-express in
Open3D's Y-up world. This is why yaw (a Z-rotation in the Z-up SFLP world) correctly renders as a
Y-rotation in Open3D — the `T_WORLD_TO_CV` sandwich remaps the rotation axis. Skipping the sandwich makes
yaw appear as roll (the original BUG-004 symptom).

## Gravity / "down"

- World gravity in the SFLP Z-up world is `[0, 0, -1]` (`sensors.py:209`).
- Body-frame gravity: `g_body = R.T @ [0, 0, -1]` (`sensors.py:210`, `ir_gravity_rot`). Used to roll the
  2D IR image so its down matches physical down, and to preserve **absolute tilt** across baseline
  resets (only yaw is zeroed on reset — pitch/roll come from the accelerometer down-vector, `3c6c93d`).
- In-plane IR components: CV Right = SFLP Y, CV Down = SFLP −X → `gx = g_body[1]`, `gy = -g_body[0]`.

## Magnetometer axis convention

`AXIS_CONVENTION = np.diag([1.0, -1.0, -1.0])` (`sensors.py:218`, write-locked) maps raw LIS2MDL mag axes
to the IMU frame — i.e. `[x, -y, -z]`. Resolved on-target in BUG-004 by evaluating all 24 axis-swap/sign
permutations for lowest tilt-variance and `slope ≈ +1.0` vs IMU yaw. On-rig calibration lives in
`mag_cal.json` (`field_ut ≈ 49.87 µT`); procedure in `docs/yaw-fusion.md`. Heading de-tilts the mag into
the SFLP world horizontal plane then `atan2` (`heading_deg`, `sensors.py:143`).

## Phase 6 SLAM implications

- **Pick one world frame and state it.** The existing render path lives in **Open3D CV world** (Y-down).
  Open3D's `t.pipelines.registration` and `VoxelBlockGrid` are frame-agnostic (they just need consistent
  poses), so the natural choice is to keep the TSDF in Open3D CV world and feed poses as
  `r_mapped`-style body→world transforms — reuse `T_WORLD_TO_CV @ R @ T_CV_TO_BODY`, don't invent a new
  frame.
- The **SFLP rotation prior** is `R` (body→SFLP-world); convert to the TSDF world with the same sandwich
  before handing it to ICP as an initial guess.
- The **baro Z-constraint** is along SFLP world **+Z (up)**, which is Open3D CV world **−Y**. A Z-drift
  constraint in the renderer's frame acts on the −Y component, not Z.
- Translation is **never** integrated from the accelerometer (drift); it comes only from ICP.
