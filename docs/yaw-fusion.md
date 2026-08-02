# Magnetometer yaw-drift correction (host-side)

The SFLP quaternion (stream 9) is **6-axis**: its tilt (roll/pitch) is gravity-referenced and
drift-free, but its **yaw drifts** — SFLP never consumes the magnetometer, and the LSM6DSV16X has no
on-chip 9-axis fusion. The host closes that loop by measuring how far the SFLP world frame's **+X
datum has wandered from magnetic north** (LIS2MDL, stream 10) and grafting a gated,
long-time-constant correction about world Z. Tilt stays 100% SFLP; only yaw is nudged. This bounds
the live gizmo's heading drift and gives the ICP rotation prior a drift-bounded orientation.

It is a **gentle drift bound, not a hard heading source** — indoor magnetic yaw is worse than
point-cloud ICP yaw (rebar/wiring distortion), so the correction is slow (default τ ≈ 20 s) and freezes
on magnetic anomalies and fast motion.

**The correction is that datum error and nothing else** (`magnetic_north_bearing_deg`, BUG-058,
2026-08-01). The filter used to difference a *device heading* against a *scalar yaw* pulled off the
attitude, and the device term did not cancel: driven at a known bearing, the fused quaternion came
out mirrored, at −bearing. The measurement now reads no body axis at all, which is also why it has
no attitude singularity — `gated:no-field` can only fire if the horizontal field itself vanishes.

There is deliberately **no gimbal gate** (BUG-051, 2026-07-31). One existed, freezing the correction
within 15° of |ZYX pitch| = 90°. Because the SFLP body frame has **X = Up**, |ZYX pitch| ≈ 90° *is*
the normal upright handheld grip, so the gate fired permanently in ordinary use — and what it was
defending was not the filter but the filter's own choice of yaw convention.

Separately, **`absolute_heading`** — the World-mode readout and the compass — is now the boresight's
compass bearing referenced to magnetic north: two bearings read in the same frame, differenced, so
the drifting datum cancels identically. It is `None` when the sensor is aimed within 10° of vertical,
where no bearing exists.

Design + rationale: `docs/superpowers/specs/2026-07-10-lsm6dsv16x-mag-yaw-correction-design.md`.

## 1. Calibrate the magnetometer (required)

Raw LIS2MDL readings are offset (hard-iron) and skewed (soft-iron); heading is meaningless without
correction. Collect a sample cloud while rotating the rig through **all** orientations, then fit:

```sh
cd host
python -m tools.mag_calibrate --seconds 30 --out mag_cal.json
# rotate the rig slowly through as many orientations as possible during the window
```

It prints the fitted field magnitude and a residual (`std/mean` of the calibrated magnitudes — lower is
better; < 0.02 is a clean fit) and writes `mag_cal.json`. Re-run whenever the rig's magnetic environment
changes materially.

The **web calibration modal** (`roomscan-web` → "Calibrate Mag") is the primary route now: it scores
sphere coverage live in 3D, steers you toward the largest gap, and gates acceptance on |B| consistency
rather than a bare fit residual. That is how the 2026-07-30 fit was made (BUG-030).

**Where the file lives.** `mag_cal_path` is **relative**, so it resolves against the *server's cwd* —
the repo root in practice, i.e. `./mag_cal.json`. Keep exactly one. A second copy under `host/` used to
shadow it for anything run from there and silently applied a superseded fit for two weeks (BUG-030 →
"The two-file trap"); a *missing* file is far safer, since it shows up immediately as
`fusion="gated:no-cal"` and `has_mag_cal=false`.

**Calibrate hand-held, in open space, with whatever is bolted to the device still bolted on.** Anything
body-fixed (e.g. a metal tripod mount plate) is calibratable and gets baked into the fit — but removing
it afterwards invalidates the calibration. Anything world-fixed is not: never calibrate on the tripod
(BUG-034).

**Then check the fit against a capture it never saw:**

```sh
host/.venv/bin/python host/tools/mag_check.py captures/<a-real-scan>.bin
```

Read its `verdict`. Do *not* judge a fit by raw |B| spread on a moving capture — indoor ambient field
varies with position by several percent, which no calibration can remove; see BUG-034 and
`magsweep.attitude_locked_error`.

## 2. Enable / tune (config)

Yaw fusion is **on by default** and falls back to the raw SFLP quat when no calibration is loaded (so it
never crashes uncalibrated). Config keys (in `roomscan.toml` `[viewer]`):

| key | default | meaning |
|-----|---------|---------|
| `yaw_fusion` | `true` | enable the correction |
| `yaw_fusion_tau` | `20.0` | complementary-filter time constant (s) — larger = gentler |
| `mag_cal_path` | `mag_cal.json` | calibration file to load |
| `yaw_anomaly_frac` | `0.3` | reject mag when \|mag\| deviates this fraction from the fitted field |
| `yaw_motion_rate_dps` | `40.0` | freeze correction above this SFLP angular rate |

(`yaw_gimbal_margin_deg` was removed by BUG-051 — see above. A stale copy left in an existing
`roomscan.toml` is harmless: the loader ignores unknown keys, so there is no migration.)

The panel logs `yaw-fusion -> active | gated:anomaly | gated:motion | gated:no-cal` on each
state change.

## 3. On-target axis-convention check (one-time)

`AXIS_CONVENTION` in `host/src/roomscan/sensors.py` (default `np.eye(3)`) reconciles the LIS2MDL mounting
with the SFLP body frame. A frame mismatch silently **mirrors or offsets** yaw with no other symptom. To
verify: with the rig level and pointed at a known magnetic heading, compare the fused/compass heading
against the known value. If it rotates the wrong way or is mirrored, set `AXIS_CONVENTION` to the
permutation/sign matrix that fixes it. Default identity is correct when the mag axes align with the LSM
body frame.

## Out of scope (see the design doc)

Temperature-based gyro-bias comp (SFLP already bias-corrects), IMU dead-reckoning (ICP owns translation),
barometer Z-constraint (a SLAM-phase concern), and MLC/FSM/ASC (need a `.ucf` blob whose reset would drop
the LSM's I3C address; the rig is tethered so their power/autonomy payoff is nil).
