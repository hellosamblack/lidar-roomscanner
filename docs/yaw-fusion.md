# Magnetometer yaw-drift correction (host-side)

The SFLP quaternion (stream 9) is **6-axis**: its tilt (roll/pitch) is gravity-referenced and
drift-free, but its **yaw drifts** — SFLP never consumes the magnetometer, and the LSM6DSV16X has no
on-chip 9-axis fusion. The host closes that loop by grafting a gated, long-time-constant,
tilt-compensated magnetometer heading (LIS2MDL, stream 10) onto the SFLP yaw. Tilt stays 100% SFLP;
only yaw is nudged toward the magnetic reference. This bounds the live gizmo's heading drift and gives
the future ICP rotation prior a drift-bounded orientation.

It is a **gentle drift bound, not a hard heading source** — indoor magnetic yaw is worse than
point-cloud ICP yaw (rebar/wiring distortion), so the correction is slow (default τ ≈ 20 s) and freezes
on magnetic anomalies, fast motion, and gimbal-lock (pointing at ceiling/floor).

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
| `yaw_gimbal_margin_deg` | `15.0` | freeze within this many degrees of \|pitch\| = 90 |

The panel logs `yaw-fusion -> active | gated:anomaly | gated:motion | gated:gimbal | gated:no-cal` on each
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
