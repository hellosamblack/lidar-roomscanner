# LSM6DSV16X: magnetometer yaw-drift correction (host-side 9-axis fusion)

**Date:** 2026-07-10
**Status:** design (approved sections; pending written-spec review)
**Depends on:** the shipped LSM6DSV16X panel integration (`docs/superpowers/specs/2026-07-09-lsm6dsv16x-orientation-env-panel-design.md`).
Stream 9 (SFLP game-rotation-vector quaternion) and stream 10 (env: pressure / **magnetometer** / temp)
are already flowing and hardware-verified. This feature is **host-side Python only** — no firmware or
protocol change.

## Goal

The SFLP quaternion (stream 9) is a **6-axis** game-rotation vector: its tilt (roll/pitch) is
gravity-referenced and drift-free, but its **yaw is free-running and drifts** (SFLP never consumes the
magnetometer — the LSM6DSV16X has no on-chip 9-axis fusion). Today the host computes a
tilt-compensated magnetic heading (`sensors.tilt_compensated_heading`) purely for the **compass display**;
it is never fed back to correct the orientation.

Close that loop: use the magnetometer (LIS2MDL, stream 10) as a **gentle, gated, long-time-constant
drift reference** that bounds SFLP's yaw drift, producing a drift-corrected orientation quaternion.
**Tilt stays 100% SFLP** (drift-free, motion-robust); only **yaw** is nudged toward the magnetic
reference. Consumers: the live scene gizmo now, and the eventual ICP rotation prior (Phase 6) over a
long scan.

**Not** a hard heading source. Indoor magnetic yaw is worse than the point-cloud ICP yaw (rebar, wiring,
the rig's own currents distort the field), so the correction is a slow bound on drift, never an override.

## Background / feasibility (confirmed)

Established from the two ST design-tip notes (dt0058 tilt-compensated eCompass, dt0060 gyro-bridged
fusion) and the LSM6DSV16X datasheet (DS13510) / application note (AN5763):

- **No onboard 9-axis fusion.** SFLP is accel+gyro only; the sensor hub reads the mag but SFLP does not
  consume it. Yaw correction **must** be a host/MCU complementary filter — there is nothing to "turn on"
  in the sensor.
- **The data already flows.** Stream 9 gives the SFLP quaternion; stream 10 carries the raw mag vector
  (µT). No firmware/protocol change is needed for the fusion itself.
- **SFLP already applies gyro-bias correction internally** — so an explicit temperature-based gyro-bias
  model is redundant and is out of scope.
- **Considered and rejected — ASC (adaptive self-configuration), MLC/FSM.** ASC lets the FSM/MLC
  autonomously rewrite config registers (DS13510 §2.7, "mutually exclusive to the host"); it needs a
  MEMS-Studio `.ucf` blob whose reset header would drop the LSM's ENTDAA-assigned I3C address (0x50) —
  the no-reset invariant we deliberately preserve. The rig is USB-tethered (power is not a constraint,
  so ASC's power-adaptation payoff is nil), and autonomous mid-scan ODR/FS changes would corrupt the
  fixed-cadence rotation prior. Shelved unless the rig ever goes battery-powered.

### Two ST-note facts the implementation must honor

- **Frame convention (the #1 silent-yaw-mirror bug).** dt0058/dt0060 assume NED (accel positive toward
  gravity). Our SFLP quaternion is the LSM body frame and the LIS2MDL may be mounted rotated relative to
  the IMU. A one-time on-target axis-convention check (rotate to a known heading, verify sign) reconciles
  SFLP-quat frame ↔ mag mounting ↔ heading math, encoded as constants. A mismatch silently mirrors/offsets
  yaw with no other symptom.
- **Validity gating.** Tilt-compensated heading is only trustworthy when (a) the mag reads near the local
  Earth-field magnitude (else magnetic anomaly) and (b) the device is not under fast motion and not near
  gimbal lock (`|pitch|→90°`, i.e. pointed at ceiling/floor). Corrections must freeze when any gate fails.

### One design choice surfaced and accepted

We do **not** stream raw accel/gyro, so the textbook `|accel|≈1g` motion gate is unavailable. We use the
**SFLP-quaternion angular rate** (successive-quat delta / Δt) as the motion proxy. This keeps the feature
zero-protocol-change. Adding a raw-accel field to a stream for a stricter gate is a noted future option,
not part of this work.

## Architecture

All host-side, in the `roomscan` package. Three units, each independently testable:

```
[ stream 10 mag (µT) ] ──► magcal: apply hard/soft-iron ──► calibrated mag ──┐
[ stream 9 SFLP quat ] ──────────────────────────────────────────────────────┤
                                                                              ▼
                                                   YawFusion (SensorState.fused_quat()):
                                                     tilt-comp heading (reuse existing helper)
                                                     → gate (anomaly / motion / gimbal)
                                                     → yaw-only NLERP toward mag heading (long τ, ±Q guard)
                                                                              │
                                        ┌─────────────────────────────────────┴───────────────┐
                                        ▼                                                       ▼
                                  scene gizmo pose (fused_quat)                     compass = corrected heading
                                                                                    + "fusion active / gated(reason)"

  magcal calibration data (offset vec + 3×3 soft-iron matrix + Earth-field magnitude)
     ← produced by an interactive collect-while-rotating routine, persisted to JSON, loaded at startup.
```

### Component 1 — `magcal.py`: hard/soft-iron calibration

**Purpose:** raw LIS2MDL readings are offset (hard-iron) and skewed/scaled (soft-iron) by nearby ferrous
material and the rig itself. Without correction, the heading is meaningless. This is the load-bearing
prerequisite for the whole feature.

- **Model:** calibrated = `S · (raw − b)`, where `b` is the hard-iron offset (3-vector) and `S` is the
  3×3 soft-iron correction matrix. Fit by an **ellipsoid fit** over a cloud of raw mag samples collected
  while the rig is rotated through all orientations (figure-8 / tumble).
- **Outputs:** `b`, `S`, and the **expected Earth-field magnitude** `|B|₀` (mean radius of the fitted
  ellipsoid after correction) — the latter drives the anomaly gate.
- **Persistence:** written to a small JSON (path in config, e.g. `mag_cal.json`), loaded at startup.
  Absent/invalid calibration → fusion disabled (fall back to raw quat) with a clear log line, never a crash.
- **Collection UX:** an interactive routine (CLI entry point and/or a panel button) that captures N seconds
  of mag while the user rotates the rig, fits, reports fit quality (residual), and saves. Deterministic and
  unit-testable by feeding it a synthetic distorted-sphere point set.

### Component 2 — axis-convention resolution

**Purpose:** reconcile the SFLP-quat body frame, the LIS2MDL mounting, and the NED heading math so yaw
comes out with the correct sign/offset.

- A documented one-time on-target procedure: point the rig at a known heading, read raw + fused heading,
  confirm sign and rotation direction; if mirrored, set the axis-flip constants.
- Encoded as a small set of constants/config (axis permutation + sign flips) applied before the heading
  math — **not** a large module. Documented in the spec's implementation notes and covered by a sign test.

### Component 3 — `YawFusion` → `SensorState.fused_quat()`

**Purpose:** graft the mag-referenced absolute yaw onto the SFLP quaternion, yaw-only, gently.

- **Per update (each new quat/env pair):**
  1. Apply magcal to the raw mag vector.
  2. Compute tilt-compensated heading from the **SFLP quat** + calibrated mag (reuse/extend
     `tilt_compensated_heading`; SFLP supplies the drift-free tilt so no raw accel is needed for de-tilting).
  3. **Gate.** Skip the correction (hold last fused yaw) if any of:
     - `| |mag_cal| − |B|₀ |` exceeds a fraction of `|B|₀` (magnetic anomaly),
     - SFLP-quat angular rate exceeds a threshold (fast motion proxy),
     - `|pitch|` within a margin of 90° (gimbal lock — ceiling/floor).
  4. **Blend.** Build a yaw-only correction and NLERP the fused quaternion a small `(1−α)` toward the
     mag-referenced yaw, with the `dot<0 → negate` sign-flip guard. Roll/pitch are taken from SFLP
     unchanged.
- **Time constant.** `α` derived from a configurable τ (default conservative, ≈20 s at the stream cadence)
  so the filter only bounds slow drift and rejects mag noise/transients. `τ = Δt·α/(1−α)`.
- **State/threading.** `fused_quat()` lives on `SensorState` (already thread-safe: reader thread `feed()`,
  UI thread reads). Fusion state (last fused quat, last timestamp, gate status) updates inside `feed()`
  under the existing lock; `fused_quat()` and a `fusion_status()` are lock-guarded reads.
- **Degraded modes.** No calibration, or fusion disabled by config → `fused_quat()` returns the raw SFLP
  quat (identity behavior — today's gizmo). Persistent gating → holds yaw, tilt continues live.

### Component 4 — consumers / UI wiring

- `panel._update_sensors`: gizmo pose uses `fused_quat()` instead of raw `latest_quat()`.
- Compass renders the **corrected** heading; optionally overlay raw vs corrected for A/B while tuning.
- **Config fields:** `yaw_fusion` (bool, default on), `yaw_fusion_tau` (s), `mag_cal_path`, and the three
  gate thresholds (anomaly fraction, motion rate, gimbal margin). The toggle lets you compare raw vs fused
  live.
- Small status line: `fusion active` / `gated (anomaly|motion|gimbal)` / `no calibration`.

## Testing (TDD, matching the repo's existing suite discipline)

Unit tests, no hardware:
- **magcal:** synthetic sphere with a known offset + scale/skew → fit recovers `b`, `S`, `|B|₀` within
  tolerance; degenerate/too-few-points input handled gracefully.
- **fusion convergence:** static synthetic quat + mag with a fixed yaw offset → fused yaw converges to the
  mag reference over ~τ; roll/pitch **provably unchanged** (the key invariant: yaw-only graft).
- **gates:** anomalous `|mag|`, high quat angular rate, and near-gimbal pitch each freeze the correction.
- **sign-flip guard:** double-cover `±Q` inputs blend correctly (no 180° flip).
- **degraded modes:** missing calibration / fusion off → `fused_quat()` == raw quat.
- **axis convention:** a sign test pinning the chosen axis constants.

Regression: full existing host suite stays green.

## Out of scope (deferred or rejected)

- **Temperature-based gyro-bias model** (dt0064) — SFLP already bias-corrects internally; redundant.
- **Gravity-subtraction / dead-reckoning** (dt0106) — point-cloud ICP owns translation (drift-bounded);
  IMU double-integration is t²-unbounded and strictly worse. Skip.
- **Barometer Z-constraint** — a SLAM (Phase 6) soft constraint, not orientation; pressure already streams.
- **MLC / FSM / ASC** — need a `.ucf` blob (I3C-reset-header hazard) and target power/autonomy use cases
  irrelevant to a tethered rig; the built-in register-only engines already cover any future auto start/stop.
- **Streaming raw accel/gyro** for a stricter motion gate — possible future protocol change; the quat-rate
  proxy is used instead here.

## Data flow summary

`stream 9 quat + stream 10 mag → SensorState.feed() → magcal + YawFusion (gate + yaw-only NLERP) →
fused_quat() → gizmo pose + corrected compass`. Calibration is a separate interactive routine writing a
small JSON consumed at startup. Nothing touches firmware or the wire protocol.
