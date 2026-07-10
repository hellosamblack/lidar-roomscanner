# LSM6DSV16X → panel: orientation + environmental sensor data

**Date:** 2026-07-09
**Status:** design (approved sections; pending written-spec review)
**Depends on:** the HUB1 native-I3C bus fix (`rs_assign_dynamic_addresses()`, on `origin/main`) — the
LSM6DSV16X is already a working native-I3C target at `0x50` on the shared I3C1 bus.

## Goal

Surface the LSM6DSV16X's data in the `roomscan` panel, shown **visually rather than as raw numbers**:
1. Device **orientation** (SFLP hardware sensor-fusion game-rotation-vector quaternion) as a live 3D
   gizmo in the point-cloud scene.
2. **Environmental sensors** reached through the LSM's I2C **sensor-hub (SHUB)** master —
   pressure (LPS22DF), magnetic field (LIS2MDL), temperature (STTS22H) — as visual panel widgets.

This is the first data-bearing step of ROADMAP Phase 4 (IKS4A1 integration). It is **visualization
only**: it does not wire orientation or any sensor into SLAM/registration (Phase 6).

## Background / feasibility (confirmed)

The bus fix put the LSM6DSV16X on the shared I3C1 bus but nothing reads it yet — there is no IMU/env
data on the wire. Getting it requires three layers (firmware read → protocol stream → host viz).

Hardware feasibility was verified against the IKS4A1 schematic (UM3239) and the LSM6DSV16X datasheet
(DS13510):
- Our current jumpering (UM3239 "Mode 3": J4 5-6, J5 5-6 → `HUB1_SDx/SCx = SENS_SDA/SCL`) routes the
  LSM's **sensor-hub master pins** (`SDx/AH1`, `SCx/AH2`) to the `SENS_I2C` bus carrying **LPS22DF
  (`0x5C`), LIS2MDL (`0x1E`), STTS22H (`0x38`)**. These three are reachable via SHUB **without
  re-jumpering**.
- **SHT40 humidity (`0x44`) is out of scope:** it is hard-bridged (SB37/SB39) to the main bus, not the
  sensor-hub bus, and is a command-response part that does not fit the SHUB register model. (It is still
  physically on the shared I3C1 bus as a legacy target — likely why the HUB1 fix held at 28 fps with
  only one legacy sensor loading the bus. A future direct legacy-I2C read is possible but deferred.)
- **SFLP and SHUB coexist:** they use different register banks (`EMB_FUNC_REG_ACCESS` vs
  `SHUB_REG_ACCESS`, same `FUNC_CFG_ACCESS` mux, config-time only) and distinct FIFO tags
  (`0x13` game-rotation-vector; `0x0E–0x11` sensor-hub slaves). The datasheet lists both as simultaneous
  FIFO sources.
- **One documentation-unanswerable risk:** whether SHUB traffic (LSM-internal, off the STM32 bus)
  perturbs the ToF frame cadence. This is a bench check, not a design question.

## Architecture

Three components, each independently understandable and testable:

```
[ LSM6DSV16X ]                      [ firmware: scanner-stream ]        [ host: roomscan panel ]
 SFLP → FIFO tag 0x13  ─────┐        acquisition loop, per ToF frame:     frame path:
 SHUB → FIFO tags 0x0E-10 ──┴─(I3C)→  drain FIFO, demux by tag,      (CDC)  decode stream 9 → quaternion
 (baro/mag/temp masters)             emit stream 9 (quat) every frame ───→  decode stream 10 → env sample
                                     emit stream 10 (env) every frame too    ├─ scene gizmo (orientation)
                                                                            └─ Sensors group (compass +
                                                                               pressure/temp sparklines)
```

### Component 1 — Firmware LSM6DSV16X driver (`scanner-stream`) — owner's bench lane

**Purpose:** configure the LSM once, then hand the acquisition loop the latest orientation + env values.

- **Bring-up** (after the ToF is up and `rs_assign_dynamic_addresses()` has put the LSM at `0x50`):
  - Enable SFLP: `SFLP_ODR ≈ 120 Hz`, game-rotation-vector batched to FIFO (tag `0x13`).
  - Configure SHUB: 3 slaves (LPS22DF `0x5C`, LIS2MDL `0x1E`, STTS22H `0x38`) via
    `MASTER_CONFIG` + `SLV0..2_ADD/SUBADD/CONFIG`, `SHUB_ODR ≈ 60 Hz` (≥ the ToF frame rate so every
    emitted ENV frame carries a fresh sample; still below the 120 Hz SFLP rate), results
    batched to FIFO (tags `0x0E–0x10`). One-time per-slave power-up writes via the `DATAWRITE_SLV0` +
    `WRITE_ONCE` channel (sequenced during bring-up, since only one write-once channel exists), or rely
    on sensor defaults where sufficient.
  - Register-bank access is arbitrated through `FUNC_CFG_ACCESS` at config time only — never toggled
    mid-stream.
  - All LSM register I/O uses the existing native-I3C private read/write to `0x50` (the same transfer
    helpers the probe already exercised).
- **Steady state:** each ToF frame, drain the FIFO and demux by tag — keep the latest quaternion and the
  latest env sample. Emit **stream 9 (quaternion) and stream 10 (env) every ToF frame** — one paired set
  per frame. With `SHUB_ODR ≥ frame rate` the per-frame env values are fresh; per-frame ENV keeps the
  data frequent enough to serve later as a SLAM input (baro Z-drift constraint, mag heading) without a
  separate low-rate path.
- **Error isolation (hard requirement):** any SFLP/SHUB failure (init error, empty FIFO, NACK tag
  `0x19`) skips that frame's IMU/ENV emission and **never** blocks, delays, or corrupts the ToF RAW/CALIB
  stream. A boot-time SFLP/SHUB init failure emits an EVENT (diagnostic) and the ToF stream proceeds
  IMU-less.

### Component 2 — Protocol (`docs/protocol.md`) — via the protocol-change skill

Two **additive** DATA streams (IDs 9 and 10 are free; 0–8 allocated). Little-endian. Hosts skip unknown
`stream_id`s, so no version bump — but the change goes through the `protocol-change` checklist (spec +
firmware C + host Python + golden vectors in lockstep).

- **stream 9 `IMU_QUAT`** — payload = 4×float32 `[w, x, y, z]` (16 B), unit quaternion, LSM body frame.
  `t_us` = capture time. Cadence: one per ToF frame.
- **stream 10 `ENV`** — payload = pressure float32 (**Pa**) + magnetic field 3×float32 `[x, y, z]`
  (**µT**) + temperature float32 (**°C**) (20 B), standard scientific units. Each sensor's native-LSB →
  unit conversion is pinned against its datasheet and frozen in the golden vector. `t_us` = capture time.
  Cadence: **one per ToF frame** (paired with `IMU_QUAT`), keeping env frequent enough for later SLAM use.

The stream registry table gets two rows; the payloads are pinned (not TBD) since we define them here.

### Component 3 — Host panel + decode (`roomscan`)

**Decode:** extend the frame path to recognize stream 9 → latest quaternion, stream 10 → latest env
sample (plus a short bounded history ring per env channel for sparklines). No change to the RAW/CALIB
path.

**Orientation gizmo (scene):** an axis-triad coordinate frame (`create_coordinate_frame`, small, anchored
at a fixed scene position) added to the `SceneWidget`; its transform is updated from the quaternion each
frame. It rotates live with the device in the same space as the cloud.
- Coordinate frame: the SFLP quaternion is in the LSM body frame; the viewer world has its own
  up/forward (`_WORLD_UP`). Apply a fixed default body→world rotation; the gizmo shows relative rotation
  faithfully regardless. Precise absolute alignment is a **fast-follow calibration** once it's seen
  moving on hardware.

**Sensors panel group:** a new collapsible `_group("Sensors")` mirroring the IR-Monitor pattern
(`numpy` render → `gui.ImageWidget`, updated each tick):
- **Compass dial** — **tilt-compensated** magnetometer heading: the raw mag vector is de-tilted using
  the SFLP orientation quaternion (roll/pitch) before the heading is computed, so the dial stays correct
  when the device is not level.
- **Pressure sparkline** — trend over the history ring (doubles as the eventual baro Z-drift indicator).
- **Temperature sparkline** — trend over the history ring.

**Config + toggles:** new `ViewerConfig` fields (`imu_gizmo: bool`, `sensors_panel: bool`, optional
`gizmo_scale: float`), persisted like the existing IR/near/surface fields; a keybind to toggle the gizmo.

**Graceful absence:** if no stream-9/10 frames arrive (old firmware, IKS4A1 absent, or IMU init failed),
the gizmo stays hidden and the sensor widgets show a neutral "no data" state — the point cloud and all
existing panel features are unaffected.

### Orientation drift & the magnetometer (verified)

The LSM6DSV16X's SFLP is a **6-axis game rotation vector** (accelerometer + gyroscope only) — confirmed
against DS13510 (§2.8; FIFO tag `0x13` is the *only* rotation-vector tag) and the reg driver (only
`sflp_game_en`). **The chip does not fuse the magnetometer into the quaternion**, and no on-chip
FSM/MLC/SFLP path produces a mag-corrected orientation. Consequence: **pitch and roll are gravity-bounded
and stable; yaw/heading drifts slowly and is uncorrected by the chip** (~0.5°/5 min — a drift rate, not
an absolute bound).

**Design decision (this visualization-only scope): accept the yaw drift.** It is cosmetic for a live
preview, and SLAM/G-ICP corrects heading in Phase 6 regardless. But **stream the LIS2MDL magnetometer
anyway** — it already comes for free with the SHUB env slice (stream 10). That makes host-side yaw
correction ("game rotation vector + tilt-compensated magnetic heading → geomagnetic heading") a
**pure-software fast-follow** whenever Phase 6 wants it: no firmware rework is ever needed because the mag
is already on the wire. The tilt-compensated compass widget is the visible, independent absolute-heading
reference in the meantime.

## Data flow

1. LSM continuously runs SFLP (→FIFO 0x13) and SHUB (→FIFO 0x0E–0x10) internally.
2. Firmware acquisition loop, per ToF frame: capture ToF RAW as today; drain LSM FIFO; demux tags; keep
   latest quaternion + env sample; emit stream 9 and stream 10 (one paired set per frame) over CDC.
3. Host decodes streams 9/10 alongside RAW/CALIB; updates the scene gizmo transform and the Sensors
   widgets each render tick.

## Testing

- **Host + protocol (no hardware):** golden vectors for streams 9 and 10; synthetic frame injection into
  the decode path; a replay capture with synthetic IMU/ENV frames appended; unit tests for
  quaternion→gizmo-transform math, env decode/scale, and widget rendering via the existing headless
  snapshotter. Host suite stays green.
- **Firmware (on-bench, owner):** rotate the board → gizmo tracks; env values plausible (baro ~101325 Pa,
  room temp in °C, non-zero mag in µT); **critical bench gate — ToF cadence unchanged** (~28 fps, 0 CRC,
  0 gaps with SHUB + SFLP active), directly testing the one documentation-unanswerable risk.
- **End-to-end:** both boards stacked, panel shows live gizmo + Sensors widgets while the cloud streams.

## Scope / landable slices (one spec)

1. **Orientation MVP** — protocol stream 9 + host decode + scene gizmo. Host-testable against synthetic
   frames before firmware lands.
2. **SHUB environmental** — firmware SHUB config + protocol stream 10 + host Sensors widgets (compass +
   pressure/temp sparklines).

Firmware SFLP and SHUB naturally land together on-bench; the host slices are independently testable.

## Out of scope

- **SHT40 humidity** — not reachable via SHUB on this board; direct legacy-I2C read deferred (would
  re-exercise a legacy PP-bus target, bench-gated).
- **SLAM / registration** wiring of orientation or any sensor (Phase 6).
- **Independent high-rate IMU streaming** — cadence is one quaternion per ToF frame by design.

## Open items / risks

- **Bench:** SHUB traffic vs ToF cadence — must confirm ~28 fps / 0 CRC holds with SHUB active.
- **Coordinate-frame alignment:** body→world mapping default now, calibrated fast-follow on hardware.
- **Env unit conversions:** pin each sensor's native-LSB → SI conversion (Pa / µT / °C) from its
  datasheet into the stream-10 golden vector.
- **SHUB write-once sequencing:** only one `SLV0` write-once channel — multi-sensor init must sequence
  (bring-up detail for the firmware plan).
