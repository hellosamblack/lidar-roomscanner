# Orientation / eCompass accuracy — resume point (2026-07-29)

**Read this first if you are picking up the orientation, IMU-fusion, or eCompass work in a fresh
session.** Everything below is landed on `main` unless explicitly marked otherwise.

---

## 1. Where this ended up (the short version)

The task started as "beat the fp16 SFLP quantization floor" (BUG-027's leftover item). Two things
changed the priorities mid-flight:

1. The owner clarified this is a **handheld** device — "maximum accuracy even over short timeframes."
   Stationary noise floors stopped being the interesting metric; **timing and calibration** during
   motion took over.
2. The owner reported the visible noise is in the **eCompass**, not the point cloud. Measurement
   confirmed that and traced it to a **direction-dependent magnetometer calibration** (BUG-030),
   which is an order of magnitude worse than everything else we had been optimising.

Error budget during a 100 °/s pan, measured:

| source | contribution | status |
|---|---|---|
| magnetometer calibration | up to **~90°** heading error | **OPEN — BUG-030**, needs owner |
| LSM tick uncalibrated (2.98%) | ~2.7° on a 90° pan | fixed (stream 12) |
| ToF↔IMU frame-stamp skew | 0.19° → 0.107° RMS | partly fixed — **BUG-031 open** |
| fp16 SFLP quantization | 0.018–0.027°/frame | transport shipped, **fusion not wired in** |

---

## 2. What is on `main` now

**Firmware** (`firmware/scanner-stream/`, flashed and verified on-target 2026-07-28/29):
- **Stream 11 `RS_STREAM_IMU_RAW`** — 480 Hz verbatim FIFO pass-through. N × 8-byte records:
  tag byte (`TAG_SENSOR<<3 | TAG_CNT<<1`, i.e. the datasheet `FIFO_DATA_OUT_TAG` register byte) +
  6 verbatim LE data bytes + reserved 0. Tags carried: GY_NC 0x01, XL_NC 0x02, TIMESTAMP 0x04,
  SFLP gbias 0x16, SFLP gravity 0x17. Record count rides the header `width`; `height` = 0.
- **Stream 12 `RS_STREAM_IMU_CAL`** — `INTERNAL_FREQ_FINE`, on the 64-frame CALIB cadence.
  Host tick period = `1 / (46080 · (1 + 0.0013 · freq_fine))`. **Measured freq_fine = −20**
  → true tick 22.2807 µs, not the nominal 21.7.
- **TIM2 microsecond clock** behind `rs_time_us()`; ToF frame stamped at the sensor **FRAME_READY**
  edge (end of integration), not at send time.
- **Sensor-hub averaging** (`RS_LSM_SHUB_AVERAGE`) for mag/baro/temp — they were keeping 1 of ~2
  samples per drain, the same defect class as BUG-027.

**Host** (`host/src/roomscan/`):
- `protocol.py` — `decode_imu_raw` → `ImuRawBatch`, `decode_imu_cal`. Scale factors: gyro
  17.5 mdps/LSB (±500 dps), accel 0.122 mg/LSB (±4 g), **SFLP gravity 0.061 mg/LSB (fixed ±2 g)**,
  **SFLP gbias 4.375 mdps/LSB (fixed ±125 dps)**. Only the SFLP *quaternion* is fp16; everything
  else is 16-bit fixed point.
- `imufusion.py` — complementary filter (gyro propagation on LSM timestamps, gbias subtracted;
  gravity tilt correction; stream-9 yaw anchor). **GATED OFF BY DEFAULT.** `SensorState(imu_fusion=None)`
  is the default and `fused_quat()` returns exactly what it always did. Guarded by explicit
  SLAM-non-regression tests. **This is the main un-landed capability — see §4.**
- `web.py` — raw orientation readouts, per-signal jitter (p95 headline / mean secondary), four
  orientation decomposition modes, zero-yaw control, magnetometer calibration modal.
- `host/tools/orientation_probe.py` — the canonical measurement CLI (`jitter` / `health` / `frame`).

---

## 3. Measurement method — read before trusting any number

Three separate plausible-looking readings were **wrong** during this task before the right one
emerged. The full list lives in the `orientation-noise-floor` auto-memory; the load-bearing ones:

- **Use p95 as the headline, mean as secondary. Never the median.** Measured stability over a fixed
  capture: p95 CV 1.45%, mean 2.46%, median **4.41% (worst)** — fp16 ties pile up at near-zero steps
  and shift the median discretely.
- **Normalise quaternions before computing angles** (float32 quats are only approximately unit; an
  unnormalised `arccos(|dot|)` clips to 1.0 and reports **zero** for the smallest steps, halving the
  mean and inventing a decay trend). **But compute exact-equality ties on the RAW decoded values** —
  re-normalising perturbs the floats and inflated k_eff from ~5 to 13–40. Opposite inputs, same
  analysis.
- **Never measure noise off the `/ws` `sensor` JSON `rot`** — it is rounded to 5 decimals, censoring
  sub-0.0006° changes. Compute statistics server-side from full-precision values.
- **Bypass `web.OrientationSmoother`** (`floor_alpha=1.0`) when measuring firmware effects, and
  restore it afterwards.
- **Let the rig settle after any disturbance and measure twice.** A first post-power-cycle pair
  disagreed 24.7%.
- **`capture.py --udp` starves `roomscan-web`** — both bind the device stream. To record while the
  web UI stays live, use the server's own recorder: `{"type":"record","on":true}` over `/ws`
  (writes `captures/web_<ts>.bin`).

---

## 4. Open items, in priority order

### 4.1 BUG-030 — magnetometer recalibration **(needs the owner, highest value)**
Current `host/mag_cal.json` (fitted 2026-07-15) is accurate at ceiling-facing and degrades
monotonically with tilt: |B| reads 47 µT at the ceiling and 85 µT horizontal, against a fitted
49.87 µT. Causes systematic heading errors up to ~90° in the horizontal wall-scanning attitude.

**Action:** owner runs a full-sphere tumble using the new calibration modal, spending real time in
the **horizontal** attitudes where the current fit is worst. The modal shows sphere coverage live,
flags missing regions, and gates acceptance on **|B| consistency** (the defining property of a good
calibration) rather than a bare fit residual. Re-run the tilt sweep afterwards to confirm |B| is flat.

### 4.2 Wire `imufusion` into the live display and A/B it
The filter is built, tested, and off. To land it: enable it behind the existing opt-in, then A/B
against the fp16 path using `orientation_probe.py jitter` with the smoother bypassed. Predicted gain
is **1.8× conservative / 6–18× optimistic** — the spread is genuine and only real-data measurement
settles it. Retune constants named in `imufusion.py` once Allan numbers exist (§4.4).
**Do not regress SLAM** — it reads `sensor_state.fused_quat()` directly.

### 4.3 BUG-031 — remaining ToF↔IMU skew (~890 µs)
Hypothesis, not yet fixed: the IMU FIFO is drained *later* in the loop than the ToF frame-ready
stamp, so the offset breathes with processing load. Principled fix is to capture the LSM timestamp
**at the frame-ready moment**. **Verification requires real motion** — invisible on a stationary rig.

### 4.4 Allan-variance characterisation (deferred, tool not written)
`captures/stationary_stream11_20260728_190311.bin` (900 s, 428 MB, stationary, stream 11) is
recorded and waiting. Compute overlapping Allan deviation on the GY_NC series: slope −1 =
quantization, −1/2 = angle random walk (compare to the datasheet's 2.8 mdps/√Hz ≡ 0.0028 °/√s),
flat minimum ÷ 0.664 = bias instability (ST does **not** specify it). Method summarised from DT0064.
Those numbers are the tuning constants for §4.2's crossover, not just validation.

### 4.5 Handheld dynamic verification **(needs the owner)**
Every claim about tracking during motion is inferred from stationary data plus arithmetic. A
deliberate pan is the only way to confirm the timing fixes and the fusion filter actually help.

---

## 5. Things established this session that are easy to re-litigate by accident

- **The fp16 floor is dither-limited, not step-limited.** Averaging 16 samples only buys √16 if input
  noise keeps the quantizer toggling; measured k_eff ≈ **5 of 16**. Consequence: **a quieter board
  measures worse.** An apparent 0.0118 → 0.0183 "regression" after a power cycle was exactly this,
  not a defect.
- **Orientation dependence of the fp16 floor is confirmed** (two-point test across a ~90° rotation):
  model exact at one pose (ratio 0.99), 21% under at the coarser pose.
- **The compass noise is magnetometer-dominated, not orientation-dominated.** Holding the quaternion
  fixed and varying only the magnetometer reproduces ~100% of heading jitter below 80° tilt.
- **The Euler singularity is real but secondary** — tilt-error contribution stays under 0.05° through
  60° tilt, reaching 0.944° at horizontal. Worth the alternative decompositions; not the main problem.
- **SFLP yaw has no magnetic input.** The game-rotation vector's yaw origin is arbitrary and
  free-runs. Absolute heading *requires* the magnetometer; gravity alone can never provide it.
- **Gravity + magnetometer is an excellent absolute reference and a poor standalone tracker** —
  linear acceleration is indistinguishable from gravity (0.1 g ≈ 5.7° apparent tilt). That is why the
  architecture is a complementary filter, not a TRIAD.

## 6. Useful artifacts

| path | what |
|---|---|
| `captures/stationary_stream11_20260728_190311.bin` | 900 s stationary, stream 11 — for §4.4 |
| `captures/web_20260729_061440.bin` | the braced tilt sweep (8 holds, 0°→90°) behind BUG-030 |
| `captures/postflash_verify.bin` | 25 s post-flash health + stream 11/12 verification |
| `host/tools/orientation_probe.py` | canonical jitter/health measurement |
| `docs/iks4a1-stacking.md` → "Orientation-noise pass" | the BUG-027 analysis this built on |
