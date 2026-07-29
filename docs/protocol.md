# roomscanner wire protocol — v1

Transport-agnostic binary framing for sensor→host streams. Little-endian throughout.
One frame = 32-byte header, payload, CRC32. See the `protocol-change` skill before editing.

## Frame layout

| Offset | Size | Field         | Notes                                                        |
|--------|------|---------------|--------------------------------------------------------------|
| 0      | 4    | `magic`       | ASCII `RSCN` (bytes `52 53 43 4E`)                           |
| 4      | 1    | `version`     | `1`                                                          |
| 5      | 1    | `frame_type`  | `1` = DATA, `2` = EVENT (device error/log), `3` = COMMAND (host→device), `4` = ACK (device→host) |
| 6      | 1    | `stream_id`   | see Stream registry below; ignored for COMMAND/ACK           |
| 7      | 1    | `flags`       | bit0 = DROPPED (DATA/EVENT only); COMMAND/ACK = 0            |
| 8      | 4    | `seq`         | DATA: sensor `frame_counter`. COMMAND: host-chosen token. ACK: echoes the COMMAND token (not a frame counter). Host requirement: `seq` may **restart** (jump backwards, typically to a low value) after a device recovery or REINIT — hosts must treat a backwards jump as a single discontinuity, never as an error or a huge gap. |
| 12     | 8    | `t_us`        | u64 µs since boot, from a free-running 1 MHz hardware timer (TIM2). **For a ToF frame and its paired IMU/env/raw frames this is the sensor's FRAME_READY instant — when the data became valid — not when the frame was transmitted**; the whole group shares one stamp. EVENT/ACK frames carry the live clock at send time. Ignored for COMMAND. |
| 20     | 2    | `width`       | zones (DATA/EVENT); 0 for COMMAND/ACK                        |
| 22     | 2    | `height`      | zones (DATA/EVENT); 0 for COMMAND/ACK                        |
| 24     | 4    | `payload_len` | bytes; DEPTH_ZF32 ⇒ `width*height*4`; COMMAND = 8, ACK = 12   |
| 28     | 4    | `reserved`    | 0                                                            |
| 32     | N    | payload       | row-major (DATA), stream-defined (EVENT), COMMAND/ACK-defined |
| 32+N   | 4    | `crc32`       | IEEE 802.3 / zlib `crc32` over bytes `[0, 32+N)`             |

## Stream registry

| stream_id | Name        | Payload encoding                                                | Status |
|-----------|-------------|-------------------------------------------------------------------|--------|
| 0 | DEPTH_ZF32  | float32 perpendicular Z, millimetres, row-major w×h. **No-return sentinel: 12000.0** (observed empirically, Task 8; treat ≥ max-range as invalid) | live (Phase 1) |
| 1 | DEPTH_ZAPC  | 4×float32 [x, y, z, confidence] per zone, row-major — on-device point cloud (calibrated intrinsics), 16 B/zone | reserved (Phase 2) |
| 2 | AMBIENT     | per-zone ambient level, format TBD from transform caps at enablement | reserved (Phase 2) |
| 3 | AMPLITUDE   | per-zone signal amplitude, format TBD | reserved (Phase 2) |
| 4 | CONFIDENCE  | per-zone confidence, format TBD | reserved (Phase 2) |
| 5 | REFLECTANCE | per-zone IR reflectance, format TBD | reserved (Phase 2) |
| 6 | STATUS      | per-zone status codes, format TBD | reserved (Phase 2) |
| 7 | RAW_3DMD | opaque vendor raw frame from the VL53L9CX (input to vl53l9-transform-c). At binning 2: `payload_len` = 14842. Header `width`/`height` carry the logical zone grid (54×42); `payload_len` is authoritative for size. `seq`/`t_us` as for DEPTH frames. | live (Phase 2) |
| 8 | CALIB | per-device calibration blob (`VL53L9_CALIB_DATA_SIZE` = 2332 B), required to run the transform host-side. `seq` = seq of the next RAW frame on the **periodic** 64-frame-cadence retransmit and on the stream-start send; on a **recovery/REINIT-triggered** retransmit (see EVENT section) `seq` instead carries the *last-captured* frame's counter, same convention as EVENT frames, because the next RAW frame's restarted counter is unknowable at send time. `width`/`height` = zone grid. Sent at stream start and **retransmitted every 64 RAW frames** so late-attaching hosts acquire it (a host must buffer or discard RAW frames until a CALIB arrives). | live (Phase 2) |
| 9 | IMU_QUAT | LSM6DSV16X SFLP game-rotation-vector: 4×float32 `[w, x, y, z]` unit quaternion (16 B), LSM body frame. One per ToF frame. `t_us` = capture time. **6-axis fusion — yaw drifts, uncorrected on-chip.** | live (Phase 4) |
| 10 | ENV | LSM6DSV16X sensor-hub environmental sample: float32 pressure (Pa) + 3×float32 magnetic field `[x, y, z]` (µT) + float32 temperature (°C) = 20 B. One per ToF frame. `t_us` = capture time. | live (Phase 4) |
| 11 | IMU_RAW | LSM6DSV16X FIFO words passed through **verbatim** (same philosophy as RAW_3DMD: sensor format on the wire, host demuxes). N × 8-byte records, `payload_len` = N × 8; **the header's `width` carries N and `height` is 0** (there is no zone grid). One frame per ToF frame, emitted only when N > 0. See the record layout below. | live (2026-07-28) |
| 12 | IMU_CAL | LSM6DSV16X clock calibration: `INTERNAL_FREQ_FINE` (register 0x4F), the factory trim that sets what a stream-11 TIMESTAMP tick is actually worth. 4 B, `width` = `height` = 0. Static per device; sent on the **same 64-frame cadence as CALIB** (and at stream start) so a late-attaching host or a mid-recording seek always has one. Emitted only when the register read succeeded — its absence means "use the nominal tick". See the payload layout below. | live (2026-07-28) |

TBD formats are pinned when the stream is first enabled (the transform library's capability
negotiation decides); pinning a TBD format is additive (no version bump); *changing* a pinned
encoding requires a version bump.

### IMU_RAW (stream 11) record layout

Why it exists: stream 9's SFLP game-rotation quaternion is encoded **fp16** inside the FIFO
(~0.056°/step), which — after the 2026-07-28 batch-averaging fix — is the orientation noise
*floor*. Stream 11 carries the 16-bit fixed-point words the fusion is built from, so the host can
fuse orientation itself and beat that floor. It is transport only: nothing consumes it yet.

Each record is exactly 8 bytes, little-endian throughout:

| Offset | Size | Field      | Notes |
|--------|------|------------|-------|
| 0      | 1    | `tag`      | the sensor's `FIFO_DATA_OUT_TAG` register byte: `TAG_SENSOR << 3 \| TAG_CNT << 1`. Bit 0 is the register's `not_used0` and is **always 0** — the ST driver (`lsm6dsv16x_fifo_out_raw_get`) decodes the register into a bitfield and discards it, so the firmware rebuilds the byte from `tag`/`cnt`. `TAG_CNT` (bits 2:1) is the 2-bit sample-time slot counter and is **preserved deliberately**: it is what groups a gyro word with the accel/timestamp words of the same sample time. |
| 1      | 6    | `data`     | the six FIFO data bytes, untouched (sensor encoding, LE) |
| 7      | 1    | `reserved` | 0 |

Tags carried (all others are dropped by the firmware — 0x13 game-rotation rides stream 9, the
sensor-hub tags 0x0E–0x10 ride stream 10):

| `TAG_SENSOR` | Meaning | `data` encoding | Sensitivity |
|--------------|---------|-----------------|-------------|
| 0x01 | `GY_NC` gyroscope | 3 × int16 `[x, y, z]` | 17.5 mdps/LSB |
| 0x02 | `XL_NC` accelerometer | 3 × int16 `[x, y, z]` | 0.122 mg/LSB |
| 0x04 | `TIMESTAMP` | uint32 tick in `data[0:4]`, BDR metadata in `data[4:6]` | **stream 12**, nominally 21.7 µs/tick |
| 0x16 | SFLP gyroscope bias | 3 × int16 `[x, y, z]` | 4.375 mdps/LSB (fixed ±125 dps) |
| 0x17 | SFLP gravity vector | 3 × int16 `[x, y, z]` | 0.061 mg/LSB (fixed ±2 g) |

**The gyro and accel sensitivities depend on the full scale the firmware configures** —
`RS_LSM_GY_FS` (±500 dps) and `RS_LSM_XL_FS` (±4 g) in
`firmware/scanner-stream/Src/rs_lsm.c`. Change those knobs and the host constants
(`IMU_RAW_GY_MDPS_PER_LSB` / `IMU_RAW_XL_MG_PER_LSB` in `host/src/roomscan/protocol.py`) must
change with them; there is no full-scale field on the wire. The two SFLP vectors are fixed-scale
regardless of the XL/GY full scale.

The timestamp word exists to put every sample on the LSM's own clock, at a resolution the frame
header cannot reach. **Scale it with stream 12, not with the nominal 21.7 µs** — see below.

Volume: gyro, accel and both SFLP vectors batch at 480 Hz plus one timestamp word per sample
time = ~2880 words/s, drained once per ToF frame (~28 Hz) ⇒ **~90–105 records/frame, ~720–840 B**,
inside a single 1400 B UDP datagram. The FIFO holds 256 words, so a drain has ~2.3× headroom but
**one missed drain overruns it**.

**Stream 9 is unchanged** by this addition — same fp16 game-rotation word, same firmware
batch-averaging, byte-for-byte identical behaviour.

### IMU_CAL (stream 12) payload layout

| Offset | Size | Field       | Notes                                                            |
|--------|------|-------------|------------------------------------------------------------------|
| 0      | 1    | `freq_fine` | `INTERNAL_FREQ_FINE` (register 0x4F), **int8, two's complement**  |
| 1      | 1    | `valid`     | 1 = `freq_fine` was read from the device; 0 = use the nominal tick |
| 2      | 2    | reserved    | 0                                                                 |

Total 4 B. `width` = `height` = 0; `seq` = the ToF frame counter it was sent alongside.

**The formula the host must apply** (AN5763 rev 4, §6.4) — this is the whole point of the stream:

```
t_tick = 1 / (46080 * (1 + 0.0013 * freq_fine))     seconds
```

`INTERNAL_FREQ_FINE` is the factory trim of the LSM's internal oscillator, in 0.13% steps. That
oscillator clocks **both** the ODRs and the FIFO timestamp counter, so the nominal 21.7 µs tick
is only right for a part that happened to land on `freq_fine = 0`. Measured on this rig
2026-07-28 over 8027 paired ToF frames / 257 s, the host clock ran **1.029790×** the LSM's —
~29790 ppm of pure scale error, entirely because the host was integrating gyro against the
nominal tick. At that error a 90° pan integrates to ~87.3°, which swamps every other orientation
error term we have.

Compatibility: this is an additive `stream_id`, so decoders that predate it skip it (see
*Decoder requirements*) and captures that predate it simply never carry one. A host **must**
default to the nominal 21.7 µs until a stream-12 frame arrives, which is byte-for-byte the
behaviour every existing recording was decoded with. Likewise `valid = 0` means fall back to
nominal — it exists so a failed register read can never be mistaken for a legitimate
`freq_fine` of 0.

## EVENT frame payload (frame_type = 2)

| Offset | Size | Field   | Notes                                   |
|--------|------|---------|-------------------------------------------|
| 0      | 4    | code    | u32 LE, see event-code registry         |
| 4      | 4    | detail  | u32 LE, code-specific (e.g. sensor status word, retry count) |
| 8      | N    | message | optional ASCII (not NUL-terminated; length = payload_len − 8) |

Header fields for EVENT frames: stream_id = 0 (ignored), width = height = 0, seq shares the DATA
counter sequence (an EVENT does not increment it — it carries the seq of the last captured frame).

Event-code registry:

| code | Name               | detail meaning                     |
|------|--------------------|--------------------------------------|
| 1 | SENSOR_INIT_FAIL   | 1-based attempt number within the bounded boot/recovery retry cycle (see below) |
| 2 | TRIGGER_TIMEOUT    | retry count at exhaustion           |
| 3 | DMA_TIMEOUT        | retry count at exhaustion (currently always 1 — no internal retry loop precedes this timeout) |
| 4 | SENSOR_ERROR_STATUS| vl53l9 status word from handle path |
| 5 | TX_OVERFLOW        | frames dropped since last report    |

Firmware emission (Phase 3 Task 5, raw-only builds only — `CONF_TRANSFORM_ONBOARD=0`):
`handle_error()` emits `SENSOR_ERROR_STATUS` (detail = packed status word: `fsm<<24 |
command<<16 | firmware`) then runs a bounded recovery loop — up to 5 full sensor
re-init attempts (100/200/400/800/1600 ms backoff), emitting `SENSOR_INIT_FAIL` per
*failed* attempt with detail = that attempt's 1-based index — before giving up and
disconnecting. On successful recovery the device retransmits a CALIB frame (calibration
is re-read during re-init and may have changed across the physical reset) before RAW
streaming resumes; its `seq` carries the last captured frame's counter (like EVENT
frames — the next RAW's restarted counter is unknowable at send time). The same bounded-retry shape wraps the pre-loop boot sequence, turning
the historical ~1-in-5 first-power-up failure into a self-healing delay; boot-time
`SENSOR_INIT_FAIL` events are emitted but drop silently (no host is attached yet at that
point in boot). `TRIGGER_TIMEOUT`/`DMA_TIMEOUT` are emitted at their respective retry
exhaustion points immediately before `handle_error()` runs. On-board-transform builds
(`CONF_TRANSFORM_ONBOARD=1`, the golden-pair regeneration path) are unchanged: no EVENT
emission, `handle_error()` still spins forever on any fault — golden-path stability, not
a wire-format distinction.

## COMMAND frame payload (frame_type = 3)

Host→device commands. Header `seq` = host-chosen token (not a frame counter); `stream_id`, `width`, `height`, `flags` all 0.

| Offset | Size | Field  | Notes                  |
|--------|------|--------|------------------------|
| 0      | 4    | cmd    | u32 LE, see command registry |
| 4      | 4    | param  | u32 LE, command-specific (e.g. usecase ID, period in µs, exposure in ms) |

All COMMAND payloads are 8 bytes; header `payload_len` = 8.

## ACK frame payload (frame_type = 4)

Device→host acknowledgement of a COMMAND. Header `seq` = echoes the COMMAND token (not the device's frame counter); `stream_id`, `width`, `height`, `flags` all 0.

| Offset | Size | Field   | Notes                  |
|--------|------|---------|------------------------|
| 0      | 4    | cmd     | u32 LE, echoes the command code from the COMMAND |
| 4      | 4    | result  | u32 LE, 0 = OK; nonzero = error (see result-code registry) |
| 8      | 4    | applied | u32 LE, command-specific: applied value, detail, or info |

ACK payloads are exactly 12 bytes (header `payload_len` = 12); longer payloads are malformed
and rejected — unlike EVENT's legitimate variable message tail. Future ACK growth would come
via a new frame revision.

### Command registry

| cmd | Name              | param meaning | applied meaning |
|-----|-------------------|---------------|-----------------|
| 1   | PING              | ignored       | firmware protocol version (u32) |
| 2   | SEND_CALIB        | ignored       | 0 — device transmits a CALIB frame immediately; lets a late-attaching host obtain calibration immediately instead of waiting the ≤63-frame retransmit cadence (closes ROADMAP's CALIB-on-DTR-connect item when wired in firmware) |
| 3   | SET_USECASE       | usecase ID (u16) | applied usecase ID (u16) |
| 4   | SET_FRAME_PERIOD_US | period in µs (u32) | applied period (u32) — stored and echoed (read back from the sensor), but has no observable effect while the app uses `VL53L9_SYNC_MANUAL` (vl53l9.h:248 — period governs AUTONOMOUS mode only); retained for the future autonomous-mode option |
| 5   | SET_EXPOSURE_MS   | exposure in ms (u32) | applied exposure (u32) |
| 6   | REINIT            | ignored       | 0 — the ACK is sent **after** the re-init completes (normally well under the host's 2 s timeout); if the first re-init attempt itself faults, the device enters its bounded recovery ladder (up to ~3.1 s) and may finish successfully after the host has already timed out — hosts must treat a REINIT timeout as "outcome unknown", not "failed" (a late ACK is silently ignored by token matching) |
| 7   | SET_STANDBY       | standby level (u32): 0 = wake/resume, 1 = soft standby, 2 = hard power-down | standby level now in effect (u32) — echoes param on success. Idles the ToF laser (VCSEL) to reduce wear when no host is viewing. **Soft** (1) = `vl53l9_stop()` → FSM STANDBY: VCSEL stops firing per frame, I3C config/calibration retained, instant resume via wake. **Hard** (2) = additionally `platform_power_disable()` (XSHUT low): sensor fully unpowered; waking re-runs the full re-init cycle (reset → re-address → init → calib → start), so a wake-from-hard ACK is sent **after** that completes (same "outcome unknown on timeout" caveat as REINIT). Applied only at the per-frame safe point (after readout ack, before the next trigger) so `vl53l9_stop()` never races an in-flight trigger. While idled the device streams no DATA frames but keeps servicing the command channel; wake resumes streaming. A wake (0) issued while already active, or a standby issued while already idled at that level, is a harmless no-op ack |

### Result-code registry

| code | Name               | meaning                          |
|------|--------------------|----------------------------------|
| 0    | OK                 | command succeeded                |
| 1    | UNKNOWN_CMD        | command code not recognized      |
| 2    | BAD_PARAM          | parameter out of valid range     |
| 3    | REJECTED_BINNING   | SET_USECASE rejected (binning mismatch) |
| 4    | SENSOR_ERROR       | sensor operation failed (applied = status word) |
| 5    | BUSY               | device not ready (e.g. frame in progress) |

## Decoder requirements

- Resync by scanning for `magic`; tolerate arbitrary garbage (e.g. ASCII boot text) between frames.
- Bound `payload_len` (reject > 1 MiB) before buffering; an oversize `payload_len` is a framing
  rejection: resync exactly as for CRC failure; count it under `bytes_skipped` (not `crc_failures`).
- On CRC failure: advance one byte past the magic candidate and rescan; count failures, never raise.
- Skip unknown `stream_id`/`frame_type` values silently (forward compatibility, no version bump needed).
- flags bit0 DROPPED: set on the first frame sent after one or more captured frames could not be
  transmitted. Hosts should treat seq gaps as the authoritative drop count; DROPPED is a cheap hint.

### Payload size bound

A single frame's payload is ≤ 1 MiB by decoder policy; firmware transports may impose tighter
bounds (UART path: ≤ 65535 B per HAL transfer — larger payloads require chunked transfers, to be
specced with the Phase 4 transport work).

## USB identification

- Milestone 1a: ST-Link VCOM (VID `0x0483`), 921600 8N1.
- Milestone 1b: native CDC ACM, VID `0xCAFE` PID `0x4001` (TinyUSB descriptors). Confirmed on
  hardware (Task 11): enumerates as its own COM port alongside the ST-Link VCOM (e.g. COM15 next
  to COM14 on Windows); `SerialSource`'s baud parameter is a no-op on this port.

## Version history

- **v1** (2026-07): initial — DATA/EVENT frame types, DEPTH_ZF32 stream.
- **v1 rev 2026-07-08**: additive — stream registry (IDs 1-6 reserved), EVENT payload defined,
  DROPPED/oversize semantics clarified, ZF32 no-return sentinel documented. No layout change.
- **v1 rev 2026-07-08 (b)**: additive — RAW_3DMD (7) and CALIB (8) allocated for the PC-side-transform architecture. No layout change.
- **v1 rev 2026-07-08 (c)**: additive — COMMAND (frame_type=3) and ACK (frame_type=4) frame types, command registry v1 (PING/SEND_CALIB/SET_USECASE/SET_FRAME_PERIOD_US/SET_EXPOSURE_MS/REINIT), result-code registry. No layout change.
- **v1 rev 2026-07-08 (d)**: semantics clarification (additive, no wire change) — SET_FRAME_PERIOD_US is stored and echoed but has no observable fps effect under the app's always-manual sync mode (driver: period governs AUTONOMOUS mode only); documented in the command-registry row, discovered during Phase 3 Task 4 hardware verification.
- **v1 rev 2026-07-08 (e)**: semantics pinned (additive, no wire change) — EVENT emission wired in firmware (Phase 3 Task 5, raw-only builds): `SENSOR_INIT_FAIL`'s detail is the bounded-retry attempt number (not a status word, superseding the earlier placeholder wording), emission points and the `handle_error()` bounded-recovery design documented in the EVENT section above.
- **v1 rev 2026-07-08 (f)**: semantics clarified (additive, no wire change) — CALIB registry row split its `seq` convention: periodic/stream-start retransmits carry the next RAW frame's counter, but a recovery/REINIT-triggered retransmit carries the last-captured counter (EVENT-frame convention), per code review deferred from Phase 3 Task 5 and applied in Task 6.
- **v1 rev 2026-07-09**: additive — IMU_QUAT (9) and ENV (10) streams for LSM6DSV16X orientation +
  sensor-hub environmental data. No layout change; hosts skip unknown stream_ids, no version bump.
- **v1 rev 2026-07-16**: additive — SET_STANDBY (cmd 7) command for laser-wear reduction: idles the
  ToF VCSEL (soft = FSM STANDBY / hard = XSHUT power-down) when no host is viewing, wakes on demand.
  New enum value only, unchanged 8-byte COMMAND / 12-byte ACK layout; no version bump. The host web
  server drives it automatically off its viewer count (debounced), depth selectable via `[viewer]`
  `idle_level`. No firmware default behavior change: the device only idles when commanded.
- **v1 rev 2026-07-28**: additive — IMU_RAW (11): verbatim LSM6DSV16X FIFO-word pass-through
  (GY_NC / XL_NC / TIMESTAMP / SFLP gbias / SFLP gravity) at 480 Hz batching, N × 8-byte records
  with the record count in the header's `width` and `height` = 0. New stream_id only, unchanged
  32-byte header layout; hosts skip unknown stream_ids, no version bump. Transport for host-side
  orientation fusion that escapes the SFLP FIFO's fp16 quaternion noise floor (~0.056°/step);
  the fusion itself is a follow-up. Streams 9 and 10 are unaffected. Firmware enables it via
  `RS_LSM_RAW_BATCH` / `RS_LSM_SFLP_BATCH_AUX` in `firmware/scanner-stream/Src/rs_lsm.c`.
- **v1 rev 2026-07-28 (b)**: additive — IMU_CAL (12) plus a `t_us` **semantics** change (no layout
  change, no version bump; the field is still a u64 µs at offset 12). Two timing defects measured
  on the rig, both of which dominate handheld accuracy during motion:
  1. *Clock scale.* Over 8027 paired ToF frames / 257 s the host-vs-LSM clock ratio was
     **1.029790** (~29790 ppm) because the host scaled stream-11 timestamp ticks by the nominal
     21.7 µs. Stream 12 carries the device's `INTERNAL_FREQ_FINE` so the host can apply the real
     period, `1 / (46080 * (1 + 0.0013 * freq_fine))` (AN5763 §6.4). New stream_id only; hosts
     skip unknown stream_ids and older captures (which never carry one) keep decoding against the
     nominal tick exactly as before.
  2. *Frame-stamp jitter.* `t_us` was `HAL_GetTick() * 1000` (1 ms granular) sampled at **send**
     time, so all the variable latency between data-ready and transmit folded in: 1.9 ms RMS /
     3.4 ms p95 / 6.2 ms max skew against the IMU clock, i.e. 0.19° RMS / 0.62° worst case of
     depth-vs-rotation misalignment at 100 °/s. `t_us` now comes from a 1 MHz free-running TIM2
     (32-bit, software-extended to the u64 wire field) and is **captured at the sensor's
     FRAME_READY edge**, then shared by that frame's RAW/CALIB/IMU/env/raw sends.
  Streams 7/9/10/11 are byte-for-byte unchanged in framing and payload; only what `t_us` means
  changed, and it changed to something strictly more accurate.
