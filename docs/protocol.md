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
| 12     | 8    | `t_us`        | u64 µs since boot (v1 source: `HAL_GetTick()*1000`, 1 ms resolution; a TIM-backed µs clock is planned with Phase 5 IMU fusion); ignored for COMMAND/ACK |
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

TBD formats are pinned when the stream is first enabled (the transform library's capability
negotiation decides); pinning a TBD format is additive (no version bump); *changing* a pinned
encoding requires a version bump.

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
