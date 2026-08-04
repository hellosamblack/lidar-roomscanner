# roomscanner wire protocol — v2

Transport-agnostic binary framing for sensor→host streams. Little-endian throughout.
One frame = 32-byte header, payload, CRC32. See the `protocol-change` skill before editing.

v2 (2026-08, see "Version history") changed only the COMMAND/ACK registry: five new
command codes (8-12) and, with them, one new COMMAND payload shape and one new ACK
payload shape. The 32-byte header and every DATA/EVENT stream are byte-for-byte
unchanged from v1. A v1 recording still decodes — `FrameHeader.unpack()` accepts either
version byte, and new encodes use v2 (`roomscan.protocol.SUPPORTED_VERSIONS = (1, 2)`).

## Frame layout

| Offset | Size | Field         | Notes                                                        |
|--------|------|---------------|--------------------------------------------------------------|
| 0      | 4    | `magic`       | ASCII `RSCN` (bytes `52 53 43 4E`)                           |
| 4      | 1    | `version`     | `1` or `2` (see "Version history"); new encodes use `2`      |
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
| 13 | IMU_SYNC | The two instants a host needs on the **LSM's own clock**: where this ToF frame's FRAME_READY edge sits (the LSM `TIMESTAMP` register read at that edge, plus the delay and the read's duration), and when the stream-9 quaternion sent alongside it is valid (the averaged batch's midpoint). 22 B, `width` = `height` = 0. **One per ToF frame**, carrying the same `seq` and `t_us` as the RAW frame it describes — the pairing is the header, not a payload field. Emitted only when the register read succeeded; its absence means "unknown". See the payload layout below. | live (2026-07-30) |

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

### IMU_SYNC (stream 13) payload layout

| Offset | Size | Field            | Notes                                                             |
|--------|------|------------------|-------------------------------------------------------------------|
| 0      | 4    | `lsm_ticks`      | u32 LE — LSM `TIMESTAMP0..3` (0x40–0x43), read just after FRAME_READY. **Same counter and units as stream 11's TIMESTAMP words**, so scale it with stream 12, not the nominal 21.7 µs |
| 4      | 4    | `latch_delay_us` | u32 LE — MCU µs from this frame's `t_us` (the FRAME_READY edge) to the **midpoint** of that register read |
| 8      | 4    | `drain_delay_us` | u32 LE — MCU µs from `t_us` to the FIFO drain that produced this frame's streams 9/10/11. Diagnostic: this is the lag the old FIFO-word inference silently absorbed (measured: 24.3 ms) |
| 12     | 4    | `quat_mid_ticks` | u32 LE — the LSM tick the **stream-9 quaternion sent alongside this frame is valid at**: the midpoint of the averaged FIFO batch. 0 when this drain produced no quaternion |
| 16     | 2    | `read_us`        | u16 LE — duration of the latch read; the uncertainty bound on `lsm_ticks` |
| 18     | 2    | `quat_n`         | u16 LE — SFLP samples averaged into that quaternion               |
| 20     | 1    | `valid`          | 1 = latched from the device (the only value ever sent)             |
| 21     | 1    | reserved         | 0                                                                  |

Total 22 B. The frame-ready edge on the LSM clock is `lsm_ticks - latch_delay_us / tick_us`
(`roomscan.protocol.ImuSync.frame_ready_ticks`).

**`quat_mid_ticks` — read the sign carefully.** Stream 9 carries the *mean* of a whole FIFO
batch (firmware `RS_LSM_SFLP_AVERAGE`, shipped for BUG-027's 2.8× noise cut), so the orientation
it holds is the batch's midpoint, not the frame's `t_us`. Measured on this rig over 3119 frames:
the batch's last sample sits **+23.1 ms after** the frame-ready edge and its midpoint **+7.8 ms
after** it (std 0.68 ms) — the batch straddles the edge, because the drain runs 24.3 ms into the
frame. So the quaternion **leads** the depth frame by ~7.8 ms (~0.3° at 38.5 °/s), roughly 9× the
frame-stamp skew this stream's other fields address. A host correcting for it must propagate the
orientation **backward**; treating it as stale and propagating forward doubles the error.
`ImuSync.quat_offset_us()` returns the signed offset.

**Why it exists (BUG-031).** `t_us` has been the FRAME_READY edge on the MCU's TIM2 clock since
2026-07-28, but orienting a depth frame needs that instant on the *IMU's* clock, and the only LSM
timestamps on the wire were stream 11's FIFO words — samples the LSM took at some point before a
drain the firmware runs 1–2 ms later, after the RAW send. Inferring the frame's position from
them carries that whole variable gap. Measured on this rig over 5331 static frames (176 s): the
residual of `t_us` against a windowed linear fit to the last FIFO word is **1082 µs RMS**, and it
is demonstrably load-dependent — a frame that also carries CALIB drains **673 µs later** than a
plain one (t = −3.9). Stream 13 replaces the inference with a measurement, and reports its own
uncertainty (`read_us`) rather than asking the host to assume one.

Compatibility: additive `stream_id`, no version bump — decoders that predate it skip it, and
captures that predate it simply never carry one (a host must fall back to the stream-11
inference, or to nothing).

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
| 6 | AUTO_WAKE_MOTION   | LSM6DSV16X `WAKE_UP_SRC` register byte (bit layout: `docs/protocol.md` links to the datasheet; `WU_IA` = wake-up detected, `X_WU`/`Y_WU`/`Z_WU` = triggering axis) |
| 7 | TX_QUEUE_STATS     | Ethernet TX-pacer queue telemetry (Task 6, 2026-08-03). Unlike codes 1–6 this event carries a **20-byte payload**, not a bare detail word: `<IIIII>` LE = `code`(7) @0, packed `detail` @4 (`queue_high_water` = bits 0–7, `active_transport` id = bits 8–15 (0 none / 1 cdc / 2 udp — the firmware's coarse hint, NOT the host's authoritative transport truth), `pending_fragments` = bits 16–31), `enqueue_drops` @8, `stack_stalls` @12, `emitted_bytes` @16 (both counters cumulative since boot). Emitted on the 64-frame CALIB cadence by raw-only builds. Host decoder: `roomscan.protocol.parse_tx_queue_stats_event()`; consumed by `UdpSource` (tolerant-skip on any other EVENT). |

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

`AUTO_WAKE_MOTION` (2026-08-03, laser-wear idle wake-on-motion): emitted from the idle-loop
branch (`RS_CMD_SET_STANDBY` soft or hard) when a periodic poll of the LSM6DSV16X's `WAKE_UP_SRC`
register finds `WU_IA` set — the device woke itself, not a host command. `seq` follows the
same "last captured frame's counter" EVENT convention (idled means no new frame exists). The
host does not need to react to this event to resume streaming — the firmware's own wake path
already restarted ranging by the time it sends this — it exists purely so the log/UI can say
*why* the sensor woke rather than reporting an unexplained resume.

## COMMAND frame payload (frame_type = 3)

Host→device commands. Header `seq` = host-chosen token (not a frame counter); `stream_id`, `width`, `height`, `flags` all 0. **v2 introduces a second payload shape** — see below; header `payload_len` tells the decoder which shape follows (this is the layout change v2 exists for: v1's fixed 8-byte payload cannot carry SET_MANUAL_PARAMS's fields).

### Legacy shape — commands 1-8, 10, 11, 12 (`payload_len` = 8)

| Offset | Size | Field  | Notes                  |
|--------|------|--------|------------------------|
| 0      | 4    | cmd    | u32 LE, see command registry |
| 4      | 4    | param  | u32 LE, command-specific (e.g. usecase ID, period in µs, exposure in ms, profile enum, rate_hz); ignored for GET_RANGING_CONFIG (10) and GET_IMU_ENV_RATE (12) |

### Manual shape — command 9 (SET_MANUAL_PARAMS) only (`payload_len` = 12)

| Offset | Size | Field             | Notes                  |
|--------|------|-------------------|------------------------|
| 0      | 4    | cmd               | u32 LE, always `9` (SET_MANUAL_PARAMS) |
| 4      | 1    | ranging_mode      | u8, see ranging-mode registry (0 = AMBIENT, 1 = PRECISION) |
| 5      | 4    | frame_period_us   | u32 LE, `round(1_000_000 / fps)`, integer FPS 1–100 |
| 9      | 2    | exposure_ms       | u16 LE, **integer milliseconds** (Task 1 finding: the current `vl53l9_set_exposure()` driver takes integer ms; a 0.5 ms step is not implemented by the firmware, so this field and the spec both use whole milliseconds — `RS_EXPOSURE_MS_STEP` / `roomscan.protocol.EXPOSURE_MS_STEP`, both `1`) |
| 11     | 1    | power_mode        | u8, see power-mode registry (0 = ULP, 1 = LP, 2 = REGULAR) |

No padding on the wire (offsets are cumulative, not struct-aligned): total 12 bytes.
`roomscan.protocol.pack_manual_command()` / `parse_manual_command()`; firmware
`rs_parse_command()`'s `RS_PARSED_CMD_MANUAL` branch (`firmware/scanner-stream/Src/rs_protocol.c`).

## ACK frame payload (frame_type = 4)

Device→host acknowledgement of a COMMAND. Header `seq` = echoes the COMMAND token (not the device's frame counter); `stream_id`, `width`, `height`, `flags` all 0. **v2 introduces a second payload shape**, selected by which command the ACK answers (not by `result`) — see below.

### Legacy shape — commands 1-8, 11, 12 (`payload_len` = 12)

| Offset | Size | Field   | Notes                  |
|--------|------|---------|------------------------|
| 0      | 4    | cmd     | u32 LE, echoes the command code from the COMMAND |
| 4      | 4    | result  | u32 LE, 0 = OK; nonzero = error (see result-code registry) |
| 8      | 4    | applied | u32 LE, command-specific: applied value, detail, or info. **For 11 (SET_IMU_ENV_RATE) and 12 (GET_IMU_ENV_RATE), `applied` IS the applied `rate_hz`** (0 = coupled) |

### Ranging-config shape — commands 9 (SET_MANUAL_PARAMS), 10 (GET_RANGING_CONFIG) (`payload_len` = 16)

| Offset | Size | Field           | Notes                  |
|--------|------|-----------------|------------------------|
| 0      | 4    | cmd             | u32 LE, echoes the command code from the COMMAND |
| 4      | 4    | result          | u32 LE, 0 = OK; nonzero = error (see result-code registry) |
| 8      | 1    | ranging_mode    | u8, the **applied/readback** ranging mode (see ranging-mode registry) |
| 9      | 4    | frame_period_us | u32 LE, the applied/readback frame period |
| 13     | 2    | exposure_ms     | u16 LE, the applied/readback exposure (integer ms) |
| 15     | 1    | power_mode      | u8, the applied/readback power mode (see power-mode registry) |

The complete applied/readback ranging configuration after `cmd`+`result` — proves what the
device actually applied rather than echoing the request back at itself. **Sent with this
16-byte shape regardless of `result`**: a BUSY/BAD_PARAM/SENSOR_ERROR ACK for cmd 9 or 10
is still 16 bytes (config fields zeroed or holding the prior known-good config, per the
firmware's implementation), never a shorter payload — the ACK's wire shape depends only on
which command it answers, so a host can always parse it the same way before even looking at
`result`. `roomscan.protocol.parse_typed_ack(cmd, payload)` dispatches on `cmd` and returns
a typed `LegacyAck` or `RangingConfigAck` (not a raw tuple); the older `parse_ack(payload)`
remains for the always-12-byte legacy shape (commands 1-8, 11, 12) unchanged.

Both ACK shapes are exact — a payload of any other length for the command in question is
malformed and rejected, unlike EVENT's legitimate variable message tail. Future ACK growth
(a third shape) would come via a new frame revision.

### Command registry

| cmd | Name              | param / payload meaning | applied / ACK meaning |
|-----|-------------------|---------------|-----------------|
| 1   | PING              | ignored       | firmware protocol version (u32) |
| 2   | SEND_CALIB        | ignored       | 0 — device transmits a CALIB frame immediately; lets a late-attaching host obtain calibration immediately instead of waiting the ≤63-frame retransmit cadence (closes ROADMAP's CALIB-on-DTR-connect item when wired in firmware) |
| 3   | SET_USECASE       | usecase ID (u16) | applied usecase ID (u16) |
| 4   | SET_FRAME_PERIOD_US | period in µs (u32) | applied period (u32) — stored and echoed (read back from the sensor), but has no observable effect while the app uses `VL53L9_SYNC_MANUAL` (vl53l9.h:248 — period governs AUTONOMOUS mode only); retained for the future autonomous-mode option |
| 5   | SET_EXPOSURE_MS   | exposure in ms (u32) | applied exposure (u32) |
| 6   | REINIT            | ignored       | 0 — the ACK is sent **after** the re-init completes (normally well under the host's 2 s timeout); if the first re-init attempt itself faults, the device enters its bounded recovery ladder (up to ~3.1 s) and may finish successfully after the host has already timed out — hosts must treat a REINIT timeout as "outcome unknown", not "failed" (a late ACK is silently ignored by token matching) |
| 7   | SET_STANDBY       | standby level (u32): 0 = wake/resume, 1 = soft standby, 2 = hard power-down | standby level now in effect (u32) — echoes param on success. Idles the ToF laser (VCSEL) to reduce wear when no host is viewing. **Soft** (1) = `vl53l9_stop()` → FSM STANDBY: VCSEL stops firing per frame, I3C config/calibration retained, instant resume via wake. **Hard** (2) = additionally `platform_power_disable()` (XSHUT low): sensor fully unpowered; waking re-runs the full re-init cycle (reset → re-address → init → calib → start), so a wake-from-hard ACK is sent **after** that completes (same "outcome unknown on timeout" caveat as REINIT). Applied only at the per-frame safe point (after readout ack, before the next trigger) so `vl53l9_stop()` never races an in-flight trigger. While idled the device streams no DATA frames but keeps servicing the command channel; wake resumes streaming. A wake (0) issued while already active, or a standby issued while already idled at that level, is a harmless no-op ack |
| 8   | SET_RANGING_PROFILE | profile enum (u32), see profile registry | applied profile enum (u32) — legacy 8-byte COMMAND payload, legacy 12-byte ACK. Presets 0-2 (ROOM_MAPPING/PRECISION/HIGH_FRAMERATE) apply immediately; MANUAL (3) reapplies the **last accepted SET_MANUAL_PARAMS candidate** and is rejected (BAD_PARAM) until one exists. **Firmware application is live** (`7598fde`, hardware-gated: readback exact, survives REINIT and both standby levels) |
| 9   | SET_MANUAL_PARAMS | manual shape (12 B, see above) | ranging-config shape (16 B, see above) — the one command that needed v2: v1's fixed 8-byte payload cannot carry `ranging_mode`+`frame_period_us`+`exposure_ms`+`power_mode` together. **Firmware application is live** (`7598fde`) — applied as one atomic candidate at the existing post-readout/pre-next-frame safe point, hardware-gated |
| 10  | GET_RANGING_CONFIG | ignored (legacy 8-byte shape) | ranging-config shape (16 B) — the complete current applied ranging configuration, read back from the device rather than assumed; needed to restore authoritative state after a web-server restart or when a second client attaches. **Live** (`7598fde`) |
| 11  | SET_IMU_ENV_RATE  | rate_hz (u32): `0` = coupled to the ToF trigger (**default**, today's behavior, byte-identical framing), `1`–`480` decouples streams 9/10/11 onto their own service tick | applied rate_hz (u32) — legacy 8-byte COMMAND payload, legacy 12-byte ACK (`applied` = rate_hz). A requested rate above the 60 Hz sensor-hub cycle sub-samples stream 10 (env) specifically; streams 9 (quat)/11 (raw) can still hit the requested rate. Rejects > 480 Hz (`RS_IMU_ENV_RATE_MAX_HZ` / `IMU_ENV_RATE_MAX_HZ`). **Decoupled draining is live** (`3f4b307`, shared `rs_lsm_service_tick()`) — hardware-gated: coupled mode is byte-identical to pre-feature firmware, decoupled 30/90 Hz hold independent of the concurrent ToF rate; see BUG-077 for the FRAME_READY-priority fix this gate found |
| 12  | GET_IMU_ENV_RATE  | ignored (legacy 8-byte shape) | applied rate_hz (u32) and coupled/decoupled state — legacy 12-byte ACK, needed for the same restart/second-client restoration as cmd 10. **Live** (`3f4b307`) |

### Ranging-profile registry (SET_RANGING_PROFILE param / ACK applied)

| id | Name           |
|----|----------------|
| 0  | ROOM_MAPPING   |
| 1  | PRECISION      |
| 2  | HIGH_FRAMERATE |
| 3  | MANUAL         |

### Ranging-mode registry (SET_MANUAL_PARAMS `ranging_mode` / the cmd 9/10 ACK's `ranging_mode`)

| id | Name      | Notes |
|----|-----------|-------|
| 0  | AMBIENT   | DSS-assisted, 450 mm min distance |
| 1  | PRECISION | no DSS, 50 mm min distance |

### Power-mode registry (SET_MANUAL_PARAMS `power_mode` / the cmd 9/10 ACK's `power_mode`)

| id | Name    |
|----|---------|
| 0  | ULP     |
| 1  | LP      |
| 2  | REGULAR |

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
- **v1 rev 2026-07-30**: additive — IMU_SYNC (13), one 16-byte frame per ToF frame carrying that
  frame's FRAME_READY edge on the **LSM's own clock** (BUG-031). It closes the half of the
  2026-07-28 timing work that stream 12 could not: `t_us` became an accurate MCU-clock stamp, but
  the host still had to guess where that instant fell on the IMU clock, from FIFO words drained
  1–2 ms later. The guess costs 1082 µs RMS on a static rig and moves with processing load (a
  CALIB-carrying frame drains 673 µs later, t = −3.9 over 5331 frames). The firmware now reads the
  LSM `TIMESTAMP` register at the edge itself, while the shared I3C bus is still idle, and ships
  the tick with its own delay and read duration so the residual uncertainty is stated, not assumed.
  The same frame also carries `quat_mid_ticks`: stream 9's quaternion is a batch *mean*
  (`RS_LSM_SFLP_AVERAGE`, BUG-027), so it is valid at the batch midpoint — measured **+7.8 ms
  after** the frame-ready edge, i.e. it leads the depth frame rather than lagging it, and by ~9×
  the skew above. That offset was previously invisible: with no measurement of where the edge sat
  on the LSM clock, the batch end was the only available proxy for it, and it is 23.1 ms out.
  New stream_id only; no version bump, no layout change, streams 7/9/10/11/12 byte-for-byte
  unchanged. Older captures never carry one — a host falls back to the stream-11 inference.
- **v1 rev 2026-08-03**: additive — event-code registry gains `AUTO_WAKE_MOTION` (6): the idle
  loop (`RS_CMD_SET_STANDBY` soft/hard) now polls the LSM6DSV16X's embedded Wake-Up function
  (`WAKE_UP_SRC`, independent of the ToF's I3C frame cadence — confirmed on an independent
  hardware block from SFLP, no conflict) and self-initiates a wake on motion instead of only
  ever waking on a host command. New event value only, EVENT payload layout unchanged, no
  version bump.
- **v1 rev 2026-08-03 (b)**: semantics clarification (additive, no wire change) — owner
  correction: `SET_STANDBY` idles the ToF LASER only, not the IMU/env. IMU_QUAT (9), ENV (10),
  and IMU_RAW (11) now keep flowing while the ToF sits parked (soft or hard) — the LSM6DSV16X
  is a separate chip on the shared I3C bus, unaffected by `vl53l9_stop()`/`platform_power_disable()`,
  and was already sampling the whole time; only the MCU's periodic drain-and-send was previously
  gated on a ToF frame existing. Targeted a ~34 ms tick (roughly the active-path rate); **on-rig
  measured at 18.2 Hz (~55 ms/tick)** instead — the per-tick command-poll + I3C drain + 3 sends
  cost more than the naive tick-count estimate assumed. Stable and clean at that rate (jitter
  ~1.4–1.9 ms over a sustained on-rig check), just slower than the design guess; not re-tuned to
  hit exactly 30 Hz since nothing depends on that specific number. `seq` for these frames is
  `g_last_seq` (frozen — no new ToF frame exists, the same convention EVENT frames already use);
  `t_us` is the live clock (no ToF FRAME_READY instant to stamp with), which is what
  `device_hz`/`host_hz` are computed from in the first place, never from `seq`. IMU_SYNC (13) is
  NOT sent during idle — its meaning ("where THIS ToF frame's edge sits on the LSM clock") has no
  referent without a ToF frame. No layout change to any stream, no version bump.
- **v2 rev 2026-08-03 (b)**: additive — EVENT code 7 `TX_QUEUE_STATS` (Ethernet TX-pacer queue
  telemetry, high-framerate plan Task 6). Unlike codes 1–6 it carries a 20-byte payload (layout in
  the EVENT table above); emitted on the 64-frame CALIB cadence by raw-only builds. Hosts that
  treat EVENT payloads as a bare detail word skip it harmlessly (tolerant-skip convention). No
  layout change to any existing frame; no version bump. **Golden vector closed 2026-08-03**:
  `golden_tx_queue_stats.bin` (a full EVENT frame, hand-packed independently of `protocol.py` by
  `host/tests/make_fixtures.py`'s `golden_tx_queue_stats()`), cross-checked in
  `host/tests/test_protocol.py` (byte-for-byte fixture match + decode + a deliberate one-byte
  perturbation proving the check is discriminating). Because this EVENT is firmware-encoded and
  host-decoded only (no Python-side encoder to feed the C parser, unlike the COMMAND cross-checks
  below), `host/tests/test_protocol_c_crosscheck.py` instead cross-checks the ENCODE side directly:
  it calls the real, non-static `rs_write_header()`/`rs_crc32()`/`rs_put_u32()` from
  `rs_protocol.c` via `ctypes` (the exact primitives `rs_send_generic_cdc()` and
  `rs_send_tx_queue_stats_event()` in `vl53l9_app.c` call) to build the same frame and checks it
  byte-for-byte against `golden_tx_queue_stats.bin`. The one thing that check cannot reach is the
  `detail`-word bit-packing formula, which is inline in `vl53l9_app.c` (not HAL-free, not
  host-compilable); that formula was instead verified by direct source reading and matches
  Python's decode bit-for-bit (`detail = high_water | (transport << 8) | (pending << 16)`).
- **v2** (2026-08-03): **layout change — version bump.** Five new command codes (8-12,
  `docs/superpowers/plans/2026-07-31-high-framerate-and-manual-ranging-modes.md` Task 2):
  `SET_RANGING_PROFILE` (8), `SET_MANUAL_PARAMS` (9), `GET_RANGING_CONFIG` (10),
  `SET_IMU_ENV_RATE` (11), `GET_IMU_ENV_RATE` (12). Commands 8, 10, 11, 12 reuse the
  existing 8-byte cmd+param COMMAND shape and 12-byte legacy ACK shape unchanged. Command
  9 (`SET_MANUAL_PARAMS`) needs a 12-byte COMMAND payload (`cmd` + `ranging_mode` u8 +
  `frame_period_us` u32 + `exposure_ms` u16 + `power_mode` u8) that does not fit v1's
  fixed 8-byte payload — this is why the version bumped rather than staying additive.
  Commands 9 and 10 also get a new 16-byte "ranging-config" ACK shape (cmd + result + the
  same four config fields) so the host gets the complete applied/readback configuration
  instead of one scalar. The 32-byte frame header, every DATA/EVENT stream, and the
  legacy 8-byte COMMAND / 12-byte ACK shapes used by commands 1-8/11/12 are all
  byte-for-byte unchanged; `FrameHeader.unpack()` accepts version 1 or 2
  (`roomscan.protocol.SUPPORTED_VERSIONS`), so existing v1 recordings still decode, and
  new encodes use version 2. **This commit is codec/registry only**: the firmware parses
  all five new commands correctly (`rs_parse_command()` returns a bounded
  `rs_parsed_command_t` deriving total frame length from the validated header's
  `payload_len` rather than one fixed 44-byte constant) and ACKs each in its command's
  registered shape, but every one of them currently ACKs `UNKNOWN_CMD` — no ranging
  profile or IMU/env rate is actually applied to the sensor yet. Application logic lands
  in later plan tasks (profile application: Task 4; IMU/env rate decoupling: Task 7).
  Golden vectors: `golden_depth_2x2_v2.bin` (a v2 DATA frame, version byte 2), the
  pre-existing `golden_depth_2x2.bin` is deliberately left frozen at version 1 forever
  (the v1-compat regression vector), plus new `golden_command_manual.bin` and
  `golden_ack_ranging_config.bin`, all hand-packed independently of `protocol.py`
  (`host/tests/make_fixtures.py`). New host-compiled C-parser cross-check
  (`host/tests/test_protocol_c_crosscheck.py`) compiles the real
  `firmware/scanner-stream/Src/rs_protocol.c` with the system C compiler and drives it
  via `ctypes` against Python-generated v2 vectors, covering concatenated 44/48-byte
  commands, a magic split across reads, garbage resync, corrupt CRC, wrong
  version/frame_type, and a reserved (neither-8-nor-12) `payload_len`.
