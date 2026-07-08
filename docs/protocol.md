# roomscanner wire protocol — v1

Transport-agnostic binary framing for sensor→host streams. Little-endian throughout.
One frame = 32-byte header, payload, CRC32. See the `protocol-change` skill before editing.

## Frame layout

| Offset | Size | Field         | Notes                                                        |
|--------|------|---------------|--------------------------------------------------------------|
| 0      | 4    | `magic`       | ASCII `RSCN` (bytes `52 53 43 4E`)                           |
| 4      | 1    | `version`     | `1`                                                          |
| 5      | 1    | `frame_type`  | `1` = DATA, `2` = EVENT (device error/log)                   |
| 6      | 1    | `stream_id`   | see Stream registry below                                    |
| 7      | 1    | `flags`       | bit0 = DROPPED: ≥1 frame was skipped since the last one sent |
| 8      | 4    | `seq`         | u32; sensor `frame_counter`, increments per *captured* frame |
| 12     | 8    | `t_us`        | u64 µs since boot (v1 source: `HAL_GetTick()*1000`, 1 ms resolution; a TIM-backed µs clock is planned with Phase 5 IMU fusion) |
| 20     | 2    | `width`       | zones                                                        |
| 22     | 2    | `height`      | zones                                                        |
| 24     | 4    | `payload_len` | bytes; DEPTH_ZF32 ⇒ `width*height*4`                         |
| 28     | 4    | `reserved`    | 0                                                            |
| 32     | N    | payload       | row-major, stream-defined encoding                           |
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
| 1 | SENSOR_INIT_FAIL   | vl53l9 status word                  |
| 2 | TRIGGER_TIMEOUT    | retry count at exhaustion           |
| 3 | DMA_TIMEOUT        | retry count at exhaustion           |
| 4 | SENSOR_ERROR_STATUS| vl53l9 status word from handle path |
| 5 | TX_OVERFLOW        | frames dropped since last report    |

Rationale: firmware today spins in `handle_error()`; emitting an EVENT first is the planned
recovery-path upgrade (ROADMAP "reference-firmware bugs" #2 + Task 8's 1-in-5 boot-failure
follow-up). This task defines the wire contract only; firmware emission is wired when the
recovery work lands.

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
- Milestone 1b: native CDC ACM, VID `0xCAFE` PID `0x4001` (TinyUSB descriptors).

## Version history

- **v1** (2026-07): initial — DATA/EVENT frame types, DEPTH_ZF32 stream.
- **v1 rev 2026-07-08**: additive — stream registry (IDs 1-6 reserved), EVENT payload defined,
  DROPPED/oversize semantics clarified, ZF32 no-return sentinel documented. No layout change.
