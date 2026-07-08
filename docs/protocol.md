# roomscanner wire protocol — v1

Transport-agnostic binary framing for sensor→host streams. Little-endian throughout.
One frame = 32-byte header, payload, CRC32. See the `protocol-change` skill before editing.

## Frame layout

| Offset | Size | Field         | Notes                                                        |
|--------|------|---------------|--------------------------------------------------------------|
| 0      | 4    | `magic`       | ASCII `RSCN` (bytes `52 53 43 4E`)                           |
| 4      | 1    | `version`     | `1`                                                          |
| 5      | 1    | `frame_type`  | `1` = DATA, `2` = EVENT (device error/log)                   |
| 6      | 1    | `stream_id`   | `0` = DEPTH_ZF32 (float32 perpendicular depth, millimetres)  |
| 7      | 1    | `flags`       | bit0 = DROPPED: ≥1 frame was skipped since the last one sent |
| 8      | 4    | `seq`         | u32; sensor `frame_counter`, increments per *captured* frame |
| 12     | 8    | `t_us`        | u64 µs since boot (v1 source: `HAL_GetTick()*1000`, 1 ms resolution) |
| 20     | 2    | `width`       | zones                                                        |
| 22     | 2    | `height`      | zones                                                        |
| 24     | 4    | `payload_len` | bytes; DEPTH_ZF32 ⇒ `width*height*4`                         |
| 28     | 4    | `reserved`    | 0                                                            |
| 32     | N    | payload       | row-major, stream-defined encoding                           |
| 32+N   | 4    | `crc32`       | IEEE 802.3 / zlib `crc32` over bytes `[0, 32+N)`             |

## Decoder requirements

- Resync by scanning for `magic`; tolerate arbitrary garbage (e.g. ASCII boot text) between frames.
- Bound `payload_len` (reject > 1 MiB) before buffering; treat reject like a CRC failure.
- On CRC failure: advance one byte past the magic candidate and rescan; count failures, never raise.
- Skip unknown `stream_id`/`frame_type` values silently (forward compatibility, no version bump needed).

## USB identification

- Milestone 1a: ST-Link VCOM (VID `0x0483`), 921600 8N1.
- Milestone 1b: native CDC ACM, VID `0xCAFE` PID `0x4001` (TinyUSB descriptors).

## Version history

- **v1** (2026-07): initial — DATA/EVENT frame types, DEPTH_ZF32 stream.
