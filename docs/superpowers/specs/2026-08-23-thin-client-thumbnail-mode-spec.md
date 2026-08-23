# Thin-client `/ws-thin` 160×160 thumbnail mode & dynamic resolution negotiation

**Date:** 2026-08-23  
**Status:** Proposed specification  
**Companion Spec:** `CrowPanelProp/docs/superpowers/specs/2026-08-23-lidar-thumbnail-mode-spec.md`  
**Extends:** `docs/thin-client.md`, `docs/web-protocol.md`, and issue #197 / #202

---

## 1. Motivation & Context

The `lidar-roomscanner` host `/ws-thin` endpoint serves live 3D point-cloud and SLAM visualisations to embedded thin clients like the **CrowPanel ESP32-P4 communicator prop**.

### The Problem
* The CrowPanel prop has two displays for the LiDAR feed:
  1. **Full View (`PK_LIDAR`)**: Dedicated screen displaying a 480×480 frame and sidebar IR preview.
  2. **Dashboard Preview (`PK_MOTION`)**: Multi-sensor scanner page embedding a mini 132×132 voxel preview and a thermal **IR FEED**.
* The CrowPanel connects to Wi-Fi via an ESP32-C6 over 1-bit SDIO, giving a throughput ceiling of **~1.5 Mbit/s (~187 KB/s)**.
* When streaming 480×480 JPEGs at quality 75 (~12 KB/frame), throughput caps at **~16–18 FPS**.
* On the dashboard preview screen, transmitting 480×480 wastes ~85% of radio airtime and client compute, since the client downscales the entire image down to 132×132.
* **IR Grid Telemetry Rate**: `THIN_TELEMETRY_INTERVAL` is currently fixed at 0.5s (2 Hz) in `host/src/roomscan/web.py`, capping the IR thermal feed at 2 FPS regardless of sensor capacity.

### The Solution
1. **160×160 Thumbnail Resolution**: Adding `160` to `THIN_HELLO_RESOLUTIONS` allows the server to render and encode lightweight **~1.5–2.5 KB** frames. At 30–60 FPS, this uses only ~60–120 KB/s, unlocking high-framerate previews while freeing radio bandwidth for other sensors.
2. **Fast IR Telemetry (10–20 Hz)**: Reducing `THIN_TELEMETRY_INTERVAL` (e.g. to 0.1s for 10 Hz, or negotiable via `thin_hello`) transmits the 54×42 IR grid at fluid thermal video rates (~30 KB/s JSON overhead).

---

## 2. Protocol Specification

### A. Resolution Whitelist
In `host/src/roomscan/web.py`, `THIN_HELLO_RESOLUTIONS` is expanded:
```python
THIN_HELLO_RESOLUTIONS = (160, 320, 480)
```

### B. Telemetry Cadence Configuration
`thin_hello` accepts an optional `telemetry_interval_s` (float, clamped to `[0.05, 1.0]`, default 0.1s for v2 flows):
* `0.1` = 10 Hz telemetry & IR grid cadence (~30 KB/s).
* `0.05` = 20 Hz telemetry & IR grid cadence (~60 KB/s).
* `0.5` = 2 Hz legacy default.

### C. Dynamic Negotiation Lifecycle
Clients may send `thin_hello` at connection establishment OR dynamically while connected (e.g. when the user switches screens between the dashboard scanner and full LiDAR view).

```
CrowPanel Client                                 Roomscanner Host
      |                                                 |
      | --- WebSocket Connect (/ws-thin) ------------> |
      | <--- 101 Switching Protocols ------------------ |
      |                                                 |
      | --- {"type":"thin_hello","width":160,          |  (Client on SCANNER page)
      |      "height":160,"fps":30,"quality":75,        |
      |      "telemetry_interval_s":0.1} -------------> |
      | <--- {"type":"thin_hello_ack","width":160,      |
      |       "height":160,"fps":30,"encoding":"jpeg",  |
      |       "telemetry_interval_s":0.1} ------------  |
      | <=== THIN_FRAME_JPEG (w=160, h=160) =========== |  (~1.8 KB / frame @ 30 fps)
      | <... thin_telemetry (ir_grid_b64) @ 10 Hz ..... |  (~3 KB / pkt @ 10 fps)
      |                                                 |
      | [User navigates to full LIDAR page]             |
      | --- {"type":"thin_hello","width":480,          |
      |      "height":480,"fps":20} -------------------> |
      | <--- {"type":"thin_hello_ack","width":480,      |
      |       "height":480,"fps":20} ------------------ |
      | <=== THIN_FRAME_JPEG (w=480, h=480) =========== |  (~12 KB / frame @ 20 fps)
```

### D. Message Contracts

#### Inbound: `thin_hello`
```json
{
  "type": "thin_hello",
  "proto": 2,
  "width": 160,
  "height": 160,
  "fps": 30,
  "quality": 75,
  "credits": 6,
  "telemetry_interval_s": 0.1
}
```
* `width` / `height`: Nearest match in `THIN_HELLO_RESOLUTIONS = (160, 320, 480)`. Non-square pairs are rejected (keeps prior resolution).
* `fps`: Clamped to `[1.0, 60.0]` for JPEG format.
* `telemetry_interval_s`: Clamped to `[0.05, 1.0]`.

#### Outbound: `thin_hello_ack`
```json
{
  "type": "thin_hello_ack",
  "proto": 2,
  "encoding": "jpeg",
  "format": "jpeg",
  "fps": 30.0,
  "width": 160,
  "height": 160,
  "quality": 75,
  "credits": 6,
  "ir_cells": 4096,
  "telemetry_interval_s": 0.1
}
```

#### Binary Wire Frame (`THIN_FRAME_JPEG`, Tag 2)
The 16-byte header dynamically reflects the negotiated dimensions:
```
Offset  Size  Field        Value
0       4     tag          2 (THIN_FRAME_JPEG)
4       2     width        160 (or 320 / 480)
6       2     height       160 (or 320 / 480)
8       4     seq          Monotonic frame counter
12      4     payload_len  JPEG byte count (e.g. 1842)
16      N     jpeg_bytes   Baseline 4:2:0 JFIF image
```

---

## 3. Host Implementation Plan

### `host/src/roomscan/web.py`
1. Update `THIN_HELLO_RESOLUTIONS = (160, 320, 480)`.
2. Add `telemetry_interval_s` negotiation to `ThinFlow` and `negotiate_thin_hello()`.
3. In `thin_render.py` / `web.py`, ensure dynamic resolution updates resize the internal offscreen render target or downscale buffer cleanly between frames without state corruption.
4. Verify `simplejpeg.encode_jpeg` encodes 160×160 buffers correctly with fast execution (<0.5 ms).

### `host/tests/test_web.py` & Tools
1. Add unit tests for 160×160 resolution clamping and `telemetry_interval_s` negotiation:
   ```python
   def test_negotiate_thin_hello_clamps_to_160_resolution():
       flow = ThinFlow()
       effective = negotiate_thin_hello({"type": "thin_hello", "width": 160, "height": 160, "telemetry_interval_s": 0.1}, flow)
       assert effective["width"] == 160
       assert effective["height"] == 160
       assert effective["telemetry_interval_s"] == 0.1
   ```
2. Update `host/tools/thin_client_probe.py` and `rig_thin_probe` MCP tool to accept `--width 160 --height 160`.

---

## 4. Verification & Validation

1. **Unit Tests**:
   * Run `pytest host/tests/test_web.py` to confirm clamping, ack formatting, and flow-control invariants.
2. **End-to-End Probe**:
   * Run `python host/tools/thin_client_probe.py --width 160 --height 160 --fps 30 --v2` and verify steady 30 FPS stream with <3 KB/frame payload and 10 Hz IR grid telemetry.
