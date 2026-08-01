# High Frame-Rate Ranging Profiles & Manual Sensor Control

## Status

Specification for implementation. Builds upon Phase 3 runtime configuration (`docs/protocol.md`), Phase 4 sensor suite (`docs/iks4a1-stacking.md`), and Phase 5 Ethernet transport (`CLAUDE.md`).

---

## 1. Overview & Objectives

This feature expands the scanner's operating profiles from the existing two presets (`AR_RANGE` / Room Mapping @ 30 FPS and `AR_PRECISION` / Precision @ 30 FPS) to a 4-mode architecture:

1. **Room Mapping (Default / Preset 1):** `AR_RANGE` — Ambient mode with DSS, 30 FPS, 6 ms exposure, ULP power mode (8m max range, 200 mW, 28.5% I3C bus duty cycle).
2. **Precision Ranging (Preset 2):** `AR_PRECISION` — Precision mode, 30 FPS, 10 ms exposure, ULP power mode (8.8m max range, 5cm min distance, 220 mW, 28.5% I3C bus duty cycle).
3. **High Frame-Rate / Gaming (Preset 3):** `HIGH_FRAMERATE` — Precision mode (no DSS), 90 FPS, 4 ms exposure, Regular power mode (5m max range, 420 mW, 85.5% I3C bus duty cycle). Optimized for low-latency SLAM tracking.
4. **Manual / Custom Mode (Mode 4):** Live interactive controls letting operators set ranging mode, frame rate (1–100 FPS), exposure time (1–16 ms), and power mode, while displaying live computed consequences (Max Range, Power Consumption, and I3C Bus Utilization).

Additionally, a visual **I3C Bus Bandwidth Bar** is integrated directly into the Web UI control card beneath the mode selector, rendering real-time bus duty cycle and warning when airtime approaches bus saturation or USB transport ceilings.

---

## 2. Technical Specification & Profile Definitions

### 2.1 Profile Table

| Mode Name | Key / Identifier | Ranging Mode | DSS | FPS (Target) | Exposure | Power Mode | Max Range (Est) | Min Distance | Typical Power | I3C Bus Utilization |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| **Room Mapping** | `room_mapping` | Ambient | Yes | 30 FPS | 6 ms | ULP | 8.0 m | 450 mm | 200 mW | 28.5% |
| **Precision** | `precision` | Precision | Yes | 30 FPS | 10 ms | ULP | 8.8 m | 50 mm | 220 mW | 28.5% |
| **High Frame-Rate** | `high_framerate` | Precision | No | 90 FPS | 4 ms | Regular | 5.0 m | 50 mm | 420 mW | 85.5% |
| **Manual** | `manual` | *Custom* | *Custom* | 1–100 FPS | 1–16 ms | *Custom* | *Dynamic* | 50 mm / 450 mm | *Dynamic* | *Dynamic* |

---

## 3. Mathematical Models for Real-Time Consequence Estimation

In **Manual Mode**, changes to FPS, exposure, ranging mode, and power mode trigger real-time client/server estimation models:

### 3.1 I3C Bus Airtime Model
The I3C bus operates at 12.5 MHz Push-Pull (SDR). For a raw 3DMD frame (binning = 2, \(14,842\text{ bytes} = 118,736\text{ bits}\)):

$$\text{Frame Transfer Time } (T_{\text{xfer}}) = \frac{118,736\text{ bits}}{12.5 \times 10^6\text{ bits/s}} = 9.49888\text{ ms}$$

$$\text{Bus Utilization (\%)} = \frac{T_{\text{xfer}}}{\text{Frame Period (ms)}} \times 100\% = 9.49888\text{ ms} \times \text{FPS} \times 100\%$$

* **At 30 FPS:** \(9.5\text{ ms} / 33.3\text{ ms} = 28.5\%\) duty cycle (\(71.5\%\) idle airtime for IMU).
* **At 60 FPS:** \(9.5\text{ ms} / 16.7\text{ ms} = 57.0\%\) duty cycle (\(43.0\%\) idle airtime for IMU).
* **At 90 FPS:** \(9.5\text{ ms} / 11.1\text{ ms} = 85.5\%\) duty cycle (\(14.5\%\) idle airtime for IMU).
* **At 100 FPS:** \(9.5\text{ ms} / 10.0\text{ ms} = 95.0\%\) duty cycle (\(5.0\%\) idle airtime for IMU).

### 3.2 Power Consumption Model
Estimated based on datasheet (DS14879 Table 9 & 36) interpolation:

$$P_{\text{total}} = P_{\text{baseline}} + P_{\text{laser}}(\text{Exposure}) \times \text{FPS} + P_{\text{mode}}$$

* **ULP Mode:** Baseline \(50\text{ mW}\) + \(5.0\text{ mW/ms/FPS}\).
* **LP Mode:** Baseline \(100\text{ mW}\) + \(5.0\text{ mW/ms/FPS}\).
* **Regular Mode:** Baseline \(240\text{ mW}\) + \(5.0\text{ mW/ms/FPS}\).

### 3.3 Max Range Model
* **Ambient Mode (with DSS):** \(R_{\text{max}} = 8.5\text{m} \times \sqrt{\text{Exposure} / 6.0\text{ms}}\) (clamped to 8.8 m max). Minimum distance = **450 mm**.
* **Precision Mode (no DSS):** \(R_{\text{max}} = 5.0\text{m} \times \sqrt{\text{Exposure} / 4.0\text{ms}}\) (clamped to 8.8 m max). Minimum distance = **50 mm**.

---

## 4. Hardware, Transport, and SLAM Safety Guards

1. **Transport Throughput Ceiling Guard (USB vs Ethernet):**
   * Raw payload bandwidth at \(\text{FPS}\) is \(14,842 \times \text{FPS}\) bytes/sec.
   * At \(\text{FPS} > 60\), bandwidth exceeds USB CDC Full-Speed's \(\sim 1.0\text{ MB/s}\) throughput cap.
   * The UI detects active transport (`CDC` vs `UDP`). If `CDC` is active and FPS is set \(> 60\), the control card displays a prominent warning: *"High frame rate (> 60 FPS) requires Ethernet UDP transport to prevent frame drops."*

2. **Host SLAM Parameter Auto-Scaling:**
   * **Barometer Drift Window (`baro_tau_frames`):** Scaled automatically with target frame rate:
     $$\text{baro\_tau\_frames} = \text{round}(30.0 \times \text{target\_fps})$$
     (e.g., 900 frames @ 30 FPS, 2700 frames @ 90 FPS).
   * **Host IMU Crossover Rate (`QUAT_REF_RATE_HZ`):** Dynamically set to match active ToF frame rate in `ImuFusion`.

3. **Web UI Pacing (`POINT_INTERVAL`):**
   * Broadcaster WebSocket pacing in `web.py` auto-adapts to incoming frame rate or decouples UI viewport rendering from frame ingest, ensuring 90 FPS stream processing while maintaining smooth browser rendering.

---

## 5. Control Protocol & Software Architecture

### 5.1 Binary & WebSocket Protocol Updates
Extends existing `COMMAND_CODE` set in `docs/protocol.md` and `/ws` JSON control channel:

* **Command Code 8: `SET_RANGING_PROFILE`** — Enum: `0=ROOM_MAPPING`, `1=PRECISION`, `2=HIGH_FRAMERATE`, `3=MANUAL`.
* **Command Code 9: `SET_MANUAL_PARAMS`** — Parameters: `ranging_mode (u8)`, `frame_period_us (u32)`, `exposure_ms (u16)`, `power_mode (u8)`.

WebSocket `/ws` JSON messages:
```json
{
  "type": "set_profile",
  "profile": "high_framerate",
  "manual_params": {
    "ranging_mode": "precision",
    "fps": 90,
    "exposure_ms": 4,
    "power_mode": "regular"
  }
}
```

### 5.2 Web UI Layout & Bus Bandwidth Bar

1. **Mode Selector:**
   * Segmented control with 4 options: `[ Room Mapping | Precision | High FPS | Manual ]`.
2. **Manual Adjustment Panel (visible when `Manual` is selected):**
   * **Ranging Mode Dropdown:** `Precision (min 5cm)` / `Ambient (min 45cm)`.
   * **FPS Slider / Input:** Range 1 to 100 FPS (steps of 1 FPS).
   * **Exposure Slider / Input:** Range 1 to 16 ms (steps of 0.5 ms).
   * **Power Mode Dropdown:** `Ultra Low Power (ULP)` / `Low Power (LP)` / `Regular`.
3. **Consequences Metrics Display:**
   * Stat readouts: **Est. Max Range** (e.g., `5.0 m`), **Est. Power** (e.g., `420 mW`), **Min Distance** (e.g., `50 mm`).
4. **I3C Bus Bandwidth Bar (Located directly beneath mode selector):**
   * Visual progress bar displaying total bus duty cycle (0% to 100%).
   * **Color Coding:**
     * `Green` (0% – 70%): Plenty of airtime reserved for IMU/sensors.
     * `Yellow` (70% – 85%): High utilization, IMU airtime constrained.
     * `Red` (> 85%): Near saturation / bus capacity limit.
   * Sub-caption label: `I3C Bus: 85.5% used (9.5ms ToF / 11.1ms frame) • 14.5% airtime left for IMU`.

---

## 6. Implementation Plan & Milestones

1. **Firmware (`firmware/scanner-stream`):**
   * Add `HIGH_FRAMERATE` profile definition and manual override setters for `frame_period_us`, `exposure_ms`, `ranging_mode`, and `power_mode` in `vl53l9_app.c`.
   * Plumb Command Codes 8 & 9 into the CDC/UDP command parser.
2. **Host Library (`roomscan`):**
   * Update `imufusion.py` and `slam/mapper.py` to auto-scale time constants (`QUAT_REF_RATE_HZ` and `baro_tau_frames`) with measured frame rate.
   * Add consequence estimation helper functions (`roomscan.profile_models`).
3. **Web Server & UI (`roomscan-web`):**
   * Plumb WebSocket commands for `set_profile` and `set_manual_params`.
   * Implement 4-way Mode Selector card, Manual Parameter controls, and real-time I3C Bus Bandwidth Bar in `static/components/controls.js` and CSS.
4. **Validation & Verification:**
   * Hardware test over Ethernet UDP @ 90 FPS (verify 0 frame drops, 0 CRC errors, clean IMU sync).
   * Verify transport ceiling warning when switching to >60 FPS over CDC.

---

## 7. Notes & Key Discoveries for Plan Writer

### 7.1 Hardware & Driver Constraints (ST VL53L9CX Datasheet DS14879)
* **Driver API Flexibility:** `vl53l9_set_frame_period()` accepts any frame period from \(10,000\ \mu\text{s}\) (100 FPS) to \(1,000,000\ \mu\text{s}\) (1 FPS). You do NOT have to choose only discrete values (30 or 100); 60 FPS or 90 FPS are fully valid continuous settings.
* **Precision vs. Ambient Minimum Distance:** 
  * **Precision Mode** allows ranging down to **50 mm (5 cm)**.
  * **Ambient Mode** enforces a minimum distance floor of **450 mm (45 cm)**.
  * *For room scanning, Precision mode's 5 cm floor is essential for scanning near objects.*
* **DSS Cutoff Frame Rate (\(\mathbf{60\text{ FPS}}\)):**
  * **Dynamic SPAD Selection (DSS)** is supported only up to **60 FPS** (frame periods \(\ge 16.7\text{ ms}\)).
  * At frame rates above 60 FPS (61–100 FPS), the on-chip sensor MCU does not have sufficient inter-frame time to run SPAD selection algorithms. The sensor **must** run in **Precision Mode with DSS disabled** (fixed SPAD array map).
* **Onboard Transform CPU Bottleneck:** If `CONF_TRANSFORM_ONBOARD == 1` is enabled, the Cortex-M33 CPU math takes \(\sim 37\text{ ms/frame}\) (\(\sim 27\text{ FPS}\) max). High frame rates (>30 FPS) **MUST** run in **Raw Streaming Mode** (`CONF_TRANSFORM_ONBOARD == 0`), which offloads math to the host.

### 7.2 Bus Bandwidth & IMU Dynamics
* **I3C 12.5 MHz Bus Utilization:** Transferring a raw 3DMD frame (14,842 bytes) takes \(\approx 9.5\text{ ms}\).
  * 30 FPS = 28.5% bus utilization (71.5% airtime for IMU).
  * 60 FPS = 57.0% bus utilization (43.0% airtime for IMU).
  * 90 FPS = 85.5% bus utilization (14.5% airtime for IMU).
  * 100 FPS = 95.0% bus utilization (5.0% airtime for IMU — near total saturation).
* **IMU Sampling Rate (LSM6DSV16X):** The IMU samples continuously at **480 Hz** into an onboard hardware FIFO. Do **NOT** attempt to change the IMU ODR when increasing LiDAR FPS. At 90 FPS ToF, the MCU simply drains smaller FIFO batches (\(\approx 5\text{--}6\) samples per readout instead of \(\approx 16\)), which actually **cuts readout latency 3×**.

### 7.3 Transport Ceilings (USB CDC FS vs. Ethernet UDP)
* **USB CDC Full-Speed Ceiling:** Payload rate at 90 FPS is \(\sim 1.34\text{ MB/s}\) (\(10.7\text{ Mbps}\)). USB CDC Full-Speed PHY tops out at \(\sim 1.0\text{ MB/s}\) (\(\approx 67\text{ FPS}\) max). Running 90 FPS over USB CDC will choke TinyUSB endpoints and drop frames heavily.
* **Ethernet UDP Requirement:** Ethernet 100 Mbps handles \(1.34\text{ MB/s}\) easily (\(\sim 10.7\%\) of link bandwidth). High FPS testing **must be conducted over Ethernet UDP**.

### 7.4 Host Processing & SLAM Performance
* **GPU Compute Margin:** Live CUDA SLAM (`SlamRunner` / Open3D VoxelBlockGrid ICP) processes frames in **\(\sim 7\text{ ms}\)** on an RTX 2000 Ada GPU, which easily fits within 90 FPS's \(11.1\text{ ms}\) budget.
* **Barometer Window Scaling:** `Mapper.baro_tau_frames` defaults to 900 frames (\(30\text{ seconds}\) @ 30 FPS). The plan writer must ensure `baro_tau_frames` is dynamically scaled (`round(30.0 * target_fps)`) to prevent barometer drift correction from becoming 3× over-aggressive at 90 FPS.
* **Complementary Filter Retuning:** `imufusion.py`'s `QUAT_REF_RATE_HZ` constant should be updated dynamically to the active ToF frame rate to preserve filter phase alignment.

### 7.5 Repository Protocol & Development Conventions
* **Vendor Directory is Read-Only:** All new firmware changes go in `firmware/scanner-stream/`, NOT `firmware/vendor/53L9A1/`.
* **Wire Protocol Changes:** Modifying command codes requires following the `protocol-change` skill checklist (`docs/protocol.md`, firmware C, host Python, golden vectors).
* **MCP Integration:** Any host-side consequence helper functions added to `host/src/roomscan` must be registered/exposed in `host/src/roomscan/mcp_server/`.
* **Shipping Checklist:** Mandate running the `status-sync` skill before landing the completed implementation.
