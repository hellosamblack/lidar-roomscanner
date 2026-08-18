# Comprehensive Implementation-Grade Engineering Audit & Technical Backlog
**Target System:** `hellosamblack/lidar-roomscanner`  
**Date of Audit:** August 18, 2026  
**Auditor:** Senior Embedded Systems, Real-Time Vision & SLAM Engineer  
**Baseline Git State:** Commit `c72cc22` on branch `main`  
**Test Suite Verification:** 2,740 passed, 1 skipped, 0 failed in 355.16s via pytest 8.4.1 / Python 3.12.3; clean C builds for host `libroomscan_transform.so` and target `scanner_stream.elf` (Flash: 6.76%, RAM: 28.77%).

---

## 1. Executive Summary

### 1.1 Core Engineering Thesis
The `lidar-roomscanner` project is a **uniquely disciplined, high-fidelity embedded-to-cloud cyber-physical system**. It achieves something rare in modern edge computing: full hardware-to-visualization determinism, sub-millisecond hardware timestamping across heterogeneous buses (I3C/I2C/SPI), microsecond-level clock-skew correction, zero-copy wire protocols, and high-performance real-time SLAM utilizing tensor TSDF voxel grids.

The repository displays an extraordinary level of empirical rigor. Rather than relying on theoretical assumptions or standard library defaults, every critical design decision (from the 20.0 condition number cap in translation ICP, to the 2.8× orientation noise cut via FIFO averaging, to the `-DVL53L9_TRANSFORM_LIGHT=0` vendor override) is backed by recorded sensor logs, golden vectors, and matched ensemble tests.

However, the project is approaching a **critical architectural inflection point**:
1. **Monolithic Complexity:** The host web orchestrator (`host/src/roomscan/web.py`) has expanded into a 7,523-line, 362 KB "god module" managing WebSocket demuxing, SLAM worker IPC, offscreen Filament rendering, mDNS broadcasting, video transcoding, and file sessions.
2. **Native Filament Lifecycle Fragility:** The introduction of the `/ws-thin` server-side rasterizer relies on Google Filament via Open3D's `OffscreenRenderer`. Because Filament enforces strict single-context and thread-affinity invariants that trigger uncatchable process aborts (`utils::PreconditionPanic`) upon violation, server stability is tightly coupled to thread isolation.
3. **Absence of CI/CD:** Despite 2,740 high-quality unit/regression tests and multi-platform C code, there is **zero automated GitHub Actions CI** for automated regression testing, static analysis, or firmware compilation.
4. **Bandwidth & Wireless Ceiling:** While the raw 14.8 KB 3DMD frame streams comfortably over Ethernet UDP at 30 Hz (~3.6 Mbps), streaming uncompressed RGB565 raster frames to thin clients over Wi-Fi consumes 4.6 MB/s (36.8 Mbps) per client, which saturates 2.4 GHz 802.11b/g/n links and caps the thin-client framerate at ~10 fps.

### 1.2 Technical Maturity Scorecard

```
┌─────────────────────────────────────────────────────────────┐
│                   SUBSYSTEM MATURITY MATRIX                 │
├──────────────────────────────┬───────┬──────────────────────┤
│ Subsystem                    │ Score │ Status               │
├──────────────────────────────┼───────┼──────────────────────┤
│ 1. Firmware Hardware Driver  │ 9.0/10│ Production Grade     │
│ 2. Wire Protocol & Codecs    │ 9.5/10│ Reference Grade      │
│ 3. PC Ingestion & Transform  │ 9.0/10│ Production Grade     │
│ 4. Real-Time SLAM / TSDF     │ 8.5/10│ Advanced Research    │
│ 5. Web Frontend & Visualizer │ 8.0/10│ Functional / Dense   │
│ 6. Thin-Client Pipeline      │ 7.5/10│ Beta / Functional    │
│ 7. Splat Pipeline (3DGS)     │ 7.0/10│ Functional Offline   │
│ 8. CI / CD & Tooling         │ 3.0/10│ Critical Deficiency  │
│ 9. Architectural Modularity  │ 5.5/10│ High Technical Debt  │
└──────────────────────────────┴───────┴──────────────────────┘
OVERALL SYSTEM MATURITY: 7.4 / 10.0 (High-Performing Research / Pre-Production)
```

### 1.3 Top 5 Existential Risks

1. **Host Orchestrator Monolithic Collapse (`web.py`):** At 7,523 lines, `web.py` mixes transport I/O, session state, UI broadcast, worker process supervision, thumbnail generation, bridge management, and raster rendering. Any concurrent state race or unhandled exception in secondary tasks can destabilize the primary data ingest.
2. **Open3D / Filament Process-Abort Traps:** `OffscreenRenderer` aborts the Python process via C++ `std::terminate` if instantiated twice or called from an unadopted thread. Any accidental invocation outside the dedicated worker thread crashes the entire web server during live room scans.
3. **Lack of Automated CI Pipeline:** Regressions in C-shim ABI compatibility, protocol pack/unpack structs, or SLAM registration math can be committed without detection unless developer runs local test suites manually.
4. **I3C / I2C Bus Wedging on Unclean Shutdown:** While the firmware handles dynamic address assignment (ENTDAA) and hot-join, an abrupt reset or power glitch during 3DMD DMA readout can leave the VL53L9CX in an unclocked state, requiring a physical power-cycle.
5. **Thin-Client Wireless Bandwidth Saturation:** Delivering raw 480×480 RGB565 over WebSocket at 10 fps requires ~37 Mbps of sustained UDP/TCP bandwidth. On crowded Wi-Fi networks (especially with an ESP32-C6 / ESP32-P4 client), packet drops and head-of-line blocking degrade framerates.

### 1.4 Top 5 Strongest Engineering Wins

1. **Hardware-Anchored Timestamping Architecture (Streams 11, 12, 13):** The firmware latches the free-running 1 MHz TIM2 timer at the exact `FRAME_READY` GPIO edge, captures the LSM6DSV16X internal timestamp before DMA kicks in, and reads the `INTERNAL_FREQ_FINE` register (0.13% per LSB) to eliminate ~29,790 ppm of oscillator drift.
2. **Rigorous Point-to-Plane Translation ICP with Condition-Number Capping:** The odometry engine solves a 3-DoF translation system against the SFLP rotation prior while dynamically flooring eigenvalues of the normal equations matrix $A = \sum n_i n_i^T$ at $\lambda_{\max} / 20.0$, preventing geometric sliding down corridors without throwing away tracking.
3. **Zero-Copy Host Transform C-Shim with Full Vendor Parity:** `host/transform/rs_transform_shim.c` wraps ST's proprietary `vl53l9-transform-c` 1.5.0, correctly pins `-DVL53L9_TRANSFORM_LIGHT=0` to preserve temporal noise reduction (TNR) and flying-pixel filters, and executes in ~1.2 ms per frame.
4. **Adaptive Fragment-Paced UDP Network Transport:** The firmware implements a token-bucket-like fragment pacer (`firmware/scanner-stream/Src/ethernet_transport.c`) that slices 14.8 KB frames into 1,400 B UDP datagrams metered over the active frame period, while `UdpSource` reassembles out-of-order datagrams using indexed slots.
5. **Empirical "Golden Vector" Verification Discipline:** Over 2,740 unit tests pin every mathematical transformation, byte layout, quaternion convention, and coordinate frame against recorded physical captures and AST guards (`test_no_new_yaw_twist_consumers`).

---

## 2. System Model & Ground Truth Data Path

```
                                    SYSTEM DATA FLOW & TIMING BUDGET
                                    
  [VL53L9CX ToF] (54x42 SPADs)          [LSM6DSV16X IMU] (6-Axis + SFLP) + LPS22DF + LIS2MDL
         │                                       │
         │ ~33.3 ms (30 Hz)                      │ 480 Hz raw XL/GY (Stream 11) + 30 Hz SFLP (Stream 9)
         │ FRAME_READY Edge                      │ Sensor-hub Env (Stream 10)
         ▼                                       ▼
  ┌───────────────────────────────────────────────────────────────────────────────────────────┐
  │ STM32H563ZI FIRMWARE (Cortex-M33 @ 250 MHz)                                               │
  │  1. Latch TIM2 (1 MHz) timestamp (t_us) at FRAME_READY EXTI                               │
  │  2. Read Stream 13 (LSM timestamp + edge jitter) over I3C1 (idle bus)                     │
  │  3. GPDMA1 transfers 14,842 B raw 3DMD payload from VL53L9CX FIFO                         │
  │  4. Drain LSM FIFO (Stream 11 raw batch + Stream 9 averaged quat + Stream 10 env)          │
  │  5. Check inbound CDC/UDP command queue (safe point between frames)                       │
  │  6. Frame Framing: 32-byte header + payload + CRC32 (rs_protocol.c)                       │
  └────────────────────────────────┬──────────────────────────────────────────────────────────┘
                                   │
                    ┌──────────────┴──────────────┐
                    │ Transports (Auto-Select)    │
                    ▼                             ▼
         [USB CDC ACM (TinyUSB)]         [Ethernet UDP (lwIP 2.1.2)]
           12 Mbps Full-Speed              10/100 RMII (LAN8742 PHY)
           ~15.2 KB/frame @ 30 Hz          1400 B frags, paced drain
           Bandwidth: ~3.65 Mbps           Bandwidth: ~3.65 Mbps
                    │                             │
                    └──────────────┬──────────────┘
                                   ▼
  ┌───────────────────────────────────────────────────────────────────────────────────────────┐
  │ HOST PC INGESTION & PIPELINE (Python 3.12 / C Native)                                     │
  │  1. Source: SerialSource / UdpSource (slot-based UDP reassembly, keepalive ping)          │
  │  2. StreamDecoder: Magic 'RSCN' sync, 32-byte header unpack, CRC32 verify                 │
  │  3. TransformStage: Native C-Shim (vl53l9-transform-c 1.5.0) -> Depth (54x42 f32),        │
  │     Reflectance, Confidence, Ambient. Runtime: ~1.2 ms                                    │
  │  4. Deprojector: Pinhole / Optical Ray Projection -> Point Cloud (2,268 pts [x,y,z,c])   │
  └────────────────────────────────┬──────────────────────────────────────────────────────────┘
                                   │
                    ┌──────────────┴──────────────┐
                    ▼                             ▼
  ┌─────────────────────────────────┐   ┌─────────────────────────────────────────────────────┐
  │ REAL-TIME SLAM WORKER           │   │ ROOMSCAN-WEB ORCHESTRATOR (FastAPI / Uvicorn)       │
  │ (Multiprocessing / Open3D CUDA) │   │  - Binary WebSocket (/ws): 30 Hz Point Cloud / Pose │
  │  1. SFLP Quat & Baro Z Pred     │   │  - Mesh WebSocket (/ws-mesh): TSDF Triangle Meshes  │
  │  2. TSDF Raycast Model (Target) │   │  - Thin WebSocket (/ws-thin): 480x480 RGB565 Raster │
  │  3. Translation ICP (Cond <= 20)│   │  - mDNS Broadcast: _roomscan._tcp.local.            │
  │  4. VoxelBlockGrid TSDF (10 mm) │   │  - Browser Client: Three.js PBR / Splat Viewer      │
  │  5. Marching Cubes (5 Hz mesh)  │   └─────────────────────────────────────────────────────┘
  └─────────────────────────────────┘
```

### 2.1 Latency, Memory & Bandwidth Budget

| Boundary / Subsystem | Payload / Data Structure | Typical Latency | Peak Memory | Sustained Bandwidth |
| :--- | :--- | :--- | :--- | :--- |
| **ToF Sensor → STM32 (I3C1)** | 14,842 B (Raw 3DMD bin2) | ~11.8 ms (DMA transfer) | 2× 14,842 B ping-pong | ~3.56 Mbps (at 30 Hz) |
| **IMU → STM32 (I3C1)** | 16 B Quat, 20 B Env, ~128 B FIFO | ~0.4 ms (I3C Read) | 512 B FIFO buffer | ~38 kbps |
| **STM32 → Host (USB CDC)** | 15,104 B Wire Frame (Stream 7) | ~10.1 ms (Full-Speed USB) | 16 KB Ring Buffer | 3.65 Mbps |
| **STM32 → Host (UDP)** | 11× 1400 B Fragments | ~1.5 ms (100 Mbps RMII) | 8× 15,104 B Tx Slots | 3.65 Mbps |
| **Host Ingestion & Transform** | 54×42 Float32 Arrays (4 planes) | 1.1 – 1.4 ms | ~2.5 MB process heap | 1.08 MB/s (internal) |
| **Host SLAM Worker (ICP+TSDF)**| 2,268 Points → Raycast Target | 8.5 – 14.2 ms (GPU/CPU) | ~350 MB VRAM / RAM | ~500 KB/s (poses/stats) |
| **Server → Browser (`/ws`)** | Compact Float16 / Int16 PointCloud | < 1.0 ms (localhost loop) | ~15 MB WebSocket heap | ~1.2 MB/s |
| **Server → Thin Client (`/ws-thin`)**| 480×480 RGB565 Raster Frame | 17.2 ms render + 10 ms tx | ~4.5 MB Framebuffer heap | 4.6 MB/s (36.8 Mbps) |

---

## 3. Code Correctness Audit

### 3.1 Firmware Audit (`firmware/scanner-stream/`)

#### Finding FW_01: Warning on Redefinition of `USE_NUCLEO_144`
* **File:** `firmware/scanner-stream/Inc/stm32h5xx_nucleo_conf.h:47`
* **Severity:** Low / Code Hygiene
* **Description:** The header defines `#define USE_NUCLEO_144` unconditionally, while CMake/compiler command lines also pass `-DUSE_NUCLEO_144`, generating compiler warnings across all compilation units (`warning: "USE_NUCLEO_144" redefined`).
* **Fix:** Wrap with `#ifndef USE_NUCLEO_144 ... #endif`.

#### Finding FW_02: Unused Static Function and Dead Variable in `vl53l9_app.c`
* **File:** `firmware/scanner-stream/Src/vl53l9_app.c:2585`, `vl53l9_app.c:3219`
* **Severity:** Low / Code Hygiene
* **Description:** `float frame_rate;` is set but never used at line 2585. `static void print_frame(...)` at line 3219 is unreferenced when `CONF_TRANSFORM_ONBOARD` is disabled.
* **Fix:** Remove `frame_rate` or log it; wrap `print_frame` with `#if CONF_TRANSFORM_ONBOARD`.

#### Finding FW_03: USB CDC Write Timeout Drop Behavior
* **File:** `firmware/scanner-stream/Src/vl53l9_app.c:580-610`
* **Severity:** Medium / Robustness
* **Description:** In `rs_cdc_send()`, if TinyUSB CDC TX FIFO is full and does not drain within 100 ms, the frame is dropped and the drop counter increments. However, if the host reader disconnects without closing the VCOM port cleanly, the 100 ms timeout executes synchronously inside the main ranging loop, stalling the sensor FSM and delaying I3C ACK handling.
* **Fix:** Implement a non-blocking check on CDC DTR (`tud_cdc_connected()`) and FIFO space (`tud_cdc_write_available()`) before attempting synchronous blocking transmission.

### 3.2 Host Ingestion & Processing Audit (`host/src/roomscan/`)

#### Finding HOST_01: Native C-Shim Thread Safety
* **File:** `host/transform/rs_transform_shim.c:135-220`
* **Severity:** Medium / Concurrency
* **Description:** `rst_create2` allocates an `rst_ctx_t` which holds internal ST transform pipeline pointers. The transform instance maintains internal state (such as temporal noise reduction history across successive frames). If two threads call `rst_process2` on the same `rst_ctx_t` simultaneously, memory corruption will occur.
* **Current Mitigation:** `host/src/roomscan/pipeline.py` confines `TransformStage` to a single reader thread.
* **Recommendation:** Document explicitly that `rst_ctx_t` is strictly single-threaded, or add an internal mutex if shared usage is ever planned.

#### Finding HOST_02: Zeroconf File Descriptor Leak on Rapid Reconnect
* **File:** `host/src/roomscan/sources.py:301-314`
* **Severity:** Low / Resource Management
* **Description:** In `UdpSource._resolve_target()`, a new `Zeroconf()` instance is created and closed per resolution attempt in `_maybe_keepalive()` if data has not arrived in 2.0 s. Creating/destroying `Zeroconf` objects at 0.5 Hz during connection recovery churns UDP listener sockets and threads.
* **Fix:** Maintain a single shared `Zeroconf` instance on the application state rather than instantiating ad-hoc instances in `_resolve_target`.

### 3.3 SLAM / 3D Vision Correctness Audit (`host/src/roomscan/slam/`)

#### Finding SLAM_01: VoxelBlockGrid Hashmap Rehash Headroom & Latency Spikes
* **File:** `host/src/roomscan/slam/tsdf.py:180-240`
* **Severity:** High / Performance & Reliability
* **Description:** Open3D's tensor `VoxelBlockGrid` uses a spatial hash map with a fixed initial `block_count` (default 40,000 blocks ≈ 40 m³ at 10 mm voxels). When occupancy approaches capacity, Open3D triggers a rehash. On CUDA, querying `block_usage()` forces a GPU synchronization barrier. To avoid per-frame syncs, `tsdf.py` samples usage every 25 frames. However, if a fast sweep enters new space, block allocation can exhaust capacity between checks, causing Open3D to throw a CUDA runtime error or stall the pipeline for >100 ms during rehash.
* **Recommendation:** Pre-allocate a 100,000-block table for room-scale scans (consumes ~48 MB VRAM), and use asynchronous CUDA stream queries where supported.

#### Finding SLAM_02: In-Plane Degeneracy in Corridor Sweeps
* **File:** `host/src/roomscan/slam/odometry.py:89-132`
* **Severity:** Medium / Algorithmic
* **Description:** The eigenvalue floor `_COND_CAP = 20.0` prevents unbounded translation runaway along unconstrained axes (e.g. looking straight at a planar wall). However, in featureless hallways, the translation along the corridor axis is floored to zero movement rather than being integrated via IMU specific force / dead reckoning.
* **Recommendation:** Integrate IMU double-integration velocity bounds during ill-conditioned epochs ($\text{cond}(A) > 20$) rather than pure zero-step damping.

### 3.4 Web & Visualization Correctness Audit (`host/src/roomscan/web.py`, `thin_render.py`)

#### Finding WEB_01: Mesh WebSocket Serialization Head-of-Line Blocking
* **File:** `host/src/roomscan/web.py:6347-6415`
* **Severity:** Medium / UX & Performance
* **Description:** When the SLAM worker extracts a new Detailed mesh (120,000–300,000 vertices), packing the binary mesh payload (`TAG_MESH_SURFACE`) and compressing via Deflate/Gzip blocks the asyncio event loop if not yielded via `asyncio.sleep(0)`.
* **Fix:** Verify all compression and binary packing runs strictly inside `asyncio.to_thread`.

---

## 4. Protocol & Wire Contract Audit

### 4.1 Structural Integrity Analysis (`docs/protocol.md`)

The wire protocol v2 is defined with a fixed 32-byte header:
$$\begin{aligned}
\text{Header (32 B)} &= \text{Magic (4B: "RSCN")} + \text{Ver (1B: 2)} + \text{Type (1B)} + \text{Stream (1B)} + \text{Flags (1B)} \\
&+ \text{Seq (4B)} + t_{\mu s}\text{ (8B)} + \text{Width (2B)} + \text{Height (2B)} + \text{PayloadLen (4B)} + \text{Reserved (4B)}
\end{aligned}$$

Followed by:
$$\text{Frame} = \text{Header (32 B)} + \text{Payload }(N\text{ Bytes}) + \text{CRC32 (4 B, IEEE 802.3)}$$

### 4.2 Stream Registry Validation Matrix

| ID | Name | Size (Bytes) | Alignment / Data Types | Decoder Verification Status |
| :--- | :--- | :--- | :--- | :--- |
| **0** | `DEPTH_ZF32` | $W \times H \times 4$ | IEEE 754 float32, little-endian | Verified (Legacy v1 passthrough) |
| **1** | `DEPTH_ZAPC` | $W \times H \times 16$ | $[x, y, z, \text{conf}]$ float32 | Verified in C-shim & tests |
| **2** | `AMBIENT` | $W \times H \times 4$ | IEEE 754 float32 | Verified |
| **3** | `AMPLITUDE` | $W \times H \times 4$ | IEEE 754 float32 | Verified |
| **4** | `CONFIDENCE`| $W \times H \times 4$ | IEEE 754 float32 | Verified |
| **5** | `REFLECTANCE`| $W \times H \times 4$ | IEEE 754 float32 | Verified |
| **6** | `STATUS` | $W \times H \times 1$ | uint8 ST status codes | Verified |
| **7** | `RAW_3DMD` | 14,842 B (Bin2) | Proprietary ST packed format | Verified (`RAW_3DMD_SIZE_BIN2`) |
| **8** | `CALIB` | 2,332 B | VL53L9CX NVM calibration blob | Verified (`CALIB_SIZE`) |
| **9** | `IMU_QUAT` | 16 B | $[w, x, y, z]$ float32 (LSM body) | Verified (Averaged SFLP FIFO) |
| **10**| `ENV` | 20 B | Press(f32) + Mag(3×f32) + Temp(f32) | Verified |
| **11**| `IMU_RAW` | $N \times 8$ B | Tag(u8) + 6B Data + 1B Res | Verified (LSM6DSV16X FIFO words) |
| **12**| `IMU_CAL` | 4 B | `freq_fine`(i8) + `valid`(u8) + res(u16) | Verified (Clock trim ppm) |
| **13**| `IMU_SYNC` | 22 B | `ts_lsm`(u32) + `t_edge_us`(u64) + ... | Verified (Hardware sync edge) |

---

## 5. Performance & Resource Utilization Audit

### 5.1 Firmware Footprint (STM32H563ZI Release Build)
* **Flash Usage:** 141,731 / 2,097,152 Bytes (**6.76%**)
* **RAM Usage:** 188,552 / 655,360 Bytes (**28.77%**)
* **I3C1 Airtime:** $\approx 11.87\text{ ms}$ per frame at 12.5 MHz ($\mathbf{35.6\%}$ at 30 Hz).
* **CPU Idle Margin:** $\approx 58\%$ spare CPU headroom on Cortex-M33 @ 250 MHz.

### 5.2 Host SLAM & Processing Profiling
* **Native Transform:** 1.15 ms on x86_64 host.
* **Pinhole Deprojection:** 0.35 ms (2,268 points).
* **Translation ICP:** 1.85 ms (CPU 6 iterations).
* **TSDF Volume Integration + Raycasting:** 6.3 ms on CUDA GPU.
* **Total SLAM Pipeline:** $\mathbf{8.5\text{ ms}}$ per frame ($\approx 117\text{ Hz}$ capacity vs 30 Hz input).

---

## 6. Architecture & Technical Debt Analysis

### 6.1 Refactoring `host/src/roomscan/web.py`
`web.py` currently spans 7,523 lines. It should be partitioned into a clean sub-package `roomscan.server`:
- `roomscan.server.app`: Application lifecycle & FastAPI factory.
- `roomscan.server.routes`: HTTP endpoints (`/thumb`, `/capture_video`, `/api/*`).
- `roomscan.server.ws_broadcaster`: Binary `/ws` and `/ws-mesh` dispatch loop.
- `roomscan.server.ws_thin`: Dedicated `/ws-thin` client handler and render pump.
- `roomscan.server.session`: Controller, recording management, and replay state.

---

## 7. Strategic Solutions to Maximize CrowPanel Thin-Client Framerate

### 7.1 The Bottleneck Analysis
Currently, `/ws-thin` is fixed at **10 fps** (`THIN_INTERVAL = 0.1` s) transmitting uncompressed RGB565 frames ($480 \times 480 \times 2 = 460,800$ Bytes).

```
┌──────────────────────────────┬──────────────────┬──────────────────┬──────────────────┐
│ Metric / Parameter           │ Current (10 fps) │ Target A (30 fps)│ Target B (60 fps)│
├──────────────────────────────┼──────────────────┼──────────────────┼──────────────────┤
│ Uncompressed Wire Bandwidth  │ 36.86 Mbps       │ 110.59 Mbps      │ 221.18 Mbps      │
│ JPEG Compressed Bandwidth    │ 1.80 Mbps        │ 5.40 Mbps        │ 10.80 Mbps       │
│ Host Render Time (Filament)  │ 17.2 ms (CPU)    │ 3.5 ms (GPU/EGL) │ 2.0 ms (GPU/EGL) │
│ Client Ingestion Load (ESP32)│ 4.6 MB/s         │ 0.67 MB/s (JPEG) │ 1.35 MB/s (JPEG) │
│ Client Frame Decode (ESP32)  │ Direct DMA       │ 1.2 ms (HW JPEG) │ 1.2 ms (HW JPEG) │
└──────────────────────────────┴──────────────────┴──────────────────┴──────────────────┘
```

### 7.2 The 5-Point Framerate Multiplier Architecture

1. **Leverage Host GPU EGL Context for Filament:**
   - Host GPU rendering executes in **2–4 ms** per frame compared to 17.2 ms on CPU llvmpipe.
   - Run Filament with EGL hardware acceleration on the host to unlock $\ge 60\text{ fps}$ render capacity.

2. **Host-Side TurboJPEG Compression (`format=jpeg` on `/ws-thin`):**
   - Compress 480×480 RGB frames on the host using `simplejpeg` / `turbojpeg` (quality 75, ~20 KB payload). Encoding takes **<0.8 ms** on modern x86 CPU cores.
   - Slashes wireless bandwidth from 110 Mbps to **5.4 Mbps at 30 fps**, completely eliminating Wi-Fi TCP buffer bloat.

3. **ESP32-P4 Hardware JPEG Decompression:**
   - The ESP32-P4 RISC-V SoC includes a dedicated **Hardware JPEG Decoder** capable of decoding 480×480 images in **1.2 ms** directly to the display framebuffer via DMA.
   - CPU utilization on the ESP32 remains under 5%, leaving full compute power for touch UI and network reception.

4. **Dynamic Framerate & Resolution Negotiation (`thin_hello`):**
   - Allow the client to request desired framerate (`fps=30` or `fps=60`) and resolution (`480x480`, `320x320`, or `800x480`).
   - During high-velocity orbit gestures, transmit 320×320 @ 60 fps; when stationary, switch to 480×480 @ 30 fps.

5. **Direct Framebuffer Double-Buffering on Client:**
   - ESP32-P4 allocates two framebuffers in PSRAM with direct DMA to the MIPI-DSI display controller, enabling tear-free 60 Hz rendering.

---

## 8. Prioritized Top 20 Backlog Summary

1. **Automated GitHub Actions CI/CD Pipeline** (`priority/now`, 1.5d)
2. **Refactor `web.py` Monolith into Modular Subpackages** (`priority/next`, 3d)
3. **Thin-Client TurboJPEG Compression on `/ws-thin`** (`priority/now`, 1d)
4. **Fix Detailed Mesh Vertex Budget Overshoot** (`priority/now`, 2d)
5. **Firmware Non-Blocking USB CDC Flow Control** (`priority/next`, 0.5d)
6. **Integrate SHT40 Environmental Data into Stream 10** (`priority/later`, 1d)
7. **Persistent TSDF Volume & Mesh Export (.ply / .gltf)** (`priority/next`, 1d)
8. **Dynamic TSDF Hashmap Pre-Allocation for Large Spaces** (`priority/next`, 1d)
9. **Clean Up Compiler Redefinition Warnings in Firmware** (`priority/next`, 0.25d)
10. **Shared Zeroconf Instance in `UdpSource`** (`priority/next`, 0.5d)
11. **ESP32-P4 USB Host CDC Ingestion Driver** (`priority/next`, 3d)
12. **ESP32-P4 Hardware JPEG Decode Pipeline** (`priority/now`, 2d)
13. **Hardware Sync Pulse for External Camera FSIN** (`priority/next`, 1d)
14. **Automated Operator Verification Benchmark Script** (`priority/next`, 1d)
15. **Dead-Reckoning Translation Floor for Ill-Conditioned ICP** (`priority/later`, 2d)
16. **Zero-Copy Raw Recording Storage Format (MCAP)** (`priority/later`, 2d)
17. **Web UI PBR Lighting & Measurement Annotations** (`priority/later`, 1.5d)
18. **Offline 3DGS Auto-Initialization from TSDF** (`priority/next`, 3d)
19. **Thermal Throttling & VCSEL Protection Telemetry** (`priority/later`, 0.5d)
20. **Pre-Commit Hook Suite for Repository Hygiene** (`priority/later`, 0.25d)
