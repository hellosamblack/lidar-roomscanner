# Roadmap — 53L9A1 3D Room Mapping

Product goal: a **tethered handheld 3D room scanner**. The STM32H563 streams timestamped sensor
frames to a PC running real-time SLAM (Open3D Tensor G-ICP + TSDF); an offline pass fuses 4K phone
video into a ToF-seeded 3D Gaussian Splat. Full design + critical review:
[`references/roadmapResearch.md`](./references/roadmapResearch.md).

Active development happens in this `roomscanner/` workspace. The existing STM32 firmware is **read-only
reference** in the sibling `53L9A1/` package; firmware paths below (`Src/…`) are relative to
`../53L9A1/Projects/NUCLEO-H563ZI/Applications/53L9A1/53L9A1_PostprocessSingle/`.

## Overriding architecture decisions

- **Transport: Ethernet is the production link, not USB.** The board has a 10/100 MAC (RMII pins already
  `AF11_ETH` in `Src/main.c`; MAC + lwIP not yet enabled). Target = lwIP/UDP with hardware PTP
  (IEEE 1588) timestamping. Native USB CDC (`USB_DRD_FS`) is a bring-up/fallback link only. This voids the
  USB-bandwidth and timestamp-drift bottlenecks in `references/roadmapResearch.md`.
- **Sensors: X-NUCLEO-IKS4A1** adds IMU (LSM6DSV16X, hardware SFLP orientation), magnetometer (yaw-drift
  correction), barometer (Z-drift constraint), temp/humidity (thermal comp). Not yet in code; I2C vs the
  ToF's I3C1 bus-sharing is unresolved.
- **Sequencing rule (owner):** mature the visualizer + UI/config on the **ToF sensor alone** before adding
  the IKS4A1 board.
- **Protocol rule:** design the frame protocol transport-agnostic from day one —
  `magic + seq + timestamp + payload + CRC32`, multi-stream — so the Ethernet cutover (Phase 4) is plumbing,
  not a redesign.

## Phases

### Phase 0 — ✅ Complete
On-device transform pipeline + ASCII depth map over ST-Link VCOM.
Enabled by `CONF_PRINT_FRAME = 1` in `Src/vl53l9_app.c:31`.

### Phase 1 — Real-time 3D visualizer
Replace ASCII printing with a **versioned binary frame protocol** and a PC app that deprojects depth into a
live-rendered point cloud.
- Transport: **native USB CDC FS** (ST-Link VCOM @115200 ≈ 1 fps for a ~9 KB frame — inadequate; USB CDC
  gives ~1 MB/s → real-time). Ethernet deferred to Phase 4.
- **First task:** capture the startup dump from `streams_inspect` / `controls_inspect`
  (`vl53l9_app.c:91-98`). It enumerates what the transform library can emit (depth / reflectance / confidence /
  possibly XYZ) and decides whether deprojection happens on-MCU or PC-side.
- Open question: does the pipeline output an XYZ/point-cloud stream, or only depth (`ZF32`)? If depth-only,
  the PC deprojects using VL53L9 per-zone angular geometry (note `radial_to_perp.c` exists → pipeline can emit
  perpendicular Z).
- PC side: Python + Open3D, or a lightweight custom viewer.

### Phase 2 — IR + additional sensor streams
Extend the protocol and PC UI to carry and toggle IR reflectance, confidence, ambient, etc.; colorize the
point cloud by IR intensity. Protocol is multi-stream from the start.

### Phase 3 — UI & runtime configuration
Host→device control channel to set usecase / binning / active streams at runtime (the transform library
exposes `controls`). Add recording/playback and config persistence.

### Phase 4 — Transport cutover to Ethernet
Enable the ETH MAC + lwIP, move the frame protocol onto UDP, add hardware PTP timestamping. Triggered when
bandwidth/sync demands it — i.e., as the sensor board approaches.

### Phase 5 — Integrate X-NUCLEO-IKS4A1
IMU/mag/baro drivers; resolve I2C-vs-I3C1 bus sharing; fuse readings into the payload with hardware
timestamps. Any new field bumps the payload version and keeps CRC32 last.

### Phase 6 — Real-time SLAM (PC)
SFLP quaternion as rotation prior → 3-DoF constrained Open3D Tensor G-ICP → scalable TSDF (VoxelBlockGrid),
IR as intensity channel, barometer as 1-DoF Z constraint.

### Phase 7 — Offline post-processing
COLMAP with ToF pose priors → depth-regularized 3D Gaussian Splatting seeded from the ToF cloud.
