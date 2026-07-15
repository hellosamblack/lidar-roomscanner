---
name: mapping-pipeline-plan
description: "The two-phase 3D room-mapping architecture the project is aiming toward, and the host-transport reconsideration (USB vs Ethernet)"
metadata: 
  node_type: memory
  type: project
  originSessionId: 75e7b29d-c8db-4ff2-98bc-b97fe95b45f2
---

Target product: tethered handheld 3D room scanner. Documented in `roadmapResearch.md` (in the app dir); that doc is forward-looking — NONE of it is in the firmware yet, which today just does on-device ToF transform + ASCII-over-VCOM.

**Phase 1 (real-time):** MCU streams timestamped binary frames (ToF depth+IR + IMU quaternion/accel) to a PC. PC parses (magic word + CRC32 + seq), injects IMU rotation as a prior so **Open3D Tensor G-ICP solves only 3-DoF translation**, integrates into a scalable TSDF (VoxelBlockGrid), IR used as intensity channel (also fixes ICP aperture problem on blank walls). Open3D chosen over RTAB-Map/ROS2.

**Phase 2 (post-process):** 4K phone video localized to the metric ToF trajectory (hand-eye calib + pose priors into COLMAP bundle adjustment), then **3D Gaussian Splatting seeded/depth-regularized by the ToF cloud** to kill floaters.

See [[hardware-stack]] for sensors. See [[roadmap-review-notes]] for issues found in the research doc.

**Transport is under reconsideration — USB FS vs Ethernet:**
The roadmap is built entirely around USB 2.0 FS (~9.2 Mbps practical) and spends two of its four "bottleneck" sections on it (bus saturation → Delta-RLE compression; timestamp drift → MCU-as-master-clock). But the **NUCLEO-H563ZI has 10/100 Ethernet** (STM32H563 has an ETH MAC; RMII pins LAN8742 are ALREADY configured with AF11_ETH in `Src/main.c` MX_GPIO_Init, though ETH MAC init / lwIP is not yet generated). Ethernet likely the better fit:
- ~100 Mbps vs ~9.2 Mbps → ~10× headroom. Full uncompressed payload w/ confidence (4 B/zone × 2268 = 9072 B) at 100 Hz ≈ 7.3 Mbps — trivial; **removes the need for on-MCU compression entirely**.
- **Hardware PTP (IEEE 1588) timestamping** on the STM32H5 ETH → collapses the timestamp-sync bottleneck far better than the SYNC_IN master-clock hack.
- More deterministic latency than host USB-CDC scheduling jitter (which the roadmap itself flags as a packet-loss failure mode).
- Cost: needs a network stack (lwIP RAW/UDP bare-metal, or NetX under Azure RTOS) — more firmware than USB CDC. Power still needed (USB/ST-Link can power the board while Ethernet carries data). Direct board↔NIC link with static IP works (modern NICs auto-MDIX); UDP for low-latency + seq numbers, or TCP for reliability.

Net: adopting Ethernet collapses roadmap Bottleneck-1 (USB saturation) and Bottleneck-3 (timestamp drift). Ethernet is now the committed target transport (USB CDC = fallback), but the *cutover* is deferred to Phase 4 — near-term visualizer work uses native USB CDC FS because it's the lowest-lift link fast enough for real-time.

**SUPERSEDED (owner, 2026-07-10): Ethernet is SHELVED.** Measurement inverted the analysis above — since P2.5's trigger-early overlap the CDC send is fully hidden inside the sensor ranging window, and the real bandwidth wall is the **I3C sensor readout** (~60-80 Hz raw ceiling), which Ethernet cannot fix. USB CDC FS is the production link. Revival triggers (I3C ceiling lifted / PTP multi-sensor sync needed / longer tether) are recorded in ROADMAP.md's transport decision.

**Committed phase roadmap** (owner's stated order: mature visualizer + UI/config on the ToF sensor ALONE before adding IKS4A1). Full version in CLAUDE.md "Target architecture":
- **P0 ✅** on-device transform + ASCII depth over ST-Link VCOM (`CONF_PRINT_FRAME=1`, `vl53l9_app.c:31`).
- **P1 ✅ (2026-07-08)** real-time 3D visualizer — protocol v1 + `roomscan` host package + `firmware/scanner-stream` fork; verified on HW over TinyUSB CDC at 13.65 fps, 0 CRC/gaps (ceiling = sensor+transform ~74 ms/frame, not the link). Key finds now in repo docs: transform lib has 6 output streams incl. ZAPC on-device point cloud (Phase 2 option), NO runtime usecase/binning controls (Phase 3 needs re-init path), ~1-in-5 boot hang needs EVENT+recovery follow-up.
- **P2 ✅ (2026-07-08)** — post-processing migrated to the PC. MCU = thin bridge (raw 3DMD 14,842 B + CALIB 2,332 B every 64 frames over CDC); PC runs vl53l9-transform-c natively (MSVC DLL). Equivalence gate: 731/731 hardware pairs ≤0.000854 mm (0% bit-exact — fp models differ; documented). Live e2e 24.6 fps at full 54×42 (send-serialization capped; trigger-early overlap deferred for ~30). Deferred items tracked in ROADMAP: reflectance/--color, trigger-early overlap, ZAPC validation, connect-time CRC transient (first occurrence, unexplained), CALIB-on-DTR-connect (63-frame blind-start). Dual-stream firmware mode retained as golden-pair regeneration path. User also added references/datasheets/VL53L9CX/ (fov/thermals/x-nucleo PDFs) — uningested as of 2026-07-08.
- **P3** UI + runtime config: host→device control channel (transform lib `controls` set usecase/binning/streams live), record/playback, config persistence.
- **P4** transport cutover to Ethernet (lwIP/UDP + hardware PTP).
- **P5** integrate X-NUCLEO-IKS4A1 (IMU/mag/baro drivers → payload).
- **P6** real-time SLAM on PC (SFLP rotation prior, 3-DoF ~~G-ICP~~ point-to-plane ICP frame-to-model, TSDF, IR intensity, baro Z-constraint).
- **P7** offline: COLMAP pose priors + depth-regularized 3D Gaussian Splatting.

**P2.5 ✅ (2026-07-08, merged)**: FoV datasheet-calibrated 55×42 + ZAPC-validated (54.65×42.50 best-fit; corner distortion → optional per-zone tables); multi-output transform (reflectance/confidence/ambient/ZAPC) + viewer --color; trigger-early overlap → 27.76 fps (send off critical path — Ethernet's value is now rate/PTP/zero-config, not fps). Open: connect-time transient (2 occurrences, same signature), CALIB-on-DTR-connect blind-start fix.

**Phase 5 prep (user-driven)**: IKS4A1 bus topology RESOLVED by owner — sensors ride the ToF's I3C1 as legacy-I2C targets (shared PB8/PB9), no separate peripheral; recipe in repo docs/iks4a1-stacking.md; IKS4A1 datasheets under references/datasheets/IKS4A1/.

**PHASES SWAPPED (owner, 2026-07-09): IKS4A1 sensors = Phase 4 (NEXT); Ethernet = Phase 5** (older docs may use pre-swap numbering). P3 ✅ merged 2026-07-09 (runtime config + robustness: 10/10 boot soak, EVENT/recovery, viewer keys; first milestone-retro executed — host/tools/ now has capture/bench/measure/analyze tools). Zero-config direct Ethernet requirement (listen-first DHCP + mDNS + auto-MDIX) lives in ROADMAP Phase 5 now. Phase 3.5 GUI panel spec exists, unscheduled.

**Status 2026-07-10 (Phase 4 retro):** P3.5 ✅ (GUI panel, merged). P4 ✅ (IKS4A1 fully integrated — see [[lsm6dsv16x-panel-integration]]; open: on-rig mag cal + AXIS_CONVENTION per [[yaw-drift-correction]]). P5 Ethernet ⏸ SHELVED (above). **Next = Phase 6, real-time SLAM on the PC** — validate against recorded captures before hardware-in-the-loop. IMU/ENV arrive at per-ToF-frame cadence (~28 Hz) on the wire, not native rate — revisit only if SLAM needs denser orientation samples.

**P6 stack decision (2026-07-10, in ROADMAP Phase 6):** "Open3D Tensor G-ICP" was a spec error — 0.19 has no tensor GICP; primary = point-to-plane tensor ICP **frame-to-model** vs VoxelBlockGrid raycast (`small_gicp` = GICP fallback). FAST-LIO2/Point-LIO/CT-ICP/PIN-SLAM/SHINE evaluated and rejected (scanning-LiDAR assumptions vs our 54×42 depth imager); KISS-ICP = offline benchmark only. CPU-first stands: Windows Open3D wheel is CPU-only (CUDA = source build or WSL2, only if TSDF integrate/raycast blows budget); the RTX 4080's real job is P7 3DGS.
