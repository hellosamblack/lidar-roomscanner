---
name: edge-ai-tooling
description: Decision on ST Edge AI Suite tools and where edge compute belongs — sensor tier (MEMS MLC/ISPU) not the M33
metadata: 
  node_type: memory
  type: reference
  originSessionId: 75e7b29d-c8db-4ff2-98bc-b97fe95b45f2
---

Evaluated the ST Edge AI Suite (https://www.st.com/content/st_com/en/st-edge-ai-suite/tools.html) for this project. See [[hardware-stack]], [[mapping-pipeline-plan]].

**Thesis: edge AI belongs at the SENSOR tier, not the MCU tier.** The STM32H563 is a Cortex-M33 — DSP ext + single-precision FPU but **NO Helium/MVE and NO Neural-ART NPU**, so any NN runs as CPU code that competes with the `-Ofast` 60 Hz transform+DMA loop. Heavy perception (SLAM/TSDF/G-ICP/3DGS) stays on the PC GPU by design. The win is in-sensor ML on the IKS4A1 MEMS (zero MCU cost, microamps): **LSM6DSV16X MLC+FSM+SFLP**, **LSM6DSO16IS ISPU**, **LIS2DUXS12 MLC**. This is a **Phase 5** concern (IKS4A1 not in code yet); must not distract from Phases 1-3.

**Tool verdicts:**
- ✅ **MEMS Studio** — download; the hub for IKS4A1 (configure/log sensors, program LSM6DSV16X MLC, profile ISPU models).
- ✅ **ST Edge AI Developer Cloud** — online, no install; benchmark any candidate model on real H5 in the board farm before committing firmware.
- ◐ **High Speed Datalog** — optional; labeled multi-sensor dataset capture over USB w/ Python SDK (for MLC/NanoEdge training sets). Reference FW, not an app base.
- ⏸ **NanoEdge AI Studio** — defer to Phase 5; AutoML tiny IMU-signal ML as MCU-side lib.
- ⏸ **STM32Cube.AI / X-CUBE-AI** and **ST Edge AI Core (CLI)** — skip until a concrete tiny on-MCU model justifies fighting the accelerator-less M33 budget.
- ⏸ **ST AIoT Craft** — skip, redundant with MEMS Studio for MLC.
- ◐ **ST Edge AI Model Zoo** — browse online; grab deploy scripts only when a model is chosen.
- ✗ **Hand posture ToF AI** — off-mission (8×8 VL53L5/L8 gesture UI, not VL53L9 mapping); only if ToF hand-gesture UI is ever wanted.
- ✗ **StellarStudioAI** (automotive Stellar), **AI for OpenSTLinux / X-LINUX-AI** (MP2 Linux MPU) — not this hardware.

Likely Phase-5 payoffs, all in-sensor: wake-on-motion (LIS2DUXS12) to idle the LiDAR; scan/activity classification (LSM6DSV16X MLC) to auto start/stop recording; fast-motion "blur-risk" flag to annotate frames for the SLAM solver.
