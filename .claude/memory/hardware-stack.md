---
name: hardware-stack
description: "Full sensor/hardware stack for the tethered 3D room-mapping rig, including the X-NUCLEO-IKS4A1 MEMS board and each sensor's intended role"
metadata: 
  node_type: memory
  type: project
  originSessionId: 75e7b29d-c8db-4ff2-98bc-b97fe95b45f2
---

The rig targets tethered handheld 3D room mapping (see [[mapping-pipeline-plan]]). Hardware stack:

- **NUCLEO-H563ZI** — STM32H563ZI host (Cortex-M33 @ 250 MHz, 640 KB SRAM), USB 2.0 FS to PC.
- **X-NUCLEO-53L9A1** — VL53L9CX dToF LiDAR (2,268 zones / 54×42, up to 100 Hz), on **I3C1 + GPDMA**. This is the only sensor the current firmware drives.
- **X-NUCLEO-IKS4A1** — MEMS motion + environmental expansion board, NOT yet in code. Sensors and intended roles:
  - **LSM6DSV16X** (6-axis IMU, hardware SFLP sensor fusion) — primary orientation; SFLP game-rotation-vector quaternions offload Kalman math from the MCU. Used to inject a rotation prior so PC-side ICP only solves 3-DoF translation.
  - **LIS2MDL** (magnetometer) — long-term yaw-drift correction ONLY; 6-axis game-rotation-vector preferred indoors due to hard/soft-iron distortion.
  - **LPS22DF** (barometer) — the highest-value bonus sensor: relative-altitude → 1-DoF vertical constraint to fight Z-axis SLAM drift in stairwells/corridors. Caveat: indoor baro Z is corrupted by HVAC/door pressure transients (opening an exterior door shifts indoor pressure several Pa; ~12 Pa/m), so treat as a soft constraint, not ground truth.
  - **SHT40AD1B** (humidity+temp) and **STTS22H** (0.5 °C temp) — ambient temperature for thermal-drift compensation of gyro bias (ToF has its own internal temp comp, so the win is mainly IMU).
  - **LSM6DSO16IS** (6-axis IMU w/ ISPU) and **LIS2DUXS12** (ultra-low-power 3-axis accel) — largely redundant given SFLP; possible use: LIS2DUXS12 as a wake-on-motion trigger to bring LiDAR/SLAM out of deep sleep.

Open integration question: how the IKS4A1 sensors bus onto the host alongside the 53L9A1. The ToF already owns I3C1; IKS4A1 sensors are I2C — STM32H5 I3C is backward-compatible with legacy I2C on the same SDA/SCL, or they can sit on a separate I2C peripheral. Pin/stacking compatibility on the Arduino connectors needs checking before assuming both boards co-exist.

Binary USB payload plan appends new fields (e.g., baro as one float after accel) — any layout change must bump the payload version and keep CRC32 at the end.
