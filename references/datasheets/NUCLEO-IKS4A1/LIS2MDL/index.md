# LIS2MDL Magnetometer Datasheet & Documentation Index

This directory contains technical datasheets, application notes, and design tips for the STMicroelectronics **LIS2MDL** ultra-low-power 3-axis digital magnetometer.

---

## LLM Quick Decision Matrix: Which Document Should You Read?

| Task / Information Needed | Recommended PDF | Markdown Summary |
| :--- | :--- | :--- |
| **Pinout, Electrical Specs, I2C/SPI Timing, Register Hex Map** | [lis2mdl.pdf](file:///home/sam/git/personal/lidar-roomscanner/references/datasheets/NUCLEO-IKS4A1/LIS2MDL/lis2mdl.pdf) | [lis2mdl.md](file:///home/sam/git/personal/lidar-roomscanner/references/datasheets/NUCLEO-IKS4A1/LIS2MDL/lis2mdl.md) |
| **Initialization Sequence, Power Modes, Hardware Offset Registers, DRDY** | [an5069...pdf](file:///home/sam/git/personal/lidar-roomscanner/references/datasheets/NUCLEO-IKS4A1/LIS2MDL/an5069-lis2mdl-ultralowpower-highperformance-3axis-magnetometer-stmicroelectronics.pdf) | [an5069...md](file:///home/sam/git/personal/lidar-roomscanner/references/datasheets/NUCLEO-IKS4A1/LIS2MDL/an5069-lis2mdl-ultralowpower-highperformance-3axis-magnetometer-stmicroelectronics.md) |
| **Full 3D Hard-Iron & Soft-Iron Calibration Math (Sphere/Ellipsoid Fitting)** | [dt0059...pdf](file:///home/sam/git/personal/lidar-roomscanner/references/datasheets/NUCLEO-IKS4A1/LIS2MDL/dt0059-ellipsoid-or-sphere-fitting-for-sensor-calibration-stmicroelectronics.pdf) | [dt0059...md](file:///home/sam/git/personal/lidar-roomscanner/references/datasheets/NUCLEO-IKS4A1/LIS2MDL/dt0059-ellipsoid-or-sphere-fitting-for-sensor-calibration-stmicroelectronics.md) |
| **Accelerometer-Assisted 2D Flat-Turn Calibration & PCB Tilt Correction** | [dt0103...pdf](file:///home/sam/git/personal/lidar-roomscanner/references/datasheets/NUCLEO-IKS4A1/LIS2MDL/dt0103-compensating-for-magnetometer-installation-error-and-hardiron-effects-using-accelerometerassisted-2d-calibration-stmicroelectronics.pdf) | [dt0103...md](file:///home/sam/git/personal/lidar-roomscanner/references/datasheets/NUCLEO-IKS4A1/LIS2MDL/dt0103-compensating-for-magnetometer-installation-error-and-hardiron-effects-using-accelerometerassisted-2d-calibration-stmicroelectronics.md) |
| **Using Magnetometer as Virtual Gyroscope / High RPM Spin Estimation** | [dt0104...pdf](file:///home/sam/git/personal/lidar-roomscanner/references/datasheets/NUCLEO-IKS4A1/LIS2MDL/dt0104-exploiting-the-magnetometer-as-a-virtual-gyroscope-at-low-and-ultrahigh-spin-rates-stmicroelectronics.pdf) | [dt0104...md](file:///home/sam/git/personal/lidar-roomscanner/references/datasheets/NUCLEO-IKS4A1/LIS2MDL/dt0104-exploiting-the-magnetometer-as-a-virtual-gyroscope-at-low-and-ultrahigh-spin-rates-stmicroelectronics.md) |
| **Simple 1-Point / 3-Point Tumble Calibration for Accelerometer/Mag** | [dt0105...pdf](file:///home/sam/git/personal/lidar-roomscanner/references/datasheets/NUCLEO-IKS4A1/LIS2MDL/dt0105-1point-or-3point-tumble-sensor-calibration-stmicroelectronics.pdf) | [dt0105...md](file:///home/sam/git/personal/lidar-roomscanner/references/datasheets/NUCLEO-IKS4A1/LIS2MDL/dt0105-1point-or-3point-tumble-sensor-calibration-stmicroelectronics.md) |
| **PCB Layout Hints, Capacitor Values, Magnetic Disturbance Avoidance** | [dt0131...pdf](file:///home/sam/git/personal/lidar-roomscanner/references/datasheets/NUCLEO-IKS4A1/LIS2MDL/dt0131-digital-magnetometer-and-ecompass-efficient-design-tips--stmicroelectronics (1).pdf) | [dt0131...md](file:///home/sam/git/personal/lidar-roomscanner/references/datasheets/NUCLEO-IKS4A1/LIS2MDL/dt0131-digital-magnetometer-and-ecompass-efficient-design-tips--stmicroelectronics (1).md) |

---

## Detailed Document Index

### 1. [lis2mdl.pdf](file:///home/sam/git/personal/lidar-roomscanner/references/datasheets/NUCLEO-IKS4A1/LIS2MDL/lis2mdl.pdf) — LIS2MDL Device Datasheet
* **Document Type**: Official Datasheet (DS12095, 36 pages)
* **Summary File**: [lis2mdl.md](file:///home/sam/git/personal/lidar-roomscanner/references/datasheets/NUCLEO-IKS4A1/LIS2MDL/lis2mdl.md)
* **Key Topics**: Mechanical package LGA-12, pin functions, electrical specifications, $\pm 50$ gauss full-scale range, $\text{I}^2\text{C}$ address (`0x1E`), SPI mode, complete register map (`0x45`-`0x6F`), sensitivity (1.5 mG/LSB).

### 2. [an5069-lis2mdl...pdf](file:///home/sam/git/personal/lidar-roomscanner/references/datasheets/NUCLEO-IKS4A1/LIS2MDL/an5069-lis2mdl-ultralowpower-highperformance-3axis-magnetometer-stmicroelectronics.pdf) — Application Note AN5069
* **Document Type**: Technical Application Note (38 pages)
* **Summary File**: [an5069...md](file:///home/sam/git/personal/lidar-roomscanner/references/datasheets/NUCLEO-IKS4A1/LIS2MDL/an5069-lis2mdl-ultralowpower-highperformance-3axis-magnetometer-stmicroelectronics.md)
* **Key Topics**: Low-power vs High-resolution operational modes, startup sequence code, hardware offset registers (`OFFSET_X/Y/Z_REG`), interrupt generation on threshold detection, temperature sensor readout (`0x6E`-`0x6F`).

### 3. [dt0059...pdf](file:///home/sam/git/personal/lidar-roomscanner/references/datasheets/NUCLEO-IKS4A1/LIS2MDL/dt0059-ellipsoid-or-sphere-fitting-for-sensor-calibration-stmicroelectronics.pdf) — Design Tip DT0059: Ellipsoid / Sphere Fitting
* **Document Type**: Mathematical Design Tip (6 pages)
* **Summary File**: [dt0059...md](file:///home/sam/git/personal/lidar-roomscanner/references/datasheets/NUCLEO-IKS4A1/LIS2MDL/dt0059-ellipsoid-or-sphere-fitting-for-sensor-calibration-stmicroelectronics.md)
* **Key Topics**: 3D matrix least-squares sphere/ellipsoid calibration equations, hard-iron offset vector $V$, soft-iron gain and cross-axis alignment matrix $R$.

### 4. [dt0103...pdf](file:///home/sam/git/personal/lidar-roomscanner/references/datasheets/NUCLEO-IKS4A1/LIS2MDL/dt0103-compensating-for-magnetometer-installation-error-and-hardiron-effects-using-accelerometerassisted-2d-calibration-stmicroelectronics.pdf) — Design Tip DT0103: Accelerometer-Assisted 2D Calibration
* **Document Type**: Algorithm Design Tip (6 pages)
* **Summary File**: [dt0103...md](file:///home/sam/git/personal/lidar-roomscanner/references/datasheets/NUCLEO-IKS4A1/LIS2MDL/dt0103-compensating-for-magnetometer-installation-error-and-hardiron-effects-using-accelerometerassisted-2d-calibration-stmicroelectronics.md)
* **Key Topics**: 2D planar rotation magnetometer calibration, tilt compensation using accelerometer roll/pitch angles, installation mounting misalignment angle correction.

### 5. [dt0104...pdf](file:///home/sam/git/personal/lidar-roomscanner/references/datasheets/NUCLEO-IKS4A1/LIS2MDL/dt0104-exploiting-the-magnetometer-as-a-virtual-gyroscope-at-low-and-ultrahigh-spin-rates-stmicroelectronics.pdf) — Design Tip DT0104: Virtual Gyroscope
* **Document Type**: Algorithm Design Tip (5 pages)
* **Summary File**: [dt0104...md](file:///home/sam/git/personal/lidar-roomscanner/references/datasheets/NUCLEO-IKS4A1/LIS2MDL/dt0104-exploiting-the-magnetometer-as-a-virtual-gyroscope-at-low-and-ultrahigh-spin-rates-stmicroelectronics.md)
* **Key Topics**: Time-derivative of magnetic vector $\frac{d\vec{B}}{dt}$, computing angular velocity $\omega$ without physical gyroscope, measuring ultra-high spin rates > 10,000 dps.

### 6. [dt0105...pdf](file:///home/sam/git/personal/lidar-roomscanner/references/datasheets/NUCLEO-IKS4A1/LIS2MDL/dt0105-1point-or-3point-tumble-sensor-calibration-stmicroelectronics.pdf) — Design Tip DT0105: Tumble Calibration
* **Document Type**: Calibration Design Tip (5 pages)
* **Summary File**: [dt0105...md](file:///home/sam/git/personal/lidar-roomscanner/references/datasheets/NUCLEO-IKS4A1/LIS2MDL/dt0105-1point-or-3point-tumble-sensor-calibration-stmicroelectronics.pdf)
* **Key Topics**: 1-point and 3-point static tumble calibration against Earth's gravity field ($1g$), fast zero-g offset and scale-factor extraction.

### 7. [dt0131...pdf](file:///home/sam/git/personal/lidar-roomscanner/references/datasheets/NUCLEO-IKS4A1/LIS2MDL/dt0131-digital-magnetometer-and-ecompass-efficient-design-tips--stmicroelectronics (1).pdf) — Design Tip DT0131: PCB & System Design Hints
* **Document Type**: Hardware Design Tip (6 pages)
* **Summary File**: [dt0131...md](file:///home/sam/git/personal/lidar-roomscanner/references/datasheets/NUCLEO-IKS4A1/LIS2MDL/dt0131-digital-magnetometer-and-ecompass-efficient-design-tips--stmicroelectronics (1).md)
* **Key Topics**: PCB layout rules, decoupling capacitors ($100\text{ nF}$ $V_{dd}$, $220\text{ nF}$ $C1$), magnetic immunity from high-current PCB traces, Block Data Update (BDU).
