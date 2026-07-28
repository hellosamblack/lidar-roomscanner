# AN5069 Application Note Summary

* **Document ID**: AN5069 (Rev 4)
* **Title**: LIS2MDL: ultralow-power, high-performance 3-axis magnetometer application note
* **PDF File**: [an5069...pdf](file:///home/sam/git/personal/lidar-roomscanner/references/datasheets/NUCLEO-IKS4A1/LIS2MDL/an5069-lis2mdl-ultralowpower-highperformance-3axis-magnetometer-stmicroelectronics.pdf)
* **Page Count**: 38 pages
* **Manufacturer**: STMicroelectronics

---

## Executive Summary
**AN5069** provides practical application guidelines, software initialization steps, operating mode transitions, temperature compensation algorithms, interrupt programming, and offset cancellation details for the LIS2MDL 3-axis magnetometer.

---

## Key Topics Covered

### 1. Operating Modes & Power Consumption
* **Continuous Mode**: Continuous measurements at 10, 20, 50, or 100 Hz.
* **Single-Trigger Mode**: Executes one conversion upon command, then returns to power-down mode. Ideal for ultra-low duty cycle sampling (< 10 Hz).
* **Power-Down Mode**: Minimal current draw (1.5 $\mu\text{A}$).
* **Low-Power vs High-Resolution**:
  * High-Resolution (`CFG_REG_A[4]=0`): Internal digital low-pass filtering and averaging enabled (current ~200 $\mu\text{A}$ @ 20 Hz).
  * Low-Power (`CFG_REG_A[4]=1`): Reduced sampling and filter bypass (current ~50 $\mu\text{A}$ @ 20 Hz).

### 2. Hard-Iron Offset Cancellation
* The LIS2MDL features internal offset cancellation registers (`OFFSET_X_REG_L/H` to `OFFSET_Z_REG_L/H` at `0x45`-`0x4A`).
* When written by host software (in 16-bit 2's complement), the sensor hardware automatically subtracts these offset values from raw magnetic measurement data before outputting to `OUTX_L/H_REG`..`OUTZ_L/H_REG`.
* Enables seamless integration with ST MotionFX dynamic magnetometer calibration libraries.

### 3. Temperature Compensation & Sensor Calibration
* Integrated 8-bit temperature sensor outputs signed 2's complement data to `TEMP_OUT_L/H_REG` (`0x6E`-`0x6F`).
* Explains how thermal drift impacts magnetic baseline offset and how to apply temperature coefficients.

### 4. Interrupt Features
* **Threshold Interrupt Generator**: Interrupt signal asserted when magnetic field along X, Y, or Z axis exceeds threshold defined in `INT_THS_L/H_REG` (`0x65`-`0x66`).
* **Interrupt Configuration**:
  * `INT_CTRL_REG` (`0x63`): Enable interrupt, configure active high/low, latch/pulse mode.
  * `INT_SOURCE_REG` (`0x64`): Read-only status flags indicating which axis triggered the threshold.
  * DRDY (Data Ready) pin multiplexing on `INT/DRDY` pin.

### 5. Recommended Initialization Sequence
```c
// 1. Soft Reset & Reboot
write_reg(CFG_REG_A, 0x60); // Soft reset + Reboot memory

// 2. High Resolution, Continuous Mode @ 100 Hz, BDU enabled
write_reg(CFG_REG_A, 0x0C); // ODR = 100 Hz, High-Res, Continuous
write_reg(CFG_REG_C, 0x10); // BDU = 1 (Block Data Update enabled)

// 3. Optional: Enable DRDY on INT pin
write_reg(CFG_REG_C, 0x11); // DRDY_on_PIN = 1, BDU = 1
```

---

## Table of Contents Summary
1. **Pin Description and Signal Connections**
2. **Operating Modes & Startup Sequences**
3. **Reading Output Data (BDU & Data Ready)**
4. **Offset Compensation Registers**
5. **Interrupt Generation & Thresholds**
6. **Digital Interfaces ($\text{I}^2\text{C}$ & SPI Read/Write Protocols)**
7. **Application Hints & PCB Layout Recommendations**

---

## LLM Routing Guide: When to Consult This File
Consult `AN5069` when:
* Writing initialization sequences and state machine drivers for LIS2MDL.
* Implementing hardware-accelerated hard-iron offset subtraction (`OFFSET_X/Y/Z_REG`).
* Setting up threshold interrupt generation or DRDY signal handling.
* Understanding Low-Power vs High-Resolution trade-offs in battery-operated applications.
