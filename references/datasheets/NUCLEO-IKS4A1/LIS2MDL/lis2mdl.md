# LIS2MDL Datasheet Summary

* **Document ID**: DS12095 (Rev 6)
* **Title**: LIS2MDL - Digital output magnetic sensor: ultralow-power, high-performance 3-axis magnetometer
* **PDF File**: [lis2mdl.pdf](file:///home/sam/git/personal/lidar-roomscanner/references/datasheets/NUCLEO-IKS4A1/LIS2MDL/lis2mdl.pdf)
* **Page Count**: 36 pages
* **Manufacturer**: STMicroelectronics

---

## Executive Summary
The **LIS2MDL** is an ultra-low-power, 3-axis digital magnetometer system-in-package with selectable $\text{I}^2\text{C}$ and 3-wire/4-wire SPI interfaces. It features a full-scale magnetic field measurement range of **$\pm 50$ gauss** with 16-bit resolution, low noise, and integrated temperature compensation. It is ideally suited for electronic compassing, orientation tracking, hard-iron offset compensation, and low-power motion sensing.

---

## Key Hardware Specifications

| Parameter | Specification | Notes |
| :--- | :--- | :--- |
| **Magnetic Field Range** | $\pm 50$ gauss ($\pm 5$ mT) | 16-bit data output |
| **Sensitivity** | 1.5 mG/LSB (0.15 $\mu\text{T/LSB}$) | Constant sensitivity across range |
| **Supply Voltage ($V_{dd}$)** | 1.71 V to 1.98 V | Digital I/O supply ($V_{dd\_IO}$) down to 1.62 V |
| **Current Consumption** | 200 $\mu\text{A}$ (High-Resolution mode @ 20 Hz)<br>50 $\mu\text{A}$ (Low-Power mode @ 20 Hz)<br>1.5 $\mu\text{A}$ (Power-Down mode) | Selectable via `CFG_REG_A` |
| **Output Data Rates (ODR)** | 10 Hz, 20 Hz, 50 Hz, 100 Hz | Selectable via `CFG_REG_A` |
| **Interfaces** | $\text{I}^2\text{C}$ (Standard, Fast mode 400 kHz, Fast mode+ 1 MHz)<br>SPI (3-wire and 4-wire) | $\text{I}^2\text{C}$ slave address: `0011110b` (`0x1E`) |
| **Package** | LGA-12 (2.0 x 2.0 x 1.0 mm) | 12-pin footprint |
| **Operating Temp** | -40 °C to +85 °C | Built-in 8-bit temperature sensor |

---

## Primary Register Map Summary

| Address (Hex) | Register Name | R/W | Default | Purpose / Function |
| :--- | :--- | :---: | :---: | :--- |
| `0x45` - `0x46` | `OFFSET_X_REG_L / H` | R/W | `0x00` | X-axis hard-iron offset cancellation (16-bit 2's complement) |
| `0x47` - `0x48` | `OFFSET_Y_REG_L / H` | R/W | `0x00` | Y-axis hard-iron offset cancellation (16-bit 2's complement) |
| `0x49` - `0x4A` | `OFFSET_Z_REG_L / H` | R/W | `0x00` | Z-axis hard-iron offset cancellation (16-bit 2's complement) |
| `0x4F` | `WHO_AM_I` | R | `0x40` | Device Identification Register (fixed `0x40`) |
| `0x60` | `CFG_REG_A` | R/W | `0x03` | Operating mode (Power-down, Single, Continuous), ODR, LP mode, reboot, soft reset |
| `0x61` | `CFG_REG_B` | R/W | `0x00` | Low-pass filter configuration, offset cancellation, LPF enable |
| `0x62` | `CFG_REG_C` | R/W | `0x00` | DRDY on INT pin, $\text{I}^2\text{C}$ disable, 4-wire SPI enable, BDU (Block Data Update) |
| `0x63` | `INT_CTRL_REG` | R/W | `0xE0` | Interrupt enable, active level (high/low), latching, X/Y/Z interrupt threshold detection |
| `0x64` | `INT_SOURCE_REG` | R | `0x00` | Interrupt source status flags (threshold exceeded X/Y/Z, internal status) |
| `0x65` - `0x66` | `INT_THS_L / H_REG` | R/W | `0x00` | 15-bit interrupt threshold value |
| `0x67` | `STATUS_REG` | R | `0x00` | Data ready flags for X, Y, Z axes and overrun flags |
| `0x68` - `0x69` | `OUTX_L / H_REG` | R | `0x00` | X-axis magnetic field output raw data (16-bit 2's complement) |
| `0x6A` - `0x6B` | `OUTY_L / H_REG` | R | `0x00` | Y-axis magnetic field output raw data (16-bit 2's complement) |
| `0x6C` - `0x6D` | `OUTZ_L / H_REG` | R | `0x00` | Z-axis magnetic field output raw data (16-bit 2's complement) |
| `0x6E` - `0x6F` | `TEMP_OUT_L / H_REG` | R | `0x00` | 16-bit temperature sensor output raw data |

---

## Operating Modes & Configuration

1. **Power-Down Mode (`CFG_REG_A[1:0] = 11` or `10`)**: Device is inactive, consuming ~1.5 $\mu\text{A}$. Registers remain accessible.
2. **Continuous Mode (`CFG_REG_A[1:0] = 00`)**: Sensor continuously measures magnetic field at configured ODR (10, 20, 50, 100 Hz).
3. **Single Mode (`CFG_REG_A[1:0] = 01`)**: Performs a single measurement, sets DRDY, then automatically returns to power-down mode.
4. **Low-Power vs High-Resolution (`CFG_REG_A[4]`)**:
   - `LP = 0`: High-Resolution mode (~200 $\mu\text{A}$ @ 20 Hz, lower noise).
   - `LP = 1`: Low-Power mode (~50 $\mu\text{A}$ @ 20 Hz).
5. **Block Data Update (`CFG_REG_C[4] = BDU`)**: Prevents reading MSB and LSB from different samples. Highly recommended to set `BDU = 1`.

---

## Pinout Overview (LGA-12)

```
        +-------------------+
  SCL   | 1               12|  NC
  NC    | 2               11|  NC
  GND   | 3    LIS2MDL    10|  C1 (External capacitor)
  NC    | 4   (Top View)   9|  Vdd_IO
  NC    | 5                8|  Vdd
  INT   | 6                7|  SDA/SDI/SDO
        +-------------------+
```

---

## LLM Routing Guide: When to Consult This File
Consult `lis2mdl.pdf` / `lis2mdl.md` when:
* Implementing the low-level C driver or HAL interface for LIS2MDL.
* Checking exact register bit definitions, hex addresses, default values, or WHO_AM_I byte (`0x40`).
* Configuring pin connections, decoupling capacitors (e.g., $100\text{ nF}$ on $V_{dd}$, $220\text{ nF}$ on $C1$), or $\text{I}^2\text{C}$ address (`0x1E`).
* Verifying electrical timing parameters (rise/fall times, SPI setup/hold times).
