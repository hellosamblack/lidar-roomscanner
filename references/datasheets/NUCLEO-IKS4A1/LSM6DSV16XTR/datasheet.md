# LSM6DSV16X Datasheet Summary

* **Document ID**: DS13745 (Rev 5)
* **Title**: LSM6DSV16X - 6-axis inertial measurement unit (IMU) and AI sensor with embedded sensor fusion, Qvar for high-end applications
* **PDF File**: [datasheet.pdf](file:///home/sam/git/personal/lidar-roomscanner/references/datasheets/NUCLEO-IKS4A1/LSM6DSV16XTR/datasheet.pdf) (See also `ds.txt`)
* **Page Count**: 198 pages
* **Manufacturer**: STMicroelectronics

---

## Executive Summary
The **LSM6DSV16X** is a flagship 6-axis IMU featuring a 3D digital accelerometer ($\pm 2/\pm 4/\pm 8/\pm 16 g$) and 3D digital gyroscope ($\pm 125/\pm 250/\pm 500/\pm 1000/\pm 2000/\pm 4000\text{ dps}$). It embeds triple core architecture (UI, EIS, OIS), a 4.5 KB FIFO, Embedded Sensor Fusion (SFI), Machine Learning Core (MLC), Finite State Machine (FSM), and Qvar electrostatic charge sensing.

---

## Key Hardware Specifications

| Parameter | Specification | Notes |
| :--- | :--- | :--- |
| **Accelerometer Ranges** | $\pm 2, \pm 4, \pm 8, \pm 16 g$ | 16-bit 2's complement |
| **Gyroscope Ranges** | $\pm 125, \pm 250, \pm 500, \pm 1000, \pm 2000, \pm 4000\text{ dps}$ | 16-bit 2's complement |
| **Supply Voltage ($V_{dd}$)** | 1.71 V to 3.6 V | $V_{dd\_IO}$ down to 1.08 V |
| **Current Consumption** | 0.65 mA (Combo High-Performance Mode)<br>0.45 mA (Low-Power Combo Mode) | Dual-channel independent core configuration |
| **Output Data Rates (ODR)** | 1.875 Hz to 7.68 kHz | High-performance and low-power modes |
| **Interfaces** | $\text{I}^2\text{C}$, SPI (3-wire/4-wire), MIPI $\text{I3C}^{\text{R}}$ | Primary $\text{I}^2\text{C}$ address: `0x6A` or `0x6B` (SA0 pin) |
| **FIFO Buffer** | 4.5 KB | Supports smart data compression (2x/3x) |
| **Package** | LGA-14L (2.5 x 3.0 x 0.83 mm) | 14-pin footprint |

---

## Primary Register Map Highlights

| Address (Hex) | Register Name | R/W | Default | Purpose / Function |
| :--- | :--- | :---: | :---: | :--- |
| `0x01` | `FUNC_CFG_ACCESS` | R/W | `0x00` | Access to embedded function registers (FSM, MLC, SFI, Qvar) |
| `0x02` | `PIN_CTRL` | R/W | `0x00` | SDO/SA0 pull-up, INT1/INT2 push-pull/open-drain select |
| `0x07` - `0x0A` | `FIFO_CTRL1..4` | R/W | `0x00` | FIFO watermark, compression, batching ODRs for Accel/Gyro |
| `0x0D` | `INT1_CTRL` | R/W | `0x00` | Interrupt 1 routing (DRDY, FIFO, FSM, MLC, Wake-up, 6D) |
| `0x0E` | `INT2_CTRL` | R/W | `0x00` | Interrupt 2 routing (DRDY, FIFO, FSM, MLC, Temperature) |
| `0x0F` | `WHO_AM_I` | R | `0x70` | Fixed device identification byte (`0x70`) |
| `0x10` | `CTRL1` | R/W | `0x00` | Accelerometer ODR (`ODR_XL`) and Full-Scale (`FS_XL`) |
| `0x11` | `CTRL2` | R/W | `0x00` | Gyroscope ODR (`ODR_G`) and Full-Scale (`FS_G`) |
| `0x12` | `CTRL3` | R/W | `0x04` | Software reset, reboot, BDU (Block Data Update), SPI mode |
| `0x17` | `CTRL8` | R/W | `0x00` | Accelerometer LPF2 / HPF configuration |
| `0x19` | `CTRL10` | R/W | `0x00` | Timestamp counter enable, ODR setting |
| `0x20` - `0x21` | `OUT_TEMP_L / H` | R | `0x00` | 16-bit Temperature output data (LSB / 256 °C) |
| `0x22` - `0x27` | `OUTX_L_G` .. `OUTZ_H_G` | R | `0x00` | 3D Gyroscope output data (16-bit 2's complement) |
| `0x28` - `0x2D` | `OUTX_L_A` .. `OUTZ_H_A` | R | `0x00` | 3D Accelerometer output data (16-bit 2's complement) |
| `0x35` | `EMB_FUNC_STATUS` | R | `0x00` | Status of embedded functions (FSM, MLC, Step Detector) |
| `0x36` - `0x37` | `FSM_STATUS_A / B` | R | `0x00` | FSM 1-8 interrupt trigger status flags |
| `0x38` | `MLC_STATUS` | R | `0x00` | MLC 1-4 decision tree output change flags |
| `0x78` - `0x7D` | `FIFO_DATA_OUT_TAG..Z_H` | R | `0x00` | FIFO output tag byte and data words |

---

## Embedded Features Summary
1. **Sensor Fusion Low Power (SFI)**: Embedded 6-axis 3D game rotation vector and eCompass vector calculations performed directly inside the IMU core, reducing host MCU processing overhead.
2. **Finite State Machine (FSM)**: Up to 8 configurable state machines for gesture recognition, motion pattern detection, and activity monitoring.
3. **Machine Learning Core (MLC)**: Up to 4 decision tree classifiers running on-chip for context-awareness (e.g. walking, running, stationary, driving).
4. **Qvar Electrostatic Sensing**: Touch, swipe, and proximity detection using external electrode.

---

## Pinout Overview (LGA-14L)

```
       +--------------------+
 SDO   | 1                14|  SDx
 SDx   | 2                13|  SCx
 CS    | 3   LSM6DSV16X   12|  INT1
 INT2  | 4   (Top View)   11|  NC
 VddIO | 5                10|  NC
 GND   | 6                 9|  Vdd
 GND   | 7                 8|  C1
       +--------------------+
```

---

## LLM Routing Guide: When to Consult This File
Consult `datasheet.pdf` / `datasheet.md` when:
* Writing basic low-level C drivers / register read/write routines for LSM6DSV16X.
* Verifying WHO_AM_I byte (`0x70`), full-scale ranges, or register hex addresses.
* Checking pin hardware connections, LGA-14 footprint, or electrical limits.
