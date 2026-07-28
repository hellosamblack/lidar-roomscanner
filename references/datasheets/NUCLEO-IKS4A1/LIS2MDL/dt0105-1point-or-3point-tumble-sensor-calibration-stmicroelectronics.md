# DT0105 Design Tip Summary

* **Document ID**: DT0105
* **Title**: 1-point or 3-point tumble sensor calibration
* **Author**: Andrea Vitali
* **PDF File**: [dt0105...pdf](file:///home/sam/git/personal/lidar-roomscanner/references/datasheets/NUCLEO-IKS4A1/LIS2MDL/dt0105-1point-or-3point-tumble-sensor-calibration-stmicroelectronics.pdf)
* **Page Count**: 5 pages
* **Manufacturer**: STMicroelectronics

---

## Executive Summary
**DT0105** outlines simple 1-point and 3-point tumble calibration techniques to determine gain and offset corrections for 3-axis motion sensors (primarily accelerometers and magnetometers). Unlike complex multi-point sphere fitting, tumble calibration requires placing the sensor in a few static orthogonal orientations relative to a known reference field (such as gravity $1g$).

---

## Calibration Approaches

### 1-Point Tumble Calibration
* **Requirement**: Sensor placed flat on a level surface (Z axis pointing upwards towards $+1g$).
* **Offset Calculation**:
  Assumes scale factor / gain is ideal ($1.0$). Offsets for X and Y axes are determined directly:
  $$V_x = A_x, \quad V_y = A_y$$
  $$V_z = A_z - 1g$$

### 3-Point Tumble Calibration
* **Requirement**: Sensor placed in 3 orthogonal positions facing gravity ($+X$, $+Y$, $+Z$ facing up).
* **Gain & Offset Calculation**:
  Measures gravity response $A_{meas\_x}, A_{meas\_y}, A_{meas\_z}$ at each position to isolate sensitivity error $K$ from bias $V$:
  $$V_i = \frac{A_{i\_max} + A_{i\_min}}{2}$$
  $$K_i = \frac{A_{i\_max} - A_{i\_min}}{2 \cdot 1g}$$
  $$A_{cal\_i} = \frac{A_{raw\_i} - V_i}{K_i}$$

---

## Applications & Benefits
* Fast factory line calibration or field calibration without specialized gimbal hardware.
* Minimal code size footprint for MCU firmware.
* Direct gain and offset recovery for accelerometer tilt measurement refinement.

---

## LLM Routing Guide: When to Consult This File
Consult `DT0105` when:
* Implementing quick manufacturing test or field calibration procedures for accelerometers/IMUs.
* Calculating zero-g offsets and gain scale factors without complex matrix inversion code.
