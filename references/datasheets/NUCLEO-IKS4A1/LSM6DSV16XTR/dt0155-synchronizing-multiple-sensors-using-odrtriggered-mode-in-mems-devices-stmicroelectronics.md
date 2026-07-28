# DT0155 Design Tip Summary

* **Document ID**: DT0155
* **Title**: Synchronizing multiple sensors using ODR-triggered mode in MEMS devices
* **Author**: Jan Sedlak
* **PDF File**: [dt0155...pdf](file:///home/sam/git/personal/lidar-roomscanner/references/datasheets/NUCLEO-IKS4A1/LSM6DSV16XTR/dt0155-synchronizing-multiple-sensors-using-odrtriggered-mode-in-mems-devices-stmicroelectronics.pdf)
* **Page Count**: 12 pages
* **Manufacturer**: STMicroelectronics

---

## Executive Summary
**DT0155** details techniques for **synchronizing multiple MEMS sensors** (such as LSM6DSV16X IMU and LIS2MDL magnetometer mounted on the X-NUCLEO-IKS4A1 expansion board) using ODR-triggered mode, trigger hardware pins, and $\text{I3C}/\text{I}^2\text{C}$ timing synchronization.

---

## Technical Principle & Hardware Modes

### 1. The Sensor Desynchronization Problem
MEMS internal RC oscillators have clock tolerances of up to $\pm 1\%$ to $\pm 3\%$. Over long data acquisition runs, sample timestamps between independent sensors (e.g. IMU accel/gyro vs external magnetometer) drift apart, causing phase errors in sensor fusion filters.

### 2. ODR-Triggered Synchronization Mode
* **Master Device**: LSM6DSV16X generates hardware ODR pulse on INT pin.
* **Slave Device**: Magnetometer / Environmental sensor samples data on the rising edge of master trigger pin.
* Ensures exact time alignment for all sensor samples stored in FIFO buffers.

---

## LLM Routing Guide: When to Consult This File
Consult `DT0155` when:
* Synchronizing sample timing across multiple sensors on X-NUCLEO-IKS4A1 (e.g. LSM6DSV16X + LIS2MDL).
* Reducing phase jitter in 9-axis sensor fusion libraries.
