# DT0103 Design Tip Summary

* **Document ID**: DT0103
* **Title**: Compensating for magnetometer installation error and hard-iron effects using accelerometer-assisted 2D calibration
* **Author**: Andrea Vitali
* **PDF File**: [dt0103...pdf](file:///home/sam/git/personal/lidar-roomscanner/references/datasheets/NUCLEO-IKS4A1/LIS2MDL/dt0103-compensating-for-magnetometer-installation-error-and-hardiron-effects-using-accelerometerassisted-2d-calibration-stmicroelectronics.pdf)
* **Page Count**: 6 pages
* **Manufacturer**: STMicroelectronics

---

## Executive Summary
**DT0103** describes an **accelerometer-assisted 2D calibration procedure** to compensate for magnetometer hard-iron offset errors and PCB installation tilt/misalignment. This method allows full magnetometer calibration by simply rotating the device in 2D on a horizontal plane (360° flat rotation) while using accelerometer gravity readings to project 3D magnetic vectors into 2D plane coordinates.

---

## Key Concepts & Algorithm

### Why 2D Calibration with Accelerometer?
* Full 3D ellipsoid fitting (DT0059) requires rotating the device through arbitrary 3D spatial angles, which is difficult or impossible for fixed vehicles, robots, or heavy devices.
* Standard 2D calibration assumes the rotation plane is strictly horizontal. If PCB installation has tilt error (roll/pitch angle $\theta, \phi \ne 0$), standard 2D calibration fails.
* **DT0103 Solution**: Use accelerometer data to measure tilt angles $\phi$ (roll) and $\theta$ (pitch) simultaneously during the 2D rotation.

### Mathematical Formulation
1. **Tilt Angle Computation**:
   $$\phi = \arctan2(A_y, A_z)$$
   $$\theta = \arctan2(-A_x, \sqrt{A_y^2 + A_z^2})$$

2. **Leveling (De-tilting) Raw Magnetometer Data**:
   $$B_{level\_x} = B_x \cdot \cos\theta + B_y \cdot \sin\phi \sin\theta + B_z \cdot \cos\phi \sin\theta$$
   $$B_{level\_y} = B_y \cdot \cos\phi - B_z \cdot \sin\phi$$

3. **2D Circle Fitting**:
   Compute 2D offsets $(V_x, V_y)$ and circle radius $R$ from 2D leveled data $(B_{level\_x}, B_{level\_y})$ collected during a single 360° horizontal turn.

4. **Installation Misalignment Angle Correction**:
   Resolves mounting misalignment angle $\psi_{error}$ between accelerometer body frame and magnetometer body frame.

---

## Advantages & Use Cases
* Fast 2D calibration in < 10 seconds.
* Ideal for land rovers, robotic vacuums, room scanners, and handheld gimbal scanners where full 3D tumbling is impractical.
* Robust against PCB mounting tolerances and structural tilt errors.

---

## LLM Routing Guide: When to Consult This File
Consult `DT0103` when:
* Implementing 2D planar magnetometer calibration for flat/surface-bound devices or robots.
* Correcting mounting tilt misalignment between an IMU (LSM6DSV16X) and magnetometer (LIS2MDL).
* Developing user-guided calibration routines (e.g., "Rotate device 360 degrees on the table").
