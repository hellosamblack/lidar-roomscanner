# DT0058 Design Tip Summary

* **Document ID**: DT0058
* **Title**: Computing tilt measurement and tilt-compensated eCompass
* **Author**: Andrea Vitali
* **PDF File**: [dt0058...pdf](file:///home/sam/git/personal/lidar-roomscanner/references/datasheets/NUCLEO-IKS4A1/LSM6DSV16XTR/dt0058-computing-tilt-measurement-and-tiltcompensated-ecompass-stmicroelectronics.pdf)
* **Page Count**: 6 pages
* **Manufacturer**: STMicroelectronics

---

## Executive Summary
**DT0058** presents the exact trigonometric and matrix equations for computing **tilt angles (Roll $\phi$, Pitch $\theta$)** from 3D accelerometer data and **tilt-compensated eCompass heading (Yaw $\psi$)** from magnetometer data, as well as converting Euler angles to quaternions.

---

## Core Equations

### 1. Roll ($\phi$) and Pitch ($\theta$) from Accelerometer
$$\phi = \arctan2(A_y, A_z)$$
$$\theta = \arctan2\left(-A_x, \sqrt{A_y^2 + A_z^2}\right)$$

### 2. Tilt Compensation of Magnetometer Vectors
To compute accurate yaw heading when tilted, project 3D raw magnetometer data $(B_x, B_y, B_z)$ onto the horizontal plane using roll ($\phi$) and pitch ($\theta$):
$$B_{x\_level} = B_x \cdot \cos\theta + B_y \cdot \sin\phi \sin\theta + B_z \cdot \cos\phi \sin\theta$$
$$B_{y\_level} = B_y \cdot \cos\phi - B_z \cdot \sin\phi$$

### 3. Electronic Compass Heading (Yaw $\psi$)
$$\psi = \arctan2(-B_{y\_level}, B_{x\_level})$$
*(Declination angle must be added to obtain True North heading).*

### 4. Euler Angles to Quaternion Conversion
$$q_0 = \cos\frac{\phi}{2} \cos\frac{\theta}{2} \cos\frac{\psi}{2} + \sin\frac{\phi}{2} \sin\frac{\theta}{2} \sin\frac{\psi}{2}$$
$$q_1 = \sin\frac{\phi}{2} \cos\frac{\theta}{2} \cos\frac{\psi}{2} - \cos\frac{\phi}{2} \sin\frac{\theta}{2} \sin\frac{\psi}{2}$$
$$q_2 = \cos\frac{\phi}{2} \sin\frac{\theta}{2} \cos\frac{\psi}{2} + \sin\frac{\phi}{2} \cos\frac{\theta}{2} \sin\frac{\psi}{2}$$
$$q_3 = \cos\frac{\phi}{2} \cos\frac{\theta}{2} \sin\frac{\psi}{2} - \sin\frac{\phi}{2} \sin\frac{\theta}{2} \cos\frac{\psi}{2}$$

---

## LLM Routing Guide: When to Consult This File
Consult `DT0058` when:
* Implementing tilt computation (roll, pitch) or eCompass yaw algorithms in C/Python.
* Writing quaternion conversion utilities for 3D orientation tracking.
