# DT0106 Design Tip Summary

* **Document ID**: DT0106
* **Title**: Residual linear acceleration by gravity subtraction to enable dead-reckoning
* **Author**: Andrea Vitali
* **PDF File**: [dt0106...pdf](file:///home/sam/git/personal/lidar-roomscanner/references/datasheets/NUCLEO-IKS4A1/LSM6DSV16XTR/dt0106-gravity-subtraction-dead-reckoning.pdf)
* **Page Count**: 6 pages
* **Manufacturer**: STMicroelectronics

---

## Executive Summary
**DT0106** provides the mathematical formulation for subtracting Earth's gravity vector ($\vec{g}$) from raw 3D accelerometer data to isolate pure **residual linear acceleration**, both in the sensor body frame and world reference frame. Isolating linear acceleration is an essential prerequisite for 3D inertial dead-reckoning (double integration for displacement tracking).

---

## Core Algorithm & Equations

### 1. Sensor Body Frame Gravity Subtraction
Given sensor orientation quaternion $q = [q_0, q_1, q_2, q_3]$ (from sensor fusion / SFI) and raw accelerometer reading $\vec{A}_{body}$:

The gravity vector in the body frame $\vec{g}_{body}$ is computed by rotating world gravity $\vec{g}_{world} = [0, 0, 1g]^T$:
$$\vec{g}_{body} = \begin{bmatrix} 2(q_1 q_3 - q_0 q_2) \\ 2(q_2 q_3 + q_0 q_1) \\ q_0^2 - q_1^2 - q_2^2 + q_3^2 \end{bmatrix} \cdot 1g$$

Residual body frame linear acceleration:
$$\vec{a}_{lin\_body} = \vec{A}_{body} - \vec{g}_{body}$$

### 2. World Reference Frame Linear Acceleration
Project linear acceleration into the fixed World Reference Frame:
$$\vec{a}_{lin\_world} = R(q) \cdot \vec{a}_{lin\_body}$$

### 3. Double Integration for Dead-Reckoning Position
$$\vec{V}_{k} = \vec{V}_{k-1} + \vec{a}_{lin\_world} \cdot \Delta t$$
$$\vec{P}_{k} = \vec{P}_{k-1} + \vec{V}_{k} \cdot \Delta t + \frac{1}{2} \vec{a}_{lin\_world} \cdot (\Delta t)^2$$

---

## LLM Routing Guide: When to Consult This File
Consult `DT0106` when:
* Implementing IMU dead-reckoning position tracking or speed estimation algorithms.
* Isolating linear motion from gravity in 3D space.
* Writing C or Python functions to extract gravity vectors from orientation quaternions.
