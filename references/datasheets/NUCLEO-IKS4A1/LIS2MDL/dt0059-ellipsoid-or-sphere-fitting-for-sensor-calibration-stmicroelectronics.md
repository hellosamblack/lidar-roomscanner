# DT0059 Design Tip Summary

* **Document ID**: DT0059
* **Title**: Ellipsoid or sphere fitting for sensor calibration
* **Author**: Andrea Vitali
* **PDF File**: [dt0059...pdf](file:///home/sam/git/personal/lidar-roomscanner/references/datasheets/NUCLEO-IKS4A1/LIS2MDL/dt0059-ellipsoid-or-sphere-fitting-for-sensor-calibration-stmicroelectronics.pdf)
* **Page Count**: 6 pages
* **Manufacturer**: STMicroelectronics

---

## Executive Summary
**DT0059** provides mathematical formulations and algorithms for calibrating 3-axis sensors (magnetometers and accelerometers) using **ellipsoid and sphere surface fitting**. It converts distorted raw sensor measurements distorted by hard-iron offsets, soft-iron scaling, and axis non-orthogonality into corrected spherical data vectors.

---

## Mathematical Model & Equations

### Raw Measurement Model
Raw sensor output $\vec{S}_{raw}$ is modeled as:
$$\vec{S}_{raw} = R \cdot D \cdot \vec{S}_{true} + V$$

Where:
* $V = [V_x, V_y, V_z]^T$: Hard-iron offset vector (center of the ellipsoid).
* $D = \text{diag}(D_x, D_y, D_z)$: Scale factor matrix (sensitivity / gain error per axis).
* $R$: $3 \times 3$ rotation / non-orthogonality matrix (soft-iron distortion & axis misalignment).
* $\vec{S}_{true}$: Ideal normalized vector on a sphere of radius $R_{norm}$ (e.g. Earth's magnetic field magnitude or $1g$).

### Calibration Correction Formula
$$\vec{S}_{cal} = D^{-1} \cdot R^{-1} \cdot (\vec{S}_{raw} - V)$$

### Ellipsoid Equation in Quadric Form
Uncalibrated measurements satisfy the general quadric equation:
$$A \cdot x^2 + B \cdot y^2 + C \cdot z^2 + 2D \cdot xy + 2E \cdot xz + 2F \cdot yz + 2G \cdot x + 2H \cdot y + 2I \cdot z = 1$$

Using linear least squares over $N \ge 9$ measurement points rotated through 3D space:
1. Construct observation matrix $M$ from sampled raw points $(x_i, y_i, z_i)$.
2. Solve linear system $M \cdot C = \mathbf{1}$ using Pseudo-Inverse or SVD.
3. Compute offset vector $V$, gain matrix $D$, and soft-iron rotation matrix $R$.

---

## Application Steps for Magnetometers
1. Rotate sensor slowly in 3D space (tumble / figure-8 pattern) to sample points across all orientations.
2. Collect $N \ge 50$ distinct 3D raw magnetometer samples $(M_x, M_y, M_z)$.
3. Run Sphere/Ellipsoid fitting algorithm to estimate hard-iron offset $V$ and soft-iron matrix $W = D^{-1} R^{-1}$.
4. Apply correction $M_{cal} = W \cdot (M_{raw} - V)$ on incoming data stream.

---

## LLM Routing Guide: When to Consult This File
Consult `DT0059` when:
* Implementing offline or online 3D hard-iron/soft-iron magnetometer calibration algorithms.
* Needing mathematical formulas for matrix least-squares sphere fitting.
* Developing custom sensor calibration tools in Python or embedded C without relying on proprietary pre-compiled binaries.
