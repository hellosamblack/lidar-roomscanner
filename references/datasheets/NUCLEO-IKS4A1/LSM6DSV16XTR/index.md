# LSM6DSV16X 6-Axis IMU Datasheet & Documentation Index

This directory contains technical datasheets, application notes, and design tips for the STMicroelectronics **LSM6DSV16X** 6-axis IMU (3D accelerometer + 3D gyroscope) with Embedded Sensor Fusion (SFI), Machine Learning Core (MLC), and Finite State Machine (FSM).

> [!NOTE]
> **Duplicate File Reference**:
> * `applicationNote.pdf` is identical to `an5763-lsm6dsv16x-6axis-imu-with-embedded-sensor-fusion-ai-qvar-for-highend-applications-stmicroelectronics.pdf`.
> * `machineLearningCore.pdf` is identical to `an5804-lsm6dsv16x-machine-learning-core-stmicroelectronics.pdf`.
> * `an.txt` and `ds.txt` are text dumps of AN5763 and the LSM6DSV16X Datasheet respectively.

---

## LLM Quick Decision Matrix: Which Document Should You Read?

| Task / Information Needed | Recommended PDF | Markdown Summary |
| :--- | :--- | :--- |
| **Pinout, Hex Register Addresses, Specs, WHO_AM_I (`0x70`), FIFO Regs** | [datasheet.pdf](file:///home/sam/git/personal/lidar-roomscanner/references/datasheets/NUCLEO-IKS4A1/LSM6DSV16XTR/datasheet.pdf) | [datasheet.md](file:///home/sam/git/personal/lidar-roomscanner/references/datasheets/NUCLEO-IKS4A1/LSM6DSV16XTR/datasheet.md) |
| **Driver Initialization, ODR Selection, Filter Cutoffs, FIFO 2x/3x, SFI Fusion, I2C/SPI** | [an5763...pdf](file:///home/sam/git/personal/lidar-roomscanner/references/datasheets/NUCLEO-IKS4A1/LSM6DSV16XTR/an5763-lsm6dsv16x-6axis-imu-with-embedded-sensor-fusion-ai-qvar-for-highend-applications-stmicroelectronics.pdf) | [an5763...md](file:///home/sam/git/personal/lidar-roomscanner/references/datasheets/NUCLEO-IKS4A1/LSM6DSV16XTR/an5763-lsm6dsv16x-6axis-imu-with-embedded-sensor-fusion-ai-qvar-for-highend-applications-stmicroelectronics.md) |
| **Machine Learning Core (MLC) Decision Trees, Feature Extraction, .ucf Loading** | [an5804...pdf](file:///home/sam/git/personal/lidar-roomscanner/references/datasheets/NUCLEO-IKS4A1/LSM6DSV16XTR/an5804-lsm6dsv16x-machine-learning-core-stmicroelectronics.pdf) | [an5804...md](file:///home/sam/git/personal/lidar-roomscanner/references/datasheets/NUCLEO-IKS4A1/LSM6DSV16XTR/an5804-lsm6dsv16x-machine-learning-core-stmicroelectronics.md) |
| **Finite State Machine (FSM) Opcodes, Program Memory Mapping, Motion Patterns** | [an5882...pdf](file:///home/sam/git/personal/lidar-roomscanner/references/datasheets/NUCLEO-IKS4A1/LSM6DSV16XTR/an5882-lsm6dsv16x-finite-state-machine-stmicroelectronics.pdf) | [an5882...md](file:///home/sam/git/personal/lidar-roomscanner/references/datasheets/NUCLEO-IKS4A1/LSM6DSV16XTR/an5882-lsm6dsv16x-finite-state-machine-stmicroelectronics.md) |
| **Tilt Calculation (Roll/Pitch) & Tilt-Compensated eCompass Yaw Math** | [dt0058...pdf](file:///home/sam/git/personal/lidar-roomscanner/references/datasheets/NUCLEO-IKS4A1/LSM6DSV16XTR/dt0058-computing-tilt-measurement-and-tiltcompensated-ecompass-stmicroelectronics.pdf) | [dt0058...md](file:///home/sam/git/personal/lidar-roomscanner/references/datasheets/NUCLEO-IKS4A1/LSM6DSV16XTR/dt0058-computing-tilt-measurement-and-tiltcompensated-ecompass-stmicroelectronics.md) |
| **Gyroscope Integration for Dynamic Motion & Complementary Filtering** | [dt0060...pdf](file:///home/sam/git/personal/lidar-roomscanner/references/datasheets/NUCLEO-IKS4A1/LSM6DSV16XTR/dt0060-exploiting-the-gyroscope-to-update-tilt-measurement-and-ecompass-stmicroelectronics.pdf) | [dt0060...md](file:///home/sam/git/personal/lidar-roomscanner/references/datasheets/NUCLEO-IKS4A1/LSM6DSV16XTR/dt0060-exploiting-the-gyroscope-to-update-tilt-measurement-and-ecompass-stmicroelectronics.md) |
| **MEMS Noise Characterization & Allan Variance ($\sigma(\tau)$) Analysis** | [dt0064...pdf](file:///home/sam/git/personal/lidar-roomscanner/references/datasheets/NUCLEO-IKS4A1/LSM6DSV16XTR/dt0064-allan-variance-noise-analysis.pdf) | [dt0064...md](file:///home/sam/git/personal/lidar-roomscanner/references/datasheets/NUCLEO-IKS4A1/LSM6DSV16XTR/dt0064-allan-variance-noise-analysis.md) |
| **Gravity Subtraction & 3D World-Frame Linear Acceleration for Dead-Reckoning** | [dt0106...pdf](file:///home/sam/git/personal/lidar-roomscanner/references/datasheets/NUCLEO-IKS4A1/LSM6DSV16XTR/dt0106-gravity-subtraction-dead-reckoning.pdf) | [dt0106...md](file:///home/sam/git/personal/lidar-roomscanner/references/datasheets/NUCLEO-IKS4A1/LSM6DSV16XTR/dt0106-gravity-subtraction-dead-reckoning.md) |
| **Multi-Sensor ODR Synchronization (IMU + Mag Trigger Pins)** | [dt0155...pdf](file:///home/sam/git/personal/lidar-roomscanner/references/datasheets/NUCLEO-IKS4A1/LSM6DSV16XTR/dt0155-synchronizing-multiple-sensors-using-odrtriggered-mode-in-mems-devices-stmicroelectronics.pdf) | [dt0155...md](file:///home/sam/git/personal/lidar-roomscanner/references/datasheets/NUCLEO-IKS4A1/LSM6DSV16XTR/dt0155-synchronizing-multiple-sensors-using-odrtriggered-mode-in-mems-devices-stmicroelectronics.md) |

---

## Detailed Document Index

### 1. [datasheet.pdf](file:///home/sam/git/personal/lidar-roomscanner/references/datasheets/NUCLEO-IKS4A1/LSM6DSV16XTR/datasheet.pdf) — LSM6DSV16X Device Datasheet
* **Document Type**: Official Datasheet (DS13745, 198 pages)
* **Summary File**: [datasheet.md](file:///home/sam/git/personal/lidar-roomscanner/references/datasheets/NUCLEO-IKS4A1/LSM6DSV16XTR/datasheet.md)
* **Key Topics**: LGA-14L footprint, 3D accelerometer ($\pm 2/\pm 4/\pm 8/\pm 16 g$), 3D gyroscope ($\pm 125$ to $\pm 4000\text{ dps}$), WHO_AM_I (`0x70`), register map (`0x01`-`0x7D`), electrical characteristics, $\text{I}^2\text{C}$ address `0x6A`/`0x6B`.

### 2. [an5763...pdf](file:///home/sam/git/personal/lidar-roomscanner/references/datasheets/NUCLEO-IKS4A1/LSM6DSV16XTR/an5763-lsm6dsv16x-6axis-imu-with-embedded-sensor-fusion-ai-qvar-for-highend-applications-stmicroelectronics.pdf) — Application Note AN5763
* **Document Type**: Technical Application Note (147 pages)
* **Summary File**: [an5763...md](file:///home/sam/git/personal/lidar-roomscanner/references/datasheets/NUCLEO-IKS4A1/LSM6DSV16XTR/an5763-lsm6dsv16x-6axis-imu-with-embedded-sensor-fusion-ai-qvar-for-highend-applications-stmicroelectronics.md)
* **Aliases**: `applicationNote.pdf`
* **Key Topics**: Operational power modes, filter setup (LPF1, LPF2, HPF), 4.5 KB FIFO batching & compression, Embedded Sensor Fusion (SFI) quaternions, interrupt routing (`INT1`/`INT2`).

### 3. [an5804...pdf](file:///home/sam/git/personal/lidar-roomscanner/references/datasheets/NUCLEO-IKS4A1/LSM6DSV16XTR/an5804-lsm6dsv16x-machine-learning-core-stmicroelectronics.pdf) — Application Note AN5804: Machine Learning Core
* **Document Type**: AI Core Application Note (67 pages)
* **Summary File**: [an5804...md](file:///home/sam/git/personal/lidar-roomscanner/references/datasheets/NUCLEO-IKS4A1/LSM6DSV16XTR/an5804-lsm6dsv16x-machine-learning-core-stmicroelectronics.md)
* **Aliases**: `machineLearningCore.pdf`
* **Key Topics**: Programming 4 decision trees, statistical feature extraction (mean, variance, zero-crossing, energy), `.ucf` configuration loading.

### 4. [an5882...pdf](file:///home/sam/git/personal/lidar-roomscanner/references/datasheets/NUCLEO-IKS4A1/LSM6DSV16XTR/an5882-lsm6dsv16x-finite-state-machine-stmicroelectronics.pdf) — Application Note AN5882: Finite State Machine
* **Document Type**: FSM Application Note (68 pages)
* **Summary File**: [an5882...md](file:///home/sam/git/personal/lidar-roomscanner/references/datasheets/NUCLEO-IKS4A1/LSM6DSV16XTR/an5882-lsm6dsv16x-finite-state-machine-stmicroelectronics.md)
* **Key Topics**: Configuring up to 8 state machines, opcode instruction set (`CHK`, `JMP`, `OUT`), program memory pages, motion pattern detection.

### 5. [dt0058...pdf](file:///home/sam/git/personal/lidar-roomscanner/references/datasheets/NUCLEO-IKS4A1/LSM6DSV16XTR/dt0058-computing-tilt-measurement-and-tiltcompensated-ecompass-stmicroelectronics.pdf) — Design Tip DT0058: Tilt & eCompass Math
* **Document Type**: Equations Design Tip (6 pages)
* **Summary File**: [dt0058...md](file:///home/sam/git/personal/lidar-roomscanner/references/datasheets/NUCLEO-IKS4A1/LSM6DSV16XTR/dt0058-computing-tilt-measurement-and-tiltcompensated-ecompass-stmicroelectronics.md)
* **Key Topics**: Roll $\phi$, Pitch $\theta$, tilt-compensated level magnetometer vectors, Yaw $\psi$, Euler-to-Quaternion conversion formulas.

### 6. [dt0060...pdf](file:///home/sam/git/personal/lidar-roomscanner/references/datasheets/NUCLEO-IKS4A1/LSM6DSV16XTR/dt0060-exploiting-the-gyroscope-to-update-tilt-measurement-and-ecompass-stmicroelectronics.pdf) — Design Tip DT0060: Gyro Integration
* **Document Type**: Fusion Design Tip (7 pages)
* **Summary File**: [dt0060...md](file:///home/sam/git/personal/lidar-roomscanner/references/datasheets/NUCLEO-IKS4A1/LSM6DSV16XTR/dt0060-exploiting-the-gyroscope-to-update-tilt-measurement-and-ecompass-stmicroelectronics.md)
* **Key Topics**: Fusing gyro angular velocity with accel/mag to eliminate dynamic motion tilt errors using complementary filtering.

### 7. [dt0064...pdf](file:///home/sam/git/personal/lidar-roomscanner/references/datasheets/NUCLEO-IKS4A1/LSM6DSV16XTR/dt0064-allan-variance-noise-analysis.pdf) — Design Tip DT0064: Allan Variance Noise Analysis
* **Document Type**: Noise Analysis Design Tip (6 pages)
* **Summary File**: [dt0064...md](file:///home/sam/git/personal/lidar-roomscanner/references/datasheets/NUCLEO-IKS4A1/LSM6DSV16XTR/dt0064-allan-variance-noise-analysis.md)
* **Key Topics**: Measuring Angle Random Walk (ARW), Rate Random Walk (RRW), Bias Instability ($B$), and Quantization Noise ($Q$) via Allan Variance log-log plots.

### 8. [dt0106...pdf](file:///home/sam/git/personal/lidar-roomscanner/references/datasheets/NUCLEO-IKS4A1/LSM6DSV16XTR/dt0106-gravity-subtraction-dead-reckoning.pdf) — Design Tip DT0106: Gravity Subtraction for Dead-Reckoning
* **Document Type**: Algorithm Design Tip (6 pages)
* **Summary File**: [dt0106...md](file:///home/sam/git/personal/lidar-roomscanner/references/datasheets/NUCLEO-IKS4A1/LSM6DSV16XTR/dt0106-gravity-subtraction-dead-reckoning.pdf)
* **Key Topics**: Subtracting body gravity vector $\vec{g}_{body}$, projecting linear acceleration to world frame $\vec{a}_{lin\_world}$, double integration for 3D displacement tracking.

### 9. [dt0155...pdf](file:///home/sam/git/personal/lidar-roomscanner/references/datasheets/NUCLEO-IKS4A1/LSM6DSV16XTR/dt0155-synchronizing-multiple-sensors-using-odrtriggered-mode-in-mems-devices-stmicroelectronics.pdf) — Design Tip DT0155: Multi-Sensor ODR Sync
* **Document Type**: Hardware Sync Design Tip (12 pages)
* **Summary File**: [dt0155...md](file:///home/sam/git/personal/lidar-roomscanner/references/datasheets/NUCLEO-IKS4A1/LSM6DSV16XTR/dt0155-synchronizing-multiple-sensors-using-odrtriggered-mode-in-mems-devices-stmicroelectronics.md)
* **Key Topics**: Synchronizing sample timestamps across IMU and magnetometer using hardware trigger pins and ODR mode.
