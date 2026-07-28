# AN5804 Application Note Summary

* **Document ID**: AN5804 (Rev 2)
* **Title**: LSM6DSV16X: machine learning core application note
* **PDF Files**: [an5804...pdf](file:///home/sam/git/personal/lidar-roomscanner/references/datasheets/NUCLEO-IKS4A1/LSM6DSV16XTR/an5804-lsm6dsv16x-machine-learning-core-stmicroelectronics.pdf) / [machineLearningCore.pdf](file:///home/sam/git/personal/lidar-roomscanner/references/datasheets/NUCLEO-IKS4A1/LSM6DSV16XTR/machineLearningCore.pdf)
* **Page Count**: 67 pages
* **Manufacturer**: STMicroelectronics

---

## Executive Summary
**AN5804** explains the configuration and programming of the **Machine Learning Core (MLC)** embedded inside the LSM6DSV16X. The MLC allows running up to **4 independent decision tree models** on-chip, classifying real-time motion states (e.g. stationary, walking, shaking, scanning) directly within the sensor to reduce host MCU power consumption.

---

## Technical Architecture

```
[ Sensor Inputs ] ---> [ Filters ] ---> [ Feature Computation ] ---> [ Decision Trees (1-4) ] ---> [ MLC Outputs / Interrupts ]
(Accel/Gyro/Qvar)     (HPF/LPF/IIR)      (Mean, Var, Energy...)        (If-Then-Else Nodes)
```

### 1. Inputs & ODR Selection
* Inputs: Accelerometer, Gyroscope, External Sensor (via Sensor Hub), Qvar.
* MLC Output Data Rate (`MLC_ODR`): 15 Hz, 30 Hz, 60 Hz, 120 Hz, or 240 Hz.

### 2. Feature Extraction Algorithms
The MLC automatically computes statistical features over configurable sliding windows:
* **Mean**, **Variance**, **Energy**, **Peak-to-Peak**
* **Zero-Crossing**, **Positive / Negative Zero-Crossing**
* **Peak Detector**, **Positive / Negative Peak Detector**
* **Minimum**, **Maximum**

### 3. Decision Tree Logic & Output Registers
* Up to 4 decision trees evaluated simultaneously.
* Results are written to output registers `MLC1_SRC` (`0x34`) .. `MLC4_SRC` (`0x37`) in embedded page space.
* Can generate host interrupt on any decision result state change.

---

## Configuration Workflow
1. Collect motion dataset using ST MEMS Studio / Unico-GUI.
2. Train decision tree model (e.g. in Python scikit-learn or Weka).
3. Export `.ucf` (Universal Configuration Format) register file.
4. Program configuration array into LSM6DSV16X embedded function memory pages via host MCU driver.

---

## LLM Routing Guide: When to Consult This File
Consult `AN5804` when:
* Programming or loading Decision Tree configurations (`.ucf` scripts) into LSM6DSV16X.
* Setting up on-chip AI classification for motion context awareness.
* Configuring MLC interrupts on `INT1` / `INT2`.
