# DT0131 Design Tip Summary

* **Document ID**: DT0131
* **Title**: Digital magnetometer and e-Compass: efficient design tips
* **Author**: Mauro Scandiuzzo
* **PDF File**: [dt0131...pdf](file:///home/sam/git/personal/lidar-roomscanner/references/datasheets/NUCLEO-IKS4A1/LIS2MDL/dt0131-digital-magnetometer-and-ecompass-efficient-design-tips--stmicroelectronics (1).pdf)
* **Page Count**: 6 pages
* **Manufacturer**: STMicroelectronics

---

## Executive Summary
**DT0131** provides hardware design, PCB layout, trace routing, decoupling, and magnetic immunity guidelines for integrating ST 3-axis digital magnetometers (LIS2MDL / IIS2MDC) into e-Compass applications.

---

## Key Hardware & Layout Recommendations

### 1. External Components & Decoupling
* **Supply Decoupling**: Place a $100\text{ nF}$ ceramic decoupling capacitor as close as possible to the $V_{dd}$ pin.
* **Internal Reservoir Capacitor ($C1$)**: LIS2MDL requires a **$220\text{ nF}$** ceramic capacitor connected between pin 10 ($C1$) and GND to supply internal set/reset current pulses.

### 2. PCB Layout & Magnetic Immunity
* **Keep Out Zones**: Keep magnetometers away from ferromagnetics (iron, nickel, steel screws/connectors, battery contacts) and high-current traces (DC-DC inductors, motor drivers, power rails).
* **Biot-Savart Effect**: High trace currents create parasitic magnetic fields:
  $$B = \frac{\mu_0 I}{2 \pi r}$$
  Maintain max distance $r$ from high current traces ($I > 100\text{ mA}$).

### 3. Software & System Design Tips
* Enable **Block Data Update (BDU)** in `CFG_REG_C[4]` to prevent reading asynchronous byte corruption.
* Use internal offset registers (`OFFSET_X/Y/Z_REG`) for hardware-level hard-iron offset removal.
* Use Built-In Self-Test mode (`CFG_REG_C[1]`) during self-diagnostics to verify sensor functionality.

---

## LLM Routing Guide: When to Consult This File
Consult `DT0131` when:
* Designing PCB schematics and board layouts featuring LIS2MDL or IIS2MDC.
* Debugging magnetic noise, measurement drift, or parasitic offset issues caused by board components.
* Verifying recommended external capacitor values ($100\text{ nF}$ $V_{dd}$, $220\text{ nF}$ $C1$).
