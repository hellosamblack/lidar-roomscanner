# DT0064 Design Tip Summary

* **Document ID**: DT0064
* **Title**: Noise analysis and identification in MEMS sensors: Allan, Time, Hadamard, Overlapping, Modified, Total variance
* **Author**: Andrea Vitali
* **PDF File**: [dt0064...pdf](file:///home/sam/git/personal/lidar-roomscanner/references/datasheets/NUCLEO-IKS4A1/LSM6DSV16XTR/dt0064-allan-variance-noise-analysis.pdf)
* **Page Count**: 6 pages
* **Manufacturer**: STMicroelectronics

---

## Executive Summary
**DT0064** presents mathematical methods for analyzing, identifying, and quantifying noise characteristics in MEMS gyroscopes and accelerometers using **Allan Variance $\sigma^2(\tau)$** and its variants (Hadamard, Overlapping, Modified Allan variance).

---

## Noise Types & Allan Variance Log-Log Characteristics

```
  Log Allan Deviation σ(τ)
      \
       \ (Angle Random Walk / White Noise: slope -1/2)
        \          ____ (Bias Instability: slope 0)
         \________/
                  \
                   \ (Rate Random Walk / Drift: slope +1/2)
      ----------------------------------------------------> Log Averaging Time τ
```

| Noise Parameter | Slope on Log-Log Plot | Identification Point |
| :--- | :---: | :--- |
| **Quantization Noise ($Q$)** | $-1$ | Read $\sigma(\tau)$ at $\tau = \sqrt{3}$ |
| **Angle / Velocity Random Walk ($N$)** | $-1/2$ | Read $\sigma(\tau)$ at $\tau = 1\text{ s}$ |
| **Bias Instability ($B$)** | $0$ (Minimum) | Flat region minimum of $\sigma(\tau)$ curve divided by 0.664 |
| **Rate / Acceleration Random Walk ($K$)** | $+1/2$ | Read $\sigma(\tau)$ at $\tau = 3\text{ s}$ |
| **Ramp Drift ($R$)** | $+1$ | Read $\sigma(\tau)$ at $\tau = \sqrt{2}\text{ s}$ |

---

## Equations
* **Allan Variance Definition**:
  $$\sigma^2(\tau) = \frac{1}{2(N-1)} \sum_{i=1}^{N-1} (\bar{y}_{i+1} - \bar{y}_i)^2$$
  where $\tau = m \cdot T_0$ is the averaging time.

---

## LLM Routing Guide: When to Consult This File
Consult `DT0064` when:
* Characterizing MEMS gyro or accel noise parameters for Kalman filter tuning ($Q$ and $R$ covariance matrices).
* Writing Allan variance Python analysis scripts for stationary sensor dataset logs.
