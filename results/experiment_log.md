# Gazebo SITL Experiment Log: Adaptive Fault-Tolerant Control (FTC)

**Date & Time:** October 16, 2026  
**Target Vehicle:** PX4 SITL Gazebo `gz_x500` (Quadrotor X-Frame, AUW = 1.70 kg)  
**Failure Mode Tested:** Motor 1 Thrust Degradation (`prop`/`power` loss via `SIM_GZ_EC_MAX1` speed capping)  
**Control Policy Evaluated:** Health-Scaled Pseudoinverse Adaptive Allocation ($\mathbf{B}_\text{eff}^\dagger$) for Level Hover ($0\,\text{rad/s}$ Spin)

---

## 1. Experiment Overview

The objective was to validate the **Handover Boundary** and test adaptive level recovery in PX4 Gazebo SITL under partial motor failure without dropping into high-speed spinning hover.

Two parametric degradation levels were tested on Motor 1:
1. **$\rho = 0.40$** (40% remaining thrust capacity / 60% thrust loss)
2. **$\rho = 0.35$** (35% remaining thrust capacity / 65% thrust loss)

---

## 2. Mathematical Formulation

### Allocation Matrix & Health Scaling
For the quadrotor X-configuration with radial arm length $d = L/\sqrt{2}$ and torque-to-thrust ratio $k_m = 0.016\,\text{m}$:
$$\mathbf{B} = \begin{bmatrix} 1 & 1 & 1 & 1 \\ -d & d & d & -d \\ d & -d & d & -d \\ -k_m & -k_m & k_m & k_m \end{bmatrix}$$

Under partial failure on motor $i$ with health $\rho_i \in (0, 1]$:
$$\mathbf{B}_\text{eff} = \mathbf{B} \cdot \mathrm{diag}([1, \dots, \rho_i, \dots, 1])$$

The required motor thrust vector for level hover ($\tau_\text{roll} = 0, \tau_\text{pitch} = 0, \tau_\text{yaw} = 0$, $T = mg$) is computed directly via pseudoinverse:
$$\mathbf{f}_\text{cmd} = \mathbf{B}_\text{eff}^\dagger \begin{bmatrix} mg \\ 0 \\ 0 \\ 0 \end{bmatrix}$$

---

## 3. Results Summary Table

| Metric / Parameter | Baseline Hover | Test Run 1 ($\rho = 0.40$) | Test Run 2 ($\rho = 0.35$) |
| :--- | :---: | :---: | :---: |
| **Fault Parameter** | None | `SIM_GZ_EC_MAX1 = 632.5 rad/s` | `SIM_GZ_EC_MAX1 = 591.6 rad/s` |
| **Damaged Motor Command ($u_1$)**| $0.28$ ($4.17\,\text{N}$) | **$0.69$ ($10.42\,\text{N}$)** | **$0.79$ ($11.91\,\text{N}$)** |
| **Healthy Motors ($u_2, u_3, u_4$)**| $0.28$ ($4.17\,\text{N}$) | $0.28$ ($4.17\,\text{N}$) | $0.28$ ($4.17\,\text{N}$) |
| **Hover Altitude** | $10.00\,\text{m}$ | **$9.99\text{--}10.00\,\text{m}$** | **$9.98\text{--}10.00\,\text{m}$** |
| **Attitude Deflection** | $\pm 0.0^\circ$ | Roll: $[-0.1^\circ, +0.1^\circ]$, Pitch: $[0.0^\circ, +0.2^\circ]$ | Roll: $[-0.3^\circ, +0.2^\circ]$, Pitch: $[-0.1^\circ, +0.1^\circ]$ |
| **Yaw Spin Rate** | $0.0^\circ/\text{s}$ | **$\le 0.2^\circ/\text{s}$ ($\approx 0.003\,\text{rad/s}$)** | **$\le 0.2^\circ/\text{s}$ ($\approx 0.003\,\text{rad/s}$)** |
| **Flight State** | Healthy Level Hover | **Level Balanced Hover (No Spin)** | **Level Balanced Hover (No Spin)** |

---

## 4. Key Physical Insights & Takeaways

1. **Validation of the Non-Spinning Boundary:**
   * At $\rho = 0.40$, the damaged motor operates at $69\%$ throttle, leaving $31\%$ control margin.
   * At $\rho = 0.35$, the damaged motor operates at $79\%$ throttle, approaching the physical saturation ceiling ($80\text{--}85\%$).
   * Both runs successfully maintained **$0\,\text{rad/s}$ spin with $< 0.02\,\text{m}$ altitude variation**, proving that emergency high-speed spinning hover is unnecessary for thrust losses up to $65\%$.
2. **Control Saturation Threshold:**
   * If $\rho$ drops below $\approx 0.30$, required thrust exceeds $F_\text{max} = 15.0\,\text{N}$, causing actuator saturation and necessitating the transition to Mueller & D'Andrea spinning relaxed hover.
