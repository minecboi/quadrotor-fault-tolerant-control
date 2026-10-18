# Quadrotor Single Motor Failure — Interview & Viva Defense Guide

> **Quick Reference Sheet for Evaluator Interviews & Technical Defense**  
> *Prepared for the Mock PS: Quadrotor Single Motor Failure Submission*

---

## 1. Executive Summary & Core Pitch (The 60-Second Answer)

> *"Our work establishes a complete, honest pipeline across 6-DOF simulation and PX4 SITL. In simulation, we benchmarked Traditional PID, Yaw-Free, INDI, and Adaptive controllers across 48 scenarios, discovering a **physical handover boundary between 30% and 40% remaining thrust** where level flight with 0 rad/s spin is preserved. In PX4 SITL, we implemented an automated MAVLink injector across 4 failure modes and an offline causal residual detector. By tuning the threshold floor from 2.0 to 0.5 rad/s², we cut detection latency from **232.4 ms to 40.8 ms** with 100% isolation accuracy, while an opposite-motor thrust ratio estimator achieved an MAE of 10.5 percentage points with 100% safe controller routing."*

---

## 2. Six Critical Questions & How to Defend Them

### Q1: How do you derive the ~30.7 rad/s (~1759°/s) equilibrium spin rate?
**Answer:**
- In normal 4-motor flight, two CW and two CCW rotors cancel net yaw torque ($\sum Q_i = 0$).
- When one rotor dies, the opposing rotor must throttle down to balance roll/pitch moments ($f_{\text{opp}} \approx 0$). The remaining two rotors rotate in the **same direction**, imparting an uncancelled motor reaction torque:
  $$\tau_{\text{react}} = k_m (f_1 + f_2) \approx k_m \cdot m g$$
  where $k_m$ is the rotor torque-to-thrust ratio ($k_m \approx 0.016\,\text{m}$ for 7-inch props).
- This torque accelerates the airframe in yaw until balanced by aerodynamic yaw drag $Q_{\text{aero}} = C_{\text{yaw}} \cdot r^2$:
  $$k_m \cdot m g = C_{\text{yaw}} \cdot r_{\text{eq}}^2 \implies r_{\text{eq}} = \sqrt{\frac{k_m m g}{C_{\text{yaw}}}}$$
- For our 1.05 kg vehicle ($m g = 10.3\,\text{N}$, $C_{\text{yaw}} \approx 1.8 \times 10^{-4}\,\text{N}\cdot\text{m}/(\text{rad/s})^2$):
  $$r_{\text{eq}} = \sqrt{\frac{0.016 \times 10.3}{1.8 \times 10^{-4}}} \approx 30.7\,\text{rad/s} \quad (1759^\circ/\text{s})$$
- **Defense takeaway:** This is a physical invariant of the airframe, not an artifact of control gains.

---

### Q2: Why did Yaw-Free and INDI perform virtually identically in your Task 1d benchmark?
**Answer (Demonstrating Academic Honesty):**
- In our Task 1d 6-DOF simulation benchmark, both Yaw-Free and INDI achieved near-identical altitude loss ($\approx 0.74\,\text{m}$) and recovery times ($\approx 1.25\,\text{s}$).
- **The root cause:** Both controllers utilize the exact same 3-motor nullspace allocation matrix ($\mathbf{B}_3^\dagger$) and roll/pitch PD tracking gains. 
- In our deterministic, nominal simulation plant without unmodeled wind gusts, aerodynamic flapping, or actuator noise, INDI's acceleration feedback residual $\Delta u = \mathbf{B}_3^\dagger (\ddot{x}_{\text{des}} - \ddot{x}_{\text{meas}})$ evaluates to near-zero steady-state correction.
- **Sim-to-real reality:** In real flight with turbulent crosswinds or model parameter mismatch (e.g. unknown payload shift), INDI's sensor-based acceleration feedback would provide superior disturbance rejection compared to pure Yaw-Free PD. We report this honestly rather than inventing artificial differences.

---

### Q3: What is your "30%–40% Handover Boundary" finding, and why does it matter?
**Answer:**
- Standard industry FTC often treats motor failure as a binary event: failure detected $\to$ immediately force emergency high-speed spinning hover.
- Our parametric severity sweeps revealed a clear physical transition:
  - **At $\ge 40\%$ remaining thrust:** The healthy rotors still have sufficient control margin to balance yaw and roll/pitch moments without hitting motor saturation ($u_{\text{max}} = 1.0$). The vehicle maintains **level, non-spinning flight ($0\,\text{rad/s}$ yaw rate)** with $< 0.1\,\text{m}$ altitude loss.
  - **At $\le 30\%$ remaining thrust:** The healthy motors saturate trying to counter yaw reaction torque. If yaw tracking is enforced, attitude diverges. The controller **must sacrifice yaw** and transition to spinning hover.
- **Flight controller implication:** An autopilot should estimate fault severity first. Blindly spinning the drone for a minor throttle cap or chipped blade disorients human pilots, destroys gimbal stabilization, and risks optical sensor divergence.

---

### Q4: Why did tilt kinematics overestimate fault severity in PX4 SITL, and why is opposite-motor ratio better?
**Answer:**
- **Tilt Kinematics (E1):** Assumes an unconstrained ballistic roll/pitch acceleration in the first $60\text{--}80\,\text{ms}$ ($\theta \approx \frac{1}{2} \alpha_{\text{eq}} \Delta t^2$). 
  - *The flaw in SITL:* PX4's high-gain inner rate loops (running at 250 Hz) react within $60\text{--}80\,\text{ms}$, producing counter-torques that distort the kinematic parabola and inflate the estimated missing thrust.
- **Opposite-Motor Ratio (E2):** In steady-state hover, moment equilibrium across the vehicle diagonal requires the actual thrusts of opposing rotors to match ($f_{\text{opp}} \approx f_{\text{failed}}$).
  - Therefore, the remaining thrust fraction is accurately reflected by the commanded throttle ratio: $\rho \approx u_{\text{opp}} / u_{\text{failed}}$.
  - In our 11 failure runs, E2 achieved an MAE of **10.5 to 10.7 percentage points**, reliably routing $\ge 50\%$ failures to level flight and $< 50\%$ failures to spinning hover with **100% correct classification**.

---

### Q5: How does your residual detector handle the latency vs. false alarm tradeoff?
**Answer:**
- The detector calculates residual $\mathbf{r} = \boldsymbol{\alpha}_{\text{meas}} - g \cdot \boldsymbol{\tau}_{\text{sp}}$ and checks for 5 consecutive samples exceeding $\gamma = \max(\mu_0 + 8\sigma_0, \gamma_{\text{min}})$.
- **At $\gamma_{\text{min}} = 2.0\,\text{rad/s}^2$ (Conservative):**
  - Detection rate: 100% (11/11).
  - False alarms: **0.00 / min** over 15.8 minutes of healthy flight.
  - Mean latency: 232.4 ms (slow power ramp took 2056 ms to cross the large threshold).
- **At $\gamma_{\text{min}} = 0.5\,\text{rad/s}^2$ (Tuned):**
  - Detection rate: 90.9% (10/11; 1 slow ramp ended before triggering).
  - Mean latency: **40.8 ms** (fastest: 36.0 ms; worst-case across step/complete faults: 48.0 ms).
  - False alarms: **0.06 / min** (only 1 false alarm in 15.8 minutes).
- **Isolation Accuracy:** **100% (10/10 and 11/11)** via four-quadrant projection onto diagonal unit vectors $[\pm 1, \pm 1] / \sqrt{2}$.

---

### Q6: If you deployed this on physical hardware tomorrow, what are the top 3 failure points?
**Answer (Sim-to-Real Gap Analysis):**
1. **Actuator Phase Lag at 30 rad/s:** Real brushless motors have an electrical/mechanical time constant $\tau_m \approx 30\,\text{ms}$. At $r = 30\,\text{rad/s}$, the airframe rotates by $\Delta \psi = r \cdot \tau_m \approx 52^\circ$ before the thrust command takes effect. This cross-couples roll into pitch, destabilizing the vehicle unless explicit phase-lead compensation $\theta_{\text{cmd}} = \theta_{\text{desired}} + 52^\circ$ is applied.
2. **Gyro Dynamic Range Clipping:** At $30.7\,\text{rad/s}$ ($1759^\circ/\text{s}$), standard commercial $\pm 2000^\circ/\text{s}$ gyros have only 12% margin and clip under turbulent yaw spikes. A $\pm 4000^\circ/\text{s}$ sensor (e.g. ICM-42688-P) is strictly required.
3. **IMU Vibration & EKF Divergence:** Running two diagonal motors at $> 85\%$ throttle introduces high-amplitude acoustic and motor-pole vibrations. If not filtered with dual hardware notch filters, the PX4 ECL EKF2 estimator will reject accelerometer data and diverge into failsafe disarm.

---

## 3. Quick Table of Key Empirical Results

| Metric | Task 1d (6-DOF Simulation) | Task 2b/2c (PX4 SITL - Conservative) | Task 2c (PX4 SITL - Tuned) |
| :--- | :--- | :--- | :--- |
| **Tested Scenarios** | 48 runs (4 ctrl $\times$ 3 speeds $\times$ 4 motors) | 16 SITL logs (11 failures, 15.8 min healthy) | 16 SITL logs |
| **Detection Latency** | Instantaneous ($\Delta t = 0$) | 232.4 ms mean | **40.8 ms mean** |
| **Isolation Accuracy** | N/A | **100.0% (11/11)** | **100.0% (10/10)** |
| **False Alarm Rate** | N/A | **0.00 / min** | **0.06 / min** (1 in 15.8 min) |
| **Severity Estimation MAE** | N/A | 10.7 percentage points | **10.5 percentage points** |
| **Altitude Loss (Complete)** | 0.74 m (Yaw-Free / INDI) | Uncontrolled in SITL (detection only) | Uncontrolled in SITL (detection only) |
| **Handover Threshold** | **$\ge 40\%$ thrust $\to$ Level Flight** | Threshold $\ge 50\% \to$ Adaptive | Threshold $\ge 50\% \to$ Adaptive |
| **Supervisory Decisions** | 100% stable recovery | **100% correct controller choice** | **100% correct controller choice** |

---

## 4. Hardware Sizing Checklist (Track B)

- **Frame:** 7-inch Carbon Fiber X-frame ($L = 0.225\,\text{m}$, AUW = $1.05\,\text{kg}$).
- **Motor/Prop:** T-Motor F90 1300KV + HQProp 7x4x3 on 6S LiPo ($14.2\,\text{N}$ max thrust per rotor).
- **TWR Requirement:** In 3-motor hover, two rotors carry the entire weight ($5.15\,\text{N}$ each). To keep them under 60% throttle for gust margin, $F_{\text{max}} \ge 8.58\,\text{N}$, yielding a 4-motor $\text{TWR} \ge 3.33$ (our design achieves **5.52 TWR**).
- **Flight Controller:** Holybro Pixhawk 6C (STM32H753 + ICM-42688-P $\pm 4000^\circ/\text{s}$).
- **Bill of Materials:** **$722** Premium Build (Pixhawk 6C + T-Motor) | **$312** Budget Build (SpeedyBee F405 + EMAX ECO II).
- **Tethered Safety Rig:** 5m overhead rail with dual ball-bearing low-friction swivel and hardware 915 MHz RF kill switch.
