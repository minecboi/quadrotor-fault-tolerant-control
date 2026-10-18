# Experimental Testing & Fault Recovery Post-Mortem

This directory contains standalone experimental test scripts and investigation logs exploring live fault-tolerant control (FTC), adaptive takeover, and ground liftoff under degraded motor conditions (50% power on Motor 1).

---

## 1. Directory Contents & Script Purpose

| Script | Purpose | Result / Findings |
| :--- | :--- | :--- |
| `main.py` | ROS2 offboard node running a parameterized Acados MPC (`ModeController`) with live takeover. | **Failed mid-air**: QP solver returned `ACADOS_MINSTEP` (status 4) upon abrupt parameter change. |
| `live_detect.py` | Real-time streaming fault detection node using IMU and MAVLink telemetry. | **Success**: Successfully triggered and confirmed Motor 1 fault within 0.20–0.23s. |
| `mode_controller.py` | Single unified Acados MPC formulation supporting nominal, adaptive, and yaw-free modes. | Solves offline with status 0, but fails under real-time parameter step changes in `SQP_RTI`. |
| `test_adaptive_ground_takeoff.py` | Test for direct ground liftoff with Motor 1 degraded to 50% power. | **Failed via PX4 mixer**: `CA_ROTOR0_CT` caused actuator saturation `[150, 150, 1000, 983]`. |
| `overdrive_test.py` / `test_overdrive_correct.py` | Testing motor speed capping in Gazebo (`SIM_GZ_EC_MAX1`). | Confirmed physical speed capping mechanics in Gazebo. |
| `closed_loop_test.py` / `yaw_damping_sweep.py` | Parameter sweeps and closed-loop validation scripts. | Standalone experimental validation. |

---

## 2. Experimental Post-Mortem & Failure Root Causes

### A. Mid-Air Adaptive Takeover Failure
- **Symptom**: Upon fault injection at 5m hover, `live_detect.py` confirmed 50% loss on Motor 1. At takeover, the drone lost altitude, causing supervisor panic and escalation to `YAW-FREE`, which ended in a tumble.
- **Root Cause**: The MPC optimization formulation uses `SQP_RTI` (Real-Time Iteration), which executes **only 1 QP iteration per time step** and relies on previous warm starts. When the effectiveness parameter jumped from $\rho = 1.0 \to 0.50$, the linear approximation at the current state caused the HPIPM interior-point solver to abort (`ACADOS_MINSTEP`, status 3). The controller fell back to holding the last command, resulting in uncompensated sink.

### B. Direct Ground Takeoff Failure via PX4 Allocation
- **Symptom**: Setting Motor 1 to 50% power (`SIM_GZ_EC_MAX1 = 707`) and updating PX4's allocation coefficient `CA_ROTOR0_CT = 3.25` resulted in zero liftoff.
- **Root Cause**: Changing `CA_ROTOR0_CT` distorted PX4's allocation matrix. To nullify roll and pitch moments, the allocator pinned Motors 1 and 2 at idle PWM (`150`), while pushing Motors 3 and 4 to maximum (`1000`). This diagonal saturation prevented the vehicle from generating uniform vertical lift.
- **Secondary Factor**: PX4 SITL defaults to `SIM_BAT_DRAIN = 60.0`, draining the simulated battery in 60 seconds and triggering an automatic failsafe landing if unsuppressed.

---

## 3. Conclusions & Recommendations
1. **Mathematical Feasibility**: Physical hover requires $4.9\,\text{N}$ per motor against vehicle weight ($mg \approx 20\,\text{N}$). A 50% motor capacity produces up to $8.45\,\text{N}$, proving that balanced hover at 50% power is physically feasible ($u_1 = 2 \cdot u_0 = 0.60, u_{2,3,4} = 0.30$).
2. **Implementation Strategy**: For production deployment, bypass PX4's internal mixer using direct motor commands (`ActuatorMotors`) or smooth the parameter transition over 100–200 ms to avoid `SQP_RTI` line-search failures.
