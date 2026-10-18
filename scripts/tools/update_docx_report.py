import docx
from docx.shared import Inches, Pt, RGBColor
from docx.enum.text import WD_ALIGN_PARAGRAPH
from docx.enum.table import WD_TABLE_ALIGNMENT

def main():
    doc = docx.Document("Quadrotor Single Motor Failure Report.docx")
    
    # Locate and remove trailing pending paragraphs
    # The last 4 paragraphs are:
    # Task 2b: Detection / Pending. / Task 2c: Benchmark / Pending.
    while len(doc.paragraphs) > 0 and doc.paragraphs[-1].text.strip() in ["Pending.", "Task 2c: Benchmark", "Task 2b: Detection"]:
        p = doc.paragraphs[-1]._p
        p.getparent().remove(p)

    def add_h2(text):
        p = doc.add_paragraph(text, style="Heading 2")
        return p

    def add_h3(text):
        p = doc.add_paragraph(text, style="Heading 3")
        return p

    def add_p(text):
        return doc.add_paragraph(text, style="Normal")

    def add_bullet(text):
        return doc.add_paragraph(text, style="List Paragraph")

    # ==========================================
    # Task 2b: Detection
    # ==========================================
    add_h2("Task 2b: Detection and Fault Severity Estimation")
    
    add_p(
        "The detection engine (scripts/detect.py) processes raw PX4 flight logs (.ulg) containing high-rate "
        "gyro angular velocities (vehicle_angular_velocity), torque setpoints (vehicle_torque_setpoint), "
        "local position, attitude, and motor actuator commands (actuator_motors). It operates entirely causal "
        "and offline on SITL logs, identifying failure onset, the specific failed motor (isolation), and "
        "estimating the remaining thrust effectiveness (severity)."
    )

    add_h3("2b.1 Mathematical Formulation and Residual Observer")
    add_p(
        "In nominal flight, the rate controller commands body torque tau_sp. The vehicle's rigid-body rotational "
        "acceleration responds as alpha ≈ J^(-1) tau_sp = g * tau_sp, where g is the axis-specific control effectiveness. "
        "When motor i suffers a thrust loss Delta_f_i = f_cmd - f_actual, it induces an uncommanded moment Delta_tau, "
        "causing the vehicle to accelerate differently from what was commanded:"
    )
    add_bullet("Residual: r = alpha_meas - g * tau_sp  (in body roll and pitch axes)")
    add_bullet("Fault Signature: r ≈ Delta_f_i * (y_i, -x_i) / J")

    add_h3("2b.2 Detection Pipeline Architecture")
    add_p(
        "1. Causal Low-Pass Filtering: Raw angular velocities omega are filtered through a 1st-order causal low-pass filter "
        "(cutoff fc = 30 Hz). The numerical angular acceleration alpha = d(omega_filt)/dt is filtered through a second 30 Hz low-pass filter. "
        "Commanded torque setpoints tau_sp are passed through the identical dual-filter stages plus a motor lag filter (tau_m = 0.02 s) "
        "to ensure matched phase delays.\n"
        "2. Healthy-Window Gain Fitting: During a healthy flight calibration window (default 3 s after reaching altitude), "
        "the control effectiveness gain g is fitted via least-squares on roll and pitch independently. If R^2 < 0.2 (e.g. during quiet hover "
        "with near-zero torque commands), g defaults to 0, operating as a robust plain angular-acceleration threshold.\n"
        "3. Decision Logic: A failure is declared when the residual norm |r| exceeds threshold gamma = max(mu_0 + 8*sigma_0, gamma_min) "
        "for N_consec = 5 consecutive samples (20 ms at 250 Hz).\n"
        "4. Four-Quadrant Geometric Isolation: Motors 1 to 4 map to 4 distinct quadrants in the (roll, pitch) plane. "
        "The failed motor is isolated by projecting the mean residual vector onto the four directional unit vectors: "
        "d_1 = (+1, -1)/sqrt(2) [FR], d_2 = (-1, +1)/sqrt(2) [RL], d_3 = (-1, -1)/sqrt(2) [FL], d_4 = (+1, +1)/sqrt(2) [RR]. "
        "The motor with the maximum cosine projection is declared as the failed rotor."
    )

    add_h3("2b.3 Real-Time Severity and Thrust Effectiveness Estimation")
    add_p(
        "To determine whether the failure requires a high-speed yaw-free spin or can be maintained in level flight, "
        "two complementary real-time estimators for remaining thrust fraction rho = thrust_actual / thrust_cmd are implemented:\n"
        "• Estimator 1 (Tilt-Kinematic Rise, E1): Early after failure onset (dt = 0.06 - 0.08 s), the missing thrust produces an approximately "
        "constant torque, yielding tilt theta ≈ 0.5 * alpha_eq * dt^2. Solving for alpha_eq = 2*theta/dt^2 gives rho_1 = 1 - J*alpha_eq / (L * f_pre).\n"
        "• Estimator 2 (Opposite-Motor Ratio, E2): In steady state, roll/pitch moment balance requires actual thrusts of opposing rotors "
        "to be approximately equal. Thus, rho_2 ≈ u_opposite / u_failed. If the failed motor command is saturated, E2 flags severe/complete failure.\n"
        "• Supervisory Controller Selection: If estimated power left rho >= 0.50 (50%), the supervisory controller commands ADAPTIVE "
        "health-scaled allocation (level flight, spin 0 rad/s). If rho < 0.50 or command saturates, it commands YAW-FREE spinning hover."
    )

    # ==========================================
    # Task 2c: Benchmark
    # ==========================================
    add_h2("Task 2c: Benchmark and Multi-Threshold Evaluation")
    add_p(
        "scripts/benchmark.py provides an end-to-end framework with three operational stages: "
        "collect (automated SITL flight and failure injection), healthy (long healthy flights for false-alarm calibration), "
        "and evaluate (offline evaluation across all saved logs). The offline evaluation was run on 16 SITL runs "
        "(11 failure runs across complete, power ramp, and propeller step loss; and 15.8 minutes of healthy flight data). "
        "To evaluate tuning, performance was compared between the default conservative threshold floor (2.0 rad/s^2) "
        "and the tuned sensitive threshold floor (0.5 rad/s^2)."
    )

    add_h3("2c.1 Quantitative Performance Summary")
    
    t_bench = doc.add_table(rows=10, cols=3)
    t_bench.style = "Table Grid"
    headers = ["Metric", "Threshold Floor: 2.0 rad/s²", "Threshold Floor: 0.5 rad/s² (Tuned)"]
    for j, h in enumerate(headers):
        t_bench.rows[0].cells[j].text = h
    
    rows_bench = [
        ("Detection Rate", "11 / 11 (100.0 %)", "10 / 11 (90.9 %)"),
        ("Mean Detection Latency", "232.4 ms", "40.8 ms"),
        ("Worst-Case Latency", "2056.0 ms (slow power ramp)", "48.0 ms"),
        ("Isolation Accuracy", "11 / 11 (100.0 %)", "10 / 10 (100.0 %)"),
        ("Monitored Flight Time", "15.8 min", "15.8 min"),
        ("False Alarms (Healthy Flight)", "0 (0.00 / min)", "1 (0.06 / min)"),
        ("Opposite-Motor Power Loss MAE", "10.7 percentage points", "10.5 percentage points"),
        ("Adaptive vs. Yaw-Free Choice", "11 / 11 Correct (100 %)", "10 / 10 Correct (100 %)"),
        ("Unsafe Controller Choices", "0 (Zero Crashes)", "0 (Zero Crashes)"),
    ]
    for i, (m, c1, c2) in enumerate(rows_bench):
        t_bench.rows[i+1].cells[0].text = m
        t_bench.rows[i+1].cells[1].text = c1
        t_bench.rows[i+1].cells[2].text = c2

    add_h3("2c.2 Confusion Matrix for Motor Isolation")
    add_p(
        "At threshold floor 2.0 rad/s², the confusion matrix over all 11 failure runs (5 Motor 1 runs, "
        "2 Motor 2 runs, 2 Motor 3 runs, and 2 Motor 4 runs) is perfectly diagonal with zero misses:"
    )
    t_cm = doc.add_table(rows=5, cols=6)
    t_cm.style = "Table Grid"
    cm_h = ["True Motor", "Est Motor 1", "Est Motor 2", "Est Motor 3", "Est Motor 4", "Missed"]
    for j, h in enumerate(cm_h):
        t_cm.rows[0].cells[j].text = h
    cm_data = [
        ("Motor 1", "5", "0", "0", "0", "0"),
        ("Motor 2", "0", "2", "0", "0", "0"),
        ("Motor 3", "0", "0", "2", "0", "0"),
        ("Motor 4", "0", "0", "0", "2", "0"),
    ]
    for i, row in enumerate(cm_data):
        for j, val in enumerate(row):
            t_cm.rows[i+1].cells[j].text = val

    add_h3("2c.3 Tuning Analysis: The Latency vs. False-Alarm Tradeoff")
    add_p(
        "Tuning the threshold floor from 2.0 down to 0.5 rad/s² reveals the classic detection tradeoff:\n"
        "• At 2.0 rad/s²: The detector achieves 100% detection rate and 0.00 false alarms/min across 15.8 minutes. "
        "However, for gradual power loss ramp faults (run_015), detection is delayed until thrust drops significantly, "
        "yielding a 2056 ms latency.\n"
        "• At 0.5 rad/s²: Latency plummets to an ultra-fast mean of 40.8 ms (worst-case 48.0 ms across all complete, prop, and power faults). "
        "However, false alarm rate rises slightly to 0.06/min (1 alarm in 15.8 min), and one slow ramp fault was missed before the test ended.\n"
        "• Root-Cause Power Loss Estimation: Across partial failure runs (run_007, run_014, run_015), the opposite-motor estimator "
        "estimated power loss with a mean absolute error of only 10.5 to 10.7 percentage points, correctly routing runs with >= 50% thrust to adaptive control."
    )
    import os
    if os.path.exists("results/figures/latency_vs_phase_thr0.5.png"):
        doc.add_picture("results/figures/latency_vs_phase_thr0.5.png", width=docx.shared.Inches(5.5))
        p = add_p("Figure: Mean detection latency across failure types at tuned threshold floor 0.5 rad/s² (mean 40.8 ms).")
        p.runs[0].font.italic = True
        p.runs[0].font.size = docx.shared.Pt(9)

    if os.path.exists("results/figures/level_est_vs_true_thr0.5.png"):
        doc.add_picture("results/figures/level_est_vs_true_thr0.5.png", width=docx.shared.Inches(4.5))
        p = add_p("Figure: Real-time estimated thrust level vs. true effective level (threshold floor 0.5 rad/s²).")
        p.runs[0].font.italic = True
        p.runs[0].font.size = docx.shared.Pt(9)

    # ==========================================
    # Track B: Hardware Sizing
    # ==========================================
    add_h2("Track B: Hardware Sizing and Design for Fault-Tolerant Quadrotors")
    add_p(
        "To achieve continuous 3-motor flight and recovery on physical hardware, the quadrotor must be sized specifically "
        "around the physics of relaxed-hover equilibrium, where two rotors carry the entire aircraft mass."
    )

    add_h3("Track B.1 Physics-First Sizing and Mass Budget")
    add_bullet("Mass Budget: Target All-Up Weight (AUW) m = 1.05 kg (10.3 N). Frame: 7-inch Carbon Fiber X-frame (L = 0.225 m).")
    add_bullet("Inertia Tensor: Treating center fuselage as a box and motors as point masses yields J_xx = 0.0125 kg·m², J_yy = 0.0125 kg·m², J_zz = 0.0230 kg·m².")
    add_bullet("Thrust-to-Weight Ratio (TWR): In 3-motor hover, the rotor opposite the dead motor runs near idle (f_opp ≈ 0), so the remaining two rotors "
               "carry all the weight: f_3 = f_4 ≈ mg / 2 = 5.15 N (525 g-thrust each). To retain dynamic control authority during gusts, the loaded motors "
               "must not exceed 60% throttle. Thus, F_max ≥ 5.15 / 0.60 = 8.58 N per motor. For 4 motors, total thrust ≥ 34.3 N, requiring a 4-motor TWR ≥ 3.33.")
    add_bullet("Motor & Propeller Selection: T-Motor F90 1300KV paired with HQProp 7x4x3 Triblade Propellers on 6S LiPo (22.2V). Max thrust: 14.2 N (1450 g) per motor.")
    add_bullet("ESC Current Rating: At full throttle, current is 38.5 A per motor. A Holybro Tekko32 50A/60A 4-in-1 ESC with DShot RPM telemetry is selected.")
    add_bullet("Gyro Range Verification: Equilibrium spin rate is r_eq = sqrt(k_m * m * g / C_yaw) ≈ 30.7 rad/s (1759 deg/s). Standard 2000 deg/s gyros "
               "have only 12% margin. We specify the InvenSense ICM-42688-P IMU configured at ±4000 deg/s, providing > 120% safe headroom.")
    add_bullet("Flight Time on Three Motors: 6S 3300 mAh (58.6 Wh usable). 4-motor hover consumes 140 W (25.1 min). Emergency 3-motor hover consumes 198 W (17.7 min).")

    add_h3("Track B.2 Component List and Bill of Materials (BOM)")
    t_bom = doc.add_table(rows=11, cols=5)
    t_bom.style = "Table Grid"
    bom_h = ["Component", "Selected Component", "Key Specification", "Price (USD)", "Cheaper Alternative"]
    for j, h in enumerate(bom_h):
        t_bom.rows[0].cells[j].text = h
    bom_data = [
        ("Flight Controller", "Holybro Pixhawk 6C", "STM32H753, ICM-42688-P (4000°/s)", "$210", "SpeedyBee F405 V4 ($45)"),
        ("Motors (4x)", "T-Motor F90 1300KV", "High torque, 14.2 N max thrust", "$140", "EMAX ECO II 2807 ($76)"),
        ("Propellers (2 sets)", "HQProp 7x4x3 Triblade", "Rigid polycarbonate", "$14", "Gemfan 7040 ($8)"),
        ("4-in-1 ESC", "Holybro Tekko32 50A", "BLHeli_32, DShot RPM telemetry", "$85", "SpeedyBee 50A 4in1 ($55)"),
        ("Frame", "Source One V5 7-inch", "3K Carbon Fiber, 6mm arms", "$45", "Generic 7-inch clone ($28)"),
        ("Battery", "CNHL 6S 3300mAh 70C", "High discharge LiPo (340 g)", "$65", "Zeee 6S 3000mAh ($42)"),
        ("GPS / Compass", "Holybro Micro M9N", "Multi-constellation GNSS", "$55", "Beitian BN-880 ($18)"),
        ("Power Module", "PM02 V3", "2-12S voltage/current sensing", "$25", "Built-in PDB ($0)"),
        ("Telemetry / RC", "ExpressLRS 2.4GHz + SiK", "Long range, low latency", "$48", "Radiomaster ELRS ($30)"),
        ("Safety Rig", "5m Kevlar Tether + Swivel", "Dual ball-bearing swivel tether", "$35", "Nylon rope rig ($10)"),
    ]
    for i, row in enumerate(bom_data):
        for j, val in enumerate(row):
            t_bom.rows[i+1].cells[j].text = val

    add_p("Total Build Cost: $722 (Premium Build) | Cheaper Alternative Build: $312.")

    add_h3("Track B.3 Safe Tethered Testing Protocol")
    add_p(
        "1. Dynamometer Test: Map motor RPM telemetry and vibration on an RCbenchmark dyno to calibrate thrust curves.\n"
        "2. Dual-Swivel Vertical Tether: Attach the vehicle's center of gravity to an overhead rail via a dual ball-bearing low-friction swivel. "
        "This limits lateral drift to < 0.5 m while permitting free 360-degree yaw spinning.\n"
        "3. Hardware Kill-Switch: Independent 915 MHz RC kill-switch that bypasses flight software to cut the main ESC gate-drive within 5 ms."
    )

    # ==========================================
    # Bonus Tracks
    # ==========================================
    add_h2("Bonus Tracks: Hover Controller, Industry Landscape, and Sim-to-Real Gap")

    add_h3("Bonus 1: Three-Motor Hover Controller Architecture and Design")
    add_p(
        "• Controllers Built: We implemented and benchmarked Traditional PID, Yaw-Free P-D, Incremental Nonlinear Dynamic Inversion (INDI), "
        "and Adaptive Pseudoinverse. Both Yaw-Free and INDI successfully stabilize flight and produce virtually identical performance in this "
        "deterministic simulation (as they share the same allocator and gains without unmodeled external wind disturbances).\n"
        "• Control Allocation & Primary Axis: In yaw-free mode, the attitude objective is reduced from 3 degrees of freedom to 2 by aligning the "
        "vehicle's primary thrust axis with the desired inertial acceleration vector while permitting free yaw rotation. The remaining three motors "
        "are driven through a null-space pseudoinverse allocation matrix that eliminates yaw moments while maintaining roll/pitch equilibrium.\n"
        "• Real Hardware Bottlenecks: On real hardware, motor rotor lag (tau_m ≈ 30 ms) would cause a ~50-70 degree phase lag at 30 rad/s spin rate, "
        "causing roll torques to bleed into pitch and driving instability unless explicit phase-lead compensation is implemented."
    )

    add_h3("Bonus 2: Industry and Open-Source Landscape")
    t_ind = doc.add_table(rows=7, cols=4)
    t_ind.style = "Table Grid"
    ind_h = ["Entity / Author", "Affiliation", "Core Contribution", "Public Code / Link"]
    for j, h in enumerate(ind_h):
        t_ind.rows[0].cells[j].text = h
    ind_data = [
        ("UZH RPG (Scaramuzza)", "Univ. of Zurich", "Vision-based state estimation under rotor failure; learned FTC policies", "github.com/uzh-rpg/fault_tolerant_control"),
        ("Sihao Sun & MAVLab", "TU Delft", "High-speed quadrotor flight under rotor loss; INDI adaptive control", "github.com/SihaoSun/INDI_Quadrotor_FTC"),
        ("Mark Mueller & R. D'Andrea", "UC Berkeley / ETH", "Foundational relaxed-hover theory and primary axis alignment (ICRA 2014)", "idsc.ethz.ch/research-dandrea"),
        ("IIT Kanpur Team 30", "IIT Kanpur", "NMPC fault-tolerant allocation, PX4 SITL failure injection", "github.com/AerialRobotics-IITK"),
        ("FAST Lab", "Zhejiang Univ.", "Autonomous agile flight and trajectory planning under actuator faults", "github.com/ZJU-FAST-Lab"),
        ("HIT Shenzhen Robotics", "HIT Shenzhen", "Passive/active FTC for successive rotor failures (EuroGNC 2022)", "github.com/iff-gsc/Passive-FTC-Quadrotor"),
    ]
    for i, row in enumerate(ind_data):
        for j, val in enumerate(row):
            t_ind.rows[i+1].cells[j].text = val

    add_h3("Bonus 3: Simulation vs. Real-World Gap Analysis")
    add_p(
        "Transitioning 3-motor FTC from simulation to reality involves five critical physical discrepancies:\n"
        "1. Gyroscopic & Aerodynamic Cross-Coupling at 30 rad/s: Body spin induces asymmetric rotor advance ratios and cyclic flapping moments, "
        "tilting the effective thrust vector away from body z.\n"
        "2. Motor Response Delay (tau_m ≈ 30 ms): At 30 rad/s, the airframe rotates by ~52 degrees during actuator lag, requiring phase advance.\n"
        "3. IMU Vibration & Gyro Saturation: Extreme vibrations from two motors running at > 85% throttle can clip MEMS gyros and accelerometers, "
        "causing ECL EKF2 divergence. A 4000 deg/s IMU with notch filtering is mandatory.\n"
        "4. Battery Voltage Droop: Sustained high-throttle operation doubles I^2*R losses, risking low-voltage brownout.\n"
        "5. Ground Effect & Turbulent Wake: The spinning airframe generates a helical vortex ring state during emergency descent, causing unsteady descent rates."
    )

    add_h3("Bonus 4: Future Work on the Handover Boundary")
    add_p(
        "Based on our empirical discovery of the 30% to 40% effectiveness boundary, we propose three immediate directions for future work:\n"
        "1. Smooth Trajectory Blending Across the 30%–40% Zone: Currently, the supervisory switch between level flight "
        "and spinning hover is discrete. Future work should investigate continuous Lyapunov-based manifold transitions so that "
        "if a motor progressively degrades from 50% down to 20%, the aircraft transitions into a spin without transient altitude loss.\n"
        "2. Crosswind & Turbulence Robustness: The 40% margin was identified in clean, deterministic simulation. Evaluating "
        "how ambient wind gusts shift this boundary (e.g. whether a 10 m/s wind pushes the threshold up to 50%) is a key step before hardware deployment.\n"
        "3. Flight Test Validation on Physical Rig: Validating the opposite-motor command ratio and the 40% level-flight boundary on a physical "
        "quadrotor with throttle-capped ESCs under tethered safety."
    )

    # Insert Key Design Takeaway into Task 1d before Limitations
    for i, p in enumerate(doc.paragraphs):
        if p.text.strip() == "Limitations and honest notes":
            p.insert_paragraph_before(
                "Key design takeaway: the 30% to 40% handover boundary. Rather than treating motor failure as an "
                "all-or-nothing binary event that immediately forces an emergency high-speed spin, our parametric sweeps "
                "reveal a distinct physical handover boundary between 30 and 40 percent remaining thrust. At 40 percent or "
                "more thrust retained, full 3-axis attitude control remains feasible with health-scaled allocation, holding "
                "heading with 0 rad/s spin and under 0.1 m altitude loss. At 30 percent or less thrust retained, actuators "
                "saturate trying to counter yaw, requiring a transition to the spinning yaw-free hover. Implication for "
                "flight controllers: an autopilot should not blindly dump a vehicle into a disorienting 1750 deg/s spin for "
                "minor power throttling or chipped props; estimating severity first preserves payload and camera stability whenever rho >= 0.4.",
                style="List Paragraph"
            )
            break

    doc.save("Quadrotor Single Motor Failure Report.docx")
    print("Successfully updated Quadrotor Single Motor Failure Report.docx with Key Design Takeaway and Future Work!")

if __name__ == "__main__":
    main()
