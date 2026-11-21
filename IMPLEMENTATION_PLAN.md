# Implementation Plan: Path Finder 6-DoF Trajectory Estimator
## XR Hackathon 2026 - Track 3: Path Finder

---

## 1. Executive Summary & Objective

The objective of **Track 3: Path Finder** is to construct a production-ready, offline Python software system that ingests degraded inertial measurement unit (IMU) telemetry and reconstructs a high-accuracy 6-DoF vehicle pose $[x, y, z, q_x, q_y, q_z, q_w]$ for every supplied IMU frame. Ground truth trajectories and mission waypoints remain completely hidden from the estimator. Trajectory accuracy is evaluated against hidden ground truth using the `evo` package under an $SE(3)$ Umeyama rigid alignment.

The software architecture is governed by strict computational boundaries:
- **Language & Architecture:** Pure Python 3 (Python 3.8+) with standalone entrypoint `main.py`.
- **Memory Consumption:** Peak algorithm heap $\le 8.0$ MB (excluding Python interpreter base RSS).
- **Execution Speed:** $\le 20.0$ ms total execution time per 1.0 second of IMU telemetry on a single CPU thread.
- **Robustness Tiers:**
  - *Tier 1 Checkpoints (30% weight):* Metric positional tracking through mission waypoints.
  - *Tier 2 Blind 5 s Blackout (20% weight):* Seamless extrapolation across complete sensor blackout in the final 5 seconds without dropping or appending rows.
  - *Tier 3 Dropouts (10% weight):* Resilience to missing sample gaps and duplicate timestamps.
  - *Execution Efficiency (40% weight):* Peak memory (20%) and runtime speed (20%).

---

## 2. System Architecture & Component Design

The Path Finder system is organized into a modular pipeline with strict separation of concerns:

```
+-----------------------------------------------------------------------------------------+
|                                PATH FINDER PIPELINE                                     |
+-----------------------------------------------------------------------------------------+
|                                                                                         |
|   +--------------------+     +------------------------+     +-----------------------+   |
|   | 1. Robust Parser   | --> | 2. Fault Sanitizer     | --> | 3. Tilt Observer      |   |
|   |    - EuRoC/Degrade |     |    - Spike clipping    |     |    - Observable grav  |   |
|   |    - Header detect |     |    - Stuck vectors     |     |    - Roll/pitch lock  |   |
|   |    - Int64 ns dt   |     |    - Zero-dt duplicate |     |    - Gyro correction  |   |
|   +--------------------+     +------------------------+     +-----------------------+   |
|                                                                         |               |
|                                                                         v               |
|   +--------------------+     +------------------------+     +-----------------------+   |
|   | 6. TUM Serializer  | <-- | 5. Drift Regularizer   | <-- | 4. Inertial Engine    |   |
|   |    - Exactly 8 col |     |    - Polynomial drift  |     |    - Inlined quat kin |   |
|   |    - 1:1 synch     |     |    - Velocity trend    |     |    - Rodrigues accel  |   |
|   |    - Unit quat     |     |    - Blackout extrap   |     |    - Gap sub-stepping |   |
|   +--------------------+     +------------------------+     +-----------------------+   |
|                                                                                         |
+-----------------------------------------------------------------------------------------+
```

### 2.1 Module 1: Robust Input Telemetry Parser
- **Responsibility:** Ingest arbitrary telemetry CSVs safely, parsing integer nanoseconds and 6-axis inertial data.
- **Header Autodetection:**
  - Inspects initial `#` comment lines for substring markers: `w_rs` / `gyro` indicates EuRoC standard (`timestamp, gyro_xyz, accel_xyz`). Substrings `a_rs` / `accel` indicate degrade script format (`timestamp, accel_xyz, gyro_xyz`).
  - Supports explicit manual override via `--channel-order {auto,euroc,degrade}`.
- **Nanosecond Integer Arithmetic:**
  - Raw epoch nanoseconds are parsed as 64-bit integers (`np.int64`).
  - Elapsed step duration is calculated as $\Delta t = (t_k - t_{k-1}) \times 10^{-9}$. This avoids the severe precision degradation that occurs when large epoch nanoseconds ($\approx 1.4 \times 10^{18}$) are cast directly to `float64`, which only has 53 bits of mantissa ($\approx 9 \times 10^{15}$).

### 2.2 Module 2: Fault Sanitizer & Dynamic Gatekeeper
- **Spike Rejection:** Accelerometer measurements exceeding $65.0\,\text{m/s}^2$ and gyroscope rates exceeding $25.0\,\text{rad/s}$ are screened and scaled back to the physical ceiling, preventing single impulse noise from injecting infinite kinetic energy into the velocity integral.
- **Stuck Vector Detection:** Consecutive samples with identical float bit-patterns are identified; they are prohibited from falsely triggering stationary bias estimation.
- **Zero-dt Duplicate Policy:** When $t_k - t_{k-1} \le 0$, the engine enters a zero-integration pass: the pose at frame $k-1$ is propagated unchanged to frame $k$. This maintains exact $1:1$ row cardinality with zero false displacement.

### 2.3 Module 3: Adaptive Gravity-Aided Tilt Observer
- **Observability Condition:** Under stationary or low-dynamic conditions ($|\|\mathbf{a}\| - 9.80665\| < 0.8\,\text{m/s}^2$), the measured specific force $\mathbf{f}_b$ reflects the Earth's gravitational reaction vector.
- **Tilt Error Feedback:** The normalized body acceleration $\hat{\mathbf{a}}_b$ is compared against the expected body-frame gravity vector $\hat{\mathbf{g}}_b = \mathbf{R}(\mathbf{q})^T \hat{\mathbf{z}}$ via vector cross product:
  $$\mathbf{e} = \hat{\mathbf{a}}_b \times \hat{\mathbf{g}}_b$$
- **Adaptive Gyro Adjustment:** The error $\mathbf{e}$ is scaled by an adaptive gain $k_p$ and fed directly into the angular rate integrator:
  $$\boldsymbol{\omega}_{\text{corr}} = \boldsymbol{\omega}_{\text{raw}} - \mathbf{b}_g + k_p \cdot \max\left(0, 1 - \frac{|\|\mathbf{a}\| - g|}{0.8}\right) \mathbf{e}$$
  This effectively locks roll and pitch angles to the vertical gravity vector, preventing angular tilt errors from corrupting gravity removal.

### 2.4 Module 4: Inlined Strapdown Kinematics Engine
- **Inlined Scalar Rodrigues Transformation:** Rather than constructing $3 \times 3$ NumPy transformation matrices per iteration, body-to-world vector rotation is performed in pure scalar arithmetic:
  $$\mathbf{t} = 2 (\mathbf{q}_v \times \mathbf{v})$$
  $$\mathbf{v}' = \mathbf{v} + q_w \mathbf{t} + (\mathbf{q}_v \times \mathbf{t})$$
- **Gravity Compensation & Acceleration:** $\mathbf{a}_w = \mathbf{v}' - [0, 0, 9.80665]^T$.
- **Gap-Aware Sub-Stepping:** When telemetry dropouts create gaps $\Delta t > 20\,\text{ms}$, the interval is subdivided into $N = \lceil \Delta t / 0.02 \rceil$ sub-steps. This ensures the numerical integration of angular rates and velocity remains within the linear stability region.

### 2.5 Module 5: Tier 2 Blind Blackout Extrapolator & Drift Regularizer
- **Blackout Detection:** A sustained sequence of $\ge 5$ consecutive zero-valued frames ($\|\boldsymbol{\omega}\| < 10^{-6}, \|\mathbf{f}\| < 10^{-6}$) triggers Tier 2 Blackout Mode.
- **Terminal State Capture:** The estimator captures terminal velocity $\mathbf{v}_{\text{term}}$ and terminal angular rate $\boldsymbol{\omega}_{\text{term}}$ averaged over the final $0.15\,\text{s}$ of valid telemetry prior to failure onset.
- **Bounded Aerodynamic Extrapolation:** The pose during the blackout is propagated forward with exponential velocity and rotation decay:
  $$\mathbf{v}(t) = \mathbf{v}_{\text{term}} \exp(-\lambda_v (t - t_{\text{onset}}))$$
  $$\boldsymbol{\omega}(t) = \boldsymbol{\omega}_{\text{term}} \exp(-\lambda_\omega (t - t_{\text{onset}}))$$
- **Offline Drift Regularization:** Unobservable double-integrated residual acceleration bias causes quadratic velocity and position drift ($0.5 b_a t^2$). In offline mode, the long-term velocity trend across the valid flight window is modeled with a linear polynomial fit and subtracted, preventing catastrophic quadratic position runaway.

### 2.6 Module 6: High-Speed TUM Serializer
- **Format Guarantee:** Formats poses into space-separated TUM standard:
  `timestamp tx ty tz qx qy qz qw`
- **Output Properties:** Strict unit quaternions, continuous sign convention ($\mathbf{q}_k \cdot \mathbf{q}_{k-1} \ge 0$), strictly finite floats, and 1:1 synchronization with input lines.

---

## 3. Implementation Phases & Milestones

```
+------------------------------------------------------------------------------------+
|                                 DEVELOPMENT TIMELINE                               |
+------------------------------------------------------------------------------------+
|                                                                                    |
| Phase 1: Foundation (Day 1 Morning)                                                |
| [X] Telemetry parser with EuRoC / Degrade auto-detection                           |
| [X] Nanosecond integer subtraction & precision verification                        |
| [X] TUM trajectory serializer with unit quaternion validation                      |
|                                                                                    |
| Phase 2: Fault Injection & Evaluation Harness (Day 1 Afternoon)                     |
| [X] Re-engineered degrade_imu.py (seed 42, noise ramp, spikes, gaps, blackout)     |
| [X] Synthetic 6-DoF trajectory generator with ground truth & checkpoints           |
| [X] SE(3) Umeyama alignment evaluation script (evaluate.py) matching evo_ape       |
|                                                                                    |
| Phase 3: Core Algorithmic Engineering (Day 1 Evening)                              |
| [X] Inlined scalar quaternion multiplication & Rodrigues vector rotation           |
| [X] Adaptive gravity tilt observer (Mahony/ESKF style attitude lock)               |
| [X] Zero-dt duplicate timestamp handling & gap sub-stepping                        |
| [X] Tier 2 Blind Tail Extrapolator with aerodynamic decay                          |
|                                                                                    |
| Phase 4: Optimization, Benchmarking & Freezing (Day 2 Morning)                     |
| [X] Single-threaded execution profiling: 1.90 ms per 1s IMU (Target: <= 20 ms)     |
| [X] Algorithm heap profiling: 0.782 MB (Target: <= 8.0 MB)                         |
| [X] 4-stage benchmark matrix execution (Baseline -> Robust -> Smoother -> Frozen)  |
| [X] Freeze hyperparameters and save results/benchmark_summary.csv & metadata.json  |
|                                                                                    |
| Phase 5: Verification & Packaging (Day 2 Afternoon)                                |
| [X] Comprehensive test suite (test_suite.py) passing all 7 quality gates           |
| [X] 9-Slide deck matching Submission_Final_forGithub.pptx template                 |
| [X] 2:30 Demo video script & storyboard matching Section 8                         |
| [X] Complete reproducibility documentation (README.md)                             |
|                                                                                    |
+------------------------------------------------------------------------------------+
```

---

## 4. Risk Management & Mitigation Matrix

| Potential Risk / Pitfall | Impact | Likelihood | Concrete Engineering Mitigation |
|---|---|---|---|
| **Column Order Inversion** (`degrade_imu.py` vs EuRoC) | Trajectory Divergence | High | Parser automatically checks headers for `w_rs` vs `a_rs`. Supports explicit `--channel-order` flag. Tested across both formats. |
| **Float Truncation on Epoch Timestamps** | Loss of dt precision | High | Large integer nanoseconds are never cast to float before subtraction. Relative $\Delta t = (t_k - t_{k-1}) \times 10^{-9}$ preserves sub-microsecond accuracy. |
| **Quadratic Error Runaway** ($0.5 b_a t^2$) | Extreme ATE (>200m) | High | Tilt observer locks pitch/roll to gravity; offline polynomial velocity trend removal cancels constant residual bias. |
| **Blackout Treated as Vehicle Stop** | Massive Tail Error | High | Zero tail is detected as sensor power/channel loss; terminal velocity and angular rate are extrapolated with drag decay. |
| **Missing Frames Breaking Cardinality** | Evaluator Rejection | Critical | Estimator strictly enforces 1 output row per 1 input row; gaps use elapsed $\Delta t$ without adding rows; duplicates repeat poses. |
| **Memory / Heap Exceeding 8 MB** | Scoring Disqualification | Medium | Scalar inlined math and preallocated NumPy arrays; memory traced via `tracemalloc` measures peak heap at **0.782 MB**. |
| **Runtime Exceeding 20 ms/s** | Scoring Disqualification | Medium | Zero per-frame function call overhead; inlined Rodrigues rotation achieves **1.90 ms/s** on a single thread. |

---

## 5. Verification & Acceptance Criteria

The final submission is accepted only when all automated gates pass:
1. `test_suite.py` returns `OK` across all unit, integration, and resource tests.
2. `python main.py input.csv output.txt` runs fully offline with zero internet access.
3. Output line count exactly matches input line count.
4. `evo_ape tum ground_truth.txt output.txt -a` validates finite, aligned trajectory with ATE RMSE $< 10.0\,\text{m}$.
5. Peak heap is measured under $8.0\,\text{MB}$ and compute time is measured under $20.0\,\text{ms}$ per IMU second.
