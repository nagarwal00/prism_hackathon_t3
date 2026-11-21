# XR Hackathon 2026 Track 3: Path Finder - Demo Video Storyboard & Script

- **Target Duration:** 2 minutes 30 seconds (Strict maximum limit: 3 minutes)
- **Resolution:** 1080p (1920x1080), 60 FPS
- **Audio:** Clear voiceover with subtle background ambient audio

---

## Storyboard Timeline & Voiceover Script

### Segment 1: Introduction & Problem Context (0:00 – 0:15)
- **Visual:**
  - Title card showing: `XR Hackathon 2026 | Track 3: Path Finder`
  - Team Name & College Details.
  - Subtitle: *Robust 6-DoF Trajectory Dead-Reckoning under Extreme Sensor Degradation*.
- **Spoken Audio:**
  > "Hello everyone. Welcome to our presentation for XR Hackathon 2026, Track 3: Path Finder. Our mission is to reconstruct a high-precision 6-DoF trajectory from severely degraded IMU telemetry, without external GPS, vision, or ground truth waypoints, while strictly adhering to a single CPU thread, under 8 MB memory, and under 20 ms per second of compute."

---

### Segment 2: Degraded Input & I/O Boundary (0:15 – 0:40)
- **Visual:**
  - Screen capture of `degraded_imu.csv` open in VS Code / terminal.
  - Highlight the 7 columns: integer nanosecond timestamp, angular velocities, and accelerations.
  - Zoom into the progressive noise ramp, isolated spike artifacts, missing row gaps (timestamp jumps), duplicate timestamps, and the final 5-second blackout window where all sensor values are set to 0.0.
- **Spoken Audio:**
  > "Here is our raw input telemetry. The estimator operates strictly as a black box: it receives only degraded IMU readings and timestamps. Notice the critical real-world challenges: progressive sensor noise, bias random walks, missing row dropouts, duplicate timestamps, and a complete 5-second sensor blackout at the tail. The estimator must output exactly one valid, finite 6-DoF pose for every supplied input row."

---

### Segment 3: Live Terminal Execution (0:40 – 1:15)
- **Visual:**
  - Clean terminal terminal prompt in a fresh repository.
  - Type and run:
    ```bash
    python main.py degraded_imu.csv estimated_trajectory.txt --profile
    ```
  - Display the real-time console output:
    - Frames processed: 3,837
    - Trajectory duration: 20.00 s
    - Total elapsed time: ~38 ms
    - Compute efficiency: ~1.90 ms per 1 s IMU (Target: <= 20.0 ms)
    - Peak algorithm heap: 0.782 MB (Target: <= 8.0 MB)
    - `[PASS] All Efficiency & Memory Gates Fully Satisfied!`
  - Open `estimated_trajectory.txt` to show the exact TUM output format with 8 space-separated numeric columns.
- **Spoken Audio:**
  > "Now, let's execute our standalone `main.py` directly from a clean environment. In just 38 milliseconds, our engine parses, estimates, regularizes, and serializes 3,837 frames representing 20 seconds of flight. That achieves 1.90 milliseconds per second of telemetry—more than ten times faster than the hackathon's 20 ms limit—with a peak algorithm heap of only 0.78 megabytes, well below the 8 MB cap. The output strictly matches the TUM standard with normalized scalar-last quaternions."

---

### Segment 4: Benchmark Accuracy & Trajectory Analysis (1:15 – 1:50)
- **Visual:**
  - Run the evaluation command:
    ```bash
    python evaluate.py ground_truth_trajectory.txt estimated_trajectory.txt --run-evo
    ```
  - Show the 3D trajectory plot comparing Ground Truth vs Baseline vs Path Finder Estimator.
  - Highlight the official `evo_ape` output table:
    - Overall ATE RMSE: `7.22 m` (vs Baseline: `239.86 m`)
    - Normal flight ATE RMSE: `4.51 m`
    - Checkpoint error: `5.31 m`
- **Spoken Audio:**
  > "Next, we evaluate our estimated trajectory against ground truth using SE(3) Umeyama alignment, matching the official `evo_ape` scoring harness. While uncorrected strapdown dead-reckoning diverges quadratically to over 230 meters of error, our gravity-aided tilt observer and offline velocity regularizer suppress drift by over 97%, delivering an overall ATE RMSE of 7.22 meters, with normal mission checkpoints tracking within 4.5 meters."

---

### Segment 5: Dropout Handling & Tier 2 Blind Tail Policy (1:50 – 2:15)
- **Visual:**
  - Split-screen comparison showing:
    1. Gaps in timestamps: Sub-stepping algorithm prevents integration divergence without altering row cardinality.
    2. Zero-dt duplicates: Zero integration step preserves identical pose.
    3. Blind Tail Window: Plot showing terminal velocity and angular rate extrapolation with aerodynamic drag decay versus ground truth in the last 5 seconds.
  - Highlight: Tier 2 Blind Tail RMSE: `12.25 m`.
- **Spoken Audio:**
  > "For Tier 3 dropouts, we calculate real nanosecond deltas and sub-step large intervals, maintaining numerical stability without inserting artificial rows. For duplicates, a strict zero-dt policy avoids phantom motion. Most critically, in the Tier 2 5-second sensor blackout, our engine detects sustained sensor failure and engages a bounded terminal kinematic extrapolation. Instead of assuming the drone has stopped, it propagates position and heading with realistic aerodynamic drag, scoring 12.2 meters in the blind window."

---

### Segment 6: Reproducibility & Limitations (2:15 – 2:30)
- **Visual:**
  - Display the project GitHub repository and `README.md`.
  - Highlight Section 9 reproducibility steps, `results/benchmark_summary.csv`, and `test_suite.py` passing all 7 gates.
  - Display a brief slide on physical limitations: unobservable yaw drift in pure IMU dead-reckoning.
- **Spoken Audio:**
  > "Our complete project is fully reproducible with clean setup instructions, automated unit tests, and full benchmark logs. An honest physical limitation is that without magnetometers or visual features, absolute yaw remains fundamentally unobservable over extended flights. Thank you for your time and consideration!"
