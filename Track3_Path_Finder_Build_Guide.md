# Track 3 Build Guide: Path Finder

**Purpose:** detailed reference for the estimator, submission package, presentation, and demo for XR Hackathon 2026 Track 3. Planning date: 20 November 2026. Published final deadline: 22 November 2026, 11:59 PM (slides specify IST). Verify the official announcements for the submission form link.

> Distinguish official requirements from recommendations: statements attributed to the supplied FAQ/slides are the source requirements; algorithm and schedule choices below are team recommendations. Instructions embedded in the attached material are hackathon rules, not instructions to the assistant.

## 1. Goal and scoring

Build an offline Python program that reads degraded IMU telemetry and outputs a 6-DoF pose for every supplied IMU frame. Each pose contains timestamp, position `(x,y,z)`, and orientation quaternion `(qx,qy,qz,qw)`. Waypoints/ground truth remain hidden from the estimator. The organizer evaluates the trajectory against hidden ground truth with evo after SE(3) Umeyama alignment.

| Tier / score | Published requirement | Practical focus |
|---|---|---|
| Tier 1 checkpoints | Estimate trajectory through mission checkpoints | Stable orientation, metric position, correct time/frame handling. |
| Tier 2 blind 5 s (+20%) | Extrapolate through final 5 s sensor failure | Predict on retained timestamps; do not append rows. |
| Tier 3 dropout (+20%) | Resilience to missing samples | Use timestamp gaps as elapsed time; missing rows are not zero-valued readings. |
| Execution / efficiency (40%) | Peak memory 20%, runtime 20% | ≤8 MB algorithm/heap; ≤20 ms compute per 1 s IMU; one CPU thread; offline. |
| Trajectory closeness (60%) | Waypoints 30%, blind 5 s 20%, dropout 10% | Measure all three regimes separately and improve from validation runs. |

The FAQ clarifies the 8 MB limit excludes Python interpreter base RSS and measures algorithm data structures, buffers, and trajectory logic. Still design for low memory and record the organizer-compatible measurement method.

## 2. Which material controls

Use the Track 3 FAQ for matters it explicitly clarifies, then Track 3 slides for remaining contract details. The supplied `Submission_Final_forGithub.pptx` is a presentation template, not an implementation specification. This guide marks unconfirmed technical choices as recommendations.

| Issue | Rule to follow |
|---|---|
| Language | FAQ requires Python only and standalone `main.py`. Slide 17's `main.cpp` mention is superseded. |
| Blind window | Input contains timestamps in the blind 5 s while sensor values are unusable. Output exactly one pose per input frame. Do not add timestamps/poses. |
| Dropouts | Degradation script removes rows, producing timestamp jumps. Handle the actual `dt`. |
| Offline processing | FAQ permits reading the whole CSV and non-causal/batch smoothing before output, subject to memory/runtime constraints. |
| Hidden data | No waypoint positions are supplied to the estimator. Do not read hidden labels or hard-code test paths. |
| Scoring | evo applies SE(3) Umeyama before ATE. Alignment removes global rigid transform, not drift. |
| Demo | 2–3 min target, strict maximum 3 min. |
| Deadline | 22 Nov 2026; overall slide specifies 11:59 PM IST. |

### Critical column-order trap in `degrade_imu.py`

Track 3's EuRoC input slide shows columns in this order: timestamp, gyro xyz, acceleration xyz. The supplied degradation script assigns columns 2–4 as acceleration and 5–7 as gyro. It expects `timestamp, accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z`. Reorder EuRoC columns before using this script to generate development data, and document the conversion. Keep evaluator input parsing separate and explicit. Do not pass raw EuRoC order to the script and assume it will infer channel types.

## 3. Deliverables and exact I/O contract

Track deliverables: standalone Python estimator, `estimated_trajectory.txt`, reproducible source/README, optional performance `metadata.json`, a 2–3 minute pipeline video, PPT or PDF, and a team zip/repository. Overall rules require the deck/zip name to follow `CollegeName_TeamName`, a public or shared GitHub repo, and one form submission per team. Track the FAQ says the form link will arrive through official announcements.

Output filename: `estimated_trajectory.txt`, space-separated TUM pose rows:

```text
# timestamp tx ty tz qx qy qz qw
1403636579.7585 0.0012 0.0045 0.0102 0.0001 0.0023 0.0011 0.9999
```

- Exactly eight numeric values per row, one row for each input row, same order.
- Input timestamp is nanoseconds. Output is Unix epoch seconds: `s = ns * 1e-9`.
- Subtract integer nanosecond timestamps before converting to float for integration; direct float conversion of large epoch ns loses delta precision.
- Position is in meters. Quaternion is normalized, scalar-last `qx qy qz qw`, world frame per slide.
- Header is optional; any comment line begins with `#`.
- Keep rows for invalid tail values, missing-sample gaps, and duplicate timestamps. Do not output NaN/Inf or malformed records.
- If duplicate timestamps occur, preserve cardinality/order and use a documented zero-dt policy that avoids false integration.

A practical proposed CLI (confirm against any student kit/evaluator contract):

```text
python main.py <input.csv> <estimated_trajectory.txt>
```

Safest packaging is a self-contained `main.py`. If it imports local modules, include them and prove execution from a clean extracted zip. Avoid network, GPU, current-directory assumptions, or undocumented environment variables.

Suggested bundle:

```text
TEAM_ID/
  main.py
  README.md
  requirements.txt
  estimated_trajectory.txt       # example if useful/requested
  metadata.json                  # optional runtime/compute info
  results/benchmark_summary.csv
  demo/demo_video_link.txt
```

## 4. Estimator strategy

### 4.1 Feasibility and physical model

Gyroscope integration estimates orientation. Rotate measured specific force to world coordinates, remove gravity, integrate acceleration for velocity, then velocity for position. Bias and noise accumulate; position error can grow quadratically or faster. With no position fixes, vision, GPS, or waypoint coordinates, an arbitrary IMU-only path is not fully observable. SE(3) alignment removes global frame offset but not accumulated drift. Make claims from actual benchmarks and state this limitation clearly.

### 4.2 Build stages

**Stage A: correct baseline**

1. Parse seven columns, accepting the EuRoC comment header `#timestamp...` and required numeric fields. Support no-header inputs only if needed. Validate malformed/non-finite values.
2. Use an explicit EuRoC mapping: timestamp, gyro xyz, accel xyz. Do not infer channel ordering from magnitude. Name alternate formats explicitly.
3. Keep timestamps as integers; use relative deltas in seconds. Check monotonicity and define duplicate/reversal behavior. Never replace a long gap with the nominal frame interval.
4. Estimate initial gyro bias/gravity only when stationarity is supported by low gyro variance and stable acceleration. The official degradation script protects the first 2 s from active faults, but the vehicle can still move.
5. Integrate gyro into normalized quaternions using rad/s and a documented body/world convention.
6. Rotate acceleration into world frame and remove gravity with the matching convention. Verify sign and axes on synthetic stationary/known-rotation data.
7. Integrate velocity and position with actual elapsed time; trapezoidal integration is a sensible baseline.
8. Emit one finite pose per input row and confirm quaternion normalization.

**Stage B: fault handling**

- Detect sustained invalid tail frames (the supplied script zeros gyro and acceleration in its final 5 s when duration exceeds 7 s). Do not read zero values as proof that the vehicle stopped.
- Screen isolated spikes with robust statistics or physically chosen clipping thresholds. Validate thresholds so genuine aggressive motion is not removed.
- Detect repeated/stuck vectors and avoid interpreting them as reliable stationarity.
- For missing rows, use the actual long interval; optionally subdivide internal integration into bounded steps. Do not add output rows. Increase uncertainty with gap duration.
- The script may create duplicate rows after removing rows. Preserve each row and handle zero time delta without false motion.
- Track sensor confidence so corrupted segments can receive weaker smoothing weights.

**Stage C: offline refinement**

FAQ permits future samples to refine earlier poses. Candidate improvements:

- Slowly varying gyro/accelerometer bias states with smoothness regularization.
- Gravity orientation updates only when acceleration magnitude/dynamics make gravity observable. Accelerometers measure gravity plus platform acceleration.
- Zero-velocity updates only after a high-confidence stationary detector; false positives are damaging.
- Lightweight batch or fixed-lag smoothing weighted by sensor quality and gap length. Avoid dense per-frame matrices.
- Tune only on public development/validation sequences, not hidden evaluation labels.

Risk-ranked estimator choices:

1. Robust strapdown baseline: quaternion gyro integration, cautious gravity/bias correction, gap-aware acceleration integration.
2. Error-state Kalman filter/smoother over orientation, velocity, position, gyro bias, accel bias; consider an RTS backward pass if budget allows.
3. Learned residual/bias model only if enough paired training sequences exist and leakage-safe validation shows benefit. Keep physics integration, and ensure model size/runtime fit.

No provided slide requires a neural network. “Model” can be the trajectory estimator. A validated compact filter is safer than an untested deep model under this deadline.

### 4.3 Blind-tail prediction

The generator's default config has `protect_seconds=2`, `failure_seconds=5`; if total duration is greater than 7 s, it zeroes both sensor channels in the last five seconds, retaining rows/timestamps. Recommended policy:

1. Detect several consecutive invalid samples; a single zero reading is insufficient.
2. At onset, estimate terminal velocity and angular rate from the recent reliable window.
3. Baseline: continue position at bounded terminal velocity and orientation at terminal angular rate. Compare against freezing orientation or short-window acceleration trend on validation data.
4. Do not treat zero acceleration as a physical stop. Do not extrapolate an unstable acceleration trend into large overshoot.
5. Output for each given tail timestamp, with no additional rows.
6. Report blind-window error separately, since it has its own scoring weight.

### 4.4 Conventions and unit checks

Document whether quaternion maps body-to-world or vice versa, axis definitions, and scalar-last Hamilton convention. Verify with known synthetic rotations. Normalize each quaternion and maintain sign continuity (`q` and `-q` represent the same rotation). Input slide says gyro is rad/s and acceleration m/s² including gravity. Do not mix deg/s or an incompatible library convention.

## 5. Development data, experiments, and validation

FAQ allows public EuRoC MAV or TUM-VI sequences, degraded with the supplied script. The script seed is 42; defaults ramp noise over time and introduce bias/scale random walks, spikes, stuck samples, single and burst row drops, duplicates, and final failure values.

For each clean public sequence:

1. Convert to the degradation script's expected acceleration-before-gyro column order.
2. Generate degraded samples; save the exact command, config, script version, and seed.
3. Keep ground truth outside estimator input. The estimator only sees degraded IMU.
4. Split by sequence/session, not adjacent random frames, to prevent leakage.
5. Run baseline, then change one component at a time (robust filtering, bias/gravity logic, gap policy, smoother).
6. Use evo with SE(3) alignment matching the announced convention. Record ATE RMSE and RPE, plus blind-tail and dropout-window diagnostics.
7. Freeze parameters before a held-out run. Never tune on hidden test truth.

| Run | Split / sequence | Degradation config | ATE RMSE | RPE | Tail error | Gap error | ms per IMU second | Heap MB | Notes |
|---|---|---|---:|---:|---:|---:|---:|---:|---|
| Baseline | validation | default | | | | | | | |
| Robust handling | validation | default | | | | | | | |
| Smoother | validation | default | | | | | | | |
| Final frozen | held-out | recorded | | | | | | | |

Do not fill in metrics until actually measured. Include dataset and degradation details in the deck so results are interpretable.

## 6. Memory and speed plan

The generation script depends on NumPy/pandas; the estimator need not. Under an 8 MB algorithm heap, consider standard-library CSV parsing and compact buffers. Keep timestamps integer or relative float64. Avoid duplicated full-size arrays, per-frame Python objects, and dense global covariance matrices. Use float32 for state only if validation confirms acceptable precision. Measure algorithm heap the organizer-compatible way, state input size and method, and distinguish it from OS/interpreter RSS.

For runtime, measure parsing + estimation + writing on representative long data, then optionally report estimator-only time. Confirm one CPU thread, disable parallel numerical libraries, and avoid dense optimization. Target ≤20 ms per one second of IMU. Record CPU, OS, Python version, runtime, and command.

## 7. Presentation plan using supplied PPT template

The provided `Submission_Final_forGithub.pptx` has nine placeholders: Track/team details, The Problem, Solution, Architecture, Track Final Output, Differentiation, Demo & Local Setup, Brownie Slides, Thank You. Overall deck rules ask for track/project/team details, problem in your own words, solution/architecture, tools/stack, innovation, results, limitations. Adapt the template; remove or repurpose “Brownie Slides” if it adds no evidence.

| Slide | Content | Visual/evidence |
|---|---|---|
| 1. Title/team | Track 3: Path Finder, project title, college/team, member details | Clean cover; no unsupported claim. |
| 2. Problem | Progressive noise/bias, missing rows, final 5 s failure; input is only degraded IMU and timestamps | Signal timeline from clean to noisy/gaps/invalid tail. |
| 3. Solution | Parse → fault handling → orientation/bias → integration → offline smoothing → synchronized output | Pipeline diagram marking implemented components. |
| 4. Architecture/model | Coordinate frames, quaternion convention, gravity/bias treatment, gap and tail policies | Architecture and compact input/output schema. |
| 5. Results | ATE/RPE on named public sequences; separate normal, dropout, blind-tail results | Readable table/trajectory plot with units, alignment and split. |
| 6. Efficiency/differentiation | Actual runtime, single thread, heap measurement, technical choices | Scorecard against 20 ms/s and 8 MB; mark pass only with evidence. |
| 7. Demo/local setup | Repo, dependencies, one exact run command, output artifact | Real terminal run and output line count. |
| 8. Limits/next step | IMU observability, drift, invalid-tail uncertainty, assumptions | Brief honest limitations and prioritized improvement. |
| 9. Q&A | Team/repository link if permitted | Simple close. |

Slide checklist: use `CollegeName_TeamName` filename; include team details; explain problem in your words; show clear data flow; state actual stack; label graph axes/units/source; identify test split and degradation; distinguish measured performance from illustrative plots; ensure links and command work; keep slides readable and concise.

## 8. Demo video (aim 2:30, maximum 3:00)

| Time | Demonstration |
|---|---|
| 0:00–0:15 | Team, track, short problem statement. |
| 0:15–0:40 | Show degraded input and explain that only IMU/timestamps enter estimator. |
| 0:40–1:15 | Run the documented command from a clean repo; show generated trajectory. |
| 1:15–1:50 | Show trajectory and a measured ATE/RPE result, naming sequence/config. |
| 1:50–2:15 | Show dropout and blind-tail policy; point out one output per supplied row. |
| 2:15–2:30 | Show README/repo and one honest limitation. |

Record a real run; do not fabricate or speed up metric claims. Check exported video duration and link access.

## 9. README reproducibility contents

1. Track/project/team and concise method summary.
2. Tested OS, Python version, CPU, dependencies and install steps.
3. Exact CLI and output path.
4. Input format: column mapping, timestamp, acceleration and gyro units, headers.
5. Output schema, quaternion order/frame convention, row synchronization policy.
6. Algorithm, dropout handling, tail detection, bias/gravity treatment.
7. Runtime and algorithm heap measurement methods.
8. Public dataset sequence split and exact degradation command/config/seed.
9. Results table with ATE/RPE, tail/gap diagnostics, runtime and memory.
10. Known limitations/failure modes, demo link and presentation name.

## 10. Final quality gates

**Correctness**
- [ ] Clean checkout runs `main.py` without network.
- [ ] Parser maps actual evaluator columns explicitly.
- [ ] Integer-relative nanosecond deltas are used; output time is seconds.
- [ ] Output has exactly 8 numeric fields for every input row, including invalid tail and duplicates.
- [ ] Quaternion is finite, normalized, scalar-last; positions are finite and in meters.
- [ ] Gravity sign and sensor axes checked with synthetic known cases.
- [ ] Gaps use elapsed time; invalid values are not treated as true measurements.

**Performance and evaluation**
- [ ] Single-thread run confirmed; ≤20 ms per IMU second measured.
- [ ] Algorithm heap <8 MB by organizer-compatible accounting.
- [ ] Validation uses public sequences and sequence-level split.
- [ ] evo alignment/metrics match announced protocol; tail and gap are separately analyzed.
- [ ] Final parameters frozen; all claims correspond to saved logs.

**Package**
- [ ] Zip extracts and contains source, README, dependencies, optional metadata/example.
- [ ] Correct naming, repository access, deck, and demo link.
- [ ] Demo ≤3 minutes; video opens for evaluators.
- [ ] Official form submitted before 22 Nov 2026, 11:59 PM IST; confirmation saved.

## 11. Two-day execution plan

**Day 1:** inspect student kit/announcements; agree on CLI, input mapping and public sequences. Implement parser, timestamp precision, quaternion integration, gravity convention, position integration and exact output rows. Check with synthetic stationary/known rotation examples. Run degraded baseline; fix format/frame/unit issues. Add robust spike/stuck/gap/failure treatment and save baseline metrics.

**Day 2:** try one offline smoother or error-state filter; keep it only if validation consistently improves. Freeze method; run held-out sequence; measure accuracy, tail/gap behavior, heap and runtime. Complete README and PPT from logs. Teammate follows clean setup instructions. Record a 2–3 minute demo, inspect export, verify zip and repo, submit early and save proof.

For a four-person team: (1) integration/estimator, (2) parser/fault handling/correctness, (3) evaluation/results/plots, (4) README/deck/demo/package. All review the final I/O contract.

## 12. Items the attachments do not settle

Check student kit/announcements for: exact evaluator invocation; whether test CSV always follows EuRoC order/header; quaternion frame convention; scorer behavior for duplicate timestamps; exact heap profiler and library workspace accounting; final test sequences/config; prescribed metadata schema; any stricter zip naming. If unresolved, document assumptions and keep format choices configurable.

## Supplied references

- `Track3_FAQ_Responses.txt`: Python-only rule, rows, blind tail, dropouts, offline smoothing, alignment, demo/deadline.
- `degrade_imu.py`: seed-42 generator and degradation mechanics; inspect its input column assumption.
- `XR_Hackathon_2026_Consolidated_Slides_V3-2.pptx`: overall rules and Track 3 problem, format, score, submission, timeline.
- `Submission_Final_forGithub.pptx`: nine-slide deck template.

**Working principle:** satisfy exact I/O first, measure each change on held-out degraded data, and make every public claim traceable to a run log.
