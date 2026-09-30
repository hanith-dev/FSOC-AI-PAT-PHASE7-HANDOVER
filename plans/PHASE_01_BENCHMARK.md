# Phase 1 — Deterministic Scientific Benchmark Engine Report

**Project:** SIH26169 — AI-Based Virtual Camera Tracking System for Coarse Alignment of Mobile FSOC Terminals  
**Role:** Senior Simulation Engineer, Senior Computer Vision Validation Engineer, Senior Control Systems Validation Engineer, Senior Experimental Software Engineer  
**Date:** September 2026  
**Phase:** Phase 1 — Deterministic Scientific Benchmark Engine  
**Status:** COMPLETE & EMPIRICALLY VALIDATED  

---

## 1. Executive Summary

In Phase 1, we designed, implemented, and empirically validated a **deterministic headless scientific benchmark engine** (Mode A) for the truth-isolated SIH26169 Free Space Optical Communication (FSOC) coarse pointing, acquisition, and tracking (PAT) system.

The benchmark engine is 100% decoupled from:
- React state rendering cycles
- Three.js WebGL canvas frame rates
- Browser `requestAnimationFrame` timing jitter
- WebSocket transport serialization and network latencies
- Display refresh rates (60 Hz / 144 Hz)
- Manual user interaction

The benchmark engine establishes the **authoritative scientific standard** for all future algorithmic comparisons and performance claims. The live web application (Mode B) remains completely operational for interactive mission control and visual demonstrations, but is officially barred from providing final benchmark figures.

In strict compliance with Phase 1 instructions:
- **Zero algorithms were modified or tuned** (no sub-pixel interpolation, no 6-state Kalman, no state machine rule alterations, no PID retuning).
- The engine measures the **Phase 0D truth-isolated baseline as-is**.
- Baseline tracking weaknesses—specifically catastrophic false-locks under optical dropouts and crossing distractors—have been honestly exposed and quantified.

---

## 2. Operating Modes Architecture

```
                                  +-------------------------------------------------------------+
                                  |                 FSOC-AI-PAT ARCHITECTURE                    |
                                  +-------------------------------------------------------------+
                                                                 |
                                +--------------------------------+--------------------------------+
                                |                                                                 |
                 [MODE A: SCIENTIFIC BENCHMARK]                                    [MODE B: LIVE MISSION CONTROL]
               Authoritative / Deterministic Engine                                Real-Time Interactive Presentation
                 - Pure Python, Headless                                            - React 19 + Three.js / R3F
                 - Fixed Integration dt = 0.02s (50 Hz)                             - Dynamic Browser rAF (60 Hz)
                 - PRNG Seed-Locked Disturbances                                    - FastAPI + WebSocket Channel
                 - Comprehensive Telemetry & 22 Metrics                             - Interactive UI & HUD Overlays
                 - Automated Publication-Grade Plots                                - Operator Parameter Overrides
                 - Bit-Exact Operational Reproducibility                            - Qualitative Demonstration Only
```

---

## 3. Fixed-Time Simulation & Seed Control

### 3.1 Deterministic Simulation Clock
Simulation time advances strictly by a discrete, configurable fixed timestep:
$$t_{k+1} = t_k + \Delta t, \quad \Delta t = 0.020\,\text{s} \ (50.0\,\text{Hz})$$

Wall-clock time (`time.perf_counter()`) is utilized **exclusively** for profiling computational latency (detector execution, estimator latency, total pipeline latency). Wall-clock measurements never influence simulation state progression or operational decisions.

### 3.2 Controlled Randomness
Every stochastic process consumes explicitly controlled seeds:
- Seed propagation: `random.seed(seed)` and `np.random.seed(seed)` initialized at the start of each benchmark execution.
- Stochastic components governed:
  - Additive Gaussian sensor noise ($\sigma = 35.0\,\text{DN}$)
  - Salt-and-pepper pixel noise
  - Platform vibration harmonic jitter
  - Atmospheric turbulence scintillation and local spatial warping
  - Random walk disturbance vectors

### 3.3 Bit-Exact Determinism Guarantee
Identical tuple `(Scenario, Algorithm Configuration, Seed)` guarantees bit-for-bit identical operational vectors:
$$\mathbf{S}_{\text{operational}}[k] = \left\{ u_{\text{cand}}[k], v_{\text{cand}}[k], \hat{\mathbf{x}}_{\text{kf}}[k], \text{State}_{\text{fsm}}[k], \psi_{\text{gimbal}}[k], \theta_{\text{gimbal}}[k], \dot{\psi}_{\text{cmd}}[k], \dot{\theta}_{\text{cmd}}[k], \epsilon_{\text{track}}[k] \right\}$$
Profiling fields (`detector_latency_ms`, `pipeline_latency_ms`, `fps`) naturally vary based on host CPU scheduling.

---

## 4. Benchmark Scenario Suite (S00 – S15)

All 16 standardized benchmark scenarios were implemented in `app/benchmark/scenarios.py`:

| ID | Scenario Name | Primary Stress Axis | Trajectory & Disturbance Configuration |
| :--- | :--- | :--- | :--- |
| **`S00_nominal`** | Nominal Static | Baseline calibration | Stationary target at $(0, 5, -100)\,\text{m}$; initial pan/tilt offset $(0.5^\circ, 0.3^\circ)$; clean atmosphere. |
| **`S01_linear`** | Linear Motion | Constant velocity | Transverse velocity $v = 5.0\,\text{m/s}$ across camera FOV; zero disturbances. |
| **`S02_sinusoidal`** | Sinusoidal Motion | Harmonic dynamics | Orbital oscillation: $x(t) = 12\sin(2\pi \cdot 0.15 t)$, $y(t) = 5 + 6\cos(2\pi \cdot 0.15 t)\,\text{m}$. |
| **`S03_high_speed`** | High-Speed Flyby | High angular rate | High-velocity transit ($v = 22.0\,\text{m/s}$ at $80\,\text{m}$); tests rate limits ($\pm 30^\circ/\text{s}$). |
| **`S04_acceleration`** | Constant Acceleration | Second-order kinematic | Constant acceleration $a = 2.0\,\text{m/s}^2$ ($v_0 = 0.5\,\text{m/s}$); tests Kalman acceleration lag. |
| **`S05_direction_change`** | Direction Reversal | Kinematic step | Target moves $+3.0\,\text{m/s}$ for $t < 5\,\text{s}$, instantaneously reverses to $-3.5\,\text{m/s}$ at $t = 5\,\text{s}$. |
| **`S06_vibration`** | Platform Vibration | Base buffeting | Stationary target, high-intensity platform vibration (intensity 65, 15 Hz harmonic jitter). |
| **`S07_sensor_noise`** | Sensor Noise & Blur | Low optical SNR | Gaussian noise (intensity 65) + blur (intensity 15); tests contour thresholding robustness. |
| **`S08_fading`** | Scintillation Fading | Optical fade | Severe atmospheric optical turbulence (intensity 65) + illumination variation (30). |
| **`S09_temporary_dropout`** | Temporary Dropout | Optical extinction | Target moving at $2.0\,\text{m/s}$; complete optical extinction between $t = 4.0\,\text{s}$ and $t = 5.5\,\text{s}$ ($1.5\,\text{s}$ blackout). |
| **`S10_fov_exit_reentry`** | FOV Exit & Reentry | Wide spatial loss | Target rapidly exits FOV ($> 4.5^\circ$), holds outside FOV, and re-enters at $t = 7.0\,\text{s}$. |
| **`S11_stationary_distractor`** | Stationary Distractor | Spatial ambiguity | Designated target moves at $2.0\,\text{m/s}$; stationary distractor beacon fixed at $1.5^\circ$ offset. |
| **`S12_moving_distractor`** | Moving Distractor | Dynamic association | **Temporally coherent moving distractor**: enters neighborhood, moves continuously, passes within $<1.0^\circ$ of beacon, crosses path, and remains visible across multiple frames. |
| **`S13_crossing_distractor`** | Crossing Distractor | High-ambiguity crossing | Target and distractor travel on collision course, crossing directly within $< 4\,\text{px}$ separation at $t = 5.0\,\text{s}$. |
| **`S14_brighter_distractor`** | Brighter Distractor | Photometric bias | Target intensity $1.5$; distractor appears nearby with $3\times$ intensity ($5.0$) and larger contour area. |
| **`S15_combined_stress`** | Combined Mission Stress | Full mission envelope | Harmonic motion + vibration (40) + turbulence (45) + noise (35) + $1.0\,\text{s}$ dropout + crossing distractor. |

---

## 5. Telemetry & Metric Schema

### 5.1 Frame-by-Frame Telemetry Schema (`telemetry.csv`)
Each simulation epoch records:
1. `simulation_time`: Deterministic simulation timestamp $t_k$ (seconds).
2. `ground_truth_target_x`, `ground_truth_target_y`: True projected 2D coordinates (**EVALUATION ONLY**).
3. `ground_truth_target_in_fov`, `ground_truth_target_active`: True target visibility flags (**EVALUATION ONLY**).
4. `detection_available`: Boolean flag indicating if detector extracted candidate(s).
5. `raw_measurement_x`, `raw_measurement_y`: Extracted candidate centroid coordinates.
6. `measurement_confidence`: Visual circularity proxy ($0.0 - 1.0$).
7. `selected_candidate_x`, `selected_candidate_y`: Selected candidate centroid coordinates.
8. `candidate_count`: Total valid visual contours detected in frame.
9. `estimated_x`, `estimated_y`: Kalman state position estimate $[\hat{x}, \hat{y}]^T$.
10. `estimated_vx`, `estimated_vy`: Kalman state velocity estimate $[\hat{v}_x, \hat{v}_y]^T$.
11. `tracking_state`: Operational FSM state (`SEARCHING`, `DETECTED`, `ACQUIRING`, `TRACKING`, `LOCKED`, `TARGET_LOST`).
12. `camera_pan`, `camera_tilt`: Actual gimbal orientation angles (deg).
13. `command_pan_rate`, `command_tilt_rate`: Output angular velocity commands from PID controller (deg/s).
14. `tracking_error_px`: True Euclidean pixel distance from optical boresight center $(320, 240)$ to true target (**EVALUATION ONLY**).
15. `tracking_error_deg`: True 3D pointing error between optical axis and target vector (**EVALUATION ONLY**).
16. `detector_latency_ms`: Execution time for OpenCV contour detection (ms).
17. `estimator_latency_ms`: Execution time for Kalman filter update (ms).
18. `pipeline_latency_ms`: Total frame processing latency (ms).
19. `controller_saturated`: Boolean flag indicating PID rate command reached output limit ($\pm 30.0^\circ/\text{s}$).
20. `evaluated_candidate_id`: Identity of ground-truth beacon matched to selected candidate (**EVALUATION ONLY**).

### 5.2 Mandatory Metric Rule: Error Paired with Coverage
Never report tracking error without accepted-measurement coverage. A tracking system that rejects difficult frames may deceptively report low error. Therefore, every benchmark artifact explicitly pairs:
$$\left[ \text{RMSE}_{\text{pointing}}, \ \text{Coverage}_{\text{accepted}} \right]$$

### 5.3 Evaluation-Only False-Lock Definition
- The operational tracker receives **zero ground-truth identity**.
- In post-processing, the evaluation engine matches the selected candidate against ground-truth beacon projections.
- A **False-Lock Event** occurs if the gimbal stabilizes on or tracks a candidate that is NOT the designated target for $\ge 0.5\,\text{s}$ ($\ge 25$ consecutive frames at $50\,\text{Hz}$).
- The evaluation engine logs `false_lock_event_count` and `false_lock_duration_s`.

---

## 6. Execution Command & CLI Interface

The benchmark engine provides a streamlined CLI interface:

```powershell
# Run nominal baseline calibration
$env:PYTHONPATH="backend"; .\.venv\Scripts\python -m app.benchmark --scenario S00_nominal --seed 42

# Run temporally coherent moving distractor
$env:PYTHONPATH="backend"; .\.venv\Scripts\python -m app.benchmark --scenario S12_moving_distractor --seed 42

# Run multi-seed statistical evaluation
$env:PYTHONPATH="backend"; .\.venv\Scripts\python -m app.benchmark --scenario S12_moving_distractor --seeds 42,43,44,45,46

# Execute the complete 16-scenario benchmark campaign
$env:PYTHONPATH="backend"; .\.venv\Scripts\python -m app.benchmark --all --seed 42
```

---

## 7. Verification & Test Results

### 7.1 Automated Test Execution
All 43 unit, integration, firewall, and benchmark tests passed:

```
============================= test session starts =============================
platform win32 -- Python 3.12.10, pytest-9.1.1, pluggy-1.6.0
rootdir: C:\Projects\SIH26169-WORKSPACE\FSOC-AI-PAT
plugins: anyio-4.15.1
collected 43 items

backend\tests\test_api.py ....                                           [  9%]
backend\tests\test_benchmark.py ......                                   [ 23%]
backend\tests\test_camera.py ....                                        [ 32%]
backend\tests\test_disturbances.py .......                               [ 48%]
backend\tests\test_experiments.py ....                                   [ 58%]
backend\tests\test_metrics.py ....                                       [ 67%]
backend\tests\test_pid.py ....                                           [ 76%]
backend\tests\test_tracking.py ...                                       [ 83%]
backend\tests\test_truth_firewall.py .......                             [100%]

======================= 43 passed, 2 warnings in 12.19s =======================
```

### 7.2 Determinism Verification
- `test_bitwise_determinism_nominal`: Ran `S00_nominal` twice with seed 42. Verified that all 19 operational simulation fields across 150 frames were **100% bit-exact identical**.
- `test_bitwise_determinism_with_disturbances`: Ran `S06_vibration` twice with seed 101. Verified identical operational trajectories despite heavy stochastic vibration.

### 7.3 Benchmark-Level Truth Firewall Verification
- `test_benchmark_truth_firewall_non_interference`: Modified evaluation target ID to `"BOGUS-TARGET-XYZ"`. Confirmed operational candidate selection, Kalman state, FSM state, and PID commands remained completely unaltered.

### 7.4 Frontend Zero-Regression Verification
- `npm run lint`: **0 errors** (33 pre-existing non-blocking warnings).
- `npm run build`: **Built successfully** in 814 ms (`dist/` ready).

---

## 8. Baseline Campaign Empirical Measurements (S00 – S15)

The complete benchmark matrix was executed using the Phase 0D truth-isolated baseline (`dt = 0.02s`, 50 Hz, seed 42):

| Scenario ID | Scenario Name | RMS Pointing Error (deg) | RMS Pixel Error (px) | Accepted Coverage (%) | False Locks (count) | False Lock Duration (s) | Actuator Saturation (%) | Actuator Jerk (deg/s³) | Pipeline Latency (ms) | Throughput (FPS) |
| :--- | :--- | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: |
| **`S00_nominal`** | Nominal Static | **0.3240** | 21.22 | **100.0%** | 0 | 0.00s | 0.00% | 39.87 | 0.96 | 1044.6 |
| **`S01_linear`** | Linear Motion | **1.4376** | 94.75 | **96.3%** | 0 | 0.00s | 0.17% | 7578.41 | 1.15 | 869.6 |
| **`S02_sinusoidal`** | Sinusoidal Motion | **4.1017** | 247.96 | **83.9%** | 0 | 0.00s | 0.13% | 7562.90 | 1.18 | 848.8 |
| **`S03_high_speed`** | High-Speed Flyby | **7.4377** | 425.82 | **64.8%** | 0 | 0.00s | 0.20% | 9601.68 | 0.98 | 1019.8 |
| **`S04_acceleration`** | Constant Accel | **4.1075** | 206.69 | **83.7%** | 0 | 0.00s | 0.17% | 7570.41 | 1.05 | 956.4 |
| **`S05_direction_change`** | Direction Reversal | **2.2138** | 88.23 | **92.2%** | 0 | 0.00s | 0.17% | 7559.75 | 1.27 | 788.6 |
| **`S06_vibration`** | Platform Vibration | **0.3645** | 23.88 | **100.0%** | 0 | 0.00s | 0.00% | 280.36 | 0.92 | 1093.0 |
| **`S07_sensor_noise`** | Sensor Noise & Blur | **0.5291** | 34.74 | **100.0%** | 0 | 0.00s | 0.00% | 80.23 | 1.28 | 781.6 |
| **`S08_fading`** | Scintillation Fading | **0.6861** | 45.06 | **100.0%** | 0 | 0.00s | 0.00% | 134.52 | 1.32 | 756.5 |
| **`S09_temporary_dropout`** | Temporary Dropout | **1.4634** | 61.76 | **83.7%** | 0 | 0.00s | 0.17% | 7574.78 | 1.00 | 1002.6 |
| **`S10_fov_exit_reentry`** | FOV Exit & Reentry | **1.4831** | 97.79 | **100.0%** | 0 | 0.00s | 0.00% | 69.00 | 1.23 | 813.5 |
| **`S11_stationary_distractor`** | Stationary Distractor | **0.8691** | 57.27 | **100.0%** | 0 | 0.00s | 0.00% | 153.71 | 0.95 | 1054.6 |
| **`S12_moving_distractor`** | Moving Distractor | **0.8532** | 56.17 | **100.0%** | 0 | 0.00s | 0.00% | 160.80 | 1.06 | 944.8 |
| **`S13_crossing_distractor`** | Crossing Distractor | **1.3819** | 57.24 | **96.7%** | 0 | 0.00s | 0.17% | 7553.72 | 1.01 | 987.6 |
| **`S14_brighter_distractor`** | Brighter Distractor | **0.6967** | 45.87 | **100.0%** | 0 | 0.00s | 0.00% | 101.78 | 1.08 | 925.5 |
| **`S15_combined_stress`** | Combined Stress | **6.0157** | 178.53 | **100.0%** | **2** | **12.62s** | 0.00% | 134.77 | 1.49 | 670.4 |

### 7.5 Multi-Seed Evaluation on Key Stress Scenarios

#### Scenario S12 (Temporally Coherent Moving Distractor) — 5 Seeds:
Across seeds 42, 43, 44, 45, 46:
- **RMS Pointing Error**: Identical $0.8532^\circ$ (deterministic trajectory)
- **Accepted Coverage**: $100.0\%$
- **False-Lock Count**: $0$
- **Actuator Jerk**: $160.799^\circ/\text{s}^3$
- **Average Throughput**: $849.5\,\text{FPS}$

#### Scenario S15 (Combined Aerospace Mission Stress) — 5 Seeds:
Across seeds 42, 43, 44, 45, 46:
- **RMS Pointing Error**: $6.0157^\circ$, $6.0197^\circ$, $6.0532^\circ$, $6.0517^\circ$, $6.0502^\circ$
- **False-Lock Events**: **Exactly 2 false-locks in all 5 runs**
- **False-Lock Duration**: $12.62\,\text{s}$, $12.60\,\text{s}$, $12.64\,\text{s}$, $12.62\,\text{s}$, $12.62\,\text{s}$
- **Accepted Coverage**: $100.0\%$, $100.0\%$, $100.0\%$, $100.0\%$, $99.9\%$

---

## 9. Failure Mode Discovery & Analysis

The headless benchmark engine performed exactly as intended: it revealed the critical weaknesses of the Phase 0D baseline:

### 1. Catastrophic False-Lock Under Optical Dropout with Clutter (`S15`)
- **Discovery**: In `S15_combined_stress`, the true target experiences an optical blackout between $t = 4.5\,\text{s}$ and $5.5\,\text{s}$. A crossing distractor beacon appears in the camera FOV during this window.
- **Root Cause**: Phase 0D's autonomous candidate selection falls back to largest-area / nearest-neighbor heuristic without Mahalanobis kinematic gating or track-before-inspect verification. The Kalman filter latch onto the crossing distractor, causing the gimbal to track the false beacon for **12.62 seconds** (90% of scenario duration).
- **Impact**: Demonstrates why advanced target association (Phase 3) is strictly required.

### 2. Derivative Kicks and Actuator Jerk Spikes (`S01`, `S04`, `S09`, `S13`)
- **Discovery**: During optical dropouts or beacon crossing events, actuator jerk spikes to over **$7500^\circ/\text{s}^3$**.
- **Root Cause**: When a target disappears and reappears (or when candidates merge and separate), pixel position error undergoes a discontinuous step. The basic PID controller has no derivative low-pass filter ($N$-filter) or rate-feedforward, transmitting massive derivative kicks directly to the gimbal.
- **Impact**: Proves the necessity of jerk-limited trajectory generation and filtered derivative actuation (Phase 5).

### 3. Lag Error Under Constant Acceleration (`S04`)
- **Discovery**: Under a modest $2.0\,\text{m/s}^2$ transverse acceleration, RMS pointing error balloons to **$4.1075^\circ$**, and coverage drops to $83.7\%$.
- **Root Cause**: The baseline Kalman filter models only constant velocity ($[\hat{x}, \hat{y}, \hat{v}_x, \hat{v}_y]^T$). It suffers from steady-state acceleration lag, driving the target toward the edge of the FOV.
- **Impact**: Confirms the necessity of a 6-state constant-acceleration kinematic estimator (Phase 4).

### 4. High-Speed Rate Saturation (`S03`)
- **Discovery**: At $v = 22\,\text{m/s}$, RMS pointing error reaches **$7.4377^\circ$** with coverage falling to $64.8\%$.
- **Root Cause**: Gimbal angular velocity commands repeatedly hit the actuator saturation clamp ($\pm 30.0^\circ/\text{s}$), leading to complete loss of visual contact.

---

## 10. Known Limitations & Architectural Boundary

1. **Reacquisition Capability**: Current FSM contains no autonomous reacquisition search patterns (spiral/raster scan). Target recovery following FOV exits remains **`NOT YET AVAILABLE`**.
2. **Sub-Pixel Centroiding**: OpenCV detector extracts integer bounding box centers ($u = x + w/2$). Quantization error floor is $\approx 0.5 - 1.5\,\text{px}$.
3. **Orbital Ephemeris**: Simulates local 3D Cartesian kinematics; full orbital mechanics (TLE / SGP4) is intentionally deferred.

---

## 11. Artifacts Generated

All benchmark outputs were persisted under `results/baseline_truth_isolated/`:
```
results/baseline_truth_isolated/
├── benchmark_campaign_summary.json
├── BENCH_S00_nominal_seed42_.../
│   ├── config.json
│   ├── metadata.json
│   ├── telemetry.csv
│   ├── summary.json
│   ├── events.json
│   └── plots/
│       ├── tracking_error.png
│       ├── gimbal_trajectory.png
│       ├── spatial_track.png
│       └── latency_profile.png
├── BENCH_S01_linear_seed42_.../
├── ...
└── BENCH_S15_combined_stress_seed42_.../
```
