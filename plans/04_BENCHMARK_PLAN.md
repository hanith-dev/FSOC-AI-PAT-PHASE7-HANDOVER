# Scientific Benchmark & Verification Specification

**Project:** SIH26169 — AI-Based Virtual Camera Tracking System for Coarse Alignment of Mobile FSOC Terminals  
**Role:** Senior Aerospace Software Architect, Computer Vision Engineer, Control Systems Engineer & Experimental Validation Engineer  
**Document Version:** 1.0.0 (Phase 0C Architecture Freeze)  
**Status:** APPROVED BENCHMARK SPECIFICATION  

---

## 1. Objective & Methodological Principles

This specification defines the rigorous scientific benchmarking framework for evaluating coarse Pointing, Acquisition, and Tracking (PAT) performance in `FSOC-AI-PAT/`.

### 1.1 Core Verification Principles
1. **Deterministic Mode A Execution:** All official benchmark metrics must be collected using the headless, seed-locked Mode A simulation framework running at a fixed integration timestep ($\Delta t = 0.01667\,\text{s}$, $60\,\text{Hz}$ nominal or $\Delta t = 0.010\,\text{s}$, $100\,\text{Hz}$).
2. **Separation of Evaluation from Control:** Ground truth is ingested exclusively by the post-run evaluation engine to measure true errors. The control loop remains 100% blind to ground truth.
3. **No Fabricated Data or Premature Claims:** All performance fields for future phases are strictly designated:  
   **`NOT YET MEASURED`**.
4. **Permanent Benchmark Provenance:** Every benchmark execution artifact must permanently bundle the algorithm configuration, scenario definition, seed, git commit SHA, complete telemetry time-series, metric summary, and failure event log.

---

## 2. Benchmark Scenario Suite (S00 – S15)

The benchmark matrix covers 16 standardized operational scenarios spanning nominal flight, aggressive dynamics, severe atmospheric fading, optical dropouts, and multi-target distractor clutter.

| ID | Scenario Name | Primary Stress Axis | Dynamics & Physical Parameters | Disturbances & Optical Environment | Pass/Fail Criteria |
| :--- | :--- | :--- | :--- | :--- | :--- |
| **`S00`** | `nominal_static` | Baseline calibration & sensor noise | Stationary target at $(0, 0, 100)\,\text{m}$. Gimbal boresight aligned within $1.0^\circ$. | Clean atmosphere ($C_n^2 = 10^{-16}\,\text{m}^{-2/3}$), Gaussian read noise $\sigma = 1.0\,\text{DN}$. | Lock retention $\ge 98\%$, steady-state error $< 0.10^\circ$. |
| **`S01`** | `linear_motion` | Constant velocity tracking | Transverse linear velocity $v = 5.0\,\text{m/s}$ across camera FOV. | Mild scintillation, zero platform vibration. | Acquisition $< 1.5\,\text{s}$, RMS error $< 0.15^\circ$. |
| **`S02`** | `sinusoidal_motion` | Periodic harmonic dynamics | Harmonic oscillation: $x(t) = 15\sin(2\pi \cdot 0.2 t)\,\text{m}$, $y(t) = 8\cos(2\pi \cdot 0.2 t)\,\text{m}$. | Moderate scintillation ($C_n^2 = 10^{-14}\,\text{m}^{-2/3}$), slight beam wander. | Zero loss of track, RMS error $< 0.25^\circ$. |
| **`S03`** | `high_speed_flyby` | High angular velocity | Close-range high-speed transit ($v = 25.0\,\text{m/s}$, angular rate up to $14^\circ/\text{s}$). | Low turbulence, zero platform vibration. | Actuator rate saturation $\le 15\%$, lock retained. |
| **`S04`** | `constant_acceleration` | Non-zero second derivative | Initial velocity $2.0\,\text{m/s}$, constant transverse acceleration $a = 3.5\,\text{m/s}^2$. | Mild turbulence. | 6-state estimator converges, RMS error $< 0.20^\circ$. |
| **`S05`** | `sudden_direction_change`| Kinematic step maneuver | Target executes instantaneous $90^\circ$ velocity vector reversal ($v = 8\,\text{m/s} \rightarrow -8\,\text{m/s}$). | Nominal atmosphere. | Overshoot $< 0.80^\circ$, recovery to `LOCK` $< 1.0\,\text{s}$. |
| **`S06`** | `platform_vibration` | Mechanical base buffeting | UAV rotor harmonic vibration modes ($15\,\text{Hz}, 30\,\text{Hz}, 60\,\text{Hz}$, amplitude up to $0.4^\circ$). | Stationary target, clean atmosphere. | Derivative filter eliminates chatter; RMS error $< 0.30^\circ$. |
| **`S07`** | `sensor_noise_clutter` | Low SNR / Background noise | Zero target motion. Strong Gaussian sensor noise + Poisson photon shot noise ($\text{SNR} \approx 6\,\text{dB}$). | High background ambient luminance. | Zero false locks, localization error $< 0.10\,\text{px}$. |
| **`S08`** | `scintillation_fading` | Gamma-Gamma atmospheric fade | Severe atmospheric optical turbulence ($C_n^2 = 5 \times 10^{-13}\,\text{m}^{-2/3}$, Rytov variance $\sigma_R^2 = 1.8$). | Deep intensity fades exceeding $20\,\text{dB}$ attenuation. | Adaptive $R$ prevents covariance collapse; lock maintained. |
| **`S09`** | `temporary_dropout` | Full signal blackout | Complete target optical extinction for duration $\Delta t_{\text{drop}} = 1.0\,\text{s}$ (60 frames). | Target continues constant-velocity motion during blackout. | FSM transitions `LOCK` $\rightarrow$ `COAST` $\rightarrow$ `TRACK` upon reappearance in $< 0.3\,\text{s}$. |
| **`S10`** | `target_fov_exit` | Wide spatial reacquisition | Target maneuvers completely out of camera FOV ($> 4.5^\circ$ offset) during occlusion. | Target resumes stationary state outside FOV. | FSM triggers spiral reacquisition; target recovered in $< 3.5\,\text{s}$. |
| **`S11`** | `stationary_distractor` | Spatial ambiguity rejection | Designated target moves at $4\,\text{m/s}$. Stationary false beacon present at fixed $1.5^\circ$ offset. | Equal intensity and color signature. | Zero identity switches, 100% target lock retention. |
| **`S12`** | `moving_distractor` | Parallel trajectory clutter | Target moves at $4\,\text{m/s}$. Second distractor beacon moves along parallel trajectory at $1.0^\circ$ separation. | Identical PSF profile. | Zero false locks, Mahalanobis gate isolates correct track. |
| **`S13`** | `crossing_distractor` | High-ambiguity intersection | Two optical beacons travel on collision course, crossing within $0.05^\circ$ ($< 4\,\text{px}$) separation. | Identical visual appearance. | Velocity/acceleration continuity maintains correct track identity. |
| **`S14`** | `brighter_distractor` | Photometric bias rejection | Target beacon intensity $I_0$. Distractor beacon appears at $3 \times I_0$ brightness nearby. | High ambient noise. | System rejects brighter distractor in favor of kinematic continuity. |
| **`S15`** | `combined_stress` | Full aerospace mission stress | Multi-axis sinusoidal motion + UAV vibration + Gamma-Gamma fading + $0.5\,\text{s}$ dropout + crossing distractor. | $C_n^2 = 2 \times 10^{-13}\,\text{m}^{-2/3}$, vibration active. | System survives full mission profile without terminal failure. |

---

## 3. Comprehensive Metric Definitions & Mathematical Formulations

The evaluation engine computes 22 rigorous performance metrics across six operational domains:

### 3.1 Localization Accuracy (Perception Domain)
1. **Sub-Pixel Localization RMSE ($\text{px}$):**
   $$\text{RMSE}_{\text{loc}} = \sqrt{\frac{1}{N_{\text{det}}} \sum_{k=1}^{N_{\text{det}}} \left((u_k - u_{\text{true},k})^2 + (v_k - v_{\text{true},k})^2\right)}$$
2. **Detection Precision:**
   $$\text{Precision} = \frac{TP}{TP + FP}$$
   *(A detection is a True Positive if it falls within the $3\sigma_{\text{psf}}$ footprint of the true optical beacon).*
3. **Detection Recall:**
   $$\text{Recall} = \frac{TP}{TP + FN}$$
   *(Evaluates target detection rate through turbulence and scintillation fades).*
4. **Severe Localization Outlier Count:**
   Number of accepted detections with spatial error $> 5.0\,\text{px}$.

### 3.2 Pointing & Tracking Accuracy (Control & Actuation Domain)
5. **RMS Tracking Error ($\text{deg}$ & $\text{mrad}$):**
   $$\text{RMSE}_{\text{track}} = \sqrt{\frac{1}{N} \sum_{k=1}^N \epsilon_{\text{point},k}^2}, \quad \epsilon_{\text{point},k} = \arccos\left(\mathbf{n}_{\text{cam},k} \cdot \mathbf{n}_{\text{target},k}\right)$$
6. **95th Percentile (P95) Tracking Error ($\text{deg}$):**
   The value $\epsilon_{95}$ such that $P(\epsilon_{\text{point}} \le \epsilon_{95}) = 0.95$.
7. **Maximum Peak Tracking Error ($\text{deg}$):**
   $$\epsilon_{\max} = \max_{k} \epsilon_{\text{point},k}$$
8. **Tracking Overshoot ($\%$ or $\text{deg}$):**
   Peak transient deviation following a step maneuver or direction reversal beyond steady-state error.
9. **Settling Time ($t_s$, $\text{s}$):**
   Time required for pointing error to enter and remain within the fine lock boundary ($\le 0.20^\circ$) following a maneuver.

### 3.3 Temporal & PAT State Machine Performance (FSM Domain)
10. **Acquisition Time ($T_{\text{acq}}$, $\text{s}$):**
    Time elapsed from initial scenario epoch $t=0$ until the FSM first reaches `LOCK` state.
11. **Reacquisition Time ($T_{\text{reacq}}$, $\text{s}$):**
    Time elapsed from target reappearance following a dropout/FOV exit until FSM re-enters `TRACK` or `LOCK`.
12. **Lock Retention Ratio ($\%$):**
    $$\text{Retention} = \frac{T_{\text{LOCK}} + T_{\text{TRACK}}}{T_{\text{target\_in\_fov}}} \times 100\%$$
13. **Accepted-Measurement Coverage ($\%$):**
    Percentage of simulation frames where a valid visual candidate successfully passed the association gate and updated the estimator.

### 3.4 Target Association & Ambiguity Metrics (Association Domain)
14. **False-Lock Count / Rate:**
    Number of times the gimbal stabilized ($\ge 0.5\,\text{s}$) on a distractor beacon or background noise artifact instead of the designated target.
15. **Identity Switches ($N_{\text{switch}}$):**
    Number of times the tracked target trajectory erroneously swapped identity with a distractor beacon.

### 3.5 Mechanical & Actuator Dynamics (Plant Domain)
16. **Controller Actuator Saturation Ratio ($\%$):**
    Percentage of time the commanded gimbal rate, acceleration, or jerk was clamped by physical actuator boundaries.
17. **RMS Angular Velocity ($\text{deg/s}$):**
    $$\text{RMS}_{\omega} = \sqrt{\frac{1}{N} \sum_{k=1}^N \left(\dot{\psi}_k^2 + \dot{\theta}_k^2\right)}$$
18. **RMS Angular Acceleration ($\text{deg/s}^2$):**
    $$\text{RMS}_{\alpha} = \sqrt{\frac{1}{N} \sum_{k=1}^N \left(\ddot{\psi}_k^2 + \ddot{\theta}_k^2\right)}$$
19. **RMS Angular Jerk ($\text{deg/s}^3$):**
    $$\text{RMS}_{j} = \sqrt{\frac{1}{N} \sum_{k=1}^N \left(\dddot{\psi}_k^2 + \dddot{\theta}_k^2\right)}$$
    *(Key metric evaluating mechanical smoothness and suppression of high-frequency gimbal chatter).*

### 3.6 Computational Latency & Throughput (System Domain)
20. **Detector Latency (Mean & P95, $\text{ms}$):**
    Execution time for sub-pixel centroid extraction per frame.
21. **Total Pipeline Latency (Mean & P95, $\text{ms}$):**
    Elapsed time from frame receipt to gimbal command generation.
22. **Effective Simulation Throughput (FPS):**
    Achievable execution frames per second during headless Mode A processing.

---

## 4. Current Baseline vs Target Architecture Status Matrix

To prevent misleading claims, all metrics are classified according to their empirical verification status:

| Metric | Legacy Baseline (Phase 0B Measured) | Target Architecture Goal | Final Measured Performance |
| :--- | :--- | :--- | :--- |
| **Execution Mode** | Live WebSocket / Browser | Mode A Headless Deterministic | **`NOT YET MEASURED`** |
| **Ground Truth Isolation** | Leaked (`gt_beacons` + `beaconPos`) | 100% Strict Firewall | **`NOT YET MEASURED`** |
| **Sub-Pixel Localization RMSE** | $\approx 1.5\,\text{px}$ (contour bbox quantization) | $< 0.05\,\text{px}$ (moment centroid) | **`NOT YET MEASURED`** |
| **Distractor False-Lock Rate** | $> 50\%$ (unprotected nearest-neighbor) | $0.0\%$ (Mahalanobis gate + TBI) | **`NOT YET MEASURED`** |
| **Steady-State Pointing RMSE (S00)** | $0.2103^\circ$ (with truth cheat) | $< 0.05^\circ$ (autonomous) | **`NOT YET MEASURED`** |
| **Dynamic Tracking RMSE (S04)** | Not Measured (failed acceleration) | $< 0.15^\circ$ (6-state CA filter) | **`NOT YET MEASURED`** |
| **Dropout Survival (S09)** | Loss of lock / erratic jump | Seamless coasting $< 0.2\,\text{s}$ | **`NOT YET MEASURED`** |
| **FOV Reacquisition Time (S10)** | Unbounded / Failure (no search) | $< 3.0\,\text{s}$ (covariance spiral) | **`NOT YET MEASURED`** |
| **Actuator RMS Jerk** | Unconstrained velocity steps | $< 250^\circ/\text{s}^3$ (limited plant) | **`NOT YET MEASURED`** |
| **Crossing Target Swaps (S13)** | $100\%$ failure (identity lost) | $0$ swaps (temporal continuity) | **`NOT YET MEASURED`** |
| **Detector Latency (OpenCV)** | $1.09\,\text{ms}$ (CPU measured) | $< 1.5\,\text{ms}$ (with sub-pixel) | **`NOT YET MEASURED`** |
| **Pipeline Latency (Total)** | $1.44\,\text{ms}$ (CPU measured) | $< 2.5\,\text{ms}$ (end-to-end) | **`NOT YET MEASURED`** |

---

## 5. Benchmark Data Retention & Provenance Architecture

Every execution run conducted in Phase 1 through Phase 12 must persist a self-contained archive under `experiments/results/` adhering to the following schema:

```
experiments/results/
└── BENCH-<timestamp>-<scenario_id>-<seed>/
    ├── config.json          # Complete JSON configuration snapshot
    ├── provenance.json      # Git commit, OS, Python version, hardware metadata
    ├── telemetry.parquet    # High-efficiency binary telemetry time series
    ├── telemetry.csv.gz     # Compressed CSV for external data inspection
    ├── metrics.json         # Computed 22 scalar metrics with pass/fail flags
    ├── events.json          # FSM state transitions, loss events, saturation alerts
    ├── error_plots.png      # High-resolution pointing error & phase plots
    └── benchmark_report.pdf # Automated ReportLab engineering summary
```

### Provenance Metadata Contract (`provenance.json`)
```json
{
  "benchmark_id": "BENCH-20260919-S04-SEED42",
  "scenario_id": "S04_constant_acceleration",
  "seed": 42,
  "git_commit_sha": "41c070b63d5cfe2bd234f19b4a7f277a7e2d2ff1",
  "git_branch": "fsoc-winning-v1",
  "timestamp_iso": "2026-09-19T14:00:00.000Z",
  "execution_mode": "MODE_A_HEADLESS",
  "timestep_dt_s": 0.0166667,
  "hardware": {
    "cpu": "Intel(R) Core(TM) Ultra 5 225H",
    "os": "Microsoft Windows 11 Build 26200",
    "python_version": "3.12.10"
  },
  "algorithm_stack": {
    "perception": "subpixel_intensity_moments",
    "association": "mahalanobis_gated_gnn",
    "estimation": "kalman_constant_acceleration_6state",
    "fsm": "pat_8state_deterministic",
    "control": "predictive_feedforward_filtered_pid",
    "plant_limiting": "jerk_accel_rate_clamped",
    "temporal_beacon_id": false,
    "ml_predictor": false
  }
}
```

---

## 6. Execution Protocol & Acceptance Verification

1. **Pre-Execution Check:** Verify working tree cleanliness with `git status`. Ensure zero uncommitted code changes.
2. **Headless Execution:** Run scenario suite via Mode A CLI runner:
   ```powershell
   $env:PYTHONPATH="backend"
   .\.venv\Scripts\python -m core.benchmark.runner --scenario=all --seeds=42,101,555 --out-dir=experiments/results/
   ```
3. **Automated Verification:** The runner validates all 22 metrics against the scenario's acceptance criteria. If any required metric exceeds threshold, the scenario exits with non-zero error code and flags the failure event in `events.json`.
4. **Summary Aggregation:** Generates master comparative CSV table and PDF report.
