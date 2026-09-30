# Master Implementation Plan: Coarse PAT Engineering Phases

**Project:** SIH26169 — AI-Based Virtual Camera Tracking System for Coarse Alignment of Mobile FSOC Terminals  
**Role:** Senior Aerospace Software Architect, Computer Vision Engineer, Control Systems Engineer & Experimental Validation Engineer  
**Document Version:** 1.0.0 (Phase 0C Architecture Freeze)  
**Status:** APPROVED EXECUTION BLUEPRINT  

---

## 1. Overview & Phased Execution Strategy

This implementation plan establishes the strict sequential roadmap to transform the inherited prototype into a competition-winning, hardware-in-the-loop coarse PAT platform.

### Phase Execution Rules
1. **Strict Monotonic Ordering:** Phases must be executed in exact numerical sequence ($0\text{D} \rightarrow 1 \rightarrow 2 \dots \rightarrow 12$). No phase may begin until the preceding phase's Acceptance Gate is 100% verified.
2. **Zero Algorithmic Regressions:** Existing functionality and unit test suites must remain fully passing after each phase.
3. **No Truth Leakage Tolerated:** Any introduction of ground-truth data into perception, association, estimation, or control instantly invalidates the build.

---

## 2. Phase-by-Phase Technical Specifications

```
Phase 0D: Truth Isolation
    │
    ▼
Phase 1: Deterministic Benchmark Framework
    │
    ▼
Phase 2: Precision Sub-Pixel Localization
    │
    ▼
Phase 3: Candidate Association & Statistical Gating
    │
    ▼
Phase 4: Adaptive 6-State Kalman State Estimation
    │
    ▼
Phase 5: Robust 8-State PAT Finite State Machine
    │
    ▼
Phase 6: Predictive Smooth Control & Gimbal Plant Limiting
    │
    ▼
Phase 7: Covariance-Guided Spiral Reacquisition
    │
    ▼
Phase 8: Temporal Beacon Identification (Optional Coded Beacon)
    │
    ▼
Phase 9: Optional Machine Learning Trajectory Predictor
    │
    ▼
Phase 10: Comprehensive Ablation Campaign
    │
    ▼
Phase 11: Judge-Facing Mission Control UI
    │
    ▼
Phase 12: Release Freeze & Delivery Package
```

---

### Phase 0D: Truth Isolation & Quarantine

* **Current Weakness:**
  - `backend/app/main.py` lines 541–590 matches visual blobs against `gt_beacons` (projected 3D ground truth). Drops any detection $> 100\,\text{px}$ from ground truth.
  - `frontend/src/scene/GimbalCamera.jsx` lines 140–161 falls back to `currentBeaconPosition` (true 3D position) when Kalman filter is ablated or uninitialized, steering the gimbal directly with ground truth.
* **Files Likely Affected:**
  - `backend/app/main.py`
  - `frontend/src/scene/GimbalCamera.jsx`
  - `frontend/src/store/simulationStore.js`
* **Files Likely Added:**
  - `backend/core/evaluation/firewall.py` (Defines strict boundary and evaluation wrapper)
  - `backend/tests/test_firewall.py` (Static and runtime AST inspection ensuring zero truth leakage)
* **Interfaces:**
  - Perception output decoupled from ground truth; input to tracking is pure `DetectionCandidate` list.
  - Frontend gimbal orientation updated purely from backend WebSocket telemetry commands; zero ground-truth angles fed to `panPid`/`tiltPid`.
* **Implementation Work:**
  - Remove all candidate-filtering logic referencing `gt_beacons` in `main.py`. Pass raw visual candidates directly to association.
  - Remove `currentBeaconPosition` fallback in `GimbalCamera.jsx`. If Kalman is disabled, the system steers by the optical camera error or remains stationary.
  - Add AST audit tests verifying that `gt_beacons` and `beaconPosition` never appear inside detection or control functions.
* **Unit Tests:**
  - `backend/tests/test_firewall.py`: AST test verifying no truth variables in `core/` or tracking handlers.
* **Integration Tests:**
  - Run closed-loop WebSocket tracking with `gt_beacons` intentionally randomized; verify tracking does not fail or use the fake values.
* **Benchmark Scenarios:**
  - S00 (Nominal stationary), S01 (Linear motion).
* **Required Telemetry:**
  - `truth_firewall_status`: Boolean indicator of active isolation.
* **Acceptance Gate:**
  - 100% of candidate association and gimbal control executes without reading ground truth.
  - All existing 30 tests pass.
* **Rollback Strategy:** Git revert of truth-isolation commits to baseline commit `41c070b`.
* **Risks:** Removing the cheat association will initially expose false positives from clutter until Phase 2 and 3 are implemented.

---

### Phase 1: Deterministic Benchmark Framework (Mode A)

* **Current Weakness:**
  - Benchmarks depend on browser execution, WebSocket frames, and wall-clock jitter. Results are non-reproducible and unscientific.
* **Files Likely Affected:**
  - `backend/app/experiments/benchmark.py`
  - `backend/app/experiments/reports.py`
* **Files Likely Added:**
  - `backend/core/simulation/engine.py` (Headless physics and optical scene generator)
  - `backend/core/benchmark/runner.py` (Headless runner for Mode A)
  - `backend/core/benchmark/metrics.py` (Comprehensive scientific metric calculator)
  - `backend/tests/test_benchmark_mode.py`
* **Interfaces:**
  - `BenchmarkConfig(scenario_name, seed, duration_s, dt_s, algorithm_config)`
  - `BenchmarkResult(telemetry_df, summary_metrics, pass_fail_flags)`
* **Implementation Work:**
  - Implement headless simulation loop: advances target trajectory, simulates virtual camera pinhole projection, renders degraded frame or analytical photon field, runs perception $\rightarrow$ estimation $\rightarrow$ control loop, logs full telemetry.
  - PRNG seed locking for all disturbance generators.
  - Structured output generator (CSV, JSON, Matplotlib curves).
* **Unit Tests:**
  - `backend/tests/test_benchmark_mode.py`: Run two instances of scenario S01 with seed `42`; assert bitwise identical telemetry arrays.
* **Integration Tests:**
  - Headless benchmark runs all 16 scenarios (`S00` to `S15`) and generates complete benchmark results.
* **Benchmark Scenarios:**
  - S00 through S15.
* **Required Telemetry:**
  - Full `TelemetryRecord` schema at each simulation tick.
* **Acceptance Gate:**
  - Exact reproducibility: Run S01 twice with seed `12345`; `max(abs(telemetry_1 - telemetry_2)) == 0.0`.
  - Headless benchmark executes at $> 100\,\text{FPS}$ on CPU.
* **Rollback Strategy:** Revert additions in `core/benchmark/`.
* **Risks:** Numerical precision differences across platforms; mitigated by strict NumPy fixed-dtype operations.

---

### Phase 2: Precision Sub-Pixel Localization

* **Current Weakness:**
  - Contour bounding box integer center ($[x + w/2, y + h/2]$) introduces $\pm 1.5\,\text{px}$ quantization jitter.
  - Severe vulnerability to background illumination gradients, scintillation fading, and beam spreading.
* **Files Likely Affected:**
  - `backend/app/core/detector.py`
  - `backend/app/main.py`
* **Files Likely Added:**
  - `backend/core/perception/psf_extractor.py`
  - `backend/core/perception/subpixel_moments.py`
  - `backend/tests/test_subpixel.py`
* **Interfaces:**
  - `extract_subpixel_candidates(image: np.ndarray, roi_candidates) -> List[DetectionCandidate]`
* **Implementation Work:**
  - Local background estimation: annular border median filter.
  - Matched filter correlation with 2D Gaussian PSF kernel ($\sigma_{\text{psf}} \approx 1.5–3.0\,\text{px}$).
  - Adaptive dual thresholding.
  - First-order intensity moments ($m_{10}/m_{00}, m_{01}/m_{00}$).
  - Dynamic measurement variance computation $\sigma_{\text{meas}}^2 = \sigma_{\text{psf}}^2 / \text{SNR}$.
* **Unit Tests:**
  - `test_subpixel.py`: Generate synthetic Gaussian spot with known sub-pixel center $(x_0, y_0) = (100.37, 50.82)$ with Gaussian noise; assert localization error $< 0.05\,\text{px}$.
* **Integration Tests:**
  - Mode A execution under heavy scintillation (Scenario S08); verify sub-pixel localization remains stable.
* **Benchmark Scenarios:**
  - S00, S07 (Sensor noise), S08 (Scintillation).
* **Required Telemetry:**
  - `subpixel_u`, `subpixel_v`, `snr_db`, `flux_total`, `measurement_variance_px`.
* **Acceptance Gate:**
  - Mean localization error on synthetic static spot $< 0.05\,\text{px}$.
  - Processing latency $< 2.0\,\text{ms}$ per frame on CPU.
* **Rollback Strategy:** Fall back to `OpenCVDetector` with integer contour centroiding.
* **Risks:** Highly turbulent frames with deep dropouts may produce low SNR; handled by adaptive variance $R_k$ and SNR thresholding.

---

### Phase 3: Candidate Association & Statistical Gating

* **Current Weakness:**
  - Simulator target ID cheating or Euclidean distance to ground truth.
  - Multi-beacon or distractor environments cause immediate false locks.
* **Files Likely Affected:**
  - `backend/app/main.py`
  - `backend/app/core/tracker.py`
* **Files Likely Added:**
  - `backend/core/association/gating.py`
  - `backend/core/association/cost_matrix.py`
  - `backend/tests/test_association.py`
* **Interfaces:**
  - `associate_candidates(candidates: List[DetectionCandidate], track_estimate: TargetKinematicEstimate) -> Tuple[Optional[DetectionCandidate], AssociationReport]`
* **Implementation Work:**
  - Innovation computation: $\mathbf{y} = \mathbf{z} - \mathbf{H}\hat{\mathbf{x}}$.
  - Innovation covariance: $\mathbf{S} = \mathbf{H}\mathbf{P}\mathbf{H}^T + \mathbf{R}$.
  - Mahalanobis distance gating: $d_M^2 = \mathbf{y}^T \mathbf{S}^{-1} \mathbf{y} \le 9.21$.
  - Multi-feature cost metric incorporating $d_M$, SNR, PSF shape consistency, and flux history.
  - Nearest-neighbor / Global Nearest Neighbor assignment.
* **Unit Tests:**
  - `test_association.py`: Provide 1 true target candidate at $1\sigma$ and 3 brighter distractor candidates at $5\sigma$; assert the filter associates the true candidate.
* **Integration Tests:**
  - Run Scenario S11 (Stationary distractor) and S14 (Brighter distractor); verify zero false locks.
* **Benchmark Scenarios:**
  - S11, S12, S13 (Crossing distractor), S14.
* **Required Telemetry:**
  - `mahalanobis_distance`, `num_gated_candidates`, `association_cost`, `association_status`.
* **Acceptance Gate:**
  - Zero distractor false locks on S11, S12, and S14.
  - Association execution time $< 0.1\,\text{ms}$.
* **Rollback Strategy:** Revert to coarse nearest-neighbor gating with generous Euclidean radius.
* **Risks:** Rapidly maneuvering targets might temporarily fall outside the gate if process noise $Q$ is tuned too small; solved by adaptive gating during maneuvers.

---

### Phase 4: Adaptive 6-State Kalman State Estimation

* **Current Weakness:**
  - Inherited 4-state $[x, y, v_x, v_y]$ filter assumes constant velocity. Target acceleration causes severe lag and steady-state offset.
  - Static measurement covariance $R$ ignores signal fading and noise spikes.
* **Files Likely Affected:**
  - `backend/app/core/tracker.py`
* **Files Likely Added:**
  - `backend/core/estimation/kalman_ca.py`
  - `backend/tests/test_kalman_ca.py`
* **Interfaces:**
  - `KalmanFilterCA.predict(dt: float)`
  - `KalmanFilterCA.update(z: np.ndarray, R: np.ndarray)`
  - `KalmanFilterCA.get_estimate() -> TargetKinematicEstimate`
* **Implementation Work:**
  - State vector: $\mathbf{x} = [u, v, \dot{u}, \dot{v}, \ddot{u}, \ddot{v}]^T$.
  - Transition matrix $\mathbf{F}(\Delta t)$ for constant acceleration.
  - Continuous-time white noise jerk process covariance $\mathbf{Q}(\Delta t)$.
  - Adaptive measurement covariance $\mathbf{R}_k = f(\text{SNR}_k)$.
  - Uncertainty propagation during coasting frames.
  - Preserve legacy 4-state filter in `tracker.py` as an ablation baseline (`--estimator=cv4`).
* **Unit Tests:**
  - `test_kalman_ca.py`: Simulate quadratic trajectory $u(t) = 100 + 10t + 2t^2$; verify acceleration state converges to $4.0\,\text{px/s}^2 \pm 0.1$ and position error $< 0.2\,\text{px}$.
* **Integration Tests:**
  - Mode A run on Scenario S04 (Acceleration) and S09 (Temporary dropout); compare 4-state vs 6-state tracking RMSE.
* **Benchmark Scenarios:**
  - S01, S02 (Sinusoidal), S04, S09.
* **Required Telemetry:**
  - `kalman_state` (6 elements), `kalman_cov_diag` (6 elements), `pos_uncertainty_px`, `vel_uncertainty_px`.
* **Acceptance Gate:**
  - Steady-state tracking error under constant acceleration reduced by $\ge 60\%$ compared to 4-state baseline.
  - Smooth coasting through 10-frame dropouts without divergent covariance collapse.
* **Rollback Strategy:** Switch configuration flag to legacy 4-state mode.
* **Risks:** Overfitting to sensor noise if process noise $q$ is set too high; tuned using Allan variance / grid search on benchmark suite.

---

### Phase 5: Robust 8-State PAT Finite State Machine

* **Current Weakness:**
  - Inherited FSM has basic 3-state logic (`SEARCHING`, `ACQUIRING`, `TRACKING`) without formal hysteresis, coasting, or reacquisition transitions.
* **Files Likely Affected:**
  - `backend/app/core/state_machine.py`
* **Files Likely Added:**
  - `backend/core/fsm/pat_fsm.py`
  - `backend/tests/test_pat_fsm.py`
* **Interfaces:**
  - `PatStateMachine.update(detection: Optional[DetectionCandidate], estimate: TargetKinematicEstimate, pointing_error_deg: float, dt: float) -> FSMState`
* **Implementation Work:**
  - Implement 8 states: `SEARCH`, `CANDIDATE`, `ACQUIRE`, `TRACK`, `LOCK`, `COAST`, `REACQUIRE`, `LOST`.
  - Frame counters for entry/exit hysteresis ($M$-of-$N$ confirmation).
  - Explicit timeouts for `COAST` ($0.5\,\text{s}$) and `REACQUIRE` ($3.0\,\text{s}$).
  - Output commanding active search behavior vs closed-loop tracking.
* **Unit Tests:**
  - `test_pat_fsm.py`: Simulate synthetic detection sequences; verify state transitions adhere strictly to contract table.
* **Integration Tests:**
  - Run Scenario S09 (Temporary dropout) and S10 (Target FOV exit); verify transition sequence: `TRACK` $\rightarrow$ `COAST` $\rightarrow$ `REACQUIRE` $\rightarrow$ `LOST` $\rightarrow$ `SEARCH`.
* **Benchmark Scenarios:**
  - S00, S09, S10, S15 (Combined stress).
* **Required Telemetry:**
  - `fsm_state`, `fsm_state_time_s`, `lock_retention_flag`.
* **Acceptance Gate:**
  - Zero state oscillation (chattering) around threshold boundaries.
  - 100% deterministic state transitions.
* **Rollback Strategy:** Revert to 3-state state machine via configuration fallback.
* **Risks:** Overly conservative hysteresis may delay acquisition; verified via S00 acquisition time metrics.

---

### Phase 6: Predictive Smooth Control & Gimbal Plant Limiting

* **Current Weakness:**
  - Unbounded velocity steps permitted by legacy PID.
  - Zero latency compensation.
  - No physical actuator rate, acceleration, or jerk limits.
  - Significant overshoot during rapid target maneuvers.
* **Files Likely Affected:**
  - `backend/app/core/controller.py`
  - `frontend/src/scene/GimbalCamera.jsx`
* **Files Likely Added:**
  - `backend/core/control/predictive_pid.py`
  - `backend/core/control/actuator_plant.py`
  - `backend/tests/test_control_plant.py`
* **Interfaces:**
  - `GimbalPlant.step(rate_cmd: Tuple[float, float], dt: float) -> Tuple[float, float]`
  - `PredictiveController.compute(target_est: TargetKinematicEstimate, gimbal_angles: Tuple[float, float], dt: float) -> GimbalCommand`
* **Implementation Work:**
  - Angular velocity feed-forward: $\boldsymbol{\omega}_{\text{ff}} = \mathbf{K}_{\text{ff}} [\hat{\dot{u}}, \hat{\dot{v}}]^T$.
  - Pipeline latency-ahead predictor: $\mathbf{e}_{\text{pred}} = \hat{\mathbf{x}}(t + \tau_{\text{latency}}) - [c_x, c_y]^T$.
  - Low-pass derivative filter $N / (1 + Ns)$ and deadband hysteresis ($1.0\,\text{px}$).
  - Integral anti-windup clamping.
  - Physical gimbal actuator limits: $\omega_{\max} = 20^\circ/\text{s}$, $\alpha_{\max} = 80^\circ/\text{s}^2$, $j_{\max} = 400^\circ/\text{s}^3$, $\tau_{\text{motor}} = 20\,\text{ms}$.
  - Preserve legacy PID as an ablation baseline (`--controller=legacy_pid`).
* **Unit Tests:**
  - `test_control_plant.py`: Issue step velocity command $100^\circ/\text{s}$; verify output rate never exceeds $20^\circ/\text{s}$ and acceleration never exceeds $80^\circ/\text{s}^2$.
* **Integration Tests:**
  - Run Scenario S03 (High speed) and S05 (Sudden direction change); evaluate tracking error overshoot and gimbal jerk.
* **Benchmark Scenarios:**
  - S01, S02, S03, S05, S06 (Vibration).
* **Required Telemetry:**
  - `gimbal_rate_deg_s`, `gimbal_accel_deg_s2`, `gimbal_jerk_deg_s3`, `feedforward_rate`, `is_actuator_saturated`.
* **Acceptance Gate:**
  - Actuator jerk reduction $\ge 50\%$ compared to legacy PID.
  - Overshoot on step direction changes reduced by $\ge 40\%$.
* **Rollback Strategy:** Toggle configuration flag to unconstrained PID.
* **Risks:** Severe actuator limits might cause target loss during extreme maneuvers; bounds validated against realistic aeronautical terminal specifications.

---

### Phase 7: Covariance-Guided Spiral Reacquisition

* **Current Weakness:**
  - When target leaves FOV or drops out, the system remains blind or static. Zero active search pattern to recover alignment.
* **Files Likely Affected:**
  - `backend/app/main.py`
  - `backend/core/fsm/pat_fsm.py`
* **Files Likely Added:**
  - `backend/core/reacquisition/spiral_search.py`
  - `backend/tests/test_reacquisition.py`
* **Interfaces:**
  - `generate_reacquisition_command(fsm_state: FSMState, estimate: TargetKinematicEstimate, search_time_s: float, dt: float) -> Tuple[float, float]`
* **Implementation Work:**
  - Extrapolated uncertainty ellipse formulation from Kalman covariance $\mathbf{P}$.
  - Local micro-scan across $3\sigma$ ellipse.
  - Expanding Archimedean spiral generator: $r(\phi) = \frac{d_{\text{fov}}}{2\pi} \phi$.
  - Multi-frame confirmation detector during search.
* **Unit Tests:**
  - `test_reacquisition.py`: Verify spiral path coordinates provide gapless spatial coverage within $3^\circ$ radius in $\le 2.5\,\text{s}$.
* **Integration Tests:**
  - Run Scenario S09 (Temporary dropout) and S10 (Target FOV exit); measure reacquisition time and lock restoration rate.
* **Benchmark Scenarios:**
  - S09, S10, S15.
* **Required Telemetry:**
  - `reacquisition_active`, `reacquisition_scan_radius_deg`, `reacquisition_time_s`.
* **Acceptance Gate:**
  - Target reacquisition achieved in $< 3.0\,\text{s}$ for Scenario S09 and S10.
* **Rollback Strategy:** Fall back to stationary standby pointing on target loss.
* **Risks:** Excessively rapid spiral scan may cause image blur; scan angular rate clamped to $< 5^\circ/\text{s}$.

---

### Phase 8: Temporal Beacon Identification (Optional Coded Beacon)

* **Current Weakness:**
  - In extreme multi-distractor scenarios (e.g. crossing flares or urban lights), spatial gating alone can experience ambiguity.
* **Files Likely Affected:**
  - `backend/core/simulation/engine.py` (Adds optional modulation to target emitter)
  - `backend/core/association/gating.py`
* **Files Likely Added:**
  - `backend/core/perception/temporal_demod.py`
  - `backend/tests/test_temporal_beacon.py`
* **Interfaces:**
  - `compute_tbi_correlation(intensity_history: np.ndarray, code_sequence: np.ndarray) -> float`
* **Implementation Work:**
  - Modulate simulated beacon with $10\,\text{Hz}$ square wave or 15-bit m-sequence code.
  - Sliding temporal intensity buffer for detected candidates.
  - Normalized cross-correlation score $\rho_{\text{TBI}} \in [0.0, 1.0]$.
  - Integration into association cost matrix.
  - Purely optional feature; benchmarks run both coded and uncoded.
* **Unit Tests:**
  - `test_temporal_beacon.py`: Correlate noisy modulated signal against reference code; verify correlation $> 0.85$ on target, $< 0.15$ on unmodulated distractor.
* **Integration Tests:**
  - Run Scenario S13 (Crossing distractor) with TBI enabled; assert 0 identity switches.
* **Benchmark Scenarios:**
  - S11, S12, S13, S14.
* **Required Telemetry:**
  - `tbi_correlation_score`, `tbi_enabled_flag`.
* **Acceptance Gate:**
  - 100% target disambiguation during crossing distractor trajectory (S13).
* **Rollback Strategy:** Disable TBI via CLI / config flag (`--enable-tbi=false`).
* **Risks:** High scintillation noise degrading temporal correlation; mitigated by robust matched filtering and running uncoded mode as standard.

---

### Phase 9: Optional Machine Learning Trajectory Predictor

* **Current Weakness:**
  - Non-linear, aggressive target maneuvers (e.g. evasive UAV or high-frequency turbulence) can exceed constant-acceleration kinematic assumptions.
* **Files Likely Affected:**
  - `backend/core/estimation/`
* **Files Likely Added:**
  - `backend/core/ml/motion_mlp.py` (Lightweight PyTorch / ONNX motion predictor)
  - `backend/tests/test_ml_predictor.py`
* **Interfaces:**
  - `MLMotionPredictor.predict(history_window: np.ndarray) -> np.ndarray`
* **Implementation Work:**
  - Train a compact 3-layer MLP on synthetic aerodynamic trajectories.
  - Predict $\hat{\mathbf{x}}(t + \tau_{\text{latency}})$ from a sliding window of past 10 state estimates.
  - Strict performance gate: Compare closed-loop tracking error of (Kalman CA + MLP) vs (Kalman CA alone).
  - If MLP does not improve closed-loop tracking RMSE by $\ge 15\%$ on non-linear scenarios, ML remains disabled.
* **Unit Tests:**
  - `test_ml_predictor.py`: Verify forward inference latency $< 0.5\,\text{ms}$ on CPU.
* **Integration Tests:**
  - Run Scenario S02 (Sinusoidal) and S05 (Sudden direction change) with and without MLP.
* **Benchmark Scenarios:**
  - S02, S03, S05, S15.
* **Required Telemetry:**
  - `ml_prediction_residual_px`, `ml_inference_latency_ms`, `ml_active_flag`.
* **Acceptance Gate:**
  - Statistically significant improvement in closed-loop metrics or clean decommission.
* **Rollback Strategy:** Set `--ml-predictor=disabled`.
* **Risks:** Inference jitter or distribution shift; mitigated by strict Kalman residual fallback.

---

### Phase 10: Comprehensive Ablation Campaign

* **Current Weakness:**
  - Lack of scientific proof demonstrating the individual value of sub-pixel centroiding, 6-state Kalman, velocity feedforward, and actuator limiting.
* **Files Likely Affected:**
  - `backend/app/experiments/`
* **Files Likely Added:**
  - `backend/core/benchmark/ablation_suite.py`
  - `experiments/ablation/` (Results storage)
* **Interfaces:**
  - `run_ablation_matrix(configurations: List[AblationConfig], scenarios: List[str], seeds: List[int])`
* **Implementation Work:**
  - Execute full combinatorial ablation matrix across all 16 scenarios:
    1. Legacy Baseline (OpenCV + 4-State Kalman + Legacy PID)
    2. + Sub-Pixel Localization
    3. + Mahalanobis Gating
    4. + 6-State Adaptive Kalman Filter
    5. + Velocity Feed-Forward & Predictive Control
    6. + Actuator Rate/Accel/Jerk Limits
    7. + Covariance-Guided Reacquisition
    8. + Temporal Beacon Identification (Optional)
  - Automated generation of comparative box plots, Pareto efficiency frontiers, and publication-quality LaTeX/PDF summary tables.
* **Unit Tests:**
  - Verify ablation engine automatically logs configuration hashes and git commit metadata.
* **Integration Tests:**
  - Automated batch execution of 16 scenarios $\times$ 5 seeds $\times$ 8 configurations.
* **Benchmark Scenarios:**
  - S00 through S15 across all seeds.
* **Required Telemetry:**
  - Full aggregate statistical tables (mean, median, P95, standard deviation).
* **Acceptance Gate:**
  - Complete, verifiable ablation dataset proving measurable superiority of the target architecture over the legacy baseline.
* **Rollback Strategy:** N/A (Analytical evaluation phase).
* **Risks:** Extended benchmark execution time; mitigated by multi-process parallelization across CPU cores.

---

### Phase 11: Judge-Facing Mission Control UI (Mode B)

* **Current Weakness:**
  - Inherited frontend has lingering ground-truth dependencies in `GimbalCamera.jsx` and does not visualize sub-pixel centroids, covariance ellipses, or reacquisition spirals.
* **Files Likely Affected:**
  - `frontend/src/scene/GimbalCamera.jsx`
  - `frontend/src/scene/SceneContent.jsx`
  - `frontend/src/components/TrackingView.jsx`
  - `frontend/src/components/TelemetryOverlay.jsx`
* **Files Likely Added:**
  - `frontend/src/scene/CovarianceEllipseVisualizer.jsx`
  - `frontend/src/scene/ReacquisitionSpiralVisualizer.jsx`
  - `frontend/src/components/AblationToggleMatrix.jsx`
* **Interfaces:**
  - WebSocket JSON Telemetry payload contract (Contract 4 & TelemetryRecord).
* **Implementation Work:**
  - Wire frontend strictly to authoritative backend telemetry stream.
  - Render 3D Kalman uncertainty ellipsoid and predicted trajectory curve in Three.js.
  - Display sub-pixel centroid crosshair, detection gate ellipse, and SNR metric on 2D camera HUD.
  - Visual indicator for 8 FSM states with dynamic color-coding.
  - Interactive ablation toggles allowing live demonstration of algorithm components to judges.
* **Unit Tests:**
  - Frontend lint (`npm run lint`) 0 errors; production build (`npm run build`) clean.
* **Integration Tests:**
  - Launch live FastAPI server + Vite frontend; verify bidirectional controls and 60 Hz smooth telemetry streaming.
* **Benchmark Scenarios:**
  - Interactive manual execution of S01, S06, S09, S13.
* **Required Telemetry:**
  - Real-time client-side FPS, WebSocket round-trip ping, frame dropped count.
* **Acceptance Gate:**
  - Flawless visual demonstration with zero console errors and zero reliance on client-side ground truth.
* **Rollback Strategy:** Revert frontend view components to Phase 0B state.
* **Risks:** WebGL rendering performance on low-spec judge laptops; mitigated by Three.js LOD (Level of Detail) optimization.

---

### Phase 12: Release Freeze & Delivery Package

* **Current Weakness:**
  - Project lacks single-command deployment scripts, frozen verification hashes, and final SIH demonstration assets.
* **Files Likely Affected:**
  - `README.md`
  - `documentation/User_Manual.md`
* **Files Likely Added:**
  - `scripts/run_headless_benchmarks.ps1`
  - `scripts/launch_mission_control.ps1`
  - `documentation/Final_Technical_Report.pdf`
  - `plans/05_FINAL_VERIFICATION_REPORT.md`
* **Interfaces:**
  - Standalone execution entry points for judges and automated grading environments.
* **Implementation Work:**
  - Complete end-to-end regression validation.
  - Generate final publication-grade verification reports and demonstration videos.
  - Archive all benchmark logs, scenario configs, and model checkpoints.
  - Code freeze and final tagging.
* **Unit Tests:**
  - 100% unit test pass across entire repository.
* **Integration Tests:**
  - Clean clone test on isolated environment verifying single-command installation and execution.
* **Benchmark Scenarios:**
  - Full suite verification.
* **Acceptance Gate:**
  - Unconditional certification by Senior Systems Architect. System ready for winning hackathon defense.
* **Rollback Strategy:** Release tag management.
* **Risks:** Documentation desynchronization; mitigated by automated markdown-to-PDF compilation.
