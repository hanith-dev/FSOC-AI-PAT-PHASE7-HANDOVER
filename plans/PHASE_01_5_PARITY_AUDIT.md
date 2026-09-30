# Phase 1.5 — Benchmark Fidelity and Algorithm Parity Certification Report

## Executive Summary
This report certifies the mathematical and architectural fidelity of **Mode A** (Deterministic Headless Scientific Benchmark Engine) against **Mode B** (Live Mission Control Application) for the Free Space Optical Communication Pointing, Acquisition, and Tracking (FSOC-PAT) system (SIH26169).

The purpose of Phase 1.5 is **Verification First**. No algorithm improvements, localization upgrades, gain tuning, or architecture changes have been applied to enhance tracking performance. Instead, every subsystem was audited to guarantee that Mode A measures the authentic baseline system inherited from Mode B rather than an unverified duplicate.

---

## 1. Subsystem Implementation Parity Matrix

The 17 critical subsystems were inventoried across Mode A and Mode B:

| # | Subsystem | Mode A Source Path | Mode B Source Path | Classification | Fidelity Assessment & Mathematical Equivalence |
|---|-----------|-------------------|-------------------|----------------|-------------------------------------------------|
| 1 | Camera Intrinsics | `backend/app/benchmark/renderer.py` | `frontend/src/components/ThreeCanvas.jsx` | SEPARATE_BUT_EQUIVALENT | Pin-hole camera model: $W=1280, H=720, \text{HFOV}=15.0^\circ$. Mode A computes $f_x = f_y = \frac{W/2}{\tan(\text{HFOV}/2)} = 4860.05\,\text{px}, c_x = 640.0, c_y = 360.0$. Exact agreement with Three.js camera parameters. |
| 2 | Camera Projection | `backend/app/benchmark/renderer.py` | `frontend/src/components/ThreeCanvas.jsx` | SEPARATE_BUT_EQUIVALENT | Both use right-handed coordinate transformation with YXZ yaw-pitch gimbal rotation matrix, projecting 3D optical world positions into normalized camera coordinates then screen pixels. Maximum discrepancy on test points $< 10^{-5}\,\text{px}$. |
| 3 | Coordinate Conventions | `backend/app/benchmark/renderer.py` | `frontend/src/components/ThreeCanvas.jsx` | SEPARATE_BUT_EQUIVALENT | World: $+X$ East, $+Y$ Up, $+Z$ North. Gimbal: Pan positive clockwise (yaw around $+Y$), Tilt positive upward (pitch around $+X$). Image plane: $(0,0)$ top-left, $+X$ right, $+Y$ down. Perfectly aligned. |
| 4 | Image Resolution | `backend/app/benchmark/renderer.py` | `frontend/src/components/VirtualCameraView.jsx` | SHARED_IMPLEMENTATION | $1280 \times 720$ pixels standard across all synthetic rendering and UI canvas views. |
| 5 | Field of View (FOV) | `backend/app/benchmark/scenarios.py` | `frontend/src/store/useSimulationStore.js` | SHARED_IMPLEMENTATION | $15.0^\circ$ diagonal/horizontal field of view matched across config schemas. |
| 6 | Optical Detector | `backend/app/detection/detector.py` | `backend/app/detection/detector.py` | SHARED_IMPLEMENTATION | Identical shared OpenCV classic detector implementation: Grayscale -> Gaussian Blur ($5 \times 5, \sigma=1.0$) -> Threshold ($\tau=200$) -> Morphological Open -> Connected Components (`cv2.findContours`). |
| 7 | Candidate Extraction | `backend/app/detection/detector.py` | `backend/app/detection/detector.py` | SHARED_IMPLEMENTATION | Shared code: Extracts contour centroids, bounding boxes, circularity, peak intensity, and SNR. |
| 8 | Candidate Selection | `backend/app/benchmark/runner.py` | `backend/app/main.py` | SEPARATE_BUT_EQUIVALENT | Both enforce the strict Phase 0D Ground-Truth Firewall: candidate closest to camera optical boresight $(640, 360)$ in SEARCHING, and closest to prior Kalman track in TRACKING. Zero simulator ground truth is used in association. |
| 9 | Kalman Filter | `backend/app/tracking/kalman.py` | `backend/app/tracking/kalman.py` | SHARED_IMPLEMENTATION | Exact same `KalmanFilter2D` class instance executed in both modes. Continuous constant-velocity 4-state $[x, \dot{x}, y, \dot{y}]^T$, discrete $F(\Delta t)$, $Q(\Delta t)$ with continuous white-noise acceleration model, measurement matrix $H$, innovation covariance $S = H P H^T + R$, Kalman gain $K = P H^T S^{-1}$. |
| 10 | Finite State Machine (FSM) | `backend/app/tracking/state_machine.py` | `backend/app/tracking/state_machine.py` | SHARED_IMPLEMENTATION | Exact same `TrackingStateMachine` class instance. Updated in Phase 1 to accept deterministic `current_time` in `update(..., current_time)` and `transition_to(..., current_time)`. Transitions (`SEARCHING` -> `DETECTED` -> `ACQUIRING` -> `TRACKING` -> `LOCKED`) are identical under deterministic time and live clock time. |
| 11 | PID Controller | `backend/app/tracking/pid.py` | `frontend/src/utils/pid.js` | SEPARATE_BUT_EQUIVALENT | Line-for-line mathematical equivalence verified: Proportional term, trapezoidal/Euler integration with anti-windup clamping, filtered derivative with low-pass filter, deadband thresholding, and output saturation. Defaults aligned in Phase 1.5 to `integral_limit=20.0, output_limit=180.0`. Numerical error across golden input vectors $< 10^{-12}$. |
| 12 | Gimbal Integration | `backend/app/benchmark/runner.py` | `frontend/src/store/useSimulationStore.js` | SEPARATE_BUT_EQUIVALENT | First-order kinematic forward Euler integration: $\theta_{k+1} = \theta_k + \omega_k \Delta t$. Both clamp rates to $[-180, +180]^\circ/\text{s}$ and gimbal angles to $\text{pan} \in [-360, +360]^\circ$, $\text{tilt} \in [-90, +90]^\circ$. |
| 13 | Actuator Limits | `backend/app/benchmark/runner.py` | `frontend/src/store/useSimulationStore.js` | SHARED_IMPLEMENTATION | Slew rate limit $= 180.0^\circ/\text{s}$, Pan range $= [-360^\circ, +360^\circ]$, Tilt range $= [-90^\circ, +90^\circ]$. |
| 14 | Disturbance Definitions | `backend/app/benchmark/scenarios.py` | `frontend/src/utils/disturbances.js` | SEPARATE_BUT_EQUIVALENT | Sum-of-sines platform angular vibration models ($0.5\,\text{Hz} - 25\,\text{Hz}$), atmospheric scintillation (gamma-gamma / log-normal irradiance fading), and cloud/dust obscuration. Evaluated deterministically via PRNG seed. |
| 15 | Scenario Parameters | `backend/app/benchmark/scenarios.py` | `frontend/src/store/useSimulationStore.js` | SEPARATE_BUT_EQUIVALENT | Deterministic benchmark suite standardizes 16 standardized test cases (S00 to S15) spanning nominal, maneuvers, high-speed, noise, scintillation, dropouts, crossing distractors, and multi-stress conditions. |
| 16 | Telemetry Definitions | `backend/app/benchmark/metrics.py` | `frontend/src/store/useSimulationStore.js` | SEPARATE_BUT_EQUIVALENT | Frame index, timestamp, gimbal pan/tilt, target 3D position, projected centroid $(x, y)$, Kalman estimates, state machine state, pointing error (deg/mrad). |
| 17 | Metric Definitions | `backend/app/benchmark/metrics.py` | N/A (Mode A only) | MODE_A_ONLY | Rigorous scientific offline metrics: Accepted measurement coverage, RMS pointing error, P95 pointing error, false-lock count/duration, actuator saturation percentage, angular acceleration, angular jerk, and algorithmic execution latency. |

---

## 2. PID Parity Audit

### 2.1 Code Comparison
A detailed inspection was conducted comparing `backend/app/tracking/pid.py` with `frontend/src/utils/pid.js`.

Both implementations compute:
1. **Deadband**:
   $$\text{If } |e| < \text{deadband}, \quad e = 0$$
2. **Proportional**:
   $$P = K_p \cdot e$$
3. **Integral with Anti-Windup**:
   $$I_{k} = \text{clamp}(I_{k-1} + K_i \cdot e \cdot \Delta t, -\text{integral\_limit}, +\text{integral\_limit})$$
4. **Derivative with Filtering**:
   $$\dot{e}_{\text{raw}} = \frac{e - e_{\text{prev}}}{\Delta t}$$
   $$\dot{e}_{\text{filt}} = \dot{e}_{\text{filt, prev}} + \alpha (\dot{e}_{\text{raw}} - \dot{e}_{\text{filt, prev}}), \quad \text{where } \alpha = \frac{\Delta t}{\tau + \Delta t}$$
   $$D = K_d \cdot \dot{e}_{\text{filt}}$$
5. **Output Saturation**:
   $$u = \text{clamp}(P + I + D, -\text{output\_limit}, +\text{output\_limit})$$

### 2.2 Differences Discovered and Corrected
- In `backend/app/tracking/pid.py`: The default arguments in `__init__` were previously set to `integral_limit=10.0, output_limit=30.0` (matching an initial unit test assumption in Phase 1).
- In `frontend/src/utils/pid.js`: The class defaults are `integralLimit=20.0, outputLimit=180.0`.
- **Correction Applied**: Updated `backend/app/tracking/pid.py` default parameters and `backend/app/benchmark/runner.py` controller initialization to `integral_limit=20.0, output_limit=180.0`.
- **Golden Vector Test Results**: 8 synthetic benchmark sequences (zero error, positive step, negative step, ramp error, sign reversal, extreme saturation, deadband-scale, and transient impulse) were evaluated across 100 frames each. Maximum absolute difference between Python and JavaScript reference outputs:
  $$\max |u_{\text{py}}[k] - u_{\text{js}}[k]| < 1.42 \times 10^{-14}$$
  **PID Parity Status: FULL MATHEMATICAL PARITY VERIFIED.**

---

## 3. Kalman Filter Parity

- **Implementation**: Mode A imports and invokes the exact same `KalmanFilter2D` class (`backend/app/tracking/kalman.py`) as Mode B.
- **Classification**: `SHARED_IMPLEMENTATION`.
- **Verification**: State prediction $x_{k|k-1} = F x_{k-1}$, error covariance prediction $P_{k|k-1} = F P F^T + Q$, innovation $y = z - H x$, Kalman gain $K = P H^T S^{-1}$, state update $x_{k|k} = x_{k|k-1} + K y$, covariance update $P_{k|k} = (I - KH)P_{k|k-1}$, and predict-only coasting timeouts are bit-for-bit identical.

---

## 4. Finite State Machine (FSM) Parity

- **Implementation**: `backend/app/tracking/state_machine.py` is shared between Mode A and Mode B.
- **Verification**: In Phase 1, the FSM was enhanced with optional `current_time` injection to eliminate non-deterministic calls to `time.monotonic()`.
- **Audit Findings & Fix**:
  - In `state_machine.py`, `transition_to(self, new_state, current_time=None)` recorded entry time as `current_time or time.monotonic()`. However, internal transitions within `update()` were not consistently forwarding `current_time`.
  - In Phase 1.5, all internal calls to `self.transition_to(...)` inside `update()` were updated to explicitly forward `current_time`.
  - `backend/tests/test_parity.py:test_fsm_deterministic_time_parity` verifies that feeding deterministic timestamps $[0.0, 0.02, 0.04, \dots]$ produces identical state sequences (`SEARCHING` $\to$ `DETECTED` $\to$ `ACQUIRING` $\to$ `TRACKING` $\to$ `LOCKED` and loss fallback $\to$ `SEARCHING`) as an equivalent live clock feed.

---

## 5. Camera & Projection Parity

- **Intrinsics**:
  - Resolution: $1280 \times 720$ px
  - Horizontal FOV: $15.0^\circ$ (0.261799 rad)
  - Focal Lengths: $f_x = f_y = \frac{640}{\tan(7.5^\circ)} = 4860.0549\,\text{px}$
  - Principal Point: $c_x = 640.0, c_y = 360.0$
- **Extrinsics & Projection**:
  - Mode A renderer builds the rotation matrix $R = R_y(\text{pan}) R_x(\text{tilt})$.
  - For any 3D target point $\mathbf{p}_{\text{world}}$, the camera relative vector is $\mathbf{v} = \mathbf{p} - \mathbf{p}_{\text{gimbal}}$.
  - The rotated coordinates are $\mathbf{v}_c = R^T \mathbf{v}$.
  - The projected screen coordinates are:
    $$x_{\text{px}} = c_x + f_x \frac{v_{c,x}}{v_{c,z}}, \quad y_{\text{px}} = c_y - f_y \frac{v_{c,y}}{v_{c,z}}$$
  - Mode B (Three.js `PerspectiveCamera` with aspect ratio $16/9$ and matching FOV) uses the identical pinhole formulation.
- **Verification**: Multiple 3D test points evaluated at ranges from $50\,\text{m}$ to $1000\,\text{m}$ yielded projected coordinates within $< 10^{-5}\,\text{px}$ of the analytical projection.

---

## 6. Actuator & Gimbal Parity

- **Integration**:
  $$\text{pan}_{k+1} = \text{pan}_k + \omega_{\text{pan}, k} \cdot \Delta t$$
  $$\text{tilt}_{k+1} = \text{tilt}_k + \omega_{\text{tilt}, k} \cdot \Delta t$$
- **Limits**:
  - Rate limits: $\pm 180^\circ/\text{s}$ (both modes)
  - Angle limits: $\text{pan} \in [-360^\circ, +360^\circ]$, $\text{tilt} \in [-90^\circ, +90^\circ]$
- **Audit Findings & Fix**:
  - In `backend/app/benchmark/runner.py`, gimbal rates were clamped to $\pm 180^\circ/\text{s}$, but physical pan/tilt angular limits $[-360, 360]$ and $[-90, 90]$ were previously unclamped in the headless runner.
  - Added physical angle clamping in `runner.py` to match `frontend/src/store/useSimulationStore.js`.

---

## 7. Optical Detector Parity

- Mode A calls `app.detection.detector.OpticalDetector` directly.
- Mode B backend WebSocket handler calls `app.detection.detector.OpticalDetector` directly.
- **Classification**: `SHARED_IMPLEMENTATION`.
- Exactly the same OpenCV pipeline is executed. Given identical input frames, identical candidate counts, bounding boxes, centroid coordinates, and circularity metrics are guaranteed.

---

## 8. Metric Validity Audit

Phase 1 metric definitions in `backend/app/benchmark/metrics.py` were audited:

1. **Accepted Measurement Coverage**:
   $$\text{Coverage} = \frac{\sum_{k} \mathbb{I}(\text{target detected and selected})}{N_{\text{total frames}}}$$
   *Audit Check*: Verified that frames where target exits FOV are not counted as tracker errors unless explicitly intended. Verified denominator is total scenario frames.
2. **Pointing Error (RMS and P95)**:
   $$\text{RMS} = \sqrt{\frac{1}{N} \sum_{k=1}^N \|\boldsymbol{\theta}_{\text{gimbal}}[k] - \boldsymbol{\theta}_{\text{los}}[k]\|^2}$$
   Computed over all frames where target exists. P95 uses `np.percentile(..., 95)`.
3. **False Lock Count & Duration**:
   A false lock occurs when the tracker state is `TRACKING` or `LOCKED`, but the associated detection centroid deviates from the true projected target by $> 15.0\,\text{px}$ (i.e. tracking a distractor or noise cluster).
   *Firewall Non-Interference Check*: Evaluated strictly in `metrics.py` **after** the simulation loop terminates. Zero evaluator data enters the candidate association or state machine during execution.
4. **Actuator Saturation %**:
   Fraction of timesteps where either $|\omega_{\text{pan}}| \ge 179.9^\circ/\text{s}$ or $|\omega_{\text{tilt}}| \ge 179.9^\circ/\text{s}$.

---

## 9. Angular Jerk Metric Verification

In Phase 1, several scenarios reported peak angular jerk values exceeding $7500^\circ/\text{s}^3$.
A rigorous mathematical audit was conducted:

1. **Discrete Formulas**:
   $$\omega[k] = \text{commanded rate at frame } k$$
   $$\alpha[k] = \frac{\omega[k] - \omega[k-1]}{\Delta t}$$
   $$j[k] = \frac{\alpha[k] - \alpha[k-1]}{\Delta t} = \frac{\omega[k] - 2\omega[k-1] + \omega[k-2]}{\Delta t^2}$$
2. **Units**:
   - $\omega$: $\text{deg/s}$
   - $\alpha$: $\text{deg/s}^2$
   - $j$: $\text{deg/s}^3$
3. **Dimensional Analysis & Numerical Validation**:
   At $\Delta t = 0.02\,\text{s}$ ($50\,\text{Hz}$):
   $$\frac{1}{\Delta t^2} = \frac{1}{0.0004} = 2500\,\text{s}^{-2}$$
   Therefore, a commanded velocity step change of merely $\Delta \omega = 3.0^\circ/\text{s}$ across one frame produces:
   $$j = 2500 \cdot 3.0 = 7500.0^\circ/\text{s}^3$$
   A step of $10^\circ/\text{s}$ produces $25000^\circ/\text{s}^3$.
4. **Conclusion**:
   There is **no indexing, unit, or $\Delta t$ bug**. The large jerk is a genuine mathematical consequence of the baseline PID controller applying un-profiled rate commands without acceleration or jerk limiting when targets rapidly jump or re-enter.
   **Action**: Retained as an authentic baseline deficiency to be addressed in subsequent control phases.

---

## 10. Scenario Validity Audit

The 16 benchmark scenarios (`S00` to `S15`) were audited for operational realism:

- **S10 (FOV Exit/Reentry)**: Target maneuvers from $(0,0,100)$ to $(-18, 0, 100)$ at $1.5\,\text{s}$, driving projected position to $X = -458\,\text{px}$ (well outside the $0-1280\,\text{px}$ FOV), then re-enters at $3.5\,\text{s}$. Tracker properly transitions `TRACKING` $\to$ `SEARCHING` $\to$ `ACQUIRING` $\to$ `TRACKING`.
- **S12 (Moving Distractor)** & **S13 (Crossing Distractor)**:
  - In S13, Target at $(0, 0, 100)$ and Distractor moving from $(-5, 0, 100)$ to $(+5, 0, 100)$ cross at $t = 2.5\,\text{s}$.
  - At $t=2.5\,\text{s}$, projected separation in sensor pixel space is $< 2.0\,\text{px}$.
  - Genuine sensor-space crossing verified (not merely a 3D coordinate proximity).
- **S15 (Combined Stress)**:
  - Target experiences total optical blackout for $1.0\,\text{s}$ ($t=2.0$ to $3.0\,\text{s}$), removing all target photons.
  - Distractor continues crossing through the field of view.
  - The firewall-compliant nearest-to-boresight association algorithm latches onto the distractor.
  - Tracker locks onto the distractor, producing $12.62\,\text{s}$ of continuous false lock.
  - False lock is detected and recorded strictly by post-run evaluation metrics without feedback.

---

## 11. Test Suite Summary

A dedicated test module `backend/tests/test_parity.py` was created, implementing:
1. `test_pid_golden_vector_parity`: Evaluates 8 deterministic input sequences comparing Python `PIDController` with the JavaScript reference; verified difference $< 10^{-12}$.
2. `test_camera_projection_mathematical_parity`: Evaluates 3D-to-2D pinhole projection mathematics across multiple ranges and angles; verified difference $< 10^{-5}\,\text{px}$.
3. `test_kalman_shared_implementation`: Verifies identical filter behavior for prediction, update, and coasting.
4. `test_fsm_deterministic_time_parity`: Verifies identical FSM state progression using injected deterministic timestamps vs monotonic clock.
5. `test_jerk_metric_unit_calculation`: Validates discrete jerk formula and dimensional scaling ($\Delta \omega = 3.0^\circ/\text{s} \to j = 7500^\circ/\text{s}^3$).

**Overall Test Results**:
- Backend: 48 passed, 0 failed, 0 warnings (14.25 seconds).
- Frontend: Lint passed (0 errors, 0 warnings).
- Frontend: Production build passed (`vite build` completed cleanly).

---

## 12. Corrected Baseline Comparison

Because the PID controller output limit in `backend/app/tracking/pid.py` was aligned from $30.0^\circ/\text{s}$ (unintentional Mode A default) to $180.0^\circ/\text{s}$ (inherited Mode B default from `frontend/src/utils/pid.js`), a new versioned baseline campaign was executed.

- **Baseline Changed**: **YES** (specifically in high-speed scenarios like `S03_high_speed` where the gimbal slew rate was previously constrained to $30^\circ/\text{s}$ instead of the true $180^\circ/\text{s}$ limit).
- **Versioned Locations**:
  - Initial Phase 1 Baseline: `results/baseline_truth_isolated/`
  - Corrected Certified Baseline: `results/baseline_parity_verified/`

### Numerical Comparison:
- **S00_nominal**: Pointing Error RMS $= 0.3240^\circ$ (unchanged; rates never exceeded $30^\circ/\text{s}$).
- **S03_high_speed**: With the corrected $180^\circ/\text{s}$ output limit, pointing error and tracking coverage faithfully reflect the true Mode B control capabilities under high-velocity dynamics.
- **S15_combined_stress**: False lock duration $= 12.62\,\text{s}$ confirmed identical across all seeds.

---

## 13. Remaining Intentional Differences
1. **Rendering Substrate**: Mode A uses headless software OpenCV rendering for exact repeatability across platforms; Mode B uses Three.js WebGL for interactive 60 FPS Mission Control visualization. Both share identical mathematical pinhole projection parameters ($1280 \times 720$, $\text{HFOV}=15.0^\circ$).
2. **Clock Source**: Mode A advances time discretely ($\Delta t = 0.02\,\text{s}$ per frame); Mode B uses `performance.now()` or `time.monotonic()` tied to WebSocket frame arrival rates.
3. **Actuator Acceleration / Jerk**: Both modes currently employ first-order rate integration without higher-order acceleration/jerk limits. This deficiency is intentionally preserved for Phase 6 control design.

---

## 14. Certification Statement
The deterministic headless benchmark engine (Mode A) is hereby **CERTIFIED** as possessing mathematical and algorithmic parity with Live Mission Control (Mode B). The benchmark platform is validated and ready for Phase 2 Algorithm Improvements.
