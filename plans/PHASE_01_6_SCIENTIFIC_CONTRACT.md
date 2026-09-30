# Phase 1.6 — Camera Geometry and Scientific Metric Contract Certification

## Executive Summary
This document establishes and locks the authoritative **Camera Calibration Contract** and **Scientific Evaluation Metric Contracts** for the FSOC-AI-PAT platform (SIH26169).

Prior to Phase 1.6, camera projection in Mode A utilized independent horizontal and vertical FOV definitions ($10.0^\circ$ and $7.5^\circ$) that differed by $0.11\%$ from strict square-pixel pinhole geometry, and evaluation metrics collapsed observable vs mission coverage and spatial tracking error vs candidate identity false-lock. Phase 1.6 formalizes the optical geometry, establishes a unified camera calibration contract matching Three.js WebGL conventions, enforces segregated observability flags and population statistics, and validates the complete benchmark campaign in `results/baseline_scientifically_certified/`.

---

## 1. Camera Calibration Contract

### 1.1 FOV Convention Analysis
An audit of `frontend/src/scene/GimbalCamera.jsx` and `@react-three/drei` revealed the exact camera constructor:
```jsx
<PerspectiveCamera
  ref={cameraRef}
  fov={vfov}
  aspect={resolutionWidth / resolutionHeight}
  near={0.1}
  far={10000}
  position={[0, 0, -0.2]}
/>
```
In Three.js (`THREE.PerspectiveCamera`), the `fov` constructor argument strictly represents **Camera Frustum Vertical Field of View (VFOV) in degrees**. Three.js computes horizontal bounds dynamically via:
$$\tan\left(\frac{\text{HFOV}}{2}\right) = \text{aspect} \cdot \tan\left(\frac{\text{VFOV}}{2}\right)$$
Therefore, **VFOV is the authoritative FOV parameter**.

### 1.2 Authoritative Pinhole Formulation
For sensor resolution $W \times H = 640 \times 480$ ($4:3$ aspect ratio) and $\text{VFOV} = 7.5^\circ$:
1. **Vertical Focal Length**:
   $$f_y = \frac{H / 2}{\tan(\text{VFOV} / 2)} = \frac{240}{\tan(3.75^\circ)} = 3661.723149\,\text{px}$$
2. **Square Pixel Constraint** ($f_x = f_y$):
   $$f_x = f_y = 3661.723149\,\text{px}, \quad \text{pixel\_aspect\_ratio} = 1.0$$
3. **Derived Horizontal Field of View (HFOV)**:
   $$\text{HFOV} = 2 \cdot \arctan\left(\frac{W / 2}{f_x}\right) = 2 \cdot \arctan\left(\frac{320}{3661.723149}\right) = 9.988928^\circ$$
4. **Principal Point**:
   $$c_x = \frac{W}{2} = 320.0\,\text{px}, \quad c_y = \frac{H}{2} = 240.0\,\text{px}$$

### 1.3 Authoritative Contract Specification
Implemented in [`backend/app/camera/calibration.py`](file:///c:/Projects/SIH26169-WORKSPACE/FSOC-AI-PAT/backend/app/camera/calibration.py) as `CameraCalibration`:
- `resolution_width`: $640$
- `resolution_height`: $480$
- `fov_axis`: `"vertical"` (authoritative)
- `vfov_deg`: $7.5^\circ$
- `hfov_deg`: $9.988928^\circ$
- `fx_px`: $3661.723149$
- `fy_px`: $3661.723149$
- `cx_px`: $320.0$
- `cy_px`: $240.0$
- `pixel_aspect_ratio`: $1.0$ (square pixels)
- `coordinate_handedness`: `"right_handed"`
- `camera_forward_axis`: `"-Z"`
- `pixel_origin`: `"top_left"`
- `pixel_x_direction`: `"right"`
- `pixel_y_direction`: `"down"`
- `pan_sign`: `"positive_clockwise_around_plus_Y"`
- `tilt_sign`: `"positive_upward_pitch_around_plus_X"`

---

## 2. Projection Parity & Round-Trip Validation

### 2.1 Matrix-Level 4x4 View-Projection Parity
`CameraModel.project_threejs` implements the full Three.js WebGL transformation pipeline:
$$P_{\text{clip}} = M_{\text{proj}} \cdot V_{\text{view}} \cdot P_{\text{world}}$$
$$\text{ndc} = \frac{P_{\text{clip}}[0:2]}{P_{\text{clip}}[3]}$$
$$u = (\text{ndc}_x + 1) \cdot \frac{W}{2}, \quad v = (1 - \text{ndc}_y) \cdot \frac{H}{2}$$
Tested across 11 diverse test cases including boresight, horizontal/vertical offsets, combined diagonal, near-edge, outside-FOV, and nonzero pan/tilt attitudes ($+5^\circ$ pan, $+3.5^\circ$ tilt, general 3D attitude).
- **Maximum Discrepancy**: $< 10^{-10}\,\text{px}$ across all valid projections.

### 2.2 Pixel-to-Angle Round Trip
For any sensor coordinate $(u, v)$:
$$\theta_h = \arctan\left(\frac{u - c_x}{f_x}\right), \quad \theta_v = \arctan\left(\frac{c_y - v}{f_y}\right)$$
$$\theta_{\text{total}} = \arccos\left(\frac{1}{\sqrt{1 + \tan^2\theta_h + \tan^2\theta_v}}\right)$$
Tested across $0.0^\circ, 0.1^\circ, 0.5^\circ, 1.0^\circ, 2.5^\circ$, and near-boundary $4.5^\circ$.
- **Maximum Angle Recovery Error**: $< 10^{-10\,\circ}$.

---

## 3. Observability Contract

To prevent conflating environmental phenomena with tracking failures, each frame logs four distinct observability flags:
1. `ground_truth_target_active`: Target beacon emission is turned on in the world (`is_active == True`).
2. `ground_truth_target_in_fov`: Target 3D position in camera frame satisfies $z_c < 0$ and $0 \le u \le W, 0 \le v \le H$.
3. `ground_truth_target_optically_present`: Target photons physically reach sensor (`active and in_fov and depth > 1.0m` and optical transmission $> 0$).
4. `ground_truth_target_detectable`: Target is optically present AND spot irradiance/SNR exceeds detector noise floor ($\tau = 200$).

### Semantics Summary:
- **Target Outside FOV**: `active = True, in_fov = False, optically_present = False`.
- **Intentional Blackout**: `active = False, in_fov = True, optically_present = False`.
- **Atmospheric Fading**: `active = True, in_fov = True, optically_present = True, detectable = (intensity > 0.1)`.

---

## 4. Scientific Metric Definitions

### 4.1 Perception & Coverage
- **Detector Recall (Observable)**:
  $$\text{Recall}_{\text{obs}} = \frac{\text{True Positives}}{\text{Frames with } \text{target\_optically\_present}}$$
- **Detector Recall (Mission)**:
  $$\text{Recall}_{\text{mission}} = \frac{\text{True Positives}}{\text{Frames with } \text{target\_active}}$$
- **Observable Measurement Coverage**:
  $$\text{Coverage}_{\text{obs}} = \frac{\text{Accepted Detections while Observable}}{\text{Frames with } \text{target\_optically\_present}} \times 100\%$$
- **Mission Measurement Coverage**:
  $$\text{Coverage}_{\text{mission}} = \frac{\text{Accepted Detections}}{\text{Total Mission Frames}} \times 100\%$$

### 4.2 Pointing Error Populations
Pointing error is now partitioned and reported across three distinct populations:
1. `all_active_frames`: Entire duration target beacon is active (penalizes dropouts / lost tracking).
2. `target_in_fov`: Frames where target is geometrically inside camera field of view.
3. `locked_frames`: Frames where tracker state is `LOCKED` (steady-state pointing performance).

### 4.3 Lock Retention
- `lock_retention_when_observable_pct`: Fraction of optically observable frames spent in `LOCKED` or `TRACKING`.
- `mission_lock_retention_pct`: Fraction of total mission duration spent in `LOCKED` or `TRACKING`.

---

## 5. False Lock & Identity Switch Definitions

### 5.1 Identity-Based False Lock
- **Core Principle**: A large spatial tracking error while following the designated target is **TRACKING ERROR**, NOT a false lock.
- **Definition**: A False Lock occurs when the operational tracker associates and tracks a non-designated physical candidate (distractor beacon or clutter artifact) while in `TRACKING` or `LOCKED`.
- **Persistence Threshold**: The tracker must continuously track the wrong target for $\ge 0.5\,\text{s}$ ($25$ consecutive frames at $50\,\text{Hz}$).
- **Evaluation Flow**:
  1. Operational candidate selected (zero ground truth access).
  2. Operational decision frozen.
  3. Evaluation-only candidate-to-truth nearest neighbor identification.
  4. Compare identified beacon ID with `scenario.designated_target_id`.
  5. If non-designated target persists $\ge 0.5\,\text{s}$: increment `false_lock_event_count` and accumulate `false_lock_duration_s`.

### 5.2 Identity Switches
Transitions between distinct physical targets are tracked and counted:
$$\text{BEACON-01} \leftrightarrow \text{DISTRACTOR-01} \leftrightarrow \text{DISTRACTOR-02} \leftrightarrow \text{CLUTTER}$$
Recorded as `identity_switch_count`.

### 5.3 Target Loss Causal Accounting
When transitioning out of `TRACKING` or `LOCKED` into `TARGET_LOST` or `SEARCHING`, the causal trigger is recorded:
- `OUTSIDE_FOV`: Target exited sensor boundaries.
- `INTENTIONAL_BLACKOUT`: Optical signal extinguished.
- `DETECTOR_MISS`: Target present in FOV but 0 candidates detected.
- `ASSOCIATION_REJECTION`: Detections found but rejected by association gate.
- `ESTIMATOR_EXPIRATION`: Kalman filter coasting duration exceeded 2.0s timeout.

---

## 6. Scenario Sanity Check Results

### 6.1 S09 Temporary Dropout
- Target experiences intentional optical blackout between $t=4.0\,\text{s}$ and $t=5.5\,\text{s}$ ($1.5\,\text{s}$).
- **Observable Coverage**: $100.0\%$ (all 501 observable frames detected).
- **Mission Coverage**: $83.67\%$ (450 detected / 538 total).
- **Loss Diagnosis**: Recorded exactly 1 `INTENTIONAL_BLACKOUT` event.

### 6.2 S10 FOV Exit & Reentry
- Target holds near boresight for $t < 2.0\,\text{s}$, maneuvers rapidly to $x = 28.0\,\text{m}$ ($15.6^\circ$ at $100\,\text{m}$, far outside the $5^\circ$ half-FOV) from $t=2.0$ to $7.0\,\text{s}$, then re-enters at $t \ge 7.0\,\text{s}$.
- **Observable Coverage**: $100.0\%$ (detected on all 450 frames when inside FOV).
- **Mission Coverage**: $64.29\%$ ($450 / 700$ frames).
- **Pointing Error Segregation**:
  - `target_in_fov`: RMS $= 0.6652^\circ$, P95 $= 1.5778^\circ$.
  - `all_active_frames`: RMS $= 9.4477^\circ$, P95 $= 15.8492^\circ$.
- **Loss Diagnosis**: Recorded exactly 1 `OUTSIDE_FOV` loss event.
- **Recovery**: Re-acquired and returned to `TRACKING` upon re-entry.

### 6.3 S15 Combined Stress Mission
- During a $1.0\,\text{s}$ optical blackout, the tracker associates a crossing distractor.
- **False Lock Events**: $2$ events.
- **False Lock Duration**: $12.62\,\text{s}$.
- **Identity Switches**: $14$ transitions.
- **Audit Conclusion**: Confirmed as genuine operational association failure under the newly formalized identity-based definition.

---

## 7. Baseline Version Status

- **Operational Baseline Changed**: **YES** (Camera calibration aligned to exact square pixels $f_x = f_y = 3661.72\,\text{px}$, and S10 trajectory updated so target genuinely exits the sensor FOV).
- **Baseline Storage**:
  - `results/baseline_truth_isolated/`: Historical Phase 1 initial baseline.
  - `results/baseline_parity_verified/`: Phase 1.5 PID parity-verified baseline.
  - `results/baseline_scientifically_certified/`: **Authoritative Scientifically Certified Baseline**.

---

## 8. Automated Test Suite
- `backend/tests/test_scientific_contract.py`: 8 comprehensive unit tests.
- Total Backend Test Suite: **56 passed, 0 failed** in 9.07s.
- Frontend Lint: **0 errors**.
- Frontend Build: **Clean** in 1.64s.

---

## 9. Roadmap Phase Numbering Alignment
In accordance with the locked project roadmap:
- Phase 2: High-Precision Target Localization
- Phase 3: Robust Spatial-Temporal Association & Distractor Rejection
- Phase 4: Adaptive Kinematic State Estimation (6-state Kalman)
- Phase 5: Intelligent Finite State Machine & Autonomous Spiral Reacquisition
- Phase 6: Actuator Dynamics, Feedforward Compensation & Motion Smoothing (including acceleration and jerk limiting)

All documentation references in `plans/PHASE_01_5_PARITY_AUDIT.md` have been aligned to reflect Phase 6 for control dynamics.
