# Ground-Truth Firewall Audit Report (Phase 0D)

**Project:** SIH26169 — AI-Based Virtual Camera Tracking System for Coarse Alignment of Mobile FSOC Terminals  
**Role:** Senior Aerospace Software Architect, Computer Vision Engineer, Control Systems Engineer & Experimental Validation Engineer  
**Date:** September 2026  
**Document Version:** 1.0.0  
**Phase:** 0D — Ground-Truth Firewall & Mathematical Isolation  
**Status:** FULLY CERTIFIED & ISOLATED  

---

## 1. Executive Summary

In Phase 0D, the operational tracking pipeline of `FSOC-AI-PAT/` was mathematically and physically decoupled from all simulator ground-truth states. 

Prior to this phase, the inherited prototype contained two critical ground-truth leakages:
1. **Candidate Association Cheat (`backend/app/main.py`):** Visual candidate detections were filtered against projected 3D ground truth (`gt_beacons`), dropping any visual detection $> 100\,\text{px}$ from truth and bypassing autonomous candidate association.
2. **Controller Cheat Fallback (`frontend/src/scene/GimbalCamera.jsx`):** When the Kalman filter state was inactive or ablated, the gimbal camera controller directly projected `currentBeaconPosition` (the true 3D scenario coordinate) to calculate azimuth/elevation angular error and drive `panPid` / `tiltPid`.

Both leakages have been completely eliminated from the production codebase. The operational loop now adheres strictly to the canonical aerospace paradigm:
$$\text{PHOTONS} \longrightarrow \text{DETECTIONS} \longrightarrow \text{MEASUREMENTS} \longrightarrow \text{ESTIMATOR} \longrightarrow \text{CONTROL}$$
with zero operational feedback or filtering from world truth.

---

## 2. Repository-Wide Ground-Truth Classification Matrix

An exhaustive AST and regular expression search was executed across all source files in `FSOC-AI-PAT/` targeting:
`beacon world position`, `ground truth`, `target id`, `projected truth`, `gt_`, `truth`, and `currentBeaconPosition`.

Every occurrence has been cataloged and classified under three strict categories:
- **`RENDERING_ALLOWED`**: Ground-truth coordinates utilized exclusively by the 3D graphics engine (Three.js/WebGL) or physical simulation engine to update world positions and render the photon field onto the synthetic camera sensor plane.
- **`EVALUATION_ALLOWED`**: Ground-truth coordinates ingested strictly in post-processing or decoupled evaluation loggers to evaluate true pointing error, localization RMSE, and link margin for post-run reporting.
- **`OPERATIONAL_FORBIDDEN`**: Ground-truth coordinates accessed, inspected, or utilized by candidate selection, association, Kalman estimation, state machine transitions, or PID/gimbal actuation. **ALL occurrences have been eliminated.**

### Detailed Inventory

| File Path | Line(s) | Symbol / Expression | Former Role | Phase 0D Classification | Phase 0D Status / Action Taken |
| :--- | :--- | :--- | :--- | :--- | :--- |
| `backend/app/main.py` | 484–524 | `gt_beacons = {}`, `designated_gt_` | Projected 3D target coordinates prior to detection/tracking | `OPERATIONAL_FORBIDDEN` | **REMOVED** from tracking loop; relocated strictly into post-tracking offline evaluation block (Step 9). |
| `backend/app/main.py` | 558–584 | `gt_beacons[best_match_id]` | Matched detected visual candidates against projected 3D truth | `OPERATIONAL_FORBIDDEN` | **REMOVED**. Replaced with autonomous candidate selection based purely on observable confidence, area, and internal filter continuity. |
| `backend/app/main.py` | 593–605 | `status = "occluded" if gt_b["in_fov"]` | Injected missed ground-truth beacons into telemetry candidate list | `OPERATIONAL_FORBIDDEN` | **REMOVED**. Candidate telemetry now reflects only observable visual detections. |
| `backend/app/main.py` | 608–628 | `designated_gt_3d`, `designated_gt_dir` | Passed to `metrics_tracker.log_frame` | `EVALUATION_ALLOWED` | **RETAINED** strictly for post-run offline error calculation and benchmark PDF generation. |
| `frontend/src/scene/GimbalCamera.jsx` | 101 | `currentBeaconPosition = store.beaconPosition` | Fetched 3D ground-truth coordinates into camera controller | `OPERATIONAL_FORBIDDEN` | **REMOVED**. Variable eliminated from component scope. |
| `frontend/src/scene/GimbalCamera.jsx` | 142–161 | `targetWorld = new Vector3(...currentBeaconPosition)` | Projected world truth to compute angular error and steer PID | `OPERATIONAL_FORBIDDEN` | **REMOVED**. Replaced with zero-error holding attitude fallback (`totalErrorDeg = 0`, PID reset). |
| `frontend/src/scene/GimbalCamera.jsx` | 240–250 | `beacons: storeState.beacons.map(...)` | Bundled world scenario state in frame capture metadata | `EVALUATION_ALLOWED` | **RETAINED**. Allows backend evaluation engine to evaluate true error for benchmark logs. |
| `frontend/src/store/useSimulationStore.js` | 201–279 | `updateBeaconPosition` | Computes 3D trajectory dynamics for WebGL rendering | `RENDERING_ALLOWED` | **RETAINED**. World physics engine generating 3D light emitters. |
| `frontend/src/store/useSimulationStore.js` | 329–350 | `setDesignatedTarget: (id)` | UI setting target designation for scenario manager | `RENDERING_ALLOWED` | **RETAINED**. Scenario configuration only. |
| `frontend/src/components/CameraFeed.jsx` | 8–139 | `targetInsideFOV`, `trackedPixelX, Y` | HUD display overlay of tracking status | `EVALUATION_ALLOWED` | **RETAINED**. Visual presentation of tracked point. |
| `backend/app/tracking/metrics.py` | 25–266 | `ground_truth_3d_position`, `gt_x, gt_y, gt_z` | Evaluation metrics recording & CSV export | `EVALUATION_ALLOWED` | **RETAINED**. Offline scientific validation. |
| `backend/tests/test_metrics.py` | 18–130 | `ground_truth_3d_position` | Unit tests for metrics logger | `EVALUATION_ALLOWED` | **RETAINED**. Test verification of metrics tracker. |

---

## 3. Operational Decoupling Architecture

### 3.1 Backend Autonomous Candidate Selection
In `backend/app/main.py`, Step 5 now executes completely independently of scenario ground truth:

```python
# 5. Autonomous Candidate Selection (Zero Ground-Truth Dependency)
detected_candidates = det_res.get("candidates", [])
target_detected = False
target_center = [0.0, 0.0]
target_bbox = [0, 0, 0, 0]
target_confidence = 0.0

if len(detected_candidates) == 1:
    best_cand = detected_candidates[0]
    target_detected = True
    target_center = best_cand["center"]
    target_bbox = best_cand["bbox"]
    target_confidence = best_cand.get("confidence", 0.0)
elif len(detected_candidates) > 1:
    # Multiple candidates: select based on observable continuity, confidence, and area
    if sim_state.ablation.get("kalman_filter", True) and kalman_filter.is_initialized:
        prev_pos = kalman_filter.state[:2]
        best_cand = min(
            detected_candidates,
            key=lambda c: (
                float(np.hypot(c["center"][0] - prev_pos[0], c["center"][1] - prev_pos[1])),
                -float(c.get("confidence", 0.0)),
                -float(c.get("area", 0.0))
            )
        )
    else:
        best_cand = max(
            detected_candidates,
            key=lambda c: (float(c.get("confidence", 0.0)), float(c.get("area", 0.0)))
        )
    target_detected = True
    target_center = best_cand["center"]
    target_bbox = best_cand["bbox"]
    target_confidence = best_cand.get("confidence", 0.0)
```

### 3.2 Frontend Zero-Cheating Fallback
In `frontend/src/scene/GimbalCamera.jsx`, the tracking controller receives angular error exclusively from the active tracking estimator:

```javascript
if (currentKalmanState.active) {
  // Driven purely by optical estimate / Kalman extrapolation
  const xPred = currentKalmanState.x + currentKalmanState.vx * elapsed;
  const yPred = currentKalmanState.y + currentKalmanState.vy * elapsed;
  ...
  hErrorDeg = THREE.MathUtils.radToDeg(Math.atan2(dx, fx));
  vErrorDeg = THREE.MathUtils.radToDeg(Math.atan2(dy, fy));
  totalErrorDeg = Math.sqrt(hErrorDeg * hErrorDeg + vErrorDeg * vErrorDeg);
} else {
  // Zero ground-truth leakage: when tracker is inactive, controller has no target measurement
  u = resolutionWidth / 2;
  v = resolutionHeight / 2;
  isInside = false;
  totalErrorDeg = 0.0;
  hErrorDeg = 0.0;
  vErrorDeg = 0.0;
  panPid.current.reset();
  tiltPid.current.reset();
}
```

---

## 4. Automated Verification Suite (`test_truth_firewall.py`)

Seven dedicated unit, regression, and AST inspection tests were implemented in `backend/tests/test_truth_firewall.py`:

1. **`test_candidate_selection_without_gt_beacons`:** Confirms candidate selection operates purely on optical confidence and area metrics without `gt_beacons`.
2. **`test_detection_enters_estimator_without_ground_truth`:** Confirms optical measurements update `KalmanFilter2D` with zero ground-truth knowledge.
3. **`test_ground_truth_non_interference` (REGRESSION TEST):** Sends an identical optical frame under two radically conflicting truth scenarios:
   - Scenario Alpha: target at $(50, 20, -100)\,\text{m}$.
   - Scenario Beta: target falsified to $(-9999, -8888, -7777)\,\text{m}$ with inactive flag and bogus target ID.
   - **Verification:** Asserted that detected pixel center, bounding box, Kalman state $(x, y)$, angular errors $(h, v, \text{total})$, and FSM tracking state are **BITWISE IDENTICAL**.
4. **`test_complete_image_blackout_yields_zero_target_truth`:** Feeds a completely black frame while metadata claims a target directly on boresight. Confirms `detected == False`, `raw_center == [0.0, 0.0]`, and zero false candidates.
5. **`test_ablation_kalman_off_does_not_leak_world_coordinates`:** Confirms that with Kalman filter ablated, output coordinates reflect only real optical detections, never world coordinates.
6. **`test_evaluation_metrics_can_access_truth`:** Validates that offline metrics tracking correctly records truth for post-run evaluation without leaking back to tracking.
7. **`test_codebase_ast_truth_quarantine`:** Static AST analysis scanning `backend/app/main.py` certifying that `gt_beacons` is completely absent from the abstract syntax tree.

---

## 5. Verification Results Summary

* **Backend Unit & Regression Suite:**
  ```powershell
  $env:PYTHONPATH="backend"; .\.venv\Scripts\pytest backend/tests
  ```
  **Result:** **37 passed, 2 warnings in 3.06s** (100% pass rate across all 37 tests).
* **Frontend Static Analysis:**
  ```powershell
  npm run lint
  ```
  **Result:** **0 errors**, 33 warnings across 17 files (`currentBeaconPosition` unused warning eliminated).
* **Frontend Production Bundle:**
  ```powershell
  npm run build
  ```
  **Result:** **Build succeeded in 985ms** (`dist/assets/index-*.js` 1,574.42 kB).
* **Live WebSocket Closed-Loop Run:**
  Executed live 10-frame session with falsified ground-truth metadata; verified seamless autonomous optical detection, Kalman convergence at $(640.5, 360.5)\,\text{px}$, and correct angular error computation.

---

## 6. Known Limitations of Phase 0D Baseline

1. **Simplified Association Policy:** In Phase 0D, multi-candidate disambiguation uses nearest-neighbor continuity and confidence/area ranking. Full Mahalanobis statistical gating and multi-feature cost matrices will be introduced in Phase 3.
2. **Attitude-Hold on Target Loss:** When target detection drops out, the gimbal currently holds attitude (zero error correction). Covariance-guided spiral reacquisition will be introduced in Phase 7.
3. **Integer Centroid Quantization:** Optical detections currently use integer contour bounding-box centers ($\pm 1.5\,\text{px}$ quantization). Sub-pixel intensity moment centroiding will be introduced in Phase 2.

---

**Certification:** The ground-truth firewall is certified active and mathematically isolated.  
**Senior Systems Architect Sign-Off:** READY FOR PHASE 1.
