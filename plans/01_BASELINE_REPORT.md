# Phase 0B — Baseline Reproduction and Execution Audit Report
**Project:** SIH26169 — Development of an AI-Based Virtual Camera Tracking System for Coarse Alignment of Mobile Free Space Optical Communication (FSOC) Terminals  
**Author:** Senior Systems Architect & Lead Development Engineer  
**Date:** September 2026  
**Document Version:** 1.0.0  
**Status:** Completed Phase 0B Deliverable  

---

## Executive Summary

Phase 0B establishes the exact inherited baseline of the `FSOC-AI-PAT/` development project. In accordance with Phase 0B rules:
- No architectural changes or algorithmic improvements were introduced.
- Known ground-truth leakages were **not** fixed in this phase; instead, they were rigorously identified, empirically verified, and classified.
- The repository was audited for environment dependencies, build systems, test suites, and operational behavior.
- All 30 existing backend automated unit tests were executed and passed.
- End-to-end closed-loop communication over WebSockets and REST was verified and measured.
- All measured tracking and control figures are explicitly designated:  
  **`LEGACY BASELINE — NOT VALID FOR FINAL PERFORMANCE CLAIMS`**.

---

## 1. Environment

| Item | Value | Verification Source |
| :--- | :--- | :--- |
| **Operating System** | Microsoft Windows 11 Home Single Language | `systeminfo` |
| **OS Version / Build** | 10.0.26200 N/A Build 26200 | `systeminfo` |
| **Python Version** | Python 3.12.10 | `python --version` |
| **pip Version** | pip 25.2 | `pip --version` |
| **Node.js Version** | v24.14.0 | `node --version` |
| **npm Version** | 11.6.2 | `npm --version` |
| **CPU Hardware** | Intel(R) Core(TM) Ultra 5 225H | `Get-CimInstance Win32_Processor` |
| **GPU Hardware** | Intel(R) Graphics (integrated) | `Get-CimInstance Win32_VideoController` |
| **Repository Branch** | `fsoc-winning-v1` | `git branch --show-current` |
| **Git Commit Hash** | `41c070b63d5cfe2bd234f19b4a7f277a7e2d2ff1` | `git rev-parse HEAD` |

---

## 2. Exact Installation Commands

### 2.1 Backend Virtual Environment & Dependency Installation
```powershell
# Inside c:\Projects\SIH26169-WORKSPACE\FSOC-AI-PAT
python -m venv .venv
.\.venv\Scripts\pip install -r backend/requirements.txt
```

### 2.2 Frontend Dependency Installation
```powershell
# Inside c:\Projects\SIH26169-WORKSPACE\FSOC-AI-PAT\frontend
npm install
```

---

## 3. Exact Backend Command

To start the FastAPI backend server with correct module path resolution:
```powershell
# Inside c:\Projects\SIH26169-WORKSPACE\FSOC-AI-PAT
$env:PYTHONPATH="backend"
.\.venv\Scripts\uvicorn app.main:app --host 127.0.0.1 --port 8000
```
*Working process verified running on PID with active WebSocket endpoint at `/ws/tracking` and REST at `/api/*`.*

---

## 4. Exact Frontend Command

To start the Vite / React 18 development server:
```powershell
# Inside c:\Projects\SIH26169-WORKSPACE\FSOC-AI-PAT\frontend
npm run dev
```
*Working process verified on local port 5173.*

---

## 5. Local Application URLs

| Service / Interface | Local URL | Status |
| :--- | :--- | :--- |
| **Frontend Web Dashboard** | `http://localhost:5173/` | Operational (HTTP 200 OK) |
| **Backend REST API Root** | `http://127.0.0.1:8000/` | Operational |
| **Backend Status Endpoint** | `http://127.0.0.1:8000/api/status` | Operational (`{"status": "online"}`) |
| **Backend Scenarios API** | `http://127.0.0.1:8000/api/scenarios` | Operational (10 scenarios returned) |
| **Backend WebSocket Stream** | `ws://127.0.0.1:8000/ws/tracking` | Operational (Bi-directional JSON/Base64) |

---

## 6. Test Results

### 6.1 Backend Automated Tests (pytest)
Executed via command: `$env:PYTHONPATH="backend"; .\.venv\Scripts\pytest backend/tests`
- **Total Test Files:** 7
- **Total Tests Collected:** 30
- **Total Tests Passed:** 30
- **Total Tests Failed:** 0
- **Execution Duration:** 4.56 s

| Test File | Tests Passed | Key Capabilities Verified |
| :--- | :---: | :--- |
| `backend/tests/test_api.py` | 4 / 4 | Status, detector mode toggle, metrics reset, scenarios list |
| `backend/tests/test_camera.py` | 4 / 4 | CameraModel intrinsics, rotation matrices, 3D-to-2D projection, angular errors |
| `backend/tests/test_disturbances.py` | 7 / 7 | DisturbanceEngine initialization, intensity setting, vibration, jitter, turbulence, noise, blur |
| `backend/tests/test_experiments.py` | 4 / 4 | ExperimentManager scenario loading, start, stop, timeout checking |
| `backend/tests/test_metrics.py` | 4 / 4 | MetricsTracker frame logging, lock rate, pixel error, angular error, CSV export |
| `backend/tests/test_pid.py` | 4 / 4 | PID proportional response, deadband, integral clamping, derivative response |
| `backend/tests/test_tracking.py` | 3 / 3 | OpenCV detector beacon finding, Kalman constant velocity & coasting, FSM transitions |

### 6.2 Frontend Static Analysis & Compilation
- **Linter (`npm run lint` via oxlint):** 17 files inspected, 104 rules checked, **0 errors**, 33 warnings (unused imports, hook dependencies).
- **Production Build (`npm run build` via Vite):** **Passed with exit code 0** in 1.19 s; generated `dist/index.html` (0.76 kB), `dist/assets/index-*.css` (29.28 kB), `dist/assets/index-*.js` (1,574 kB).

---

## 7. Functional Validation Matrix

| Subsystem / Capability | Verification Method | Status | Observations |
| :--- | :--- | :---: | :--- |
| **3D Scene Rendering** | React Three Fiber / WebGL canvas | **Functional** | Renders 3D pad, emissive beacon spheres, coordinate axes, starfield |
| **Target Trajectory Generation** | Store clock step in Three.js | **Functional** | Linear, circular, sinusoidal, spiral, random-walk position updates |
| **Virtual Pinhole Camera** | `CameraModel` math & Three.js camera | **Functional** | Focal length, FOV, and world-to-camera matrix transformations valid |
| **OpenCV Beacon Detection** | Synthetic frame test & E2E stream | **Functional** | HSV thresholding + contour circularity detects beacon centroid |
| **YOLO Detection** | `DetectorInterface` initialization | **Fallback Active** | Gracefully disabled: `models/beacon_detector.pt` weights absent; OpenCV active |
| **Kalman Filter Estimation** | `test_tracking.py` & live stream | **Functional** | 4-state CV filter updates $[x, y, v_x, v_y]$, predicts during dropout |
| **Tracking State Machine** | 6-state FSM transitions | **Functional** | Transitions `SEARCHING` $\rightarrow$ `DETECTED` $\rightarrow$ `TRACKING` $\rightarrow$ `LOCKED` / `TARGET_LOST` |
| **Gimbal PID Controller** | `pid.js` / `test_pid.py` | **Functional** | Decoupled yaw/pitch rate commands with anti-windup clamping |
| **Disturbance Injection** | `DisturbanceEngine` affine/blur/noise | **Functional** | Modulates image arrays via vibration, jitter, turbulence, noise, blur, flicker |
| **Telemetry Emission** | WebSocket `/ws/tracking` JSON payload | **Functional** | Streams frame index, tracking state, Kalman coords, angular error, metrics |
| **Scenario Management** | REST `/api/scenarios` & start/stop | **Functional** | All 10 predefined benchmark scenarios load and apply configurations |
| **E2E Dynamic Loop** | WebSocket frame stream + telemetry | **Functional** | Frame transmit $\rightarrow$ detection $\rightarrow$ Kalman $\rightarrow$ telemetry response loop active |
| **PDF Technical Reporting** | `reports.py` via ReportLab | **Functional** | Compiled publication-grade PDF report (`EXP-baseline-*.pdf`, ~149 kB) with Matplotlib plots |

---

## 8. Current Architecture

```
[ Frontend: React 18 / Three.js ]
     │ 
     ├── (Offscreen WebGL Canvas) ──[ Base64 JPEG Frame ]──> [ Backend: FastAPI ]
     │                                                               │
     │                                                               ├── Disturbance Engine (Warp/Noise)
     │                                                               ├── OpenCV HSV Detector
     │                                                               ├── Ground-Truth Data Association [LEAKAGE A]
     │                                                               ├── 2D Kalman Filter
     │                                                               └── 6-State PAT State Machine
     │                                                               │
     └──<──[ Telemetry JSON (Kalman state, Error) ]──────────────────┘
     │
     ├── If Kalman Active: Extrapolates Kalman state to drive PID
     └── If Kalman Inactive: Direct 3D Ground-Truth Projection to drive PID [LEAKAGE B]
```

---

## 9. Current Detector Modes

1. **`opencv` Mode (Default & Active):**
   - Color space: BGR $\rightarrow$ HSV.
   - Threshold range: $[35, 50, 50]$ to $[95, 255, 255]$ (tuned for green optical beacons).
   - Morphological filtering: $3 \times 3$ rectangular kernel, `MORPH_OPEN` followed by `MORPH_CLOSE`.
   - Contour analysis: Gated by area ($10 < A < 0.5 \cdot W \cdot H$), circularity metric $4\pi A / P^2 > 0.4$.
   - Output: Bounding box integer rect $[x_{\min}, y_{\min}, x_{\max}, y_{\max}]$, center midpoint $[(x_1+x_2)/2, (y_1+y_2)/2]$, circularity-derived confidence score.
2. **`yolo` Mode (Inactive / Fallback):**
   - Configured weights path: `models/beacon_detector.pt`.
   - Audit finding: Model weights file is not committed in repository (`models/` contains only `README.md`). Ultralytics dependency is optional; `DetectorInterface` catches `ImportError` or `FileNotFoundError` and defaults safely to `opencv` mode with `fallback_warning_active = True`.

---

## 10. Current Estimator

- **Model:** 4-state Linear Discrete Kalman Filter (`KalmanFilter2D` in `backend/app/tracking/kalman.py`).
- **State Vector:** $\mathbf{x} = [x, y, v_x, v_y]^T$ where $x, y$ are sensor pixel coordinates, and $v_x, v_y$ are pixel velocities (px/s).
- **Measurement Vector:** $\mathbf{z} = [x, y]^T$.
- **Process Noise ($Q$):** Scaled identity with default $\sigma_q = 0.5$.
- **Measurement Noise ($R$):** Scaled identity with default $\sigma_r = 1.0$.
- **Coasting Logic:** If no detection occurs, calls `predict_only()`; tracks missed frame counter; enforces `max_prediction_seconds = 2.0` before signaling timeout.

---

## 11. Current State Machine

- **Implementation:** `TrackingStateMachine` in `backend/app/tracking/state_machine.py`.
- **States:** 6 discrete states:
  1. `SEARCHING` (initial idle state)
  2. `DETECTED` (candidate observed)
  3. `ACQUIRING` (target in FOV, angular error $> 0.5^\circ$)
  4. `TRACKING` (target in FOV, angular error $\le 0.5^\circ$)
  5. `LOCKED` (angular error $\le 0.15^\circ$ sustained for $> 1.0\text{s}$)
  6. `TARGET_LOST` (occlusion or timeout)
- **Search Pattern Trigger:** Passive flag only; does not actuate autonomous raster/spiral scan commands to the gimbal.

---

## 12. Current Controller

- **Implementation:** Decoupled dual-axis PID (`PIDController` in `frontend/src/utils/pid.js`).
- **Input:** Angular tracking error in degrees (horizontal for pan, vertical for tilt).
- **Output:** Angular rate command in deg/s ($\omega_{\text{pan}}, \omega_{\text{tilt}}$).
- **Default Parameters:** $k_p = 1.5, k_i = 0.05, k_d = 0.1$, `integralLimit = 20.0`, `outputLimit = 180.0` deg/s, `deadband = 0.01^\circ`.
- **Anti-Windup:** Integral clamping to $[-\text{integralLimit}, \text{integralLimit}]$.
- **Actuation:** Direct Euler integration in Three.js `useFrame`: $\theta_{\text{next}} = \theta_{\text{current}} + \omega \cdot \Delta t$.

---

## 13. Current Telemetry

Emitted frame-by-frame over WebSocket as a structured JSON object:
- `frame_index`: Monotonically increasing sequence number.
- `timestamp`: Epoch seconds.
- `tracking_state`: Current FSM state string.
- `detected`: Boolean flag indicating designated target detection.
- `active_detector`: `"opencv"` or `"yolo"`.
- `inference_time_ms`: Processing time of the vision detector.
- `fps`: Instantaneous simulation frame rate.
- `kalman`: Dictionary containing estimated $x, y, v_x, v_y$ and timestamp.
- `bbox`: Array $[x_1, y_1, x_2, y_2]$.
- `raw_center`: Unfiltered detector center $[cx, cy]$.
- `angular_error_total`, `angular_error_h`, `angular_error_v`: Errors in degrees.
- `candidates`: List of all detected/tracked candidate dictionaries.
- `live_metrics`: Summary statistics dictionary from `MetricsTracker`.

---

## 14. Current Scenarios

Loaded from `configs/scenarios.json` (10 scenarios):
1. `baseline`: Slow-moving linear target under zero disturbances ($v = 5\text{ m/s}$, duration 20 s).
2. `moving_target`: Target moving in a circular orbit under zero disturbances.
3. `fast_target`: High-speed crossing ($v = 10\text{ m/s}$).
4. `vibration`: Platform vibration at 65% intensity (15 Hz harmonic affine jitter).
5. `noise`: Low SNR with 65% Gaussian + salt-and-pepper noise, 15% blur, 10% flicker.
6. `turbulence`: 65% Atmospheric turbulence (spatial coordinate remap wobble + blur).
7. `multiple_targets`: 4 simultaneous optical beacons (Emerald, Blue, Amber, Pink).
8. `target_loss`: Designated target occluded between $t = 5\text{s}$ and $t = 10\text{s}$.
9. `camera_disturbance`: Camera jitter at 70% intensity (shock jumps with exponential recovery).
10. `stress_test`: All 6 disturbances enabled at 40–45% + fast random target trajectory.

---

## 15. Ground-Truth Flow Audit

### Leakage A: Candidate Association (`backend/app/main.py` lines 541–590)
- **Description:** The backend loops over all detected visual candidates and matches them against `gt_beacons` (the projected pixel positions of ground-truth 3D beacons). If and only if the closest ground truth beacon within 100 pixels is `BEACON-01`, it marks `target_detected = True`.
- **Classification:** **`ASSOCIATION` — FORBIDDEN [CRITICAL VIOLATION]**.
- **Downstream Consequences:**
  1. The vision system cannot track an optical beacon based on visual attributes alone.
  2. If the ground-truth 3D position is slightly offset or unknown, the detector treats valid visual detections as false positives.
  3. Pre-filters measurements fed to the Kalman filter using ground truth knowledge, masking real-world association ambiguity.

### Leakage B: Fallback Gimbal Controller (`frontend/src/scene/GimbalCamera.jsx` lines 140–161)
- **Description:** When the Kalman filter state is inactive or ablated, the frontend projects `currentBeaconPosition` (the exact 3D ground truth coordinate of the target) directly through the camera matrix to compute angular errors `hErrorDeg` and `vErrorDeg`. These ground-truth errors are fed directly into the PID controller (`panPid.update`, `tiltPid.update`).
- **Classification:** **`CONTROL` — FORBIDDEN [CRITICAL VIOLATION]**.
- **Downstream Consequences:**
  1. The gimbal continues tracking the target even when the detector loses the target or when the Kalman filter is turned off.
  2. Ablation benchmarks comparing "with Kalman" vs "without Kalman" are completely invalid because the "without Kalman" branch cheats using ground truth.
  3. Gives a false illusion of 100% lock retention during sensor blackouts.

---

## 16. Additional Leakage Discovered

A comprehensive codebase audit across all writable project files identified the following additional occurrences:

| Location | Code Construct | Classification | Evaluation |
| :--- | :--- | :--- | :--- |
| `frontend/src/scene/SimulationScene.jsx:10-18` | `updateBeaconPosition(dt)` | **`RENDERING-ONLY`** | **Allowed.** Updates 3D world positions to render the physical/optical scene. |
| `frontend/src/scene/SimulationScene.jsx:23` | `<Beacon position={b.position} />` | **`RENDERING-ONLY`** | **Allowed.** Positions emissive mesh in the 3D WebGL viewport. |
| `frontend/src/components/TelemetryPanel.jsx:26` | `const [bx, by, bz] = beaconPosition;` | **`EVALUATION-ONLY`** | **Allowed.** Displays ground-truth coordinates in the user telemetry panel for monitoring. |
| `backend/app/main.py:484-524` | `target_camera = camera_model.world_to_camera` | **`EVALUATION-ONLY`** / **Leakage A** | **Hybrid.** Allowed for computing evaluation metrics, but currently leaked into Candidate Association (Leakage A). |
| `backend/app/main.py:657-681` | `metrics_tracker.log_frame(ground_truth_3d_position=...)` | **`EVALUATION-ONLY`** | **Allowed.** Passes ground truth solely to calculate TP/FP/FN/TN and offline error statistics. |
| `backend/app/tracking/metrics.py:83-93` | `if is_in_fov: ... tp/fp/fn/tn` | **`EVALUATION-ONLY`** | **Allowed.** Computes objective evaluation metrics; does not feed back into perception or control. |

---

## 17. Runtime Blockers Fixed

In accordance with Phase 0B rules, only runtime-blocking errors required to install, test, and run the inherited project were addressed.

### Blocker 1: Missing Starlette TestClient Dependency (`httpx`)
- **Error:** `RuntimeError: The starlette.testclient module requires the httpx package to be installed.`
- **Root Cause:** `httpx` was omitted from `backend/requirements.txt`, causing `backend/tests/test_api.py` to fail during collection.
- **Minimum Correction:** Installed `httpx` into `.venv`.
- **Verification:** Re-ran `pytest backend/tests/test_api.py`; all 4 API tests passed.

### Blocker 2: Undeclared Report Generation Dependencies (`reportlab`, `matplotlib`)
- **Error:** `ModuleNotFoundError: No module named 'reportlab'` when invoking `ExperimentManager.stop_experiment()`.
- **Root Cause:** `backend/app/experiments/reports.py` imports `reportlab` and `matplotlib.pyplot`, but neither was declared in `backend/requirements.txt`.
- **Minimum Correction:** Installed `reportlab` and `matplotlib` into `.venv`.
- **Verification:** Successfully executed experiment stop; generated complete PDF technical report `EXP-baseline-*.pdf` (149 kB).

### Blocker 3: Python Module Import Path Resolution
- **Error:** `ModuleNotFoundError: No module named 'app'` when running pytest from repository root.
- **Root Cause:** Python import resolution did not include the `backend/` directory in `sys.path`.
- **Minimum Correction:** Standardized execution with `$env:PYTHONPATH="backend"`.
- **Verification:** All 30 backend tests collected and passed cleanly.

---

## 18. Actual Measured Legacy Metrics

The following metrics were captured during an actual end-to-end executed run on this host machine (60 frames over WebSocket at 60 Hz nominal pace).

> [!CAUTION]
> **LEGACY BASELINE — NOT VALID FOR FINAL PERFORMANCE CLAIMS**  
> Because the inherited system relies on ground-truth matching during candidate association, the numbers below represent the legacy starting point only and cannot be cited as evidence of autonomous tracking accuracy.

| Metric | Measured Value | Measurement Status |
| :--- | :--- | :--- |
| **Backend Execution FPS (WebSocket Active)** | **32.8 FPS** (avg: 40.7, max: 247.9) | Actual Measured Legacy Run |
| **Detector Inference Latency (OpenCV)** | **1.09 ms** (avg across 40 frames) | Actual Measured Legacy Run |
| **Kalman Tracker Update Latency** | **0.07 ms** (avg across 40 frames) | Actual Measured Legacy Run |
| **End-to-End Backend Processing Latency** | **1.44 ms** (avg across 40 frames) | Actual Measured Legacy Run |
| **Pointing Error (Static Offset Target)** | **$0.2103^\circ$** (mean: $0.21^\circ$, max: $0.21^\circ$) | Actual Measured Legacy Run |
| **Detection Precision** | **1.0** (when ground truth matched) | Actual Measured Legacy Run |
| **Detection Recall (Ground-Truth Matched)** | **1.0** | Actual Measured Legacy Run |
| **Detection Recall (Ground-Truth Offset)** | **0.0** (100% false negative rejection) | Actual Measured Legacy Run |
| **YOLOv8 Inference Latency** | *NOT YET MEASURED* (weights absent) | Not Yet Measured |
| **Physical Gimbal Acceleration Limits** | *NOT YET MEASURED* (not modeled) | Not Yet Measured |

---

## 19. Known Limitations of the Baseline

1. **Association Brittleness:** True visual detections are rejected if the 3D target coordinates in metadata differ from the pixel detection by $> 100$ px.
2. **Cheat Fallback in Control:** Ablating or losing the Kalman filter causes the frontend to directly slew the gimbal toward the 3D beacon.
3. **No Sub-Pixel Centroiding:** Contour bounding box integer center introduces $\pm 1.5$ px quantization jitter.
4. **No Actuator Inertia or Torque Limits:** Instantaneous velocity steps permitted by the PID controller.
5. **No Active Spatial Reacquisition:** Gimbal remains stationary upon target loss rather than executing an autonomous expanding scan pattern.

---

## 20. Recommended Prerequisites Before Valid Benchmarking

Before any valid numerical performance claims can be established in future phases:
1. **Complete Truth Isolation (Phase 0C / Phase 1):** Remove all references to `gt_beacons` in `main.py` and `currentBeaconPosition` in `GimbalCamera.jsx`. The perception-to-control loop must consume only camera pixels and gimbal encoder angles.
2. **Sub-Pixel Localizer Implementation:** Replace bounding box centers with intensity-weighted connected-component moments.
3. **Autonomous 7-State FSM with Spiral Reacquisition:** Integrate an active Archimedean spiral search generator to reacquire targets after dropouts.
4. **Physically Rate/Acceleration-Limited Gimbal Model:** Add second-order inertia smoothing and strict $\alpha_{\max}$ clamping.
5. **Deterministic Benchmark Runner:** Create a headless automated benchmark harness that runs fixed-seed reproducible scenarios without browser network variance.

---

## 21. Reproducibility Cleanup (Phase 0B.5)

To guarantee that any fresh clone of the repository can immediately replicate the verified baseline environment without manual troubleshooting or undocumented pip invocations, the following reproducibility corrections were committed:

### 21.1 Files Changed
- `backend/requirements.txt`: Added missing runtime and testing dependencies.
- `documentation/User_Manual.md`: Streamlined backend installation instructions to reference the unified `requirements.txt`.
- `plans/01_BASELINE_REPORT.md`: Updated installation instructions and documented cleanup audit.

### 21.2 Dependencies Added
| Dependency | Version Constraint | Reason |
| :--- | :--- | :--- |
| `httpx` | `>=0.24.0` | Required by Starlette / FastAPI `TestClient` for executing backend unit tests in `backend/tests/test_api.py`. |
| `matplotlib` | `>=3.7.0` | Required by `backend/app/experiments/reports.py` for rendering telemetry plots embedded into experiment PDFs. |
| `reportlab` | `>=4.0.0` | Required by `backend/app/experiments/reports.py` for automated technical PDF report document generation. |

### 21.3 Clean Installation Commands
```powershell
# Backend clean install
cd c:\Projects\SIH26169-WORKSPACE\FSOC-AI-PAT
python -m venv .venv
.\.venv\Scripts\pip install -r backend/requirements.txt

# Frontend clean install
cd c:\Projects\SIH26169-WORKSPACE\FSOC-AI-PAT\frontend
npm install
```

### 21.4 Verification Results
- **Backend Tests:** `$env:PYTHONPATH="backend"; .\.venv\Scripts\pytest backend/tests`  
  **Result:** **30 passed in 2.39s** (100% pass rate).
- **Frontend Linter:** `npm run lint` (oxlint)  
  **Result:** **0 errors**, 33 warnings across 17 files.
- **Frontend Production Build:** `npm run build` (vite)  
  **Result:** **Build succeeded in 896ms** (`dist/assets/index-*.js` 1,574 kB).
- **Algorithmic Invariance:** Verified zero modifications to tracking, perception, Kalman filtering, PID control, FSM state logic, or simulation physics.

---
**Baseline Reproduction Certified by:** Senior Systems Architect & Lead Development Engineer  
**Status:** READY FOR ARCHITECTURE FREEZE
