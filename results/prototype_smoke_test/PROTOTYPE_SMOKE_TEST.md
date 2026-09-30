# Prototype Freeze Checkpoint — End-to-End Live Validation Smoke Test Report

**Date:** 2026-09-29  
**Platform:** Windows 10/11 x64  
**Python Environment:** Python 3.12.10 (`.venv`)  
**Frontend Framework:** React 19 + Vite 8.2.2 + Three.js + TailwindCSS  
**Validation Methodology:** Native Windows runtime chain testing over live WebSocket (`ws://127.0.0.1:8000/ws/tracking`) and HTTP REST APIs (`http://127.0.0.1:8000/api/*`).

---

## 1. Executive Summary

This smoke test certifies prototype submission readiness for the Free-Space Optical Communications (FSOC) Pointing, Acquisition, and Tracking (PAT) coarse alignment simulator.

The full operational pipeline was executed live end-to-end on this Windows PC:
```text
Virtual FSOC Environment (Three.js WebGL)
        ↓
Virtual Camera / Optical Image (640x480 RGB / Base64 DataURL stream)
        ↓
Disturbances Engine (Platform Vibration, Turbulence, Noise)
        ↓
Beacon Detection (OpenCV HSV Thresholding + Weighted Centroid Localization)
        ↓
Robust Association (Multi-feature Mahalanobis Gating & Cost Matrix)
        ↓
CA6 Adaptive Estimator (6-state Kinematic Filter with Quality Inflation)
        ↓
PAT Lifecycle State Machine (SEARCH → CANDIDATE → ACQUIRE → TRACK → LOCK)
        ↓
Filtered PID Controller (1st-order Derivative Filter + Anti-Windup)
        ↓
Camera / Gimbal Response (Closed-loop Pan/Tilt Actuation)
        ↓
Mission Control Telemetry (Real-time WebSockets @ 15–50 Hz)
```

---

## 2. Startup Verification & System Profile

| Component | Target / URL | Startup Command | Verified Status |
| :--- | :--- | :--- | :--- |
| **Backend** | `http://127.0.0.1:8000` | `$env:PYTHONPATH="backend"; .\.venv\Scripts\uvicorn.exe app.main:app --host 127.0.0.1 --port 8000` | **PASS** (Started cleanly in 210 ms, PID confirmed) |
| **Frontend** | `http://localhost:5173` | `npm run dev` (in `frontend/`) | **PASS** (Vite v8 ready in 219 ms) |
| **Health API** | `GET /api/status` | `urllib.request.urlopen` | **PASS** (`{"status":"online","detector_mode":"opencv"}`) |
| **WebSocket** | `ws://127.0.0.1:8000/ws/tracking` | WebSocket Handshake | **PASS** (`{"type":"connection_status","status":"connected"}`) |
| **Offline Inference** | Local CPU | OpenCV weighted centroid | **PASS** (No GPU or Docker required) |

---

## 3. Operational Chain & Telemetry Flow

| Subsystem | Input | Processing | Output Verified | Status |
| :--- | :--- | :--- | :--- | :--- |
| **Detector** | 640×480 Image Frame | Color space segmentation + weighted centroid | Bounding box `[u1, v1, u2, v2]` & centroid | **PASS** |
| **Localization** | Detected blob pixels | Intensity-weighted centroid subpixel computation | Centroid `[x, y] = [320.0, 240.5]` | **PASS** |
| **Association** | Detected candidates | Multi-feature gating against Kalman innovation | Accepted designated candidate | **PASS** |
| **CA6 Estimator** | Associated measurement | 6-state CA kinematic update ($x, y, v_x, v_y, a_x, a_y$) | State estimate + innovation covariance | **PASS** |
| **PAT FSM** | Filter health, residual | 8-state transition logic | Explicit state progression | **PASS** |
| **Filtered PID** | Angular error ($^\circ$) | Filtered derivative + anti-windup integration | Rate commands ($\Delta\text{Pan}, \Delta\text{Tilt}$) | **PASS** |
| **Gimbal** | Rate commands | Simulated physical actuation | Updated camera orientation | **PASS** |
| **Telemetry** | Full pipeline snapshot | JSON serialization streamed over WebSocket | Received by UI store in $< 20$ ms | **PASS** |

---

## 4. Live PAT State Machine Transitions

### Sequence 1: Initial Acquisition
- **SEARCH:** Initial frame with empty field $\to$ `state = SEARCH`, `detected = False`.
- **CANDIDATE:** First optical beacon detected $\to$ confirmed after 2 frames $\to$ `state = CANDIDATE`.
- **ACQUIRE:** Candidate confirmed $\to$ estimator initialized $\to$ `state = ACQUIRE`.
- **TRACK:** Gate convergence within 2 frames $\to$ `state = TRACK`, pointing error reducing.
- **LOCK:** Pointing error $\le 0.011^\circ \le 0.15^\circ$ held for $> 0.40$ s $\to$ `state = LOCK` (FPS: ~50 Hz).
- **Result:** **PASS**

### Sequence 2: Target Interruption & Phase 7 Reacquisition
- **Target Loss:** Simulated optical blackout (beacon extinguished).
- **COAST:** Immediately transitioned to `COAST` (Reason: `MEASUREMENT_LOST`). Estimator extrapolated kinematic trajectory.
- **REACQUIRE:** Coast timeout elapsed (0.20 s) $\to$ entered `REACQUIRE` (Reason: `COAST_TIMEOUT`).
  - Search pattern active: `covariance_ellipse_scan`.
  - Search active: `True`.
  - Autonomous scan offsets dynamically produced: e.g., $\Delta\text{Pan} = +0.012^\circ, \Delta\text{Tilt} = -0.045^\circ$.
- **Recovery:** Optical beacon restored $\to$ detected during reacquisition $\to$ transitioned to `TRACK` (Reason: `LOCAL_REACQUIRE_SUCCESS`) within 2 confirmation frames.
- **Relock:** Smoothly converged back to `LOCK` with residual pointing error $0.011^\circ$.
- **Result:** **PASS**

---

## 5. Multi-Candidate Distractor Smoke Test

- **Setup:** Placed designated target beacon at `(320, 240)` and a brighter distractor beacon at `(450, 320)`.
- **Observation:**
  - Detector reported 2 optical candidates: `CANDIDATE-01` (distractor) and `CANDIDATE-02` (designated target).
  - Robust Association matched `CANDIDATE-02` based on kinematic gate and operational cost matrix.
  - Tracked position distance to designated target: **0.71 px**.
  - Distance to distractor: **151.96 px**.
- **Result:** **PASS** (Designated target maintained lock with zero false-latch onto distractor).

---

## 6. Mission Control Telemetry Inventory

| Metric / Parameter | Classification | Notes |
| :--- | :--- | :--- |
| **PAT state** | VISIBLE IN UI | State badge in TelemetryPanel, Overview card, and CameraFeed |
| **LOCK status** | VISIBLE IN UI | Top status pill and Overview card |
| **Target detection marker** | VISIBLE IN UI | Green reticle and bounding box on CameraFeed |
| **Estimated target position** | VISIBLE IN UI | Blue crosshair on CameraFeed |
| **Gimbal pan/tilt** | VISIBLE IN UI | Live numerical readouts in TelemetryPanel |
| **Pointing error (Total, H, V)** | VISIBLE IN UI | Degrees breakdown in TelemetryPanel |
| **Association confidence** | AVAILABLE IN TELEMETRY ONLY | Sent in `response.candidates[i].confidence` |
| **Localization confidence** | AVAILABLE IN TELEMETRY ONLY | Sent in `response.confidence` |
| **Estimator uncertainty ($\sigma_{pos}$)** | AVAILABLE IN TELEMETRY ONLY | Derived from `covariance` matrix |
| **Covariance ellipse / Search state** | AVAILABLE IN TELEMETRY ONLY | Sent in `response.pat_fsm` (`search_pattern`, `search_offsets`) |
| **Reacquisition state** | VISIBLE IN UI | Badge pulses purple labeled `REACQUIRE` |
| **FPS & Network latency** | VISIBLE IN UI | Real-time FPS and latency (ms) |
| **Disturbance status** | VISIBLE IN UI | Sliders and indicators in Hardware & Ablations tab |
| **False-lock / Evaluation metrics** | AVAILABLE IN TELEMETRY ONLY | Telemetry logs, exported CSV, and generated PDF report |

---

## 7. Issue Classification & Fixes Applied

### P0/P1 Blockers Addressed
- **P1 Blocker:** `PATLifecycleStateMachine` had not been instantiated in `backend/app/main.py` WebSocket handler (was using pre-Phase-5 `TrackingStateMachine`), preventing live demonstration of `CANDIDATE`, `COAST`, `REACQUIRE`, and `covariance_ellipse_scan`.
  - **Resolution:** Initialized `pat_fsm = PATLifecycleStateMachine(config=PATFSMConfig())` in `websocket_endpoint` and wired update calls and rich telemetry diagnostics (`pat_fsm` block).
  - **Frontend Parity:** Updated `TelemetryPanel.jsx`, `Dashboard.jsx`, `CameraFeed.jsx`, and `GimbalCamera.jsx` to handle both `PATState` and legacy aliases, and initialized client-side gimbal controller in `filtered_pid` mode.

### P2 Issues Recorded (Presentation Weaknesses — Non-Blockers)
- Covariance ellipse is currently transmitted in telemetry as numerical scan offsets rather than drawn as a 2D confidence ellipse graphic on the 2D camera feed overlay.
- Reacquisition cycle count is logged in telemetry diagnostics but not displayed in the top KPI summary bar.

### P3 Issues Recorded (Future Enhancements — Post-Freeze)
- Adaptive scan rate scaling for high-jerk disturbance profiles.
- Native WebGL shader bloom on camera feed canvas.

---

## 8. Test Regression Certification

- **Backend Pytest Suite:** **159 / 159 PASS** (100% pass rate in 22.18s)
- **Frontend Linter (`oxlint`):** **0 errors** (34 informational unused import warnings)
- **Frontend Production Build (`vite build`):** **PASS** (Zero errors, bundles created in `dist/`)
