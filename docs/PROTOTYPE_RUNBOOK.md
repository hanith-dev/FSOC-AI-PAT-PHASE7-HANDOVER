# FSOC-AI-PAT Prototype Runbook & Operational Guide
**System Version**: Phase 7 Certified Prototype Handover  
**Architecture**: Closed-Loop Hardware-In-The-Loop Software Surrogate  
**Status**: 160/160 Backend Tests Passing | Production Build Verified  

---

## 1. Quick Launch Commands

### A. Start Backend Server (Python FastAPI)
```bash
# Set PYTHONPATH and start WebSocket server
export PYTHONPATH=backend
python3 -m uvicorn app.main:app --host 127.0.0.1 --port 8000
```
- **REST Status**: `http://127.0.0.1:8000/api/status`
- **Live Stream Endpoint**: `ws://127.0.0.1:8000/ws/tracking`

### B. Start Frontend Client (React 19 + Three.js + Vite)
```bash
cd frontend
npm run dev
```
- **Local Dashboard URL**: `http://localhost:5173/`

---

## 2. System Architecture

```
[ WebGL 3D Scene ] ──(Render Target)──> [ Pinhole Camera Model (fx=fy=3661.7px) ]
         ▲                                                │
         │ (Pan/Tilt Velocity Commands)                   ▼ (Base64 JPEG Stream)
[ Filtered PID Actuator ] <── [ PAT State Machine ] <── [ CA6 Kalman Filter ] <── [ OpenCV / YOLOv8 Detector ]
```

### Subsystems Breakdown:
1. **Pinhole Virtual Camera**:
   - Resolution: 640 x 480 pixels
   - Field of View: 10.0° Horizontal x 7.5° Vertical
   - Optical Center: (320, 240)
2. **AI & Vision Pipeline**:
   - Primary: OpenCV HSV color thresholding + contour filtering + circularity constraint ($4\pi \cdot \text{Area} / \text{Perimeter}^2 > 0.4$)
   - Fallback: Robust color segmentation
   - Sub-pixel centroiding via weighted spatial moments
3. **Multi-Feature Association Engine**:
   - Strict Mahalanobis distance gating ($\chi^2 \le 9.21$)
   - Kinematic, color, and intensity distance scoring
   - Rejects non-designated distractors autonomously without ground truth assistance
4. **Kinematic Estimator**:
   - Canonical 6-state CA6 Kalman Filter: $[x, y, v_x, v_y, a_x, a_y]^T$
   - Continuous white-noise jerk model ($Q(dt)$)
   - Joseph-form covariance update with adaptive measurement noise $R$
5. **PAT Lifecycle State Machine**:
   - 8 Operational States: `SEARCH` ➔ `CANDIDATE` ➔ `ACQUIRE` ➔ `TRACK` ➔ `LOCK` ➔ `COAST` ➔ `REACQUIRE` ➔ `LOST`
   - Lock criteria: angular error $\le 0.15^\circ$, position uncertainty $\sigma_{pos} \le 5.0\text{ px}$, sustained for 0.40s
6. **Actuation Controller**:
   - First-order derivative low-pass filter ($\tau_d = 0.04\text{s}$)
   - Integral anti-windup clamping ($\pm 20.0^\circ$)
   - Decoupled Yaw/Pitch actuation with rate limiting ($60^\circ/\text{s}$)
7. **Phase 7 Covariance Reacquisition**:
   - Guided `covariance_ellipse_scan` active during target dropouts
   - Dynamically scans predicted 2-DOF covariance ellipse

---

## 3. Step-by-Step 5-Minute Live Demonstration Script

| Timestamp | Phase | User Action | Visible Proof / Telemetry |
| :--- | :--- | :--- | :--- |
| **0:00–0:30** | **Intro** | Launch Dashboard | Point out Header status, connection `● LIVE`, and 7-stage pipeline visualization. |
| **0:30–1:30** | **Acquisition** | Click `START / RESET MISSION` | Observe state chain: `SEARCH` ➔ `CANDIDATE` ➔ `ACQUIRE` ➔ `TRACK` ➔ `LOCK`. Optical beacon visibly converges into reticle crosshair. Pointing error drops $<0.15^\circ$. |
| **1:30–2:30** | **Precision Tracking** | Adjust `VIBRATION` (50%) & `TURBULENCE` (60%) | Telemetry displays stable pointing error $<0.15^\circ$. Estimator health remains `ACTIVE`. Link remains locked. |
| **2:30–3:30** | **Occlusion Handling** | Click `SIMULATE OCCLUSION` | Beacon disappears. State transitions: `LOCK` ➔ `COAST` (0.2s CA6 prediction) ➔ `REACQUIRE`. Covariance ellipse scanning pattern activates. |
| **3:30–4:15** | **Recovery** | Click `RESTORE BEACON` | Beacon reappears. State transitions immediately: `REACQUIRE` ➔ `TRACK` ➔ `LOCK`. Optical link restored. |
| **4:15–5:00** | **Distractor Defense** | Click `ADD DISTRACTOR` | Second blue beacon passes through FOV. Multi-feature association rejects distractor; system maintains lock on green target. |

---

## 4. Known Operational Limits
- Sensor FOV bounds: Targets outside 10° x 7.5° require raster acquisition scan.
- Maximum gimbal slew rate: $60.0^\circ/\text{s}$ per axis (enforced in software actuator model).
- Trusted coast window: 0.20s before covariance expansion triggers reacquisition.
