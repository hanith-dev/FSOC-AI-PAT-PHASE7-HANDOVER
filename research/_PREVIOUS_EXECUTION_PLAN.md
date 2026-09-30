## SIH26169: AI-Based Virtual Camera Tracking for FSOC Coarse Alignment

## Professional Execution Flow (4–6 Weeks)

---

## Phase 0: Setup & Baseline (Days 1–3)

**Goal:** Get a minimal working simulation running end-to-end.

## Tasks

1. **Clone & Run Existing MVP**

   * Repository: `LumiTrack` (luvmangla05/LumiTrack)[svggithub](https://github.com/luvmangla05/LumiTrack/blob/main/Readme.md)
   * Run locally: `docker-compose up` or `python main.py`
   * Verify: Virtual camera tracks a moving beacon in a simple 3D scene
2. **Document Baseline Metrics**

   * Track one scenario: linear motion, target starts in FOV
   * Record: acquisition time, steady-state tracking error
3. **Clarify ISRO Requirements**

   * Extract from SIH26169 statement: target motion types, FOV size, performance criteria[svggithub](https://github.com/NoBugNinja/Smart-India-Hackathon-SIH-2026-Problem-Statements/blob/main/README_THEMED.md)
   * Write a 1-page **Requirements Spec** (bullet list is fine)

**Deliverable:** Running demo + 1-page requirements doc

**ChatGPT Prompt:**
"Here's the SIH26169 problem statement and the LumiTrack repo. Help me extract the key performance requirements (FOV size, acceptable tracking error, acquisition time) and map them to measurable metrics."

---

## Phase 1: Core Simulation Loop (Week 1)

**Goal:** Backend runs a closed-loop simulation: beacon → camera → detection → tracking → control → telemetry.

## Architecture (Simplified)

```
text
```

svg

`[Beacon Trajectory] → [Virtual Camera Renderer] → [Detector (YOLO/CV)]        ↑                                              ↓ [Gimbal Controller (PID)] ← [Tracker (Kalman)] ← [Detection Output]        ↓ [Telemetry Stream (WebSocket)]`

## Tasks

1. **Beacon Motion Model**

   * Implement 3 motion modes: linear, sinusoidal, random-walk
   * Parameters: max velocity, max acceleration, trajectory duration
2. **Virtual Camera Kinematics**

   * Azimuth/elevation limits (e.g., ±30°)
   * Max slew rate (e.g., 10°/s)
   * FOV size (e.g., 5° × 5°)
3. **Detection Pipeline**

   * Option A: YOLOv8 pretrained on synthetic beacon images
   * Option B: Simple color/shape thresholding + centroid extraction (faster to implement)
4. **Tracking Algorithm**

   * Extended Kalman Filter (EKF) with state: `[x, y, vx, vy, ax, ay]`
   * Tune process noise for rapid motion scenarios
5. **Control Loop**

   * PID controller: input = tracking error (beacon center − image center), output = gimbal rate command
   * Add anti-windup and saturation handling
6. **Telemetry Schema**

   ```
   json
   ```

   svg

   `{   "timestamp": 1234567890.123,   "beacon_pos": {"az": 5.2, "el": -3.1},   "camera_pos": {"az": 5.0, "el": -3.0},   "tracking_error": {"dx": 0.2, "dy": -0.1},   "gimbal_command": {"rate_az": 0.5, "rate_el": 0.3},   "in_fov": true }`

**Deliverable:** Backend runs closed-loop simulation, streams telemetry to a simple frontend (can be terminal-based or basic HTML page)

**ChatGPT Prompt:**
"I'm implementing an EKF for tracking a moving beacon in a virtual camera system. State is [x, y, vx, vy, ax, ay]. Help me write the prediction and update equations in Python, and suggest reasonable process noise values for rapid motion (up to 10°/s² acceleration)."

---

## Phase 2: Frontend Visualization (Week 2)

**Goal:** Real-time 2D camera feed + 3D scene view + telemetry dashboard.

## Tasks

1. **2D Camera View**

   * Render virtual camera image (beacon as bright spot on dark background)
   * Overlay: crosshair, FOV boundary, tracking error vector
2. **3D Scene View (Three.js)**

   * Show beacon position, camera orientation, gimbal axes
   * Toggle between 2D and 3D views
3. **Telemetry Dashboard**

   * Real-time plots: tracking error (dx, dy), gimbal command, FOV margin
   * Status indicators: "Acquiring", "Tracking", "Lost"
4. **WebSocket Integration**

   * Connect frontend to backend telemetry stream
   * Handle reconnection gracefully

**Deliverable:** Interactive web UI showing live simulation

**ChatGPT Prompt:**
"Help me set up a Three.js scene with a moving sphere (beacon) and a camera that rotates based on azimuth/elevation angles. Show how to update the camera orientation in real-time from a WebSocket stream."

---

## Phase 3: Scenario Testing & Performance Tuning (Week 3)

**Goal:** Validate system against ISRO-like scenarios, tune for robustness.

## Test Scenarios

| ScenarioMotion TypeInitial ConditionSuccess Criterion |                                 |                    |                                      |
| ----------------------------------------------------- | ------------------------------- | ------------------ | ------------------------------------ |
| S1                                                    | Linear constant velocity        | Target in FOV      | Error < 5% FOV                       |
| S2                                                    | Sinusoidal (1 Hz, 5° amplitude) | Target in FOV      | Error < 10% FOV                      |
| S3                                                    | Random walk (max accel 10°/s²)  | Target outside FOV | Acquisition < 5s, then track         |
| S4                                                    | Step change (instant 10° jump)  | Target in FOV      | Recovery < 2s                        |
| S5                                                    | Occlusion (2s blackout)         | Target in FOV      | Predictive tracking during occlusion |

## Tasks

1. **Implement Scenario Runner**

   * Load scenario config (JSON/YAML)
   * Run simulation, log telemetry to file
2. **Metrics Computation**

   * Acquisition time: first frame where error < threshold and stays < threshold for N seconds
   * RMS tracking error (steady-state)
   * Peak error (worst-case)
   * FOV violation count
3. **Tuning**

   * Adjust EKF process noise for better rapid-motion tracking
   * Tune PID gains to reduce overshoot, improve response time
   * Add feedforward compensation if needed (e.g., predict beacon motion)

**Deliverable:** Performance report with metrics for all 5 scenarios

**ChatGPT Prompt:**
"Here's my telemetry log (CSV) from a tracking simulation. Help me compute acquisition time, RMS tracking error, and peak error. Also suggest how to tune EKF process noise if tracking error spikes during rapid acceleration."

---

## Phase 4: Acquisition Logic & Robustness (Week 4)

**Goal:** Handle cases where target starts outside FOV or is lost during tracking.

## Tasks

1. **Search Pattern Implementation**

   * Spiral or raster scan when target is outside FOV
   * Transition to track mode when detection confidence > threshold
2. **Lost-Target Recovery**

   * If tracking error > FOV size for N frames, switch to search mode
   * Use last known velocity to predict search region
3. **Robustness Enhancements**

   * Add measurement noise to detection (simulate real camera noise)
   * Test under varying beacon intensity (simulate atmospheric turbulence)

**Deliverable:** System autonomously acquires target from unknown initial position and recovers from temporary loss

**ChatGPT Prompt:**
"Design a search pattern (spiral or raster) for a gimbal-mounted camera to find a moving beacon when the initial position is unknown. How do I decide when to switch from search mode to track mode?"

---

## Phase 5: Documentation & Demo Prep (Week 5–6)

**Goal:** Polish system for ISRO evaluation.

## Tasks

1. **Demo Script**

   * Scenario 1: Target in FOV → show stable tracking
   * Scenario 2: Target outside FOV → show autonomous acquisition
   * Scenario 3: Rapid motion → show robustness
2. **Architecture Diagram**

   * Draw high-level block diagram (beacon → detector → tracker → controller → gimbal)
3. **Performance Report**

   * Table of metrics for all test scenarios
   * Compare against ISRO criteria (if specified)
4. **Code Cleanup**

   * Add README with setup instructions
   * Document configuration options (motion models, controller gains)

**Deliverable:** Demo video + performance report + clean GitHub repo

**ChatGPT Prompt:**
"Help me write a professional README for my SIH26169 project. Include: problem statement, architecture overview, setup instructions, how to run test scenarios, and how to interpret the performance metrics."

---

## Decision Points (Where ChatGPT Adds Maximum Value)

| PhaseDecisionChatGPT Role |                                      |                                                       |
| ------------------------- | ------------------------------------ | ----------------------------------------------------- |
| Phase 1                   | EKF vs. Particle Filter for tracking | Compare trade-offs, help implement chosen algorithm   |
| Phase 1                   | PID tuning strategy                  | Suggest Ziegler-Nichols or manual tuning approach     |
| Phase 3                   | Metrics computation logic            | Write Python code to parse telemetry and compute KPIs |
| Phase 4                   | Search pattern design                | Generate spiral/raster scan algorithm pseudocode      |
| Phase 5                   | Demo narrative                       | Craft compelling story for ISRO judges                |

---

## Minimal Tech Stack (Keep It Simple)

| ComponentTechnologyRationale |                              |                                                            |
| ---------------------------- | ---------------------------- | ---------------------------------------------------------- |
| Backend                      | Python + FastAPI             | Fast to implement, easy WebSocket support                  |
| Frontend                     | HTML + JavaScript + Three.js | No build tools needed, runs in browser                     |
| Detection                    | OpenCV (color thresholding)  | Faster than training YOLO, sufficient for synthetic beacon |
| Tracking                     | Custom EKF (NumPy/SciPy)     | Full control, easy to tune                                 |
| Control                      | PID (custom Python class)    | Simple, well-understood                                    |
| Data                         | CSV logs + SQLite (optional) | Minimal overhead                                           |
