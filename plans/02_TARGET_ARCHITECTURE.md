# Target System Architecture Specification: Competition-Grade Coarse PAT Platform

**Project:** SIH26169 — AI-Based Virtual Camera Tracking System for Coarse Alignment of Mobile FSOC Terminals  
**Role:** Senior Aerospace Software Architect, Computer Vision Engineer, Control Systems Engineer & Experimental Validation Engineer  
**Document Version:** 2.0.0 (Phase 0C Architecture Freeze)  
**Status:** ARCHITECTURALLY FROZEN — BASELINE SPECIFICATION FOR PHASES 0D THROUGH 12  

---

## 1. Executive Summary & Core Architectural Tenets

This document specifies the authoritative, competition-grade software-in-the-loop (SITL) Pointing, Acquisition, and Tracking (PAT) system architecture for mobile Free-Space Optical Communication (FSOC) coarse terminal alignment.

### 1.1 Core Engineering Mandates
1. **Absolute Truth Isolation (Zero Ground-Truth Leakage):** The perception, association, estimation, prediction, FSM, and control loops must NEVER receive, inspect, or implicitly depend on simulator ground truth. Ground truth is strictly segregated into world simulation/rendering and offline evaluation engines.
2. **Dual-Execution Architecture (Unified Algorithms):**
   - **Mode A (Deterministic Benchmark Mode):** Authoritative, headless Python simulation running at fixed, configurable timesteps ($\Delta t$). Completely independent of browser clocks, WebSockets, or UI rendering. Produces bit-reproducible, seed-locked scientific benchmark datasets.
   - **Mode B (Live Mission Control Mode):** Interactive visualization layer where React + Three.js renders 3D world geometry, sensor viewports, and HUD overlays. Telemetry and state updates are streamed from the authoritative Python backend over WebSockets. The browser is never the authoritative arbiter of tracking or control decisions.
3. **Photon-Centric Precision Perception:** Optical FSOC beacons are treated as Point Spread Function (PSF) diffraction patterns, not generic large bounding-box objects. Coarse candidate detection is followed by localized background subtraction, matched filtering, and intensity-weighted centroiding with sub-pixel resolution.
4. **Kinematic Rigor in Estimation & Control:**
   - 6-state constant-acceleration ($CA$) Kalman state estimator with Mahalanobis statistical gating.
   - Latency-aware feed-forward angular velocity compensation combined with an anti-windup, derivative-filtered PID feedback controller.
   - Second-order, rate-, acceleration-, and jerk-limited gimbal actuator plant model with mechanical lag and inertia.
5. **Deterministic 8-State PAT Finite State Machine:** Governs operational transitions across `SEARCH`, `CANDIDATE`, `ACQUIRE`, `TRACK`, `LOCK`, `COAST`, `REACQUIRE`, and `LOST` with explicit temporal and statistical hysteresis.

---

## 2. Global Coordinate Conventions, Units & Notation

To eliminate spatial discrepancies between the 3D world simulation, virtual camera sensor, and electro-mechanical gimbal plant, the following conventions are strictly enforced across all modules:

```
                            +Y_world (Up / Zenith)
                               ^
                               |   +Z_world (North / Boresight Initial)
                               |  /
                               | /
                               |/
   -X_world (West) <-----------+-----------> +X_world (East)
                              /|
                             / |
                            /  |
                           v   v
```

| Domain | Symbol / Frame | Standard Units | Conventions & Coordinate Directions |
| :--- | :--- | :--- | :--- |
| **Simulation Time** | $t, \Delta t$ | seconds ($\text{s}$) | Mode A: Fixed step $\Delta t \in \{0.005, 0.010, 0.01667\}\,\text{s}$. Mode B: Monotonic wall clock. |
| **World Cartesian Frame** | $\mathcal{F}_W: [X_W, Y_W, Z_W]^T$ | meters ($\text{m}$) | Right-handed Cartesian. $+X$: East, $+Y$: Up (Zenith), $+Z$: North. |
| **Terminal Base Frame** | $\mathcal{F}_B: [X_B, Y_B, Z_B]^T$ | meters ($\text{m}$) | Rigidly attached to host platform (UAV/Satellite/Ground Station). |
| **Gimbal Azimuth** | $\psi, \dot{\psi}, \ddot{\psi}$ | $\text{deg}, \text{deg/s}, \text{deg/s}^2$ | Rotation about $+Y_B$ (Yaw). Positive: Counter-Clockwise looking down $+Y_B$. |
| **Gimbal Elevation** | $\theta, \dot{\theta}, \ddot{\theta}$ | $\text{deg}, \text{deg/s}, \text{deg/s}^2$ | Rotation about transverse horizontal axis $+X_{\text{gimbal}}$ (Pitch). Positive: Upward toward $+Y_B$. |
| **Camera Optical Frame** | $\mathcal{F}_C: [X_C, Y_C, Z_C]^T$ | meters ($\text{m}$) | Pinhole optical frame. $+Z_C$: Forward along optical boresight. $+X_C$: Camera Right. $+Y_C$: Camera Down. |
| **Image Pixel Frame** | $[u, v]^T$ | pixels ($\text{px}$) | Origin $(0,0)$ at top-left corner. $u \in [0, W-1]$ (horizontal, right), $v \in [0, H-1]$ (vertical, down). Nominal $W=1920, H=1080$. |
| **Principal Point** | $[c_x, c_y]^T$ | pixels ($\text{px}$) | Nominal geometric sensor center: $c_x = (W-1)/2 = 959.5\,\text{px}$, $c_y = (H-1)/2 = 539.5\,\text{px}$. |
| **Normalized Device Coords (NDC)** | $[x_{\text{ndc}}, y_{\text{ndc}}]^T$ | dimensionless | $x_{\text{ndc}} \in [-1.0, 1.0]$ (left to right), $y_{\text{ndc}} \in [-1.0, 1.0]$ (bottom to top). |
| **Camera Angular Offset** | $[\delta\psi, \delta\theta]^T$ | milliradians ($\text{mrad}$) | Angular deviation of target from camera boresight axis $+Z_C$. |
| **Pointing Error** | $\epsilon_{\text{point}}$ | $\text{mrad}$ or $\text{deg}$ | Total Euclidean angular deviation: $\epsilon = \sqrt{\delta\psi^2 + \delta\theta^2}$. |

### 2.1 Pinhole Camera Projection Formulation
Given intrinsic focal lengths $(f_x, f_y)$ in pixels and principal center $(c_x, c_y)$:
$$\begin{bmatrix} u \\ v \\ 1 \end{bmatrix} = \frac{1}{Z_C} \begin{bmatrix} f_x & 0 & c_x \\ 0 & f_y & c_y \\ 0 & 0 & 1 \end{bmatrix} \begin{bmatrix} X_C \\ Y_C \\ Z_C \end{bmatrix}$$
Angular deviations in the camera frame are computed inversely from pixel measurements without ground-truth distance knowledge:
$$\delta\psi = \arctan\left(\frac{u - c_x}{f_x}\right) \approx \frac{u - c_x}{f_x} \quad (\text{rad})$$
$$\delta\theta = -\arctan\left(\frac{v - c_y}{f_y}\right) \approx -\frac{v - c_y}{f_y} \quad (\text{rad})$$

---

## 3. Authoritative State Ownership & Data Flow

```
┌────────────────────────────────────────────────────────────────────────────────────────────────────────┐
│                                       SIMULATION WORLD (GROUND TRUTH)                                   │
│  - Target Kinematics: p_true(t), v_true(t)                                                            │
│  - Atmospheric Turbulence: Cn2(h), Rytov Variance, Scintillation Index, Fried Parameter r0           │
│  - Platform Disturbances: Vibration Modes, Wind Gusts, Base Jitter                                    │
└───────────────────────────────────┬────────────────────────────────────────────────────────────────────┘
                                    │
                                    │ Photons / Optical Wavefront (Degraded Viewport Frame)
                                    ▼
┌────────────────────────────────────────────────────────────────────────────────────────────────────────┐
│                                     PERCEPTION SUBSYSTEM (FIREWALLED)                                  │
│  [Degraded Frame I(u,v)] ──> [ROI Candidate Detection] ──> [Background Subtraction & Matched Filter]  │
│                                                                        │                               │
│  [Intensity-Weighted Centroid & Sub-Pixel Refinement] <────────────────┘                               │
│        │                                                                                               │
│        ▼                                                                                               │
│  Raw Detections: { z_k = [u, v]^T, Intensity I_k, SNR_k, SpotSize sigma_k, Score s_k }                │
└───────────────────────────────────┬────────────────────────────────────────────────────────────────────┘
                                    │
                                    ▼
┌────────────────────────────────────────────────────────────────────────────────────────────────────────┐
│                                    ASSOCIATION & ESTIMATION SUBSYSTEM                                  │
│  - Innovation & Covariance: y_k = z_k - H*x_hat_pred,  S_k = H*P_pred*H^T + R_k                       │
│  - Statistical Gating: d_M^2 = y_k^T * inv(S_k) * y_k <= gamma_gate                                   │
│  - Multi-Feature Cost Matrix (Spatial + Photometric + Temporal)                                       │
│  - 6-State Adaptive Kalman Filter: [x, y, vx, vy, ax, ay]^T                                           │
└───────────────────────────────────┬────────────────────────────────────────────────────────────────────┘
                                    │
                                    ▼
┌────────────────────────────────────────────────────────────────────────────────────────────────────────┐
│                                    PAT FINITE STATE MACHINE (FSM)                                      │
│  States: SEARCH ──> CANDIDATE ──> ACQUIRE ──> TRACK ──> LOCK ──> COAST ──> REACQUIRE ──> LOST         │
│  Output: FSM State, Pointing Target [u_cmd, v_cmd], Reacquisition Command                            │
└───────────────────────────────────┬────────────────────────────────────────────────────────────────────┘
                                    │
                                    ▼
┌────────────────────────────────────────────────────────────────────────────────────────────────────────┐
│                                   PREDICTIVE CONTROL & GIMBAL PLANT                                    │
│  - Feed-Forward: omega_ff = K_ff * [vx_hat, vy_hat]                                                    │
│  - Latency Predictor: e_pred = x_hat(t + tau_lag) - [c_x, c_y]                                         │
│  - Filtered Anti-Windup PID: u_pid = Kp*e + Ki*int(e) + Kd*de_filt/dt                                  │
│  - Gimbal Actuator Plant: Rate, Acceleration, and Jerk Saturation Clamps + Motor Lag Filter           │
│  - Output: Commanded Gimbal Pose [psi(t+dt), theta(t+dt)]                                             │
└───────────────────────────────────┬────────────────────────────────────────────────────────────────────┘
                                    │
                                    ▼ (Closes Physical Loop)
┌───────────────────────────────────┴────────────────────────────────────────────────────────────────────┐
│                                       CAMERA POSE REPOSITIONING                                        │
│  Camera Frame Orientation Updated: q_cam(t+dt) = q_base(t+dt) * q_gimbal(t+dt)                        │
└────────────────────────────────────────────────────────────────────────────────────────────────────────┘

═════════════════════════════════════ PARALLEL EVALUATION ONLY ═══════════════════════════════════════════
┌────────────────────────────────────────────────────────────────────────────────────────────────────────┐
│                                   EVALUATION & BENCHMARK LOGGING                                       │
│  Inputs: True Target State p_true(t)  ||  Estimated Target x_hat(t)  ||  Gimbal Boresight n_cam(t)     │
│  Metrics: Pointing Error (True), Estimation Error, Lock Retention, Settling Time, Actuator Jerk        │
└────────────────────────────────────────────────────────────────────────────────────────────────────────┘
```

---

## 4. The Strict Truth Firewall

### 4.1 Boundary Enforcements
1. **Forbidden Contexts:** The modules responsible for Candidate Detection, Sub-Pixel Centroiding, Candidate Association, State Estimation, PAT State Decisions, and Gimbal Control are strictly quarantined. They must never:
   - Ingest `gt_beacons`, `currentBeaconPosition`, or world scenario target arrays.
   - Use simulated target IDs (e.g., `"BEACON-01"`) for candidate selection.
   - Evaluate pixel error against projected 3D ground truth.
2. **Permitted Contexts:**
   - **World Simulation / Sensor Synthesis:** Generates true 3D spatial trajectories and renders photon distributions onto the synthetic camera sensor plane with atmospheric noise.
   - **Offline Evaluation Engine:** Subscribes to true world coordinates and estimated gimbal poses *after* control actuation to compute true localization RMSE, true pointing error, and false-lock rates.

### 4.2 Data Contracts & Schemas

#### Contract 1: Sensor Frame Interface (World $\rightarrow$ Perception)
```python
@dataclass(frozen=True)
class SensorFrame:
    frame_id: int               # Monotonically increasing frame index
    timestamp: float            # Simulation time (s)
    image: np.ndarray           # Grayscale or BGR image (H, W), dtype=uint8
    focal_length_px: float      # Camera focal length (px)
    principal_point: Tuple[float, float] # (c_x, c_y) in px
    gimbal_angles: Tuple[float, float]   # Current gimbal encoder feedback (psi, theta) in degrees
    gimbal_rates: Tuple[float, float]    # Current gimbal rate feedback (psi_dot, theta_dot) in deg/s
```
*Zero ground truth target data is contained in `SensorFrame`.*

#### Contract 2: Optical Detection Candidate (Perception $\rightarrow$ Association)
```python
@dataclass(frozen=True)
class DetectionCandidate:
    u: float                    # Sub-pixel horizontal coordinate (px)
    v: float                    # Sub-pixel vertical coordinate (px)
    intensity_peak: float       # Peak pixel intensity in spot [0, 255]
    flux_total: float           # Integrated photon flux over spot footprint
    snr: float                  # Estimated Signal-to-Noise Ratio (dB)
    semi_major_axis: float      # Spatial spot footprint width (px)
    semi_minor_axis: float      # Spatial spot footprint height (px)
    spot_eccentricity: float    # Measure of PSF circularity
    detector_confidence: float  # Score from coarse detector [0.0, 1.0]
    measurement_variance_px: float # Dynamic R variance estimate: sigma_meas^2
```

#### Contract 3: Kinematic Target Estimate (Estimator $\rightarrow$ FSM / Controller)
```python
@dataclass(frozen=True)
class TargetKinematicEstimate:
    timestamp: float            # Time of estimate
    x_hat: np.ndarray           # State vector [u, v, u_dot, v_dot, u_ddot, v_ddot]^T
    P_diag: np.ndarray          # State covariance diagonal
    innovation: Optional[np.ndarray] # Measurement residual y_k (px)
    mahalanobis_dist: float     # Gating distance d_M
    is_updated: bool            # True if measurement was incorporated, False if coasted
    consecutive_misses: int     # Count of sequential frames without valid update
    pos_uncertainty_px: float   # 1-sigma spatial position uncertainty: sqrt(P[0,0] + P[1,1])
```

#### Contract 4: Actuator Command (Controller $\rightarrow$ Gimbal Plant)
```python
@dataclass(frozen=True)
class GimbalCommand:
    timestamp: float
    psi_rate_cmd: float         # Commanded azimuth rate (deg/s)
    theta_rate_cmd: float       # Commanded elevation rate (deg/s)
    feedforward_rate: Tuple[float, float] # (ff_psi, ff_theta) in deg/s
    feedback_rate: Tuple[float, float]    # (pid_psi, pid_theta) in deg/s
    fsm_state: str              # Active PAT state name
    is_saturated: bool          # True if rate/accel/jerk limits were clamped
```

---

## 5. Execution Modes

### 5.1 Mode A — Deterministic Benchmark Mode
* **Architecture:** Standalone Python execution engine.
* **Timing:** Headless fixed-step clock: $t_{k+1} = t_k + \Delta t$.
* **Physics & Disturbances:** Pseudo-random number generators (PRNG) parameterized by a mandatory integer `seed`.
* **Execution Guarantee:** Running the same scenario configuration with the same seed guarantees exact bitwise telemetry reproducibility across multiple runs regardless of system CPU load.
* **Output:** JSON telemetry logs, structured CSV metrics, and automated PDF engineering reports.

### 5.2 Mode B — Live Mission Control Mode
* **Architecture:** FastAPI asynchronous server communicating with React 18 + Three.js frontend.
* **Server Role:** Executes the identical backend tracking loop (`core/perception/`, `core/estimation/`, `core/control/`). Evaluates the simulation clock and streams JSON telemetry payloads over WebSocket at 30–60 Hz.
* **Frontend Role:** Ingests telemetry, updates 3D terminal models and target nodes in Three.js, visualizes the 2D virtual camera feed, displays PID/Kalman HUD graphs via Recharts, and exposes parameter configuration sliders.
* **Decoupling:** If the WebSocket experiences latency or dropped frames, the backend tracking loop remains authoritative and unaffected.

---

## 6. Subsystem Architecture Deep-Dives

### 6.1 Perception Subsystem: Precision Optical Beacon Extraction

Standard bounding-box detectors (YOLO, Haar, simple contours) fail in FSOC coarse alignment because atmospheric turbulence, Rytov scintillation, and optical defocus introduce significant morphology changes and pixel quantization noise ($\pm 1.5\,\text{px}$). 

The competition-grade perception pipeline employs a 6-stage sub-pixel extraction process:

```
[Raw Frame I] ──> [Stage 1: Coarse ROI Candidate Detection]
                          │
                          ▼
                  [Stage 2: Local Background Estimation & Subtraction]
                          │
                          ▼
                  [Stage 3: Matched Filter / PSF Correlation]
                          │
                          ▼
                  [Stage 4: Adaptive Local Dual-Thresholding]
                          │
                          ▼
                  [Stage 5: Intensity-Weighted Centroiding (Moments)]
                          │
                          ▼
                  [Stage 6: Sub-Pixel Refinement & SNR Uncertainty Estimation]
```

1. **Stage 1: Coarse Candidate Detection:**
   - Detects salient bright regions using multi-scale morphological top-hat filtering and adaptive thresholding or optional lightweight neural detection.
   - Extracts localized $K \times K$ Regions of Interest (ROIs) around each candidate (nominal $K=32\,\text{px}$).
2. **Stage 2: Local Background Estimation:**
   - Background intensity $B_{\text{local}}$ is computed from the median of the outermost 2-pixel annular border of the ROI:
     $$\hat{B} = \text{median}\{I(u,v) \mid (u,v) \in \text{ROI Border}\}$$
     $$I_{\text{sub}}(u,v) = \max(0, I(u,v) - \hat{B})$$
3. **Stage 3: Matched Filter / PSF Correlation:**
   - Correlates $I_{\text{sub}}$ with a parameterized Airy disk / 2D Gaussian kernel matching the optical system's nominal beam point-spread function:
     $$\mathcal{K}(u,v) = \frac{1}{2\pi \sigma_{\text{psf}}^2} \exp\left(-\frac{u^2 + v^2}{2\sigma_{\text{psf}}^2}\right)$$
4. **Stage 4: Adaptive Dual-Thresholding:**
   - Threshold $T_{\text{high}} = \hat{B} + k_1 \hat{\sigma}_{\text{noise}}$ isolates candidate peaks.
   - Threshold $T_{\text{low}} = \hat{B} + k_2 \hat{\sigma}_{\text{noise}}$ ($k_2 < k_1$) retains the low-intensity diffraction skirt for centroiding.
5. **Stage 5: Intensity-Weighted Centroiding (First-Order Moments):**
   - Precise centroid position $[u_c, v_c]$ is computed over the connected component $\Omega$:
     $$m_{00} = \sum_{(u,v)\in\Omega} I_{\text{sub}}(u,v), \quad m_{10} = \sum_{(u,v)\in\Omega} u \cdot I_{\text{sub}}(u,v), \quad m_{01} = \sum_{(u,v)\in\Omega} v \cdot I_{\text{sub}}(u,v)$$
     $$u_c = \frac{m_{10}}{m_{00}}, \quad v_c = \frac{m_{01}}{m_{00}}$$
6. **Stage 6: Sub-Pixel Refinement & Variance Formulation:**
   - Calculates the spatial covariance (second-order central moments $\mu_{20}, \mu_{02}, \mu_{11}$) to evaluate spot circularity and beam spreading.
   - Formulates measurement uncertainty $\sigma_{\text{meas}}^2$ dynamically based on Cramér-Rao lower bound scaling:
     $$\sigma_{\text{meas}} \approx \frac{\sigma_{\text{psf}}}{\sqrt{\text{SNR}}}, \quad R_k = \begin{bmatrix} \sigma_{\text{meas}}^2 & 0 \\ 0 & \sigma_{\text{meas}}^2 \end{bmatrix}$$

---

### 6.2 Candidate Association Subsystem

To discard distractors, background clutter, and false visual triggers without cheating:
1. **Validation Gating (Mahalanobis Distance):**
   - For each candidate measurement $\mathbf{z}_j = [u_j, v_j]^T$ and predicted target position $\hat{\mathbf{z}}_{k|k-1} = \mathbf{H} \hat{\mathbf{x}}_{k|k-1}$:
     $$\mathbf{y}_j = \mathbf{z}_j - \hat{\mathbf{z}}_{k|k-1}$$
     $$\mathbf{S}_k = \mathbf{H} \mathbf{P}_{k|k-1} \mathbf{H}^T + \mathbf{R}_{k,j}$$
     $$d_{M,j}^2 = \mathbf{y}_j^T \mathbf{S}_k^{-1} \mathbf{y}_j$$
   - A candidate is rejected if $d_{M,j}^2 > \gamma_{\text{gate}}$ (where $\gamma_{\text{gate}} = 9.21$ corresponds to a 99% gating ellipse for a 2-DoF $\chi^2$ distribution).
2. **Multi-Feature Cost Function:**
   - Validated candidates within the gate are scored using a normalized composite metric:
     $$C_j = w_1 \left(\frac{d_{M,j}^2}{\gamma_{\text{gate}}}\right) + w_2 \left(1.0 - \frac{\text{SNR}_j}{\text{SNR}_{\max}}\right) + w_3 (1.0 - s_{\text{shape},j}) + w_4 (1.0 - s_{\text{temporal},j})$$
   - Candidate with $\min(C_j)$ is selected. If no candidates pass the gate, a `MISS` is declared and the estimator enters prediction-only mode.

---

### 6.3 Adaptive 6-State Kalman State Estimator

To accurately track accelerating targets and maintain pointing through scintillation dropouts without step velocity errors:

* **State Vector:**
  $$\mathbf{x} = \begin{bmatrix} u & v & \dot{u} & \dot{v} & \ddot{u} & \ddot{v} \end{bmatrix}^T \in \mathbb{R}^6$$
  where $(u, v)$ are pixel positions, $(\dot{u}, \dot{v})$ are pixel velocities ($\text{px/s}$), and $(\ddot{u}, \ddot{v})$ are pixel accelerations ($\text{px/s}^2$).
* **State Transition Model ($\mathbf{F}$):**
  $$\mathbf{F}(\Delta t) = \begin{bmatrix} 
  1 & 0 & \Delta t & 0 & \frac{1}{2}\Delta t^2 & 0 \\ 
  0 & 1 & 0 & \Delta t & 0 & \frac{1}{2}\Delta t^2 \\ 
  0 & 0 & 1 & 0 & \Delta t & 0 \\ 
  0 & 0 & 0 & 1 & 0 & \Delta t \\ 
  0 & 0 & 0 & 0 & 1 & 0 \\ 
  0 & 0 & 0 & 0 & 0 & 1 
  \end{bmatrix}$$
* **Process Noise Matrix ($\mathbf{Q}$):** Continuous white noise jerk model parameterized by spectral density $q$:
  $$\mathbf{Q} = \begin{bmatrix} \mathbf{Q}_{2\times 2} & \mathbf{0} \\ \mathbf{0} & \mathbf{Q}_{2\times 2} \end{bmatrix}, \quad \mathbf{Q}_{1D}(\Delta t) = q \begin{bmatrix} \frac{\Delta t^5}{20} & \frac{\Delta t^4}{8} & \frac{\Delta t^3}{6} \\ \frac{\Delta t^4}{8} & \frac{\Delta t^3}{3} & \frac{\Delta t^2}{2} \\ \frac{\Delta t^3}{6} & \frac{\Delta t^2}{2} & \Delta t \end{bmatrix}$$
* **Measurement Model ($\mathbf{H}$):**
  $$\mathbf{H} = \begin{bmatrix} 1 & 0 & 0 & 0 & 0 & 0 \\ 0 & 1 & 0 & 0 & 0 & 0 \end{bmatrix}$$
* **Adaptive Measurement Covariance ($\mathbf{R}_k$):** Dynamically modulated by the measured SNR from the perception stage:
  $$\mathbf{R}_k = \left(\sigma_{\text{base}}^2 + \frac{\kappa}{\max(\text{SNR}_k, 1.0)}\right) \mathbf{I}_{2\times 2}$$
* **Coasting Mode:** When a detection is missed, the filter executes the propagation step $\hat{\mathbf{x}}_{k|k-1} = \mathbf{F}\hat{\mathbf{x}}_{k-1|k-1}$ and $\mathbf{P}_{k|k-1} = \mathbf{F}\mathbf{P}_{k-1|k-1}\mathbf{F}^T + \mathbf{Q}$, setting $\hat{\mathbf{x}}_{k|k} = \hat{\mathbf{x}}_{k|k-1}$ and $\mathbf{P}_{k|k} = \mathbf{P}_{k|k-1}$. The spatial covariance grows monotonically, signaling increasing uncertainty to the FSM.

---

### 6.4 PAT Finite State Machine (8 States)

The 8-state deterministic FSM controls system operational phases:

```
                  ┌───────────────┐
       ┌─────────>│    SEARCH     │<───────────────┐
       │          └───────┬───────┘                │
       │                  │ candidate found        │
       │                  ▼                        │
       │          ┌───────────────┐                │
       │          │   CANDIDATE   │                │
       │          └───────┬───────┘                │
       │                  │ M-of-N confirmed       │
       │                  ▼                        │
       │          ┌───────────────┐                │
       │          │    ACQUIRE    │                │
       │          └───────┬───────┘                │
       │                  │ error < coarse gate    │
       │                  ▼                        │
       │          ┌───────────────┐                │
       │          │     TRACK     │                │
       │          └───────┬───────┘                │
       │                  │ error < fine gate      │
       │                  ▼                        │
       │          ┌───────────────┐                │
       │          │     LOCK      │                │
       │          └───────┬───────┘                │
       │                  │ miss detected          │
       │                  ▼                        │
       │          ┌───────────────┐                │
       │          │     COAST     │                │
       │          └───────┬───────┘                │
       │                  │ coast timeout          │
       │                  ▼                        │
       │          ┌───────────────┐                │
       │          │  REACQUIRE    │                │
       │          └───────┬───────┘                │
       │                  │ reacquire timeout      │
       │                  ▼                        │
       │          ┌───────────────┐                │
       └──────────┤     LOST      ├────────────────┘
                  └───────────────┘
```

| State | Entry Condition | Operational Behavior | Exit Condition |
| :--- | :--- | :--- | :--- |
| **`SEARCH`** | System boot or transition from `LOST`. | Drives gimbal in autonomous Archimedean spiral or raster search pattern. | Unassociated visual candidate detected $\rightarrow$ `CANDIDATE`. |
| **`CANDIDATE`** | First visual detection passed initial SNR gate. | Halts wide search. Observes target position without aggressive gimbal slew. | Confirmed in $\ge 3$ of last 5 frames $\rightarrow$ `ACQUIRE`. Gating failure / disappearance $\rightarrow$ `SEARCH`. |
| **`ACQUIRE`** | Target presence confirmed. | Initializes 6-state Kalman filter. Slew gimbal at maximum allowable acceleration toward target. | Tracking error $\epsilon_{\text{point}} < 1.5^\circ$ ($\approx 26\,\text{mrad}$) $\rightarrow$ `TRACK`. Target leaves FOV $\rightarrow$ `LOST`. |
| **`TRACK`** | Target stabilized in central FOV. | Coarse tracking active. Closed-loop PID + Feedforward driving gimbal. | $\epsilon_{\text{point}} < 0.2^\circ$ for $\ge 10$ consecutive frames $\rightarrow$ `LOCK`. Detection missed $\rightarrow$ `COAST`. |
| **`LOCK`** | High-precision alignment established. | Full fine-pointing mode. Telemetry records high-precision link margin. | $\epsilon_{\text{point}} > 0.5^\circ$ for $\ge 3$ frames $\rightarrow$ `TRACK`. Detection missed $\rightarrow$ `COAST`. |
| **`COAST`** | Target detection missed while in `TRACK` or `LOCK`. | Gimbal driven purely by extrapolated Kalman velocity/acceleration predictions. Covariance grows. | Target re-detected within Mahalanobis gate $\rightarrow$ `TRACK`. Miss duration exceeds $T_{\text{coast}}$ (e.g. 0.5 s) $\rightarrow$ `REACQUIRE`. |
| **`REACQUIRE`** | Target still missing after `COAST`. | Executes localized elliptical spiral scan centered around the predicted covariance centroid. | Target re-detected and confirmed $\rightarrow$ `TRACK`. Search time exceeds $T_{\text{reacq}}$ (e.g. 3.0 s) $\rightarrow$ `LOST`. |
| **`LOST`** | Target lost and local search exhausted. | Resets Kalman filter states and covariances. Reinitializes gimbal to nominal standby attitude. | Transition after 1 cycle $\rightarrow$ `SEARCH`. |

---

### 6.5 Predictive Feed-Forward & Smooth Control Architecture

```
Target State Estimate x_hat
       │
       ├─────────────────────────────> [Feed-Forward Velocity Block]
       │                                     │  omega_ff = K_ff * [v_x, v_y]
       ▼                                     │
[Latency-Ahead Predictor]                    │
  x_pred = x_hat(t + tau)                    │
       │                                     │
       ▼                                     │
[Pointing Error Calculator]                  │
  e = x_pred - [c_x, c_y]                    │
       │                                     │
       ▼                                     │
[Deadband / Hysteresis Filter]                │
       │                                     │
       ▼                                     │
[Anti-Windup PID with Filtered Derivative]    │
       │                                     │
       ▼ u_pid                               │
       └──────────────┬──────────────────────┘
                      │
                      ▼
               u_raw = omega_ff + u_pid
                      │
                      ▼
       [Gimbal Plant Actuator Model]
       - Rate Clamping (|omega| <= omega_max)
       - Acceleration Limiting (|alpha| <= alpha_max)
       - Jerk Limiting (|j| <= j_max)
       - 1st Order Motor Lag Filter (tau_m)
                      │
                      ▼
        Commanded Gimbal Pose [psi, theta]
```

1. **Estimated Angular Velocity Feed-Forward:**
   - Translates estimated target pixel velocity into camera angular rate commands:
     $$\omega_{\text{ff},\psi} = -\frac{\hat{\dot{u}}}{f_x}, \quad \omega_{\text{ff},\theta} = \frac{\hat{\dot{v}}}{f_y}$$
2. **Latency-Aware Prediction:**
   - Compensates for total pipeline processing and communication delay $\tau_{\text{latency}}$:
     $$\hat{\mathbf{z}}_{\text{future}} = \hat{\mathbf{x}}_{\text{pos}} + \hat{\mathbf{x}}_{\text{vel}} \tau_{\text{latency}} + \frac{1}{2} \hat{\mathbf{x}}_{\text{acc}} \tau_{\text{latency}}^2$$
     $$\mathbf{e}(t) = \hat{\mathbf{z}}_{\text{future}} - [c_x, c_y]^T$$
3. **Deadband & Hysteresis:**
   - If $\|\mathbf{e}(t)\| \le e_{\text{deadband}}$ (nominal $1.0\,\text{px} \approx 0.015^\circ$), error is clamped to zero to eliminate actuator hunting and servo jitter.
4. **Anti-Windup PID with Derivative Filter:**
   - Derivative action is computed on the measurement error using a low-pass filter ($N \approx 10–20$):
     $$D(s) = \frac{K_d N s}{s + N} E(s)$$
   - Integral term uses back-calculation clamping: integration is halted whenever actuator rate commands saturate.

---

### 6.6 Rate-, Acceleration-, and Jerk-Limited Gimbal Plant Model

Real electro-mechanical gimbals cannot undergo instantaneous velocity steps. The actuator model enforces physical second-order limits:

1. **Jerk Limiting:**
   $$\dddot{\mathbf{q}} = \text{clamp}\left(\frac{\ddot{\mathbf{q}}_{\text{des}} - \ddot{\mathbf{q}}(t)}{\Delta t}, -j_{\max}, +j_{\max}\right)$$
2. **Acceleration Limiting:**
   $$\ddot{\mathbf{q}}(t + \Delta t) = \text{clamp}\left(\ddot{\mathbf{q}}(t) + \dddot{\mathbf{q}} \Delta t, -\alpha_{\max}, +\alpha_{\max}\right)$$
3. **Velocity Limiting:**
   $$\dot{\mathbf{q}}(t + \Delta t) = \text{clamp}\left(\dot{\mathbf{q}}(t) + \ddot{\mathbf{q}}(t + \Delta t) \Delta t, -\omega_{\max}, +\omega_{\max}\right)$$
4. **First-Order Motor Inertia Lag:**
   $$\dot{\mathbf{q}}_{\text{actual}}(t + \Delta t) = \dot{\mathbf{q}}_{\text{actual}}(t) + \frac{\Delta t}{\tau_{\text{motor}}} \left(\dot{\mathbf{q}}(t + \Delta t) - \dot{\mathbf{q}}_{\text{actual}}(t)\right)$$
5. **Physical Parameter Bounds:**
   - $\omega_{\max} = 20.0\,\text{deg/s}$ ($349\,\text{mrad/s}$)
   - $\alpha_{\max} = 80.0\,\text{deg/s}^2$ ($1396\,\text{mrad/s}^2$)
   - $j_{\max} = 400.0\,\text{deg/s}^3$ ($6981\,\text{mrad/s}^3$)
   - $\tau_{\text{motor}} = 0.020\,\text{s}$ ($20\,\text{ms}$)

---

### 6.7 Recovery & Reacquisition Architecture

When target loss occurs (e.g. scintillation fade or obstacle occlusion):

1. **Phase 1: Inertial Coasting (0 to $0.5\,\text{s}$):**
   - Kalman estimator coasts with zero measurement update. The gimbal continues tracking along the predicted trajectory.
2. **Phase 2: Covariance-Guided Uncertainty Ellipse Search ($0.5$ to $1.5\,\text{s}$):**
   - The spatial covariance $\mathbf{P}_{\text{pos}} = \begin{bmatrix} P_{00} & P_{01} \\ P_{10} & P_{11} \end{bmatrix}$ defines an uncertainty ellipse with semi-major axis $a = \sqrt{\lambda_1}$ and semi-minor axis $b = \sqrt{\lambda_2}$.
   - The gimbal executes a micro-nutation scan covering the $3\sigma$ uncertainty footprint.
3. **Phase 3: Expanding Archimedean Spiral ($1.5$ to $3.0\,\text{s}$):**
   - If the target is not found within the local uncertainty zone, the camera executes an expanding Archimedean spiral:
     $$r(\phi) = \frac{d_{\text{fov}}}{2\pi} \phi, \quad \phi(t) = \sqrt{\frac{4\pi v_{\text{scan}}}{d_{\text{fov}}} t}$$
   - Ensures continuous, gapless FOV coverage without overshooting.
4. **Phase 4: Multi-Frame Confirmation:**
   - Any visual signal detected during reacquisition must be confirmed across $M=3$ consecutive frames before the FSM transitions back to `TRACK` or `LOCK`, preventing lock on transient noise spikes.

---

### 6.8 Optional Temporal Beacon Identification (TBI)

To provide definitive distractor rejection without relying on simulator IDs:
* **Principle:** The simulated target beacon encodes a known, subtle, low-depth square-wave or pseudo-random noise (PRN) intensity modulation (e.g., $10\,\text{Hz}$ on/off or Gold code modulation).
* **Demodulation:** The perception candidate extractor maintains a sliding intensity history buffer for all persistent visual spots:
  $$\mathbf{I}_{\text{hist}} = [I(t-N\Delta t), \dots, I(t)]$$
* **Correlation:** The temporal correlation coefficient $\rho_{\text{TBI}}$ is evaluated against the expected code template $\mathbf{T}_{\text{code}}$:
  $$\rho_{\text{TBI}} = \frac{\sum (I_k - \bar{I})(T_k - \bar{T})}{\sqrt{\sum (I_k - \bar{I})^2 \sum (T_k - \bar{T})^2}}$$
* **Integration:** Included in candidate association cost metric $s_{\text{temporal}} = \max(0.0, \rho_{\text{TBI}})$. Completely optional; benchmark suite supports both TBI-coded and uncoded modes.

---

## 7. Telemetry & Experiment Logging Architecture

All benchmark runs and live mission sessions record standardized telemetry:

```python
@dataclass(frozen=True)
class TelemetryRecord:
    timestamp: float
    frame_id: int
    fsm_state: str
    
    # Ground Truth (Offline Evaluation Only)
    target_true_pos_world: Tuple[float, float, float]
    target_true_pos_pixel: Tuple[float, float]
    true_pointing_error_deg: float
    true_pointing_error_mrad: float
    
    # Perception
    num_candidates_detected: int
    selected_candidate_pixel: Optional[Tuple[float, float]]
    selected_candidate_snr_db: float
    selected_candidate_flux: float
    
    # Estimation
    kalman_state_pixel: Tuple[float, float, float, float, float, float]
    kalman_pos_uncertainty_px: float
    innovation_pixel: Optional[Tuple[float, float]]
    mahalanobis_distance: float
    is_kalman_updated: bool
    
    # Control & Actuation
    gimbal_az_deg: float
    gimbal_el_deg: float
    gimbal_az_rate_deg_s: float
    gimbal_el_rate_deg_s: float
    gimbal_az_accel_deg_s2: float
    gimbal_el_accel_deg_s2: float
    pid_proportional: Tuple[float, float]
    pid_integral: Tuple[float, float]
    pid_derivative: Tuple[float, float]
    feedforward_rates: Tuple[float, float]
    is_actuator_saturated: bool
    
    # Timing & Performance
    detector_latency_ms: float
    estimator_latency_ms: float
    controller_latency_ms: float
    total_pipeline_latency_ms: float
```

---

## 8. Architectural Invariant Summary

1. **NO Ground Truth in Decision Logic:** `main.py`, `GimbalCamera.jsx`, and all control loops operate strictly on extracted pixel coordinates and internal state estimates.
2. **Deterministic Mode A as Source of Truth:** Scientific benchmark claims are valid ONLY when executed via the headless Mode A benchmark framework.
3. **No Unbounded Actuation:** All gimbal rate commands pass through rate, acceleration, and jerk limiters.
4. **Sub-Pixel Centroiding Standard:** Bounding box center coordinates are deprecated in favor of intensity-weighted moments.
5. **Architectural Status:** FROZEN. Proceed to `plans/03_IMPLEMENTATION_PLAN.md`.
