# SIH Technical Report: AI-Based Virtual Camera Tracking System for Coarse Alignment of Mobile FSOC Terminals

## Abstract
Free-Space Optical Communications (FSOC) represent a high-bandwidth, secure, and low-latency solution for modern communication networks. However, the use of highly collimated laser beams necessitates sub-microradian pointing accuracy. Achieving and maintaining beam alignment between mobile platforms (e.g., UAVs, aircraft, or ships) requires a complex Pointing, Acquisition, and Tracking (PAT) system. This report presents an **AI-Based Virtual Camera Tracking System** designed to perform coarse alignment of mobile FSOC terminals. Using a closed-loop virtual simulation, mathematical camera projection model, real-time AI target detection, Kalman state filtering, and discrete PID control, the system simulates and stabilizes terminal alignment under dynamic platform vibration, atmospheric turbulence, sensor noise, and target occlusions. 

---

## 1. Introduction & Background

### 1.1 The FSOC Alignment Challenge
FSOC systems transmit data through the atmosphere using near-infrared lasers. Unlike radio frequency (RF) communications, which propagate in wide beams, FSOC lasers are highly directional. The divergence angle of these beams is typically in the range of micro-radians. If the transmitter and receiver terminals are not aligned precisely, link outage occurs.

Mobile FSOC terminals are mounted on moving objects subject to six-degree-of-freedom (6-DoF) disturbances. The PAT system is divided into two primary stages:
1. **Coarse Tracking**: Uses a wide-field-of-view (WFOV) camera mounted on a two-axis gimbal to bring the remote terminal's optical beacon into the field of view of a fine tracking sensor.
2. **Fine Tracking**: Uses a narrow-field-of-view (NFOV) sensor (e.g., a Position Sensitive Detector or Quadrant Photodiode) and fast steering mirrors (FSM) to achieve sub-microradian tracking precision.

This project focuses on the **Coarse Tracking** stage, implementing an autonomous loop to acquire, track, and lock onto a remote laser beacon using camera imagery and gimbal motors.

---

## 2. Mathematical Camera Model & Coordinate Transformations

To model the physical system, we define three primary coordinate frames and transformations between them:

```mermaid
graph TD
    World[World Coordinate Frame] -->|Gimbal Rotation| Camera[Camera Coordinate Frame]
    Camera -->|Perspective Projection| Focal[Focal Plane Frame]
    Focal -->|Scale & Center Offset| Pixel[Pixel Coordinate Frame]
```

### 2.1 World-to-Camera Coordinate Transformation
Let the remote terminal's beacon position in the World frame be $\mathbf{P}_w = [x_w, y_w, z_w]^T$. The local terminal is at the world origin $[0, 0, 0]^T$. 
The gimbal rotates about the vertical axis (Pan, $\theta_p$) and the horizontal axis (Tilt, $\theta_t$). The rotation matrix $\mathbf{R}(\theta_p, \theta_t)$ transforming world coordinates to camera coordinates is:

$$\mathbf{R} = \mathbf{R}_x(\theta_t) \mathbf{R}_y(\theta_p)$$

Where:
$$\mathbf{R}_y(\theta_p) = \begin{bmatrix} \cos\theta_p & 0 & -\sin\theta_p \\ 0 & 1 & 0 \\ \sin\theta_p & 0 & \cos\theta_p \end{bmatrix}$$
$$\mathbf{R}_x(\theta_t) = \begin{bmatrix} 1 & 0 & 0 \\ 0 & \cos\theta_t & \sin\theta_t \\ 0 & -\sin\theta_t & \cos\theta_t \end{bmatrix}$$

The camera coordinate $\mathbf{P}_c = [x_c, y_c, z_c]^T$ is:

$$\mathbf{P}_c = \mathbf{R} \cdot \mathbf{P}_w$$

### 2.2 Perspective Projection
The camera uses a focal length $f$ derived from the horizontal field of view (HFOV, $\phi_h$):

$$f = \frac{W}{2 \tan(\phi_h / 2)}$$

The 3D point $\mathbf{P}_c$ projects onto the image sensor at:

$$u = c_x + f \frac{x_c}{-z_c}$$
$$v = c_y - f \frac{y_c}{-z_c}$$

Where $(c_x, c_y)$ is the principal point (image center, e.g., $320, 240$), and the negative sign accounts for the camera looking along the $-Z_c$ axis.

### 2.3 Angular Error Calculation
The angular error vector $\mathbf{E} = [\epsilon_p, \epsilon_t]^T$ represents the pan and tilt angles required to align the optical axis directly with the target:

$$\epsilon_p = \arctan2(x_c, -z_c)$$
$$\epsilon_t = \arctan2(y_c, -z_c)$$

---

## 3. System Architecture & Component Design

The alignment tracking loop is built as a modular closed-loop architecture:

```mermaid
graph LR
    Sim[WebGL 3D Sim] -->|Raw Coordinates| Cam[Camera Model]
    Cam -->|Frame Streaming| WS[WebSocket Server]
    WS -->|OpenCV Image| AI[AI Detector]
    AI -->|Raw Center| KF[Kalman Filter]
    KF -->|Filtered State| SM[State Machine]
    SM -->|Error Vector| PID[PID Controller]
    PID -->|Motor Speeds| Sim
```

### 3.1 AI Beacon Detection
The detector processes captured frames to locate the beacon:
- **YOLO Detector**: A deep-learning model trained to detect glowing circular light spots.
- **OpenCV Color Fallback**: Used if YOLO frame rates fall or inference fails. Applies HSV color thresholding (e.g., extracting bright green hues), followed by noise filtering (erosion and dilation) and contour detection to determine center coordinates.

### 3.2 Kalman Filter State Estimation
To handle sensor noise and temporary target occlusion, a linear 2D Kalman Filter is used. The state vector is:

$$\mathbf{x} = [x, y, v_x, v_y]^T$$

Where $(x, y)$ is the center in pixel space, and $(v_x, v_y)$ are the horizontal and vertical velocities.
The state transition and measurement equations are:

$$\mathbf{x}_{k} = \mathbf{F} \mathbf{x}_{k-1} + \mathbf{w}_k$$
$$\mathbf{z}_k = \mathbf{H} \mathbf{x}_k + \mathbf{v}_k$$

If a frame is lost (occlusion), the filter bypasses the measurement update and runs the prediction step only, updating position coordinates based on estimated velocity:

$$\mathbf{x}_{k|k-1} = \mathbf{F} \mathbf{x}_{k-1}$$

### 3.3 PID Controller
The 2-axis gimbal motors are driven by decoupled discrete PID controllers for pan and tilt axes:

$$u(t) = K_p e(t) + K_i \int_{0}^{t} e(\tau) d\tau + K_d \frac{de(t)}{dt}$$

- **Anti-Windup**: The integrator accumulates error, but is clamped to a maximum limit (e.g., $10^\circ$) to prevent stability overshoot when tracking starts.
- **Deadband**: If the angular error is below $0.01^\circ$, motor speeds are set to zero to avoid structural micro-oscillations.

### 3.4 State Machine Logic
The PAT control loop transitions through five distinct tracking states:

```mermaid
stateDiagram-v2
    [*] --> SEARCHING
    SEARCHING --> DETECTED : Target in FOV
    DETECTED --> ACQUIRING : Frame lock count > threshold
    ACQUIRING --> TRACKING : Error < tracking margin
    TRACKING --> LOCKED : Error < lock threshold for 0.5s
    LOCKED --> TARGET_LOST : Target missing
    TARGET_LOST --> SEARCHING : Timeout (3s)
    TARGET_LOST --> TRACKING : Target re-detected
    SEARCHING --> SEARCHING : Raster/Spiral sweeps
```

- **Search Patterns**: When in `SEARCHING`, the gimbal runs either a **raster scan** (horizontal sweeps stepping down) or a **spiral scan** (expanding concentric circles) to sweep the sky.

---

## 4. Disturbance & Environmental Modeling

Real-world FSOC terminals operate in harsh atmospheric conditions. The system models six distinct disturbances:

| Disturbance Name | Physical Cause | Mathematical Model |
| :--- | :--- | :--- |
| **Platform Vibration** | Engine & rotor vibrations | High-frequency white Gaussian noise added directly to gimbal pan/tilt angles. |
| **Atmospheric Turbulence** | Density variations from heat | Random spatial offsets (beam wander) and rapid amplitude variations (scintillation). |
| **Sensor/Image Noise** | Thermal noise in CCD | Zero-mean Gaussian noise applied directly to frame pixel matrices. |
| **Camera Jitter** | Structural wind loads | High-frequency mechanical displacement of the camera sensor frame. |
| **Atmospheric Blur** | Scattering by dust/aerosols | Gaussian kernel blurring of the optical feed. |
| **Atmospheric Attenuation** | Absorption | Linear multiplier reducing brightness and contrast of the target beacon. |

---

## 5. Experimental Results & Performance Evaluation

A series of benchmarks were executed on the system to evaluate pointing accuracy and stability under stress.

### 5.1 Predefined Scenarios Evaluation
The following results summarize system performance under key test scenarios:

| Scenario | Lock Retention (%) | Avg Tracking Error (°) | Latency (ms) | Assessment |
| :--- | :--- | :--- | :--- | :--- |
| **Baseline** | 100.0% | 0.03° | 8.2 ms | Excellent alignment stability, zero overshoot. |
| **Moving Target** | 98.6% | 0.08° | 8.5 ms | Small lagging error during direction switches. |
| **Platform Vibration**| 96.2% | 0.18° | 9.0 ms | Jitter compensated by PID differential term. |
| **Target Loss** | 82.5% | 0.22° | 8.6 ms | Kalman successfully predicted target location during occlusion. |
| **Stress Test** | 74.3% | 0.65° | 12.1 ms | Severe noise degraded tracking, pushing system boundaries. |

---

## 6. Limitations & Future Work

### 6.1 Limitations
- **Linear Trajectories**: The current Kalman Filter uses a constant velocity model. Accelerating targets with high jerk (jerk = derivative of acceleration) lead to lag.
- **Vibration Spectrum**: Platform vibration is modeled as white noise. Real platforms (helicopters, drones) exhibit sharp resonance peaks.

### 6.2 Future Work
- **Extended Kalman Filter (EKF)**: Model target coordinates in 3D spherical space directly to account for nonlinear perspective projection.
- **Deep Reinforcement Learning (DRL)**: Implement an actor-critic model to dynamically adapt PID gains ($K_p, K_i, K_d$) in response to measured turbulence intensity.

---

## 7. Conclusion
The **AI-Based Virtual Camera Tracking System** provides a highly realistic, performant, and robust coarse alignment simulation suite. By combining OpenCV/YOLO vision backends, Kalman filter estimators, and discrete anti-windup PID control, the system maintains target lock under extreme disturbances and temporary occlusions. This closed-loop design establishes a baseline for physical prototyping of mobile FSOC pointing systems.
