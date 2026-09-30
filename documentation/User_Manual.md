# FSOC Coarse Alignment Suite: User Manual & Operator Guide

## 1. System Overview
The **FSOC AI Virtual Camera Tracking Simulator** is a full closed-loop laboratory test bench and testing environment for Free-Space Optical Communications. It enables research into coarse alignment pointing, acquisition, and tracking algorithms. 

---

## 2. Installation & Quick Start

### 2.1 System Prerequisites
- **Python 3.10 to 3.13** (tested with 3.13)
- **Node.js v18+** & **npm**

### 2.2 Backend Installation
1. Navigate to the backend folder:
   ```bash
   cd backend
   ```
2. Create and activate a Python virtual environment:
   ```bash
   python -m venv .venv
   .venv\Scripts\activate
   ```
3. Install required dependencies:
   ```bash
   pip install -r requirements.txt
   ```
4. Start the FastAPI uvicorn server:
   ```bash
   python -m uvicorn app.main:app --host 127.0.0.1 --port 8000 --reload
   ```

### 2.3 Frontend Installation
1. Navigate to the frontend folder:
   ```bash
   cd ../frontend
   ```
2. Install npm dependencies:
   ```bash
   npm install
   ```
3. Launch the development server:
   ```bash
   npm run dev
   ```
4. Open the displayed URL in your browser (usually `http://localhost:5173`).

---

## 3. Dashboard Operation Guide

### 3.1 The Landing Screen
Upon loading the web application, you are presented with the aerospace telemetry entry screen:
- **Signal Flow Visualizer**: Shows the active pipeline (`Camera` $\rightarrow$ `AI` $\rightarrow$ `Kalman` $\rightarrow$ `PID` $\rightarrow$ `Lock`) with glowing indicator states.
- **Entry Points**: 
  - **Launch Simulator**: Starts standard tracking session in Overview tab.
  - **Hardware & Ablations**: Directs to configuration adjustments.
  - **Scenarios & Benchmarks**: Opens predefined experiments list.
  - **Run SIH Demo Mode**: Launches a fully automated 2-minute demonstration.

### 3.2 Tab 1: System Overview (Live Viewport)
- **3D WebGL Scene**: Visualizes the local FSOC terminal stand, two-axis camera gimbal, and remote optical beacons in 3D. Click and drag to orbit; scroll to zoom.
- **Camera Feed**: Overlay HUD displaying:
  - Bounding box around target.
  - **Green Dashed Error Vector**: A line showing the alignment error vector from the optical center (crosshair) to the beacon.
  - AI confidence percentage and inference time.
  - FALLBACK WARNING (yellow) flashes if YOLO inference frames drop, showing backup OpenCV color-threshold tracking.
- **HUD telemetry panels**: Displays tracking states (`SEARCHING`, `DETECTED`, `TRACKING`, `LOCKED`, etc.), error rates, and coordinate readouts.

### 3.3 Tab 2: Hardware Tuning & Ablation Filters
- **Gimbal Actuation (PID)**: Sliders and number inputs to adjust Proportional ($K_p$), Integral ($K_i$), and Derivative ($K_d$) coefficients for Pan and Tilt motors.
- **Target Configurations**: Change designated target IDs (`BEACON-01` to `BEACON-04`), select trajectories (`linear`, `circular`, `spiral`, `sine`), and set kinematics velocities.
- **Ablation Bypass Filters**:
  - Toggle **AI Detection**: Bypassing AI falls back to coordinate projection (ground truth).
  - Toggle **Kalman Filter**: Disabling estimation runs tracking directly on noisy camera pixels.
  - Toggle **PID Controller**: Disabling PID freezes gimbal actuators (no motor corrections).
  - Toggle **Disturbance Model**: Bypassing ignores all active environmental degradation.
- **Environmental Disturbances**: Sliders to increase vibration, noise, atmospheric turbulence, and attenuation intensities in real-time.

### 3.4 Tab 3: Real-Time Analytics
Five charts plotting live-streamed telemetry data:
1. **Pointing Angular Error**: Plots total pointing error, yaw (pan) error, and pitch (tilt) error in degrees.
2. **System Latency & Frame Rate**: Measures WebSocket rate and AI inference duration.
3. **Beacon Detector Confidence**: Graphs target confidence over time.
4. **Gimbal Axis Angles**: Yaw and pitch positioning in degrees.
5. **Timeline Lock State**: Horizontal bar showing locked vs. search durations.

### 3.5 Tab 4: Scenarios & Benchmarks
- **Predefined Scenarios Directory**: Pick from 10 stress tests and click to execute.
- **Automated Demo**: Click `RUN AUTOMATED SIH DEMO MODE` to run the E2E E2E testing sequence.
- **Comparative Metrics Matrix**: Select checkboxes of completed experiments to generate a comparative side-by-side table analyzing lock rate, avg error, and response latencies.
- **Reports Export**:
  - Click **PDF** to download a compiled 2-page performance evaluation report with embedded charts and technical assessments.
  - Click **CSV** to download raw frame-by-frame coordinate logs.

---

## 4. Troubleshooting

### 4.1 WebSocket Connection Error
- **Symptom**: Overview tab shows "CAMERA disconnected" and red WEBSOCKET indicator.
- **Resolution**: Check that your FastAPI uvicorn backend is running on `http://127.0.0.1:8000`. Ensure that no firewall is blocking port 8000.

### 4.2 Report Generation Fails
- **Symptom**: PDF download fails or results in empty file.
- **Resolution**: Verify that the libraries `matplotlib` and `reportlab` are installed in your active virtual environment. Check the backend logs for folder permission issues in the `experiments/results/` directory.

### 4.3 Low Frame Rates (< 15 FPS)
- **Symptom**: Interface is laggy, WebGL viewport stutters.
- **Resolution**: If your machine lacks a discrete GPU, switch the **Detector Mode** from `yolo` or `hybrid` to `opencv`. OpenCV color-thresholding consumes significantly less CPU/GPU compute.
