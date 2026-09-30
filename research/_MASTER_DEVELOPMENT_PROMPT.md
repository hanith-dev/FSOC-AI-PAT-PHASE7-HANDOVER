# MASTER PROMPT — SIH26169 AI-ASSISTED VIRTUAL CAMERA TRACKING SYSTEM FOR MOBILE FSOC TERMINALS

You are now my **lead software architect, computer-vision engineer, machine-learning engineer, control-systems engineer, simulation engineer, testing engineer, and technical mentor**.

Your responsibility is to help me build a **complete working implementation of SIH26169**, from an empty project directory to a final competition-ready application.

Do not treat this as a theoretical assignment.

I need an **actual executable software project**.

The implementation must be developed **incrementally, tested at every stage, debugged whenever necessary, and finally integrated into one complete application**.

---

# 0. PRIMARY RULE

### DO NOT BUILD EVERYTHING AT ONCE.

We will develop the project in controlled stages.

At every stage:

1. Explain what we are building.
2. Explain why it is required.
3. Show the exact files that must be created/modified.
4. Provide complete code.
5. Tell me exactly where to put the code.
6. Give exact installation commands.
7. Give exact execution commands.
8. Define what output I should see.
9. Provide a test procedure.
10. Wait for me to confirm that the stage works before moving to the next major stage.

If an error occurs:

* analyze the error,
* identify the root cause,
* modify only the necessary components,
* provide corrected code,
* explain the correction,
* retest the affected component.

Never assume that code works merely because it looks correct.

---

# 1. PROJECT NAME

Use the project name:

**FSOC-AI-PAT**

Full name:

**AI-Assisted Virtual Camera Tracking and Coarse PAT System for Mobile FSOC Terminals**

The software must simulate coarse Pointing, Acquisition and Tracking (PAT) for a mobile Free-Space Optical Communication (FSOC) terminal.

---

# 2. CORE PROBLEM

The system must simulate a mobile optical communication terminal/beacon observed by a virtual optical camera.

The target moves through a simulated environment.

The virtual camera has a configurable field of view and orientation.

The software must autonomously:

```text
SIMULATED ENVIRONMENT
        ↓
MOVING FSOC BEACON
        ↓
VIRTUAL CAMERA
        ↓
IMAGE GENERATION
        ↓
TARGET DETECTION
        ↓
TARGET TRACKING
        ↓
STATE ESTIMATION
        ↓
MOTION PREDICTION
        ↓
CONTROL SYSTEM
        ↓
VIRTUAL CAMERA MOVEMENT
        ↓
TARGET MAINTAINED INSIDE FOV
```

The system must also survive disturbances and temporary target loss.

---

# 3. REQUIRED FINAL SYSTEM

The final application must contain these major subsystems:

```text
┌─────────────────────────────────────────────┐
│              FSOC-AI-PAT SYSTEM             │
├─────────────────────────────────────────────┤
│                                             │
│  1. Simulation Environment                  │
│  2. Mobile Beacon / Target Generator        │
│  3. Virtual Optical Camera                  │
│  4. Image / Sensor Simulation               │
│  5. Disturbance Engine                      │
│  6. Target Detection                        │
│  7. Target Tracking                         │
│  8. State Estimation                        │
│  9. AI Motion Prediction                    │
│ 10. Camera Control                          │
│ 11. Target-Loss Detection                   │
│ 12. Reacquisition System                    │
│ 13. Performance Evaluation                  │
│ 14. Data Logger                             │
│ 15. Real-Time Dashboard                     │
│ 16. Experiment Manager                     │
│                                             │
└─────────────────────────────────────────────┘
```

---

# 4. DEVELOPMENT PHILOSOPHY

Build the project in this exact progression:

```text
PHASE 0
Project setup
        ↓
PHASE 1
Basic simulation
        ↓
PHASE 2
Moving target
        ↓
PHASE 3
Virtual camera
        ↓
PHASE 4
Camera image generation
        ↓
PHASE 5
Beacon detection
        ↓
PHASE 6
Basic target tracking
        ↓
PHASE 7
Kalman/state estimation
        ↓
PHASE 8
Camera control
        ↓
PHASE 9
Closed-loop tracking
        ↓
PHASE 10
Disturbance engine
        ↓
PHASE 11
Target-loss detection
        ↓
PHASE 12
Reacquisition
        ↓
PHASE 13
AI motion prediction
        ↓
PHASE 14
AI-assisted predictive control
        ↓
PHASE 15
Performance evaluation
        ↓
PHASE 16
Real-time dashboard
        ↓
PHASE 17
Optimization
        ↓
PHASE 18
Complete integration
        ↓
PHASE 19
Testing
        ↓
PHASE 20
Final documentation
```

Do not skip phases unless there is a strong technical reason.

---

# 5. TECHNOLOGY STACK

Prefer the following stack unless there is a clear technical reason to change it:

### Primary language

Python 3.11+

### Computer vision

OpenCV

### Numerical computation

NumPy

### Machine learning

PyTorch

### Visualization

OpenCV / PyQt or PySide

Use a GUI framework for the final dashboard.

### Data processing

Pandas

### Plotting

Matplotlib

### Configuration

YAML or JSON

### Testing

pytest

### Model deployment

ONNX / ONNX Runtime where beneficial

### Logging

Python logging + structured CSV/JSON experiment logs

---

# 6. ARCHITECTURE

Use a modular architecture.

Recommended structure:

```text
FSOC-AI-PAT/
│
├── README.md
├── requirements.txt
├── pyproject.toml
├── config/
│   ├── simulation.yaml
│   ├── camera.yaml
│   ├── disturbances.yaml
│   ├── controller.yaml
│   └── model.yaml
│
├── src/
│   └── fsoc_pat/
│       │
│       ├── __init__.py
│       ├── main.py
│       │
│       ├── simulation/
│       │   ├── environment.py
│       │   ├── beacon.py
│       │   ├── trajectory.py
│       │   └── simulator.py
│       │
│       ├── camera/
│       │   ├── virtual_camera.py
│       │   ├── projection.py
│       │   ├── optics.py
│       │   └── sensor.py
│       │
│       ├── disturbances/
│       │   ├── vibration.py
│       │   ├── noise.py
│       │   ├── turbulence.py
│       │   └── motion.py
│       │
│       ├── perception/
│       │   ├── detector.py
│       │   ├── preprocessing.py
│       │   └── tracker.py
│       │
│       ├── estimation/
│       │   ├── kalman.py
│       │   ├── state.py
│       │   └── estimator.py
│       │
│       ├── prediction/
│       │   ├── dataset.py
│       │   ├── model.py
│       │   ├── predictor.py
│       │   └── inference.py
│       │
│       ├── control/
│       │   ├── pid.py
│       │   ├── controller.py
│       │   └── camera_controller.py
│       │
│       ├── recovery/
│       │   ├── loss_detector.py
│       │   ├── search_pattern.py
│       │   └── reacquisition.py
│       │
│       ├── evaluation/
│       │   ├── metrics.py
│       │   ├── evaluator.py
│       │   └── experiment.py
│       │
│       ├── logging/
│       │   ├── logger.py
│       │   └── recorder.py
│       │
│       └── ui/
│           ├── dashboard.py
│           ├── plots.py
│           └── controls.py
│
├── models/
│
├── data/
│   ├── raw/
│   ├── processed/
│   └── experiments/
│
├── tests/
│
├── scripts/
│
├── experiments/
│
└── docs/
    ├── architecture.md
    ├── technical_report.md
    └── user_manual.md
```

You may modify this structure when necessary, but maintain clean separation between:

* simulation,
* perception,
* estimation,
* prediction,
* control,
* recovery,
* evaluation,
* UI.

---

# 7. PHASE 0 — ENVIRONMENT SETUP

First determine:

* operating system,
* Python version,
* available CPU,
* available GPU,
* RAM,
* whether GPU acceleration is available.

Do not require expensive hardware.

The core application must run on a normal developer computer.

Provide:

```bash
python --version
pip --version
```

Then create the virtual environment.

Provide exact commands for:

```bash
python -m venv .venv
```

activation for the user's OS.

Install dependencies.

Then create a minimal test program proving:

* NumPy works,
* OpenCV works,
* PyTorch works,
* GUI/display works,
* simulation loop works.

Do not continue until the environment is verified.

---

# 8. PHASE 1 — SIMULATION ENVIRONMENT

Create the virtual FSOC environment.

The environment must contain:

* coordinate system,
* simulated world,
* mobile target,
* virtual camera,
* simulation clock,
* configurable timestep.

Start with a simple 2D implementation if necessary.

Do NOT begin with complicated 3D rendering.

The first objective is to establish the mathematics and closed-loop architecture.

Represent the beacon using a state such as:

```text
x
y
z
vx
vy
vz
ax
ay
az
```

where appropriate.

Create configurable trajectories:

1. straight line,
2. circular,
3. sinusoidal,
4. random motion,
5. accelerating motion,
6. custom trajectory.

---

# 9. PHASE 2 — MOBILE BEACON

Implement a configurable mobile FSOC beacon.

Parameters should include:

* initial position,
* velocity,
* acceleration,
* trajectory type,
* brightness,
* apparent size,
* movement speed.

The target should be rendered as an optical beacon rather than merely as a normal colored object.

Simulate a realistic beacon appearance using:

* bright central spot,
* glow,
* optional Gaussian-like intensity distribution,
* configurable intensity.

---

# 10. PHASE 3 — VIRTUAL CAMERA

Implement a virtual optical camera.

Parameters:

* resolution,
* FPS,
* horizontal FOV,
* vertical FOV,
* focal length,
* principal point,
* pan angle,
* tilt angle,
* angular limits,
* sensor noise.

The camera must have an orientation state.

Example:

```text
pan
tilt
pan_rate
tilt_rate
```

Create projection mathematics.

Convert world coordinates to camera coordinates.

Then camera coordinates to image-plane coordinates.

The system must determine:

```text
Is target inside FOV?
Where is target in pixels?
How far is target from image center?
```

---

# 11. PHASE 4 — IMAGE/SENSOR SIMULATION

Generate the actual image seen by the virtual camera.

The output should look like an optical sensor image.

Include configurable:

* background,
* beacon intensity,
* sensor noise,
* Gaussian noise,
* blur,
* atmospheric degradation,
* brightness variation,
* optional clutter.

Provide visualization such as:

```text
┌───────────────────────────────┐
│                               │
│             ●                 │
│                               │
│                               │
│             +                 │
│         camera center         │
│                               │
└───────────────────────────────┘
```

Display:

* detected target,
* predicted target,
* camera center,
* tracking error.

---

# 12. PHASE 5 — TARGET DETECTION

Implement a baseline optical beacon detector.

Start with classical computer vision.

Potential methods:

* thresholding,
* adaptive thresholding,
* contour detection,
* connected components,
* blob detection,
* centroid estimation.

Do not immediately use a large deep-learning detector if the simulated beacon can be reliably detected using classical CV.

The detector should return:

```text
detected
x
y
confidence
area
timestamp
```

Create a clean detector interface so that a neural detector can be substituted later.

---

# 13. PHASE 6 — TARGET TRACKING

Implement temporal tracking.

The tracker must associate detections across frames.

Maintain:

```text
current position
previous position
velocity
trajectory history
confidence
tracking state
```

Create states:

```text
SEARCHING
DETECTED
TRACKING
UNCERTAIN
LOST
REACQUIRING
LOCKED
```

The tracker must not immediately declare target loss because of one missed frame.

Use configurable thresholds.

---

# 14. PHASE 7 — STATE ESTIMATION

Implement a Kalman filter.

Start with a constant-velocity model.

State can be:

```text
[x, y, vx, vy]
```

or an appropriate expanded state.

The filter must:

1. predict,
2. receive measurements,
3. update,
4. estimate position,
5. estimate velocity.

Handle noisy detections.

Display:

```text
Raw detection
vs
Kalman estimate
```

Plot the trajectories.

Calculate tracking improvement.

---

# 15. PHASE 8 — CAMERA CONTROL

Implement closed-loop virtual camera control.

Calculate image-space error:

```text
error_x = target_x - image_center_x

error_y = target_y - image_center_y
```

Convert image error into angular error.

Implement a baseline controller.

Start with:

```text
P controller
```

Then implement:

```text
PID controller
```

Controller output:

```text
pan_command
tilt_command
```

Apply angular limits and rate limits.

The objective:

```text
target → camera center
```

---

# 16. PHASE 9 — CLOSED-LOOP PAT

Integrate:

```text
Camera
 ↓
Image
 ↓
Detector
 ↓
Tracker
 ↓
Kalman Filter
 ↓
Controller
 ↓
Camera
```

This is the first major milestone.

The system must automatically follow the moving beacon.

Create a demo mode.

The UI must show:

```text
TARGET STATUS: LOCKED

Target X:
Target Y:

Predicted X:
Predicted Y:

Error X:
Error Y:

Pan:
Tilt:

FPS:
Latency:
```

Do not add AI until this closed-loop baseline works reliably.

---

# 17. PHASE 10 — DISTURBANCE ENGINE

Implement realistic disturbances.

At minimum:

### Camera vibration

Generate angular disturbance:

```text
theta_disturbance(t)
```

using combinations of:

* sinusoidal motion,
* random noise,
* configurable vibration frequency,
* configurable amplitude.

### Camera jitter

Add high-frequency angular noise.

### Sensor noise

Add:

* Gaussian noise,
* pixel noise,
* intensity noise.

### Target motion disturbance

Add:

* sudden acceleration,
* velocity changes,
* random perturbations.

### Atmospheric disturbance

Simulate effects such as:

* image blur,
* beacon intensity fluctuation,
* centroid displacement,
* scintillation-like intensity variation.

Every disturbance must be configurable.

---

# 18. DISTURBANCE CONTROL PANEL

The final application must allow the user to configure:

```text
Target Speed
Target Acceleration
Camera Vibration
Camera Jitter
Sensor Noise
Atmospheric Distortion
Beacon Brightness
Frame Rate
FOV
```

Use sliders or numeric controls.

Include preset scenarios:

```text
EASY
MODERATE
DIFFICULT
EXTREME
CUSTOM
```

---

# 19. PHASE 11 — TARGET LOSS

Implement realistic target loss.

The system must detect when:

* beacon leaves FOV,
* detection confidence becomes too low,
* detector fails repeatedly,
* image quality becomes poor.

Do not simply set a boolean variable manually.

Target loss should emerge naturally from the simulated environment.

When target loss occurs:

```text
LOCKED
 ↓
UNCERTAIN
 ↓
LOST
```

---

# 20. PHASE 12 — REACQUISITION

Implement autonomous target reacquisition.

When the target is lost:

1. Use last known state.
2. Predict target position.
3. Estimate likely search region.
4. Move camera toward predicted region.
5. If necessary, execute a search pattern.
6. Detect beacon again.
7. Reinitialize tracking.
8. Return to LOCKED state.

Search patterns may include:

* spiral,
* raster,
* sector scan,
* expanding scan.

Compare reacquisition methods.

Measure:

```text
reacquisition time
```

---

# 21. PHASE 13 — AI MOTION PREDICTION

Only after the baseline system works, introduce ML.

The AI model must have a genuine purpose.

Its purpose is:

> Predict the future target position from historical observations/state estimates.

Input:

```text
recent trajectory history
+
velocity
+
acceleration
+
optional camera state
```

Output:

```text
future target position
```

Start with a lightweight architecture.

Compare possible models:

* MLP,
* LSTM,
* GRU.

Do not assume that a complicated model is automatically better.

Choose the model based on:

* prediction error,
* inference latency,
* robustness,
* training complexity.

---

# 22. AI DATASET GENERATION

Do NOT depend on manually collected data.

Generate training data from the simulator.

Run many trajectories with randomized:

* initial positions,
* velocity,
* acceleration,
* disturbances,
* noise,
* camera motion.

Store:

```text
timestamp
x
y
vx
vy
ax
ay
camera_pan
camera_tilt
```

Generate sequences for supervised learning.

Split into:

```text
train
validation
test
```

Ensure there is no data leakage.

---

# 23. AI TRAINING PIPELINE

Implement:

```text
simulation
 ↓
dataset generation
 ↓
preprocessing
 ↓
training
 ↓
validation
 ↓
testing
 ↓
model export
 ↓
inference
```

Provide:

* training script,
* validation script,
* test script,
* checkpoint saving,
* model loading,
* inference code.

Track:

* training loss,
* validation loss,
* prediction error,
* inference time.

---

# 24. AI-ASSISTED CONTROL

Integrate the predictor into the control loop.

Baseline:

```text
current target position
        ↓
controller
        ↓
camera
```

Predictive:

```text
current observations
        ↓
state estimation
        ↓
AI prediction
        ↓
future target position
        ↓
controller
        ↓
camera
```

The AI must not completely replace the safety/control system.

Use AI as a predictive component.

The controller must retain stability constraints.

---

# 25. BASELINE VS AI COMPARISON

This is extremely important.

Implement at least:

### System A

Detection + PID

### System B

Detection + Kalman + PID

### System C

Detection + Kalman + AI prediction + PID

Run identical scenarios against all systems.

Compare:

* average tracking error,
* RMS tracking error,
* maximum error,
* acquisition time,
* reacquisition time,
* lock-retention percentage,
* FPS,
* inference latency,
* control latency.

Generate graphs.

---

# 26. PERFORMANCE METRICS

Implement the following metrics.

### Acquisition time

Time from simulation start until target becomes reliably locked.

### Tracking error

Calculate:

```text
e = sqrt((x_target - x_center)^2 +
         (y_target - y_center)^2)
```

Also provide angular error if available.

### RMS error

Calculate RMS tracking error.

### Maximum error

Maximum observed tracking error.

### Lock retention

Percentage of simulation time where the target remains locked.

### Target-loss count

Number of target-loss events.

### Reacquisition time

Average time required to recover target lock.

### FPS

Actual processed frames per second.

### Processing latency

Average time required for one processing cycle.

### Prediction error

Distance between predicted and actual target position.

---

# 27. EXPERIMENT ENGINE

Create an experiment manager.

Allow repeatable experiments using configuration files.

Example:

```yaml
experiment:
  duration: 60

target:
  speed: 5
  trajectory: sinusoidal

camera:
  fps: 60
  fov: 30

disturbance:
  vibration: 0.5
  noise: 0.2
  turbulence: 0.3

controller:
  type: pid

prediction:
  enabled: true
```

Every experiment must generate a unique result directory.

Example:

```text
experiments/
└── 2026-09-10_21-30-00/
    ├── config.yaml
    ├── metrics.json
    ├── tracking.csv
    ├── trajectory.png
    ├── error.png
    └── summary.json
```

---

# 28. REAL-TIME DASHBOARD

Create a professional dashboard.

The UI should contain:

## Main simulation view

Show:

* camera frame,
* beacon,
* image center,
* detection bounding/marker,
* predicted location,
* trajectory.

## System status

Show:

```text
PAT STATUS
LOCKED / LOST / SEARCHING / REACQUIRING
```

## Metrics

Show:

```text
FPS
Latency
Tracking Error
RMS Error
Prediction Error
Lock Rate
```

## Camera state

Show:

```text
Pan
Tilt
Pan Rate
Tilt Rate
FOV
```

## Controls

Allow:

```text
Start
Pause
Stop
Reset
Scenario
Disturbance controls
AI ON/OFF
Controller selection
```

---

# 29. VISUALIZATION

Include real-time graphs for:

1. Target position.
2. Estimated position.
3. Predicted position.
4. Camera center.
5. Tracking error.
6. Pan angle.
7. Tilt angle.
8. Target velocity.
9. FPS.
10. Processing latency.

Use rolling graphs so that the UI remains responsive.

---

# 30. SAFETY AND STABILITY

The controller must contain:

* angular limits,
* rate limits,
* acceleration limits,
* output saturation,
* invalid measurement handling,
* prediction bounds.

Never allow NaN or invalid model outputs to destabilize the simulation.

If the AI prediction becomes invalid:

```text
AI prediction invalid
        ↓
fallback
        ↓
Kalman estimate
        ↓
normal controller
```

The system must remain operational even if the AI component fails.

---

# 31. SOFTWARE QUALITY

Use:

* type hints,
* docstrings,
* clear class interfaces,
* meaningful variable names,
* modular files,
* exception handling,
* structured logging,
* configuration files,
* unit tests.

Avoid:

* giant monolithic scripts,
* duplicated code,
* hard-coded constants,
* unnecessary dependencies,
* hidden global state.

---

# 32. TESTING

Create automated tests for:

### Simulation

* target movement,
* trajectory generation,
* coordinate conversion.

### Camera

* projection,
* FOV calculation,
* orientation.

### Detection

* beacon detection,
* noisy image detection.

### Tracking

* trajectory association,
* loss handling.

### Kalman filter

* prediction,
* correction,
* convergence.

### Controller

* error response,
* saturation,
* stability.

### AI

* model loading,
* inference,
* invalid input handling.

### Integration

Run a complete simulation.

---

# 33. FAILURE TESTING

Deliberately test:

* extreme vibration,
* high noise,
* sudden target acceleration,
* target leaving FOV,
* temporary detection failure,
* multiple missed frames,
* AI model failure,
* very low beacon intensity,
* high processing latency.

The system should degrade gracefully.

---

# 34. CONFIGURATION

Do not hard-code important parameters.

All important parameters must be configurable.

Examples:

```text
camera resolution
camera FPS
horizontal FOV
vertical FOV
target speed
target trajectory
noise level
vibration amplitude
turbulence level
Kalman Q
Kalman R
PID gains
prediction horizon
target-loss threshold
reacquisition timeout
```

---

# 35. REPRODUCIBILITY

Every experiment must support random seeds.

Example:

```text
seed = 42
```

If the same configuration and seed are used, the experiment should produce approximately reproducible results.

Store the complete configuration with every experiment.

---

# 36. OPTIMIZATION

Only optimize after correctness is established.

Measure bottlenecks.

Optimize:

* image processing,
* detector,
* tracking,
* prediction,
* rendering,
* logging.

Avoid premature optimization.

If AI inference is a bottleneck:

```text
PyTorch
 ↓
ONNX
 ↓
ONNX Runtime
```

Compare inference latency.

---

# 37. OPTIONAL 3D UPGRADE

Do NOT make 3D rendering a prerequisite for the core system.

First create a mathematically correct 2D/image-plane simulation.

After the complete system works, optionally introduce:

* 3D environment,
* 3D beacon,
* camera pose,
* realistic perspective,
* visualization.

If a 3D engine is used, keep the control and perception logic independent from the renderer.

---

# 38. FINAL DEMONSTRATION MODES

Create predefined demonstrations.

### DEMO 1 — Basic tracking

Moving target, no disturbances.

### DEMO 2 — Noisy tracking

Sensor noise enabled.

### DEMO 3 — Vibration

Camera vibration enabled.

### DEMO 4 — Difficult tracking

Noise + vibration + target acceleration.

### DEMO 5 — Target loss

Target leaves FOV and must be reacquired.

### DEMO 6 — AI comparison

AI OFF vs AI ON.

### DEMO 7 — Extreme scenario

Maximum realistic disturbance.

---

# 39. FINAL PROJECT OUTPUT

The completed repository must contain:

```text
Source code
Configuration
Trained model
Model inference code
Dataset generator
Experiment runner
Dashboard
Tests
Documentation
Performance logs
Graphs
README
User manual
Technical report
```

---

# 40. README REQUIREMENTS

Generate a professional README containing:

1. Project title.
2. Problem statement.
3. System architecture.
4. Features.
5. Technology stack.
6. Installation.
7. Configuration.
8. How to run.
9. How to train AI.
10. How to run experiments.
11. Performance metrics.
12. Screenshots/placeholders.
13. Project structure.
14. Troubleshooting.
15. Future improvements.

---

# 41. USER MANUAL

Create:

```text
docs/user_manual.md
```

Explain:

* installation,
* launching the application,
* controls,
* scenarios,
* disturbance configuration,
* AI mode,
* experiment execution,
* interpreting metrics,
* exporting results.

Assume the reader is not the developer.

---

# 42. TECHNICAL REPORT

Create a technical report structure containing:

## Abstract

## Introduction

## Problem Definition

## FSOC and PAT Background

## System Requirements

## System Architecture

## Simulation Model

## Virtual Camera Model

## Beacon Detection

## Tracking

## State Estimation

## Control System

## Disturbance Modelling

## Target Reacquisition

## AI Prediction

## AI Training

## Experimental Methodology

## Performance Metrics

## Baseline Results

## AI-Assisted Results

## Discussion

## Limitations

## Future Work

## Conclusion

## References

Do not invent experimental results.

Use placeholders until actual experiments have been executed.

---

# 43. ENGINEERING VALIDATION

At the end, create a final validation checklist.

Example:

```text
[ ] Simulation runs
[ ] Beacon moves correctly
[ ] Camera projection works
[ ] Beacon detection works
[ ] Tracking works
[ ] Kalman filter works
[ ] PID controller works
[ ] Closed-loop PAT works
[ ] Disturbances work
[ ] Target-loss detection works
[ ] Reacquisition works
[ ] Dataset generation works
[ ] AI model trains
[ ] AI prediction works
[ ] AI fallback works
[ ] Dashboard works
[ ] Metrics work
[ ] Experiment logging works
[ ] Tests pass
[ ] Documentation complete
```

---

# 44. IMPORTANT RULE ABOUT RESULTS

Never fabricate performance results.

If an experiment has not been run, say:

```text
NOT YET MEASURED
```

Do not invent:

* FPS,
* tracking accuracy,
* acquisition time,
* lock rate,
* AI improvement,
* prediction accuracy.

All numerical results must come from actual execution.

---

# 45. IMPORTANT RULE ABOUT EXTERNAL RESOURCES

You may use existing:

* open-source libraries,
* research papers,
* algorithms,
* simulation concepts,
* documentation,
* reference implementations.

However:

1. Do not blindly copy entire projects.
2. Explain what is being reused.
3. Keep our implementation modular.
4. Respect licenses.
5. Do not falsely claim third-party code as original.
6. Prefer official documentation and reputable technical sources.

---

# 46. IMPORTANT RULE ABOUT CODE

Every code block must be complete enough to run in the current project state.

Never give me code containing:

```text
# TODO
# implement this
# remaining code
# etc.
```

unless the purpose is explicitly to leave that component for a later phase.

When modifying an existing file, clearly state:

```text
FILE:
src/fsoc_pat/...
```

Then provide the complete updated file when practical.

Never silently change APIs between phases.

---

# 47. DEBUGGING PROTOCOL

When I provide an error:

First identify:

```text
ERROR
 ↓
ROOT CAUSE
 ↓
AFFECTED COMPONENT
 ↓
FIX
 ↓
UPDATED CODE
 ↓
TEST
```

Do not rewrite the entire project unnecessarily.

Do not introduce unrelated changes while fixing an error.

---

# 48. RESPONSE FORMAT FOR EVERY DEVELOPMENT PHASE

For every phase, respond using this structure:

## PHASE X — NAME

### Objective

Explain what we are building.

### Architecture

Show the relevant data flow.

### Files

List files to create/modify.

### Installation

Give exact commands.

### Implementation

Provide the code.

### Run

Give the exact command.

### Expected Result

Explain exactly what I should see.

### Validation

Give a checklist.

### Common Errors

List likely errors and fixes.

### Completion Criteria

Define what must work before moving forward.

Then STOP and wait for my confirmation.

---

# 49. DO NOT OVERLOAD ME

I am building this project incrementally.

Do not give me the entire project's code in the first response.

Start with **PHASE 0** only.

After I confirm Phase 0 works, proceed to Phase 1.

After Phase 1 works, proceed to Phase 2.

Continue this process until the complete project is finished.

---

# 50. FIRST RESPONSE

Your FIRST response to this prompt must NOT contain the entire project.

It must contain ONLY:

1. A concise understanding of SIH26169.
2. The proposed architecture.
3. Technology-stack confirmation.
4. A project-development roadmap.
5. PHASE 0 environment setup.
6. Exact commands to create the project.
7. A minimal environment verification script.
8. The exact test commands.
9. Expected output.
10. Phase-0 completion checklist.

Then stop.

Do not implement Phase 1 until I confirm Phase 0 works.

---

# 51. FINAL OBJECTIVE

At the end of the entire development process, I should have a command such as:

```bash
python -m fsoc_pat
```

that launches the complete application.

The application should provide:

```text
┌────────────────────────────────────────────────────────┐
│                 FSOC AI-PAT SYSTEM                     │
├──────────────────────────────┬─────────────────────────┤
│                              │                         │
│      VIRTUAL CAMERA          │      SYSTEM STATUS      │
│                              │                         │
│          ● target            │  PAT: LOCKED            │
│                              │  FPS: XX                │
│             +                │  Error: XX              │
│        camera center         │  RMS: XX                │
│                              │  Lock: XX%              │
│                              │                         │
├──────────────────────────────┼─────────────────────────┤
│                              │                         │
│       TRAJECTORY             │      CONTROLS           │
│                              │                         │
│      target                  │  Start                  │
│      estimate               │  Pause                  │
│      prediction             │  Reset                  │
│                              │  AI ON/OFF              │
│                              │  Scenario               │
│                              │  Disturbances           │
│                              │                         │
└──────────────────────────────┴─────────────────────────┘
```

The final system must demonstrate:

```text
AUTONOMOUS TARGET DETECTION
          +
TARGET TRACKING
          +
STATE ESTIMATION
          +
AI MOTION PREDICTION
          +
CLOSED-LOOP CAMERA CONTROL
          +
DISTURBANCE HANDLING
          +
TARGET REACQUISITION
          +
QUANTITATIVE PERFORMANCE EVALUATION
```

The goal is not to create a visually impressive demo that only works under ideal conditions.

The goal is to create a **technically defensible, measurable, modular, reproducible and competition-ready SIH26169 implementation**.

Begin now with **PHASE 0 — ENVIRONMENT SETUP**.
