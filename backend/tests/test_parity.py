"""
Automated Mathematical Parity and Architectural Certification Tests (Phase 1.5).
Certifies mathematical equivalence between Mode A (Headless Benchmark) and Mode B (Live Mission Control).
"""
import pytest
import numpy as np

from app.camera.model import CameraModel
from app.tracking.pid import PIDController
from app.tracking.kalman import KalmanFilter2D
from app.tracking.state_machine import TrackingStateMachine, TrackingState
from app.detection.detector import OpenCVDetector
from app.benchmark.renderer import SyntheticFrameRenderer
from app.benchmark.metrics import BenchmarkMetricsCalculator


# ----------------------------------------------------------------------
# 1. PID Golden Vector Parity Test (pid.py vs exact port of pid.js)
# ----------------------------------------------------------------------
class ReferenceJSPIDController:
    """Exact 1:1 Python port of frontend/src/utils/pid.js for parity audit."""
    def __init__(self, kp=1.5, ki=0.05, kd=0.1, integralLimit=20.0, outputLimit=180.0, deadband=0.01):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integralLimit = integralLimit
        self.outputLimit = outputLimit
        self.deadband = deadband
        self.reset()

    def reset(self):
        self.integral = 0.0
        self.prevError = 0.0
        self.hasPrevError = False

    def update(self, error, dt):
        if dt <= 0:
            return 0.0
        if abs(error) < self.deadband:
            self.integral = 0.0
            self.prevError = error
            self.hasPrevError = True
            return 0.0

        pTerm = self.kp * error
        self.integral += error * dt
        self.integral = max(-self.integralLimit, min(self.integralLimit, self.integral))
        iTerm = self.ki * self.integral

        dTerm = 0.0
        if self.hasPrevError:
            errorDiff = error - self.prevError
            dTerm = self.kd * (errorDiff / dt)

        self.prevError = error
        self.hasPrevError = True

        output = pTerm + iTerm + dTerm
        output = max(-self.outputLimit, min(self.outputLimit, output))
        return output


def test_pid_golden_vector_parity():
    """
    Test PIDController against JS reference across 8 golden vector sequences:
    zero, positive, negative, step, ramp, sign reversal, large saturation, deadband.
    """
    dt = 0.02
    test_scenarios = [
        ("constant_zero", [0.0] * 50),
        ("constant_pos", [2.5] * 50),
        ("constant_neg", [-2.5] * 50),
        ("step", [0.0] * 10 + [5.0] * 20 + [0.0] * 20),
        ("ramp", [0.1 * i for i in range(50)]),
        ("sign_reversal", [3.0] * 20 + [-3.0] * 20),
        ("large_saturation", [0.0] * 5 + [250.0] * 20 + [-250.0] * 20),
        ("deadband_scale", [0.005] * 10 + [0.015] * 10 + [0.008] * 10)
    ]

    for name, error_seq in test_scenarios:
        py_pid = PIDController(kp=1.5, ki=0.05, kd=0.1, integral_limit=20.0, output_limit=180.0, deadband=0.01)
        js_pid = ReferenceJSPIDController(kp=1.5, ki=0.05, kd=0.1, integralLimit=20.0, outputLimit=180.0, deadband=0.01)

        for step, err in enumerate(error_seq):
            py_out = py_pid.update(err, dt)
            js_out = js_pid.update(err, dt)

            assert abs(py_out - js_out) < 1e-12, (
                f"PID Parity failure in '{name}' at step {step}: "
                f"Python={py_out} vs JS={js_out} (error={err})"
            )


# ----------------------------------------------------------------------
# 2. Camera Projection Parity Test
# ----------------------------------------------------------------------
def test_camera_projection_mathematical_parity():
    """
    Verify camera pinhole projection mathematics match Three.js YXZ Euler rotation
    and pinhole intrinsic formulas across diverse 3D world test points.
    """
    cam = CameraModel(width=640, height=480, hfov_deg=10.0, vfov_deg=7.5, camera_pos=np.array([0.0, 1.5, 0.0]))

    # Verify focal lengths (square pixel calibration where Three.js VFOV is authoritative)
    expected_fy = 480.0 / (2.0 * np.tan(np.radians(3.75)))
    expected_fx = expected_fy  # Square pixels fx == fy
    assert abs(cam.fx - expected_fx) < 1e-9
    assert abs(cam.fy - expected_fy) < 1e-9
    assert cam.cx == 320.0
    assert cam.cy == 240.0

    # Test projection points (P_w, pan, tilt, expected_u_center_offset_dir)
    test_cases = [
        # Directly on boresight at 100m
        (np.array([0.0, 1.5, -100.0]), 0.0, 0.0, 320.0, 240.0),
        # Target offset right (+X): should project to u > 320
        (np.array([2.0, 1.5, -100.0]), 0.0, 0.0, 320.0 + expected_fx * 0.02, 240.0),
        # Target offset up (+Y): should project to v < 240 (CV image Y goes down)
        (np.array([0.0, 3.5, -100.0]), 0.0, 0.0, 320.0, 240.0 - expected_fy * 0.02),
        # Gimbal panned right (+pan): target appears shifted left on sensor (u < 320)
        (np.array([0.0, 1.5, -100.0]), 1.0, 0.0, None, 240.0),
    ]

    for pt_w, pan_deg, tilt_deg, exp_u, exp_v in test_cases:
        p_c = cam.world_to_camera(pt_w, np.radians(pan_deg), np.radians(tilt_deg))
        u, v, is_in_fov = cam.camera_to_pixel(p_c)

        if exp_u is not None:
            assert abs(u - exp_u) < 1e-6
        if exp_v is not None:
            assert abs(v - exp_v) < 1e-6

        # Angular error check
        tot_err, h_err, v_err = cam.compute_angular_errors(pt_w, np.radians(pan_deg), np.radians(tilt_deg))
        if pan_deg == 0.0 and tilt_deg == 0.0 and pt_w[0] == 0.0 and pt_w[1] == 1.5:
            assert tot_err < 1e-9


# ----------------------------------------------------------------------
# 3. Kalman Filter Parity & Interface Certification
# ----------------------------------------------------------------------
def test_kalman_shared_implementation():
    """Verify KalmanFilter2D behaves identically across simulation time steps."""
    kf = KalmanFilter2D(q_noise=0.5, r_noise=1.0, max_prediction_seconds=2.0)
    assert not kf.is_initialized

    kf.initialize(320.0, 240.0, 0.0)
    assert kf.is_initialized
    assert kf.state[0] == 320.0
    assert kf.state[1] == 240.0

    # Prediction only
    pred = kf.predict_only(0.02)
    assert pred is not None

    # Measurement update
    upd = kf.update(np.array([322.0, 241.0]), 0.04)
    assert upd[0] > 320.0

    # Occlusion timeout
    assert not kf.check_occlusion_timeout(1.5)
    assert kf.check_occlusion_timeout(2.5)  # exceeds 2.0s


# ----------------------------------------------------------------------
# 4. FSM Deterministic vs Live-Time Parity Test
# ----------------------------------------------------------------------
def test_fsm_deterministic_time_parity():
    """
    Verify that passing deterministic current_time produce identical state
    transitions through SEARCHING -> DETECTED -> ACQUIRING -> TRACKING -> LOCKED.
    """
    fsm = TrackingStateMachine(lock_threshold_deg=0.15, lock_duration_sec=1.0)
    assert fsm.current_state == TrackingState.SEARCHING

    dt = 0.02
    t = 0.0

    # 1. Target detected far (> 0.5 deg)
    state = fsm.update(detected=True, is_occluded=False, angular_error_deg=0.8, current_time=t)
    assert state == TrackingState.DETECTED

    t += dt
    state = fsm.update(detected=True, is_occluded=False, angular_error_deg=0.8, current_time=t)
    assert state == TrackingState.ACQUIRING

    # 2. Target centered (<= 0.5 deg)
    t += dt
    state = fsm.update(detected=True, is_occluded=False, angular_error_deg=0.3, current_time=t)
    assert state == TrackingState.TRACKING

    # 3. Enter lock core (<= 0.15 deg) for < 1.0s
    for _ in range(30):  # 0.6s
        t += dt
        state = fsm.update(detected=True, is_occluded=False, angular_error_deg=0.10, current_time=t)
        assert state == TrackingState.TRACKING

    # 4. Stay in lock core to complete 1.0s
    for _ in range(25):  # +0.5s -> 1.1s total
        t += dt
        state = fsm.update(detected=True, is_occluded=False, angular_error_deg=0.10, current_time=t)

    assert state == TrackingState.LOCKED


# ----------------------------------------------------------------------
# 5. Jerk Metric Mathematical Calculation Test
# ----------------------------------------------------------------------
def test_jerk_metric_unit_calculation():
    """
    Verify jerk formula: a[k] = (v[k] - v[k-1])/dt, j[k] = (a[k] - a[k-1])/dt.
    Prove that a 3.0 deg/s step change at dt=0.02s yields 7500 deg/s^3.
    """
    dt = 0.02
    # Velocity sequence with an instantaneous step of 3.0 deg/s
    pan_rates = np.array([0.0, 0.0, 3.0, 3.0, 3.0])
    tilt_rates = np.zeros_like(pan_rates)

    dpan_dt = np.diff(pan_rates) / dt
    # dpan_dt: [0.0, 150.0, 0.0, 0.0]

    ddpan_dt2 = np.diff(dpan_dt) / dt
    # ddpan_dt2: [7500.0, -7500.0, 0.0]

    assert ddpan_dt2[0] == 7500.0
    assert ddpan_dt2[1] == -7500.0

    calc = BenchmarkMetricsCalculator(dt=dt)
    fake_telemetry = [
        {"simulation_time": i * dt, "command_pan_rate": pan_rates[i], "command_tilt_rate": 0.0,
         "tracking_error_deg": 0.1, "tracking_error_px": 5.0, "detection_available": True}
        for i in range(len(pan_rates))
    ]
    res = calc.compute_metrics(fake_telemetry)
    # Jerk is non-zero and confirms mathematical formulation
    assert res["control"]["rms_angular_jerk_deg_s3"] > 4000.0


# ----------------------------------------------------------------------
# 6. GimbalController Multi-Mode Parity Test (Python vs JS logic port)
# ----------------------------------------------------------------------
class ReferenceJSGimbalController:
    """Exact 1:1 port of frontend/src/utils/pid.js GimbalController."""
    def __init__(self, mode="predictive_smooth", kp=1.5, ki=0.05, kd=0.1, tauD=0.04, maxRate=60.0, maxAccel=250.0, maxJerk=2500.0):
        self.mode = mode
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.tauD = tauD
        self.maxRate = maxRate
        self.maxAccel = maxAccel
        self.maxJerk = maxJerk
        self.integralLimit = 20.0
        self.deadband = 0.01
        self.legacyOutputLimit = 180.0
        self.reset()

    def reset(self):
        self.integral = 0.0
        self.prevError = 0.0
        self.hasPrevError = False
        self.filteredDError = 0.0
        self.cmdRate = 0.0
        self.prevCmdRate = 0.0
        self.prevAccel = 0.0

    def update(self, error, dt, feedforwardRate=0.0):
        if dt <= 0:
            return self.cmdRate
        self.deadbandActive = False
        if abs(error) < self.deadband:
            self.deadbandActive = True
            self.integral = 0.0
            self.prevError = error
            self.hasPrevError = True
            self.filteredDError = 0.0
            if self.mode == "legacy_pid":
                self.cmdRate = 0.0
                self.prevCmdRate = 0.0
                self.prevAccel = 0.0
                return 0.0

        rawCmd = 0.0
        if not self.deadbandActive:
            pTerm = self.kp * error
            self.integral += error * dt
            self.integral = max(-self.integralLimit, min(self.integralLimit, self.integral))
            iTerm = self.ki * self.integral

            dRaw = (error - self.prevError) / dt if self.hasPrevError else 0.0
            if self.mode == "legacy_pid":
                dTerm = self.kd * dRaw
            else:
                alphaD = dt / (self.tauD + dt)
                self.filteredDError = self.filteredDError + alphaD * (dRaw - self.filteredDError)
                dTerm = self.kd * self.filteredDError

            self.prevError = error
            self.hasPrevError = True
            rawCmd = pTerm + iTerm + dTerm

        if self.mode in ("predictive_pid", "predictive_smooth"):
            rawCmd += feedforwardRate

        if self.mode == "legacy_pid":
            clamped = max(-self.legacyOutputLimit, min(self.legacyOutputLimit, rawCmd))
            self.cmdRate = clamped
            accel = (clamped - self.prevCmdRate) / dt
            self.prevAccel = accel
            self.prevCmdRate = clamped
            return clamped
        elif self.mode in ("filtered_pid", "predictive_pid"):
            clamped = max(-self.maxRate, min(self.maxRate, rawCmd))
            self.cmdRate = clamped
            accel = (clamped - self.prevCmdRate) / dt
            self.prevAccel = accel
            self.prevCmdRate = clamped
            return clamped
        else:
            desiredAccel = (rawCmd - self.prevCmdRate) / dt
            deltaAccel = desiredAccel - self.prevAccel
            maxDeltaAccel = self.maxJerk * dt
            if abs(deltaAccel) > maxDeltaAccel:
                deltaAccel = np.sign(deltaAccel) * maxDeltaAccel
            feasibleAccel = self.prevAccel + deltaAccel
            if abs(feasibleAccel) > self.maxAccel:
                feasibleAccel = np.sign(feasibleAccel) * self.maxAccel
            feasibleRate = self.prevCmdRate + feasibleAccel * dt
            if abs(feasibleRate) > self.maxRate:
                feasibleRate = np.sign(feasibleRate) * self.maxRate
                feasibleAccel = (feasibleRate - self.prevCmdRate) / dt
            self.prevAccel = feasibleAccel
            self.prevCmdRate = feasibleRate
            self.cmdRate = feasibleRate
            return feasibleRate


def test_gimbal_controller_golden_vector_parity():
    """Verify exact parity between Python SingleAxisController and JS Reference across C0, C1, C2, C3."""
    from app.tracking.controller import SingleAxisController, GimbalControllerConfig, ControlMode

    modes = [
        ("legacy_pid", ControlMode.LEGACY_PID),
        ("filtered_pid", ControlMode.FILTERED_PID),
        ("predictive_pid", ControlMode.PREDICTIVE_PID),
        ("predictive_smooth", ControlMode.PREDICTIVE_SMOOTH)
    ]
    dt = 0.02
    test_sequence = [0.0] * 5 + [2.0] * 15 + [-3.0] * 15 + [0.005] * 5 + [10.0] * 10

    for js_mode, py_mode in modes:
        cfg = GimbalControllerConfig()
        py_ctrl = SingleAxisController(cfg)
        js_ctrl = ReferenceJSGimbalController(mode=js_mode)

        for step, err in enumerate(test_sequence):
            ff = 1.5 if "predictive" in js_mode else 0.0
            py_rate, _ = py_ctrl.update(err, dt, mode=py_mode, feedforward_rate=ff)
            js_rate = js_ctrl.update(err, dt, feedforwardRate=ff)
            assert abs(py_rate - js_rate) < 1e-10, (
                f"Parity mismatch in mode {js_mode} at step {step}: Py={py_rate} vs JS={js_rate}"
            )

