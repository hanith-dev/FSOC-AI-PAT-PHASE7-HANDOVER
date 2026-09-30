import pytest
import numpy as np
import math
from app.tracking.controller import (
    GimbalController,
    GimbalControllerConfig,
    ControlMode,
    SingleAxisController
)


def test_zero_error_produces_zero_output():
    """C01: zero error produces zero command and inactive saturation."""
    ctrl = GimbalController(mode=ControlMode.PREDICTIVE_SMOOTH)
    pan_cmd, tilt_cmd, telem = ctrl.compute_control(
        kf_state=np.array([320.0, 240.0, 0.0, 0.0, 0.0, 0.0]),
        camera_cx=320.0,
        camera_cy=240.0,
        camera_fx=3661.72,
        camera_fy=3661.72,
        dt=0.02,
        pat_state="TRACK"
    )
    assert abs(pan_cmd) < 1e-6
    assert abs(tilt_cmd) < 1e-6
    assert telem["deadband_active"] is True
    assert telem["rate_saturated"] is False
    assert telem["acceleration_saturated"] is False
    assert telem["jerk_saturated"] is False


def test_small_constant_error_convergence():
    """C02: Small constant error engages integral without windup."""
    cfg = GimbalControllerConfig(kp=1.5, ki=0.5, kd=0.1, deadband_deg=0.001)
    axis = SingleAxisController(cfg)
    err = 0.05  # 0.05 deg
    dt = 0.02
    for _ in range(50):
        rate, telem = axis.update(err, dt, mode=ControlMode.FILTERED_PID)
    assert rate > 0.0
    assert axis.integral > 0.0
    assert axis.integral <= cfg.integral_limit


def test_step_input_jerk_and_acceleration_limiting():
    """C03: Step input does not violate acceleration or jerk limits in C3."""
    cfg = GimbalControllerConfig(
        max_rate_deg_s=60.0,
        max_accel_deg_s2=250.0,
        max_jerk_deg_s3=2500.0
    )
    axis = SingleAxisController(cfg)
    dt = 0.02
    rates = []
    # Step error: jump to 10 deg
    for _ in range(30):
        rate, telem = axis.update(10.0, dt, mode=ControlMode.PREDICTIVE_SMOOTH)
        rates.append(rate)

    rates = np.array(rates)
    accels = np.diff(rates) / dt
    jerks = np.diff(accels) / dt

    # Check that acceleration never exceeds max_accel (+ tolerance for discrete dt)
    assert np.all(np.abs(accels) <= cfg.max_accel_deg_s2 + 1e-3)
    # Check that jerk never exceeds max_jerk
    assert np.all(np.abs(jerks) <= cfg.max_jerk_deg_s3 + 1e-3)
    # Check that rate never exceeds max_rate
    assert np.all(np.abs(rates) <= cfg.max_rate_deg_s + 1e-3)


def test_constant_velocity_target_feedforward():
    """C04 & C05: Moving target engages velocity feed-forward reducing tracking lag."""
    ctrl_c0 = GimbalController(mode=ControlMode.LEGACY_PID)
    ctrl_c3 = GimbalController(mode=ControlMode.PREDICTIVE_SMOOTH)

    # Target moving horizontally at 100 px/s at center
    kf_state = np.array([320.0, 240.0, 100.0, 0.0, 0.0, 0.0])
    dt = 0.02
    fx = 3661.72
    fy = 3661.72

    # In C0, feed-forward is 0; only error drives command
    p0, t0, telem0 = ctrl_c0.compute_control(kf_state, 320.0, 240.0, fx, fy, dt, pat_state="TRACK")
    assert telem0["feedforward_pan_rate"] == 0.0

    # In C3, feed-forward is active: (vx/fx) / (1 + ((x-cx)/fx)^2) in degrees
    p3, t3, telem3 = ctrl_c3.compute_control(kf_state, 320.0, 240.0, fx, fy, dt, pat_state="TRACK")
    expected_ff = math.degrees((100.0 / fx) / (1.0 + ((320.0 - 320.0) / fx) ** 2))
    assert pytest.approx(telem3["feedforward_pan_rate"], abs=1e-3) == expected_ff
    assert p3 > 0.0


def test_direction_reversal_smoothness():
    """C08: Target direction reversal does not cause unbounded jerk."""
    cfg = GimbalControllerConfig(max_accel_deg_s2=250.0, max_jerk_deg_s3=2500.0)
    axis = SingleAxisController(cfg)
    dt = 0.02

    # Positive command
    for _ in range(20):
        axis.update(5.0, dt, mode=ControlMode.PREDICTIVE_SMOOTH)

    rates = []
    # Abrupt sign reversal
    for _ in range(30):
        rate, telem = axis.update(-5.0, dt, mode=ControlMode.PREDICTIVE_SMOOTH)
        rates.append(rate)

    rates = np.array(rates)
    accels = np.diff(rates) / dt
    jerks = np.diff(accels) / dt

    assert np.all(np.abs(accels) <= cfg.max_accel_deg_s2 + 1e-3)
    assert np.all(np.abs(jerks) <= cfg.max_jerk_deg_s3 + 1e-3)


def test_derivative_kick_protection_on_lifecycle_handoff():
    """C14: Returning from COAST to TRACK clears derivative memory to prevent kick."""
    ctrl = GimbalController(mode=ControlMode.PREDICTIVE_SMOOTH)
    dt = 0.02
    fx = fy = 3661.72

    # Tracking normally
    kf_state = np.array([320.0, 240.0, 0.0, 0.0, 0.0, 0.0])
    ctrl.compute_control(kf_state, 320.0, 240.0, fx, fy, dt, pat_state="TRACK")

    # Enter COAST for 5 frames
    ctrl.compute_control(kf_state, 320.0, 240.0, fx, fy, dt, pat_state="COAST")

    # Reappear at offset: 350.0 px (error jumped 30 px = ~0.47 deg)
    kf_resumed = np.array([350.0, 240.0, 0.0, 0.0, 0.0, 0.0])
    p_cmd, t_cmd, telem = ctrl.compute_control(kf_resumed, 320.0, 240.0, fx, fy, dt, pat_state="TRACK")

    # Should flag reset event and avoid massive derivative spike
    assert telem["controller_state_reset_event"] is True
    # The acceleration and jerk should remain bounded
    assert abs(telem["pan_acceleration"]) <= ctrl.config.max_accel_deg_s2 + 1e-3
    assert abs(telem["pan_jerk"]) <= ctrl.config.max_jerk_deg_s3 + 1e-3


def test_anti_windup_under_large_saturation():
    """C11: Large saturation clamps integral accumulation and respects all actuator limits."""
    cfg = GimbalControllerConfig(
        integral_limit=15.0,
        max_rate_deg_s=60.0,
        max_accel_deg_s2=250.0,
        max_jerk_deg_s3=2500.0
    )
    axis = SingleAxisController(cfg)
    dt = 0.02

    for _ in range(100):
        rate, telem = axis.update(50.0, dt, mode=ControlMode.PREDICTIVE_SMOOTH)
        assert not math.isnan(rate) and not math.isinf(rate)
        assert abs(telem["limited_rate"]) <= cfg.max_rate_deg_s + 1e-4
        assert abs(telem["accel"]) <= cfg.max_accel_deg_s2 + 1e-4
        assert abs(telem["jerk"]) <= cfg.max_jerk_deg_s3 + 1e-4

    assert axis.integral == cfg.integral_limit
    assert axis.rate_saturated is True


def test_search_and_lost_deferral_to_search_pattern():
    """C13 & C15: In SEARCH / LOST state, search offsets drive pointing."""
    ctrl = GimbalController(mode=ControlMode.PREDICTIVE_SMOOTH)
    dt = 0.02
    fx = fy = 3661.72

    # In SEARCH with 1.0 deg pan offset from spiral generator
    p_cmd, t_cmd, telem = ctrl.compute_control(
        kf_state=None,
        camera_cx=320.0,
        camera_cy=240.0,
        camera_fx=fx,
        camera_fy=fy,
        dt=dt,
        pat_state="SEARCH",
        search_pan_offset_deg=1.0,
        search_tilt_offset_deg=0.5
    )
    assert p_cmd > 0.0
    assert t_cmd > 0.0


def test_prediction_horizon_bounding():
    """Ensure latency horizon is clamped strictly to Phase 4.5 safe window (20ms - 200ms)."""
    ctrl = GimbalController(mode=ControlMode.PREDICTIVE_PID)
    fx = fy = 3661.72
    kf_state = np.array([320.0, 240.0, 50.0, 0.0, 0.0, 0.0])

    # Injected 500 ms delay -> must clamp to 200 ms
    _, _, telem_high = ctrl.compute_control(
        kf_state, 320.0, 240.0, fx, fy, 0.02, pat_state="TRACK", injected_latency_s=0.50
    )
    assert telem_high["prediction_horizon_ms"] == 200.0

    # Injected 5 ms delay -> must clamp to 20 ms
    _, _, telem_low = ctrl.compute_control(
        kf_state, 320.0, 240.0, fx, fy, 0.02, pat_state="TRACK", injected_latency_s=0.005
    )
    assert telem_low["prediction_horizon_ms"] == 20.0


def test_candidate_chase_attenuation():
    """In CANDIDATE state, target chasing is attenuated to avoid premature jerks."""
    ctrl = GimbalController(mode=ControlMode.FILTERED_PID)
    fx = fy = 3661.72
    kf_state = np.array([350.0, 240.0, 0.0, 0.0, 0.0, 0.0])

    # Unconfirmed candidate
    p_cand, _, telem_cand = ctrl.compute_control(
        kf_state, 320.0, 240.0, fx, fy, 0.02, pat_state="CANDIDATE"
    )
    ctrl.reset()
    # Confirmed track
    p_track, _, telem_track = ctrl.compute_control(
        kf_state, 320.0, 240.0, fx, fy, 0.02, pat_state="TRACK"
    )
    assert abs(p_cand) < abs(p_track)


# ======================================================================
# Phase 6.5 Verification & Audit Test Suite
# ======================================================================

def test_exact_actuator_limits_and_post_limiter_telemetry():
    """Verify rate, accel, and jerk limits and exact post-limiter telemetry consistency."""
    cfg = GimbalControllerConfig(
        max_rate_deg_s=60.0,
        max_accel_deg_s2=250.0,
        max_jerk_deg_s3=2500.0
    )
    axis = SingleAxisController(cfg)
    dt = 0.02
    # Apply abrupt step command
    rate, telem = axis.update(50.0, dt, mode=ControlMode.PREDICTIVE_SMOOTH)
    
    assert abs(rate) <= cfg.max_rate_deg_s + 1e-9
    assert abs(telem["accel"]) <= cfg.max_accel_deg_s2 + 1e-9
    assert abs(telem["jerk"]) <= cfg.max_jerk_deg_s3 + 1e-9
    assert telem["limited_rate"] == rate
    assert telem["raw_cmd"] > rate  # Pre-limiter vs post-limiter distinction


def test_analytical_feedforward_projection_derivative():
    """Verify exact analytical derivative of pinhole camera projection vs finite difference."""
    cx = 320.0
    fx = 3661.72
    x = 550.0
    vx = -120.0
    dt = 1e-5
    
    # Finite difference
    theta0 = math.degrees(math.atan((x - cx) / fx))
    theta1 = math.degrees(math.atan((x + vx * dt - cx) / fx))
    num_dtheta = (theta1 - theta0) / dt
    
    # Analytical
    ana_dtheta = math.degrees((vx / fx) / (1.0 + ((x - cx) / fx) ** 2))
    
    assert pytest.approx(ana_dtheta, rel=1e-4) == num_dtheta


def test_no_prediction_feedforward_double_counting():
    """Verify position prediction is not doubled when velocity feedforward is active."""
    ctrl = GimbalController(mode=ControlMode.PREDICTIVE_PID)
    fx = fy = 3661.72
    # Target at center moving right
    kf_state = np.array([320.0, 240.0, 100.0, 0.0, 0.0, 0.0])
    
    pan_cmd, tilt_cmd, telem = ctrl.compute_control(
        kf_state, 320.0, 240.0, fx, fy, 0.02, pat_state="TRACK"
    )
    # Target is at center, so tracking error is driven by predicted offset
    # and feed-forward is reported as separate component
    assert telem["feedforward_pan_rate"] > 0.0
    assert telem["raw_command_pan_rate"] == pytest.approx(
        telem["pid_pan_component"] + telem["feedforward_pan_rate"], abs=1e-4
    )


def test_delay_buffer_frame_alignment_and_determinism():
    """Verify deterministic frame alignment across time steps."""
    ctrl1 = GimbalController(mode=ControlMode.FILTERED_PID)
    ctrl2 = GimbalController(mode=ControlMode.FILTERED_PID)
    fx = fy = 3661.72
    
    for step in range(50):
        err_x = 320.0 + 10.0 * math.sin(step * 0.1)
        kf = np.array([err_x, 240.0, 5.0, 0.0, 0.0, 0.0])
        p1, t1, _ = ctrl1.compute_control(kf, 320.0, 240.0, fx, fy, 0.02, pat_state="TRACK")
        p2, t2, _ = ctrl2.compute_control(kf, 320.0, 240.0, fx, fy, 0.02, pat_state="TRACK")
        assert p1 == p2
        assert t1 == t2


def test_s15_safety_regression_zero_false_locks():
    """Verify S15 combined stress with filtered_pid preserves 0 false locks."""
    from app.benchmark.runner import BenchmarkEngine
    from app.benchmark.scenarios import SCENARIO_REGISTRY
    
    eng = BenchmarkEngine(fsm_mode="new_pat_fsm", controller_mode="filtered_pid")
    res = eng.run_scenario(SCENARIO_REGISTRY["S15_combined_stress"], seed=42, duration=5.0, save_plots=False)
    assert res.metrics["association"]["false_lock_duration_s"] == 0.0

