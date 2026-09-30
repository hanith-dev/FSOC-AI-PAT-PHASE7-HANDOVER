import pytest
import numpy as np
from app.tracking.kalman import KalmanFilter2D, QUALITY_INFLATION_FACTORS
from app.tracking.association import AssociationEngine, CandidateMeasurement


def test_canonical_state_ordering():
    """Verify canonical state ordering: [x, y, vx, vy, ax, ay]^T."""
    kf = KalmanFilter2D(mode="ca6_adaptive_r")
    assert kf.state_dim == 6
    kf.initialize(100.0, 200.0, timestamp=0.0)
    assert kf.state.shape == (6,)
    assert kf.state[0] == 100.0
    assert kf.state[1] == 200.0
    assert kf.state[2] == 0.0
    assert kf.state[3] == 0.0
    assert kf.state[4] == 0.0
    assert kf.state[5] == 0.0


def test_f_matrix_exact_values():
    """Verify exact algebraic values of F(dt) for CA6."""
    kf = KalmanFilter2D(mode="ca6_fixed")
    dt = 0.04
    F = kf._get_F(dt)
    assert F.shape == (6, 6)

    expected_F = np.array([
        [1.0, 0.0, dt,  0.0, 0.5 * dt**2, 0.0],
        [0.0, 1.0, 0.0, dt,  0.0, 0.5 * dt**2],
        [0.0, 0.0, 1.0, 0.0, dt,  0.0],
        [0.0, 0.0, 0.0, 1.0, 0.0, dt],
        [0.0, 0.0, 0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 0.0, 0.0, 1.0]
    ], dtype=np.float32)

    np.testing.assert_allclose(F, expected_F, atol=1e-6)


def test_q_symmetry_and_psd():
    """Verify continuous white jerk Q(dt) symmetry and positive semi-definite eigenvalues."""
    kf = KalmanFilter2D(mode="ca6_adaptive_r", q_jerk=10000.0)
    for dt in [0.01, 0.02, 0.05, 0.1]:
        Q = kf._get_Q(dt)
        assert Q.shape == (6, 6)
        # Symmetry
        np.testing.assert_allclose(Q, Q.T, atol=1e-5)
        # Positive semi-definite (all eigenvalues >= -1e-6)
        eigenvals = np.linalg.eigvalsh(Q)
        assert np.all(eigenvals >= -1e-6), f"Negative eigenvalue found in Q for dt={dt}: {eigenvals}"


def test_measurement_matrix_h():
    """Verify H matrix strictly observes positions [x, y] with zero velocity/acceleration leakage."""
    kf = KalmanFilter2D(mode="ca6_adaptive_r")
    assert kf.H.shape == (2, 6)
    expected_H = np.array([
        [1, 0, 0, 0, 0, 0],
        [0, 1, 0, 0, 0, 0]
    ], dtype=np.float32)
    np.testing.assert_allclose(kf.H, expected_H)


def test_adaptive_r_bounds_and_inflation():
    """Verify Phase 2 uncertainty clamping and quality flag inflation."""
    kf = KalmanFilter2D(mode="ca6_adaptive_r")

    # Clamping test: extremely small uncertainty 0.01 px should clamp to 0.2 px (variance 0.04)
    R_tiny = np.diag([0.0001, 0.0001])
    R_clamped = kf._compute_R(R_custom=R_tiny)
    assert R_clamped[0, 0] >= 0.04
    assert R_clamped[1, 1] >= 0.04

    # Quality flag inflation
    R_base = kf._compute_R(R_custom=np.diag([1.0, 1.0]))
    R_inflated = kf._compute_R(R_custom=np.diag([1.0, 1.0]), quality_flags=["TOUCHING_SENSOR_EDGE", "LOW_SNR"])
    # 1.0 * 2.0 * 1.5 = 3.0
    assert np.isclose(R_inflated[0, 0], R_base[0, 0] * 3.0)


def test_uncertainty_gain_monotonicity():
    """Verify: higher measurement uncertainty -> lower Kalman gain; lower uncertainty -> higher gain."""
    kf_high_r = KalmanFilter2D(mode="ca6_adaptive_r")
    kf_high_r.initialize(100.0, 100.0, timestamp=0.0)
    kf_high_r.correct(np.array([102.0, 100.0]), timestamp=0.02)
    # High R update
    kf_high_r.correct(np.array([105.0, 100.0]), timestamp=0.04, R_custom=np.diag([16.0, 16.0]))
    gain_high_r = kf_high_r.last_kalman_gain[0, 0]

    kf_low_r = KalmanFilter2D(mode="ca6_adaptive_r")
    kf_low_r.initialize(100.0, 100.0, timestamp=0.0)
    kf_low_r.correct(np.array([102.0, 100.0]), timestamp=0.02)
    # Low R update
    kf_low_r.correct(np.array([105.0, 100.0]), timestamp=0.04, R_custom=np.diag([0.1, 0.1]))
    gain_low_r = kf_low_r.last_kalman_gain[0, 0]

    assert gain_low_r > gain_high_r, f"Low R gain ({gain_low_r}) should exceed High R gain ({gain_high_r})"


def test_two_stage_initialization():
    """Verify: Frame 1 initializes position; Frame 2 initializes velocity via finite difference; acceleration remains 0."""
    kf = KalmanFilter2D(mode="ca6_adaptive_r")
    kf.initialize(50.0, 80.0, timestamp=0.0)
    assert kf.consecutive_updates == 1
    assert kf.state[2] == 0.0
    assert kf.state[3] == 0.0
    assert kf.state[4] == 0.0
    assert kf.state[5] == 0.0

    # Second detection at dt = 0.02s with displacement dx=2.0px, dy=1.0px -> vx=100px/s, vy=50px/s
    kf.correct(np.array([52.0, 81.0]), timestamp=0.02)
    assert kf.consecutive_updates == 2
    assert np.isclose(kf.state[0], 52.0)
    assert np.isclose(kf.state[1], 81.0)
    assert np.isclose(kf.state[2], 100.0, atol=1e-2)
    assert np.isclose(kf.state[3], 50.0, atol=1e-2)
    assert kf.state[4] == 0.0  # Acceleration remains 0 on frame 2
    assert kf.state[5] == 0.0


def test_prediction_only_covariance_growth():
    """Verify that during coasting (predict_only), covariance monotonically grows."""
    kf = KalmanFilter2D(mode="ca6_adaptive_r")
    kf.initialize(100.0, 100.0, timestamp=0.0)
    kf.correct(np.array([102.0, 100.0]), timestamp=0.02)

    initial_trace = np.trace(kf.covariance)
    traces = [initial_trace]

    for step in range(1, 10):
        t = 0.02 + step * 0.02
        kf.predict_only(t)
        traces.append(np.trace(kf.covariance))
        assert traces[-1] > traces[-2], f"Covariance did not grow at step {step}: {traces[-1]} <= {traces[-2]}"


def test_future_prediction_no_state_mutation():
    """Verify predict_state(horizon_s) does not mutate internal filter state or covariance."""
    kf = KalmanFilter2D(mode="ca6_adaptive_r")
    kf.initialize(100.0, 100.0, timestamp=0.0)
    kf.correct(np.array([102.0, 100.0]), timestamp=0.02)

    state_before = kf.state.copy()
    cov_before = kf.covariance.copy()

    x_fut, P_fut = kf.predict_state(horizon_s=0.200)

    np.testing.assert_allclose(kf.state, state_before)
    np.testing.assert_allclose(kf.covariance, cov_before)
    assert x_fut[0] > state_before[0]  # Projected forward along velocity


def test_nis_and_uncertainty_ellipse():
    """Verify online NIS calculation and error ellipse extraction."""
    kf = KalmanFilter2D(mode="ca6_adaptive_r")
    kf.initialize(100.0, 100.0, timestamp=0.0)
    kf.correct(np.array([102.0, 100.0]), timestamp=0.02)
    kf.correct(np.array([104.0, 100.0]), timestamp=0.04)

    assert kf.last_nis >= 0.0
    ellipse = kf.get_uncertainty_ellipse(n_sigma=2.0)
    assert "major_axis_px" in ellipse
    assert "minor_axis_px" in ellipse
    assert "orientation_deg" in ellipse
    assert ellipse["major_axis_px"] >= ellipse["minor_axis_px"] > 0.0


def test_health_state_transitions():
    """Verify ACTIVE -> DEGRADED -> EXPIRED health transitions based on coasting and timeout."""
    kf = KalmanFilter2D(mode="ca6_adaptive_r", max_prediction_seconds=0.10)
    kf.initialize(100.0, 100.0, timestamp=0.0)
    assert kf.get_health_status(0.0) == "ACTIVE"

    # Coast for 2 frames
    kf.predict_only(0.02)
    assert kf.get_health_status(0.02) == "DEGRADED"

    # Coast past 0.10s
    kf.predict_only(0.12)
    assert kf.get_health_status(0.12) == "EXPIRED"
    assert kf.check_occlusion_timeout(0.12) is True
    assert kf.is_initialized is False


def test_legacy_cv4_preservation():
    """Verify backward compatibility: cv4_fixed mode retains 4-state CV dynamics exactly."""
    kf_cv4 = KalmanFilter2D(mode="cv4_fixed")
    assert kf_cv4.state_dim == 4
    kf_cv4.initialize(100.0, 100.0, timestamp=0.0)
    assert kf_cv4.state.shape == (4,)
    kf_cv4.correct(np.array([102.0, 100.0]), timestamp=0.02)
    assert kf_cv4.state.shape == (4,)
    assert np.isclose(kf_cv4.state[2], 100.0, atol=1e-2)


def test_association_engine_compatibility_with_ca6():
    """Verify that AssociationEngine seamlessly gates and ranks candidates with CA6."""
    kf = KalmanFilter2D(mode="ca6_adaptive_r")
    kf.initialize(640.0, 360.0, timestamp=0.0)
    kf.correct(np.array([642.0, 360.0]), timestamp=0.02)

    engine = AssociationEngine(mode="robust_multifeature", image_width=1280, image_height=720)

    # Near target candidate (within gate) and distant candidate (outside gate)
    cands = [
        CandidateMeasurement.from_detector_candidate(
            {"center": [644.0, 360.0], "confidence": 0.95, "integrated_signal": 2000.0, "snr_db": 20.0,
             "localization": {"center": [644.0, 360.0], "uncertainty_x_px": 0.5, "uncertainty_y_px": 0.5}},
            candidate_index=0,
            timestamp=0.04
        ),
        CandidateMeasurement.from_detector_candidate(
            {"center": [750.0, 360.0], "confidence": 0.90, "integrated_signal": 3000.0, "snr_db": 25.0,
             "localization": {"center": [750.0, 360.0], "uncertainty_x_px": 0.5, "uncertainty_y_px": 0.5}},
            candidate_index=1,
            timestamp=0.04
        )
    ]

    res = engine.associate(cands, kf, timestamp=0.04)
    assert res.accepted is True
    assert res.selected_candidate_index == 0
    assert res.gated_candidate_count == 1


def test_prediction_horizon_consistency():
    """Verify that CA6 yields lower prediction error at lead latencies <= 100ms under acceleration."""
    kf_ca6 = KalmanFilter2D(mode="ca6_adaptive_r")
    kf_cv4 = KalmanFilter2D(mode="cv4_fixed")

    # Simulate accelerated trajectory x = 0.5 * 200 * t^2
    for step in range(5):
        t = step * 0.02
        pos = float(0.5 * 200.0 * (t ** 2))
        kf_ca6.update(np.array([pos, 100.0]), t)
        kf_cv4.update(np.array([pos, 100.0]), t)

    # 40ms lead prediction
    t_lead = 0.04
    gt_future = 0.5 * 200.0 * ((0.08 + t_lead) ** 2)
    x_ca6, _ = kf_ca6.predict_state(t_lead)
    x_cv4, _ = kf_cv4.predict_state(t_lead)

    err_ca6 = abs(x_ca6[0] - gt_future)
    err_cv4 = abs(x_cv4[0] - gt_future)
    assert err_ca6 < err_cv4, f"CA6 lead error ({err_ca6}) should be lower than CV4 ({err_cv4})"

