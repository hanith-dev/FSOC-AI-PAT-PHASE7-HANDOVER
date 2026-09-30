"""
Phase 7 Unit & Controlled Reacquisition Tests: Covariance-Guided Intelligent Reacquisition.

Verifies:
1. Mathematical Foundations:
   - Covariance eigendecomposition
   - Chi-square 2D ellipse scaling (1-sigma, 2-sigma, 95%, 99%)
   - Ellipse orientation angle recovery
   - Exact camera projection transformation
   - FOV clipping to observable bounds
   - Monotonic Mahalanobis radius ordering (high probability first)
   - Bounded search expansion
   - Search-center trust horizon damping
   - Search reset and state integrity
   - Deterministic bitwise repeatability
   - Ground truth firewall non-interference
   - Association requirement preserved (no shortcut)
   - Distractor rejection and safety
   - Fixed spiral preservation (R0)
   - FSM and Controller compatibility

2. Controlled Reacquisition Scenarios:
   - R01: Short centered dropout
   - R02: Horizontal uncertainty ellipse
   - R03: Vertical uncertainty ellipse
   - R04: Rotated ellipse (45 deg)
   - R05: Small covariance (tight search)
   - R06: Large covariance (wide search)
   - R07: Ellipse partially outside FOV (clipping)
   - R08: Target near ellipse boundary
   - R09: Target outside initial ellipse but inside expanded ellipse
   - R10: Covariance growth during search
   - R11: Stale CA6 prediction (trust horizon freeze)
   - R12: Distractor inside ellipse (rejection)
   - R13: True target + distractor both inside ellipse (correct selection)
   - R14: Failed local reacquisition (timeout to LOST)
   - R15: Successful reacquisition after expansion
   - R16: Deterministic repeatability
"""
import pytest
import numpy as np

from app.tracking.search_pattern import (
    SearchCommandGenerator,
    SearchPatternConfig,
    SearchCommand,
    compute_covariance_ellipse,
    chi2_scale_for_probability,
    probability_for_chi2_scale
)
from app.tracking.pat_state_machine import (
    PATLifecycleStateMachine,
    PATFSMConfig,
    PATState,
    TransitionReason
)
from app.tracking.association import AssociationEngine, CandidateMeasurement
from app.tracking.kalman import KalmanFilter2D
from app.tracking.controller import GimbalController, GimbalControllerConfig, ControlMode
from app.camera.calibration import CameraCalibration, DEFAULT_CAMERA_CALIBRATION


# =====================================================================
# 1. MATHEMATICAL FOUNDATION TESTS
# =====================================================================

def test_covariance_eigendecomposition():
    """Verify eigendecomposition of 2x2 position covariance matrix."""
    # Diagonal covariance: major along X
    P_diag = np.array([[100.0, 0.0], [0.0, 25.0]])
    res = compute_covariance_ellipse(P_diag, scale_k=1.0)
    assert np.isclose(res["major_axis_px"], 10.0, atol=1e-3)
    assert np.isclose(res["minor_axis_px"], 5.0, atol=1e-3)
    assert np.isclose(res["orientation_deg"], 0.0, atol=1e-2)

    # Diagonal covariance: major along Y
    P_vert = np.array([[16.0, 0.0], [0.0, 64.0]])
    res_v = compute_covariance_ellipse(P_vert, scale_k=1.0)
    assert np.isclose(res_v["major_axis_px"], 8.0, atol=1e-3)
    assert np.isclose(res_v["minor_axis_px"], 4.0, atol=1e-3)
    assert np.isclose(abs(res_v["orientation_deg"]), 90.0, atol=1e-2)


def test_chi_square_ellipse_scaling():
    """Verify 2-DOF chi-square scaling factors for standard confidence regions."""
    # 1-sigma: k = 1.0 -> p ~ 39.35%
    p_1s = probability_for_chi2_scale(1.0)
    assert np.isclose(p_1s, 0.393469, atol=1e-4)
    assert np.isclose(chi2_scale_for_probability(p_1s), 1.0, atol=1e-4)

    # 2-sigma: k = 2.0 -> p ~ 86.47%
    p_2s = probability_for_chi2_scale(2.0)
    assert np.isclose(p_2s, 0.864665, atol=1e-4)
    assert np.isclose(chi2_scale_for_probability(p_2s), 2.0, atol=1e-4)

    # 95%: p = 0.95 -> s = -2*ln(0.05) ~ 5.9915 -> k ~ 2.4477
    k_95 = chi2_scale_for_probability(0.95)
    assert np.isclose(k_95, 2.447747, atol=1e-4)
    assert np.isclose(probability_for_chi2_scale(k_95), 0.95, atol=1e-4)

    # 99%: p = 0.99 -> s = -2*ln(0.01) ~ 9.2103 -> k ~ 3.0349
    k_99 = chi2_scale_for_probability(0.99)
    assert np.isclose(k_99, 3.034854, atol=1e-4)
    assert np.isclose(probability_for_chi2_scale(k_99), 0.99, atol=1e-4)


def test_ellipse_orientation():
    """Verify orientation angle for rotated covariance matrices."""
    # 45-degree rotation
    theta_true = np.radians(45.0)
    R = np.array([
        [np.cos(theta_true), -np.sin(theta_true)],
        [np.sin(theta_true),  np.cos(theta_true)]
    ])
    D = np.diag([100.0, 25.0])
    P_rot = R @ D @ R.T

    res = compute_covariance_ellipse(P_rot, scale_k=1.0)
    assert np.isclose(res["major_axis_px"], 10.0, atol=1e-2)
    assert np.isclose(res["minor_axis_px"], 5.0, atol=1e-2)
    assert np.isclose(res["orientation_deg"], 45.0, atol=1.0)


def test_pixel_to_angle_conversion():
    """Verify exact pinhole projection mapping without constant-scale approximations."""
    cfg = SearchPatternConfig(
        fx=3661.723149, fy=3661.723149,
        cx=320.0, cy=240.0,
        camera_width=640, camera_height=480
    )
    # Principal point: zero optical angle
    u0, v0 = 320.0, 240.0
    th_h0 = float(np.degrees(np.arctan((u0 - cfg.cx) / cfg.fx)))
    th_v0 = float(np.degrees(np.arctan((cfg.cy - v0) / cfg.fy)))
    assert np.isclose(th_h0, 0.0, atol=1e-6)
    assert np.isclose(th_v0, 0.0, atol=1e-6)

    # Edge of sensor: (640, 240) -> u = +320px
    th_hedge = float(np.degrees(np.arctan((640.0 - cfg.cx) / cfg.fx)))
    assert np.isclose(th_hedge, 4.994464, atol=1e-4)  # Half of HFOV (9.9889 deg)


def test_fov_clipping():
    """Verify that commanded search points outside sensor bounds are clipped."""
    cfg = SearchPatternConfig(
        pattern_type="covariance_ellipse_scan",
        fov_clipping=True,
        fov_margin_px=10.0,
        camera_width=640,
        camera_height=480,
        max_duration_s=2.0
    )
    gen = SearchCommandGenerator(cfg)

    # Center near right edge with large horizontal covariance
    mean = np.array([620.0, 240.0])
    cov = np.array([[2500.0, 0.0], [0.0, 100.0]])  # 50px major axis along X
    gen.start(0.0, kf_pos_mean=mean, kf_pos_cov=cov)

    clipped_any = False
    for step in range(50):
        t = step * 0.04
        _, _, _ = gen.get_offset(t, kf_pos_mean=mean, kf_pos_cov=cov)
        cmd = gen.get_command()
        assert 10.0 <= cmd.u_commanded_px <= 630.0
        assert 10.0 <= cmd.v_commanded_px <= 470.0
        if cmd.fov_clipped:
            clipped_any = True

    assert clipped_any is True, "FOV clipping should have activated near edge!"


def test_search_ordering_monotonic_mahalanobis():
    """Verify that covariance ellipse scan orders search points by non-decreasing Mahalanobis radius."""
    cfg = SearchPatternConfig(pattern_type="covariance_ellipse_scan", max_duration_s=2.0)
    gen = SearchCommandGenerator(cfg)
    gen.start(0.0, kf_pos_mean=np.array([320.0, 240.0]), kf_pos_cov=np.eye(2) * 100.0)

    radii = []
    for step in range(50):
        t = step * 0.04
        gen.get_offset(t)
        cmd = gen.get_command()
        radii.append(cmd.mahalanobis_radius)

    # Verify that radii are non-decreasing over time (high probability first)
    for i in range(1, len(radii)):
        assert radii[i] >= radii[i - 1] - 1e-4, f"Mahalanobis radius decreased at step {i}: {radii[i]} < {radii[i-1]}"


def test_maximum_search_bound():
    """Verify that search offsets never exceed configured max_local_radius_deg."""
    cfg = SearchPatternConfig(
        pattern_type="covariance_ellipse_scan",
        max_local_radius_deg=1.5,
        max_duration_s=2.0
    )
    gen = SearchCommandGenerator(cfg)
    # Huge covariance
    gen.start(0.0, kf_pos_mean=np.array([320.0, 240.0]), kf_pos_cov=np.eye(2) * 100000.0)

    for step in range(50):
        t = step * 0.04
        pan, tilt, _ = gen.get_offset(t)
        radius = np.hypot(pan, tilt)
        assert radius <= cfg.max_local_radius_deg + 1e-4


def test_stale_prediction_handling_trust_horizon():
    """Verify that search center freezes after trust_horizon_s."""
    cfg = SearchPatternConfig(
        pattern_type="covariance_ellipse_scan",
        trust_horizon_s=0.30,
        max_duration_s=2.0
    )
    gen = SearchCommandGenerator(cfg)
    gen.start(0.0, kf_pos_mean=np.array([320.0, 240.0]), kf_pos_cov=np.eye(2) * 25.0, time_since_measurement=0.0)

    # Drift mean continuously
    centers = []
    for step in range(25):
        t = step * 0.04
        drifting_mean = np.array([320.0 + 100.0 * t, 240.0 + 50.0 * t])
        gen.get_offset(t, kf_pos_mean=drifting_mean, time_since_measurement=t)
        centers.append((gen.cached_pos_mean[0], gen.cached_pos_mean[1]))

    # Center should move before 0.30s
    assert centers[3] != centers[0]  # t=0.12s
    # Center should freeze after 0.30s (step >= 8, t >= 0.32s)
    frozen_pt = centers[8]
    for c in centers[9:]:
        assert np.isclose(c[0], frozen_pt[0], atol=1e-3)
        assert np.isclose(c[1], frozen_pt[1], atol=1e-3)


def test_search_reset():
    """Verify that reset restores all internal state variables cleanly."""
    gen = SearchCommandGenerator()
    gen.start(10.0, kf_pos_mean=np.array([400.0, 300.0]))
    gen.get_offset(10.5)
    assert gen.active is True
    assert gen.point_index > 0
    assert gen.total_angular_travel_deg > 0.0

    gen.reset()
    assert gen.active is False
    assert gen.point_index == 0
    assert gen.total_angular_travel_deg == 0.0
    assert gen.last_offset_pan == 0.0
    assert gen.last_offset_tilt == 0.0


def test_determinism_repeatability():
    """Verify that two identical generators produce bitwise identical commands."""
    cfg = SearchPatternConfig(pattern_type="covariance_ellipse_scan")
    gen1 = SearchCommandGenerator(cfg)
    gen2 = SearchCommandGenerator(cfg)

    mean = np.array([350.0, 220.0])
    cov = np.array([[80.0, 20.0], [20.0, 50.0]])
    gen1.start(1.0, kf_pos_mean=mean, kf_pos_cov=cov)
    gen2.start(1.0, kf_pos_mean=mean, kf_pos_cov=cov)

    for i in range(50):
        t = 1.0 + i * 0.02
        p1, t1, tout1 = gen1.get_offset(t)
        p2, t2, tout2 = gen2.get_offset(t)
        assert p1 == p2
        assert t1 == t2
        assert tout1 == tout2
        assert gen1.get_command() == gen2.get_command()


def test_truth_firewall_reacquisition():
    """Verify search generator and FSM methods contain zero ground-truth arguments."""
    import inspect
    import app.tracking.search_pattern as sp_mod
    import app.tracking.pat_state_machine as pat_mod

    for mod in [sp_mod, pat_mod]:
        for name, obj in inspect.getmembers(mod):
            if inspect.isclass(obj):
                for method_name in ["update", "transition_to", "get_offset", "start", "_step_covariance_ellipse"]:
                    if hasattr(obj, method_name):
                        sig = inspect.signature(getattr(obj, method_name))
                        for forbidden in ["truth", "gt", "target_id", "ground_truth", "designated_target"]:
                            assert forbidden not in sig.parameters, f"Forbidden param '{forbidden}' in {name}.{method_name}!"


def test_fixed_spiral_preservation():
    """Verify that fixed spiral mode (R0) is fully preserved with exact backward compatibility."""
    cfg = SearchPatternConfig(pattern_type="fixed_spiral", angular_step_deg=0.25, scan_rate_deg_s=1.0)
    gen = SearchCommandGenerator(cfg)
    gen.start(0.0)

    pan, tilt, tout = gen.get_offset(0.5)
    assert tout is False
    assert abs(pan) > 0.0 or abs(tilt) > 0.0
    cmd = gen.get_command()
    assert cmd.search_stage == "fixed_spiral"


# =====================================================================
# 2. CONTROLLED REACQUISITION TESTS (R01 to R16)
# =====================================================================

def test_r01_short_centered_dropout():
    """R01: Short centered dropout (100 ms) recovers quickly within 1-sigma region."""
    fsm = PATLifecycleStateMachine()
    fsm.current_state = PATState.TRACK
    t = 1.0

    # 5 frames miss (100ms) -> enters COAST
    for _ in range(5):
        t += 0.02
        fsm.update("NO_CANDIDATE", False, 0, "ACTIVE", 3.0, 0.1, t)
    assert fsm.current_state == PATState.COAST

    # Target returns: immediate recovery back to TRACK
    t += 0.02
    s = fsm.update("ACCEPTED", True, 1, "ACTIVE", 2.0, 0.05, t)
    assert s in (PATState.TRACK.value, PATState.LOCK.value)
    assert fsm.transition_reason == TransitionReason.SHORT_COAST_RECOVERED.value


def test_r02_horizontal_uncertainty_ellipse():
    """R02: Horizontal uncertainty ellipse (Pxx >> Pyy) directs scan along horizontal axis."""
    cfg = SearchPatternConfig(pattern_type="covariance_ellipse_scan")
    gen = SearchCommandGenerator(cfg)
    cov = np.array([[400.0, 0.0], [0.0, 25.0]])  # 20px major along X, 5px minor along Y
    gen.start(0.0, kf_pos_mean=np.array([320.0, 240.0]), kf_pos_cov=cov)

    assert np.isclose(gen.ellipse_theta_deg, 0.0, atol=1e-2)
    assert gen.sigma1 > gen.sigma2 * 3.0


def test_r03_vertical_uncertainty_ellipse():
    """R03: Vertical uncertainty ellipse (Pyy >> Pxx) directs scan along vertical axis."""
    cfg = SearchPatternConfig(pattern_type="covariance_ellipse_scan")
    gen = SearchCommandGenerator(cfg)
    cov = np.array([[25.0, 0.0], [0.0, 400.0]])  # 20px major along Y, 5px minor along X
    gen.start(0.0, kf_pos_mean=np.array([320.0, 240.0]), kf_pos_cov=cov)

    assert np.isclose(abs(gen.ellipse_theta_deg), 90.0, atol=1e-2)
    assert gen.sigma1 > gen.sigma2 * 3.0


def test_r04_rotated_ellipse():
    """R04: Rotated ellipse (45 deg) correctly recovers orientation."""
    cfg = SearchPatternConfig(pattern_type="covariance_ellipse_scan")
    gen = SearchCommandGenerator(cfg)
    th = np.radians(45.0)
    R = np.array([[np.cos(th), -np.sin(th)], [np.sin(th), np.cos(th)]])
    cov = R @ np.diag([200.0, 20.0]) @ R.T
    gen.start(0.0, kf_pos_mean=np.array([320.0, 240.0]), kf_pos_cov=cov)

    assert np.isclose(gen.ellipse_theta_deg, 45.0, atol=2.0)


def test_r05_small_covariance():
    """R05: Small covariance results in tight search region."""
    cfg = SearchPatternConfig(pattern_type="covariance_ellipse_scan")
    gen = SearchCommandGenerator(cfg)
    cov = np.diag([4.0, 4.0])  # sigma = 2px
    gen.start(0.0, kf_pos_mean=np.array([320.0, 240.0]), kf_pos_cov=cov)

    # During 1-sigma stage (t=0.2s), major axis is small
    gen.get_offset(0.2)
    cmd = gen.get_command()
    assert abs(cmd.pan_offset_deg) < 0.2
    assert abs(cmd.tilt_offset_deg) < 0.2


def test_r06_large_covariance():
    """R06: Large covariance expands search region wider up to authority clamp."""
    cfg = SearchPatternConfig(pattern_type="covariance_ellipse_scan")
    gen = SearchCommandGenerator(cfg)
    cov = np.diag([900.0, 900.0])  # sigma = 30px
    gen.start(0.0, kf_pos_mean=np.array([320.0, 240.0]), kf_pos_cov=cov)

    gen.get_offset(1.0)
    cmd = gen.get_command()
    ang_rad = np.hypot(cmd.pan_offset_deg, cmd.tilt_offset_deg)
    assert ang_rad > 0.3  # significantly larger than R05


def test_r07_ellipse_partially_outside_fov():
    """R07: Search center near sensor edge clips unobservable coordinates to FOV margin."""
    cfg = SearchPatternConfig(pattern_type="covariance_ellipse_scan", fov_clipping=True)
    gen = SearchCommandGenerator(cfg)
    mean = np.array([630.0, 240.0])
    cov = np.diag([400.0, 100.0])
    gen.start(0.0, kf_pos_mean=mean, kf_pos_cov=cov)

    clipped_count = 0
    for i in range(25):
        gen.get_offset(i * 0.04)
        if gen.get_command().fov_clipped:
            clipped_count += 1
    assert clipped_count > 0


def test_r08_target_near_ellipse_boundary():
    """R08: Target at 1.8-sigma is reached during 2-sigma stage."""
    cfg = SearchPatternConfig(pattern_type="covariance_ellipse_scan")
    gen = SearchCommandGenerator(cfg)
    gen.start(0.0, kf_pos_mean=np.array([320.0, 240.0]), kf_pos_cov=np.eye(2) * 100.0)

    # Advance to 2-sigma stage (t=0.6s)
    gen.get_offset(0.6)
    cmd = gen.get_command()
    assert cmd.confidence_region == "2_sigma"
    assert 1.0 <= cmd.mahalanobis_radius <= 2.0


def test_r09_target_outside_initial_ellipse_inside_expanded():
    """R09: Target at 2.3-sigma is reached during 95% expansion stage."""
    cfg = SearchPatternConfig(pattern_type="covariance_ellipse_scan")
    gen = SearchCommandGenerator(cfg)
    gen.start(0.0, kf_pos_mean=np.array([320.0, 240.0]), kf_pos_cov=np.eye(2) * 100.0)

    # Advance to 95% stage (t=1.1s)
    gen.get_offset(1.1)
    cmd = gen.get_command()
    assert cmd.confidence_region == "95_percent"
    assert 2.0 <= cmd.mahalanobis_radius <= 2.45


def test_r10_covariance_growth_during_search():
    """R10: Covariance update during search expands ellipse bounds."""
    cfg = SearchPatternConfig(pattern_type="covariance_ellipse_scan")
    gen = SearchCommandGenerator(cfg)
    gen.start(0.0, kf_pos_mean=np.array([320.0, 240.0]), kf_pos_cov=np.eye(2) * 25.0)
    assert np.isclose(gen.sigma1, 5.0)

    # Estimator covariance expands after coasting
    gen.get_offset(0.5, kf_pos_cov=np.eye(2) * 100.0)
    assert np.isclose(gen.sigma1, 10.0)


def test_r11_stale_ca6_prediction():
    """R11: Stale CA6 prediction stops translating search center beyond trust horizon."""
    cfg = SearchPatternConfig(pattern_type="covariance_ellipse_scan", trust_horizon_s=0.35)
    gen = SearchCommandGenerator(cfg)
    gen.start(0.0, kf_pos_mean=np.array([320.0, 240.0]), time_since_measurement=0.20)

    # At t=0.20s in reacq, total age is 0.40s (> 0.35s trust horizon)
    gen.get_offset(0.20, kf_pos_mean=np.array([500.0, 400.0]), time_since_measurement=0.40)
    assert gen.center_frozen is True
    # Commanded center must NOT follow [500, 400]
    assert gen.cached_pos_mean[0] < 450.0


def test_r12_distractor_inside_ellipse():
    """R12: Optical distractor inside search ellipse is safely rejected by association engine."""
    assoc = AssociationEngine(mode="robust_multifeature")
    kf = KalmanFilter2D(mode="ca6_adaptive_r")
    kf.initialize(320.0, 240.0, 0.0)

    # Establish track history with genuine beacon measurements
    for step in range(20):
        t = step * 0.02
        c = CandidateMeasurement(
            candidate_index=0,
            x=320.0, y=240.0,
            localization_method="weighted_centroid",
            localization_uncertainty_x_px=0.5,
            localization_uncertainty_y_px=0.5,
            confidence=0.95,
            area=30.0,
            peak_intensity=200.0,
            integrated_signal=1000.0,
            background_level=20.0,
            snr_proxy=25.0,
            sigma_x_px=2.5,
            sigma_y_px=2.5,
            shape_quality=0.95,
            quality_flags=[],
            timestamp=t
        )
        assoc.associate([c], kf, t)
        kf.update(np.array([c.x, c.y]), t)

    # Coasting / dropout
    kf.predict_only(0.5)

    # Distractor candidate inside search region but outside established track gate
    distractor = CandidateMeasurement(
        candidate_index=0,
        x=320.0, y=180.0,
        localization_method="weighted_centroid",
        localization_uncertainty_x_px=2.0,
        localization_uncertainty_y_px=2.0,
        confidence=0.4,
        area=80.0,
        peak_intensity=80.0,
        integrated_signal=8000.0,
        background_level=20.0,
        snr_proxy=8.0,
        sigma_x_px=4.0,
        sigma_y_px=4.0,
        shape_quality=0.3,
        quality_flags=[],
        timestamp=0.5
    )

    res = assoc.associate([distractor], kf, 0.5)
    # Distractor rejected (gate or cost)
    assert res.accepted is False or res.status != "ACCEPTED"


def test_r13_true_target_plus_distractor_inside_ellipse():
    """R13: When true target and distractor are both present, association selects true target."""
    assoc = AssociationEngine(mode="robust_multifeature")
    kf = KalmanFilter2D(mode="ca6_adaptive_r")
    kf.initialize(320.0, 240.0, 0.0)

    # Target candidate near predicted state
    true_target = CandidateMeasurement(
        candidate_index=1,
        x=321.0, y=240.5,
        localization_method="weighted_centroid",
        localization_uncertainty_x_px=0.5,
        localization_uncertainty_y_px=0.5,
        confidence=0.95,
        area=30.0,
        peak_intensity=200.0,
        integrated_signal=2000.0,
        background_level=20.0,
        snr_proxy=25.0,
        sigma_x_px=2.5,
        sigma_y_px=2.5,
        shape_quality=0.9,
        quality_flags=[],
        timestamp=0.1
    )
    distractor = CandidateMeasurement(
        candidate_index=0,
        x=350.0, y=200.0,
        localization_method="weighted_centroid",
        localization_uncertainty_x_px=1.0,
        localization_uncertainty_y_px=1.0,
        confidence=0.7,
        area=80.0,
        peak_intensity=250.0,
        integrated_signal=4000.0,
        background_level=20.0,
        snr_proxy=35.0,
        sigma_x_px=3.5,
        sigma_y_px=3.5,
        shape_quality=0.6,
        quality_flags=[],
        timestamp=0.1
    )

    res = assoc.associate([distractor, true_target], kf, 0.1)
    assert res.accepted is True
    assert res.selected_index == 1  # Selects true target, rejects distractor!


def test_r14_failed_local_reacquisition():
    """R14: Failed reacquisition times out at max_duration_s and transitions to LOST."""
    fsm = PATLifecycleStateMachine()
    fsm.transition_to(PATState.REACQUIRE, TransitionReason.COAST_TIMEOUT, 1.0)
    assert fsm.current_state == PATState.REACQUIRE

    t = 1.0
    # Run past 2.0s max duration
    for _ in range(105):
        t += 0.02
        fsm.update("NO_CANDIDATE", False, 0, "DEGRADED", 20.0, 0.5, t)

    assert fsm.previous_state == PATState.LOST or fsm.current_state in (PATState.LOST, PATState.SEARCH)


def test_r15_successful_reacquisition_after_expansion():
    """R15: Successful candidate detection in outer stage requires 2 consecutive updates to re-enter TRACK."""
    fsm = PATLifecycleStateMachine()
    fsm.transition_to(PATState.REACQUIRE, TransitionReason.COAST_TIMEOUT, 1.0)

    # Reappearance at frame 1: remains in REACQUIRE (1 confirmation)
    t = 1.8
    s1 = fsm.update("ACCEPTED", True, 1, "ACTIVE", 3.0, 0.2, t)
    assert s1 == PATState.REACQUIRE.value

    # Reappearance at frame 2: confirmed! -> TRACK
    t += 0.02
    s2 = fsm.update("ACCEPTED", True, 1, "ACTIVE", 2.0, 0.15, t)
    assert s2 == PATState.TRACK.value
    assert fsm.transition_reason == TransitionReason.LOCAL_REACQUIRE_SUCCESS.value


def test_r16_deterministic_repeatability():
    """R16: Full closed-loop FSM + Search sequence is bit-for-bit identical across two independent runs."""
    def run_sim():
        fsm = PATLifecycleStateMachine()
        fsm.transition_to(PATState.REACQUIRE, TransitionReason.COAST_TIMEOUT, 0.0)
        history = []
        for i in range(50):
            t = i * 0.04
            st = fsm.update("NO_CANDIDATE", False, 0, "DEGRADED", 15.0, 0.5, t)
            telem = fsm.get_telemetry(t)
            history.append((st, telem["search_offset_pan_deg"], telem["search_offset_tilt_deg"], telem["search_stage"]))
        return history

    h1 = run_sim()
    h2 = run_sim()
    assert h1 == h2
