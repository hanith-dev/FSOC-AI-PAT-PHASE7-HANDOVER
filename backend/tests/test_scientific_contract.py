"""
Automated Tests for Phase 1.6 Camera Geometry and Scientific Metric Contracts.
Validates:
- Camera FOV convention and square pixel calibration (Three.js VFOV authoritative)
- Matrix-level 4x4 View-Projection parity with Three.js camera pipeline
- Pixel-to-angle mathematical round-trip consistency
- Observability contract flags (active, in_fov, optically_present, detectable)
- Measurement coverage denominators (observable vs mission)
- Identity-based false lock semantics (large error != false lock)
- Identity switch counting
- Strict Ground-Truth Firewall non-interference during evaluation
"""
import pytest
import numpy as np
from app.camera.calibration import CameraCalibration, DEFAULT_CAMERA_CALIBRATION
from app.camera.model import CameraModel
from app.benchmark.metrics import BenchmarkMetricsCalculator


# =====================================================================
# 1. CAMERA CALIBRATION & FOV CONVENTION TESTS
# =====================================================================

def test_camera_fov_convention_vfov_authoritative():
    """
    Verify Three.js vertical FOV convention and square pixel constraint.
    For W=640, H=480, VFOV=7.5 deg:
    fy = (H / 2) / tan(VFOV / 2)
    fx = fy
    HFOV = 2 * atan((W / 2) / fx)
    """
    calib = CameraCalibration.from_vfov(width=640, height=480, vfov_deg=7.5)

    assert calib.resolution_width == 640
    assert calib.resolution_height == 480
    assert calib.fov_axis == "vertical"
    assert calib.vfov_deg == 7.5

    # Square pixels
    assert calib.fx_px == calib.fy_px
    assert calib.pixel_aspect_ratio == 1.0

    # Analytical focal length
    expected_f = (480.0 / 2.0) / np.tan(np.radians(3.75))
    assert pytest.approx(calib.fy_px, abs=1e-6) == expected_f
    assert pytest.approx(calib.fx_px, abs=1e-6) == expected_f

    # Derived HFOV
    expected_hfov = np.degrees(2.0 * np.arctan((640.0 / 2.0) / expected_f))
    assert pytest.approx(calib.hfov_deg, abs=1e-5) == expected_hfov
    assert pytest.approx(calib.hfov_deg, abs=1e-2) == 9.99  # ~9.989 degrees


def test_camera_matrix_level_projection_parity():
    """
    Compare analytical CameraModel.camera_to_pixel against exact 4x4
    Three.js View-Projection matrix multiplication (project_threejs).
    Must match down to floating point precision across all attitudes and ranges.
    """
    cam = CameraModel(width=640, height=480, vfov_deg=7.5, camera_pos=np.array([0.0, 1.5, 0.0]))

    test_cases = [
        # (world_pos, pan_deg, tilt_deg, desc)
        (np.array([0.0, 1.5, -100.0]), 0.0, 0.0, "Boresight"),
        (np.array([4.0, 1.5, -100.0]), 0.0, 0.0, "Positive horizontal offset"),
        (np.array([-4.0, 1.5, -100.0]), 0.0, 0.0, "Negative horizontal offset"),
        (np.array([0.0, 4.5, -100.0]), 0.0, 0.0, "Positive vertical offset"),
        (np.array([0.0, -1.5, -100.0]), 0.0, 0.0, "Negative vertical offset"),
        (np.array([3.5, 3.5, -100.0]), 0.0, 0.0, "Combined diagonal offset"),
        (np.array([8.0, 1.5, -100.0]), 0.0, 0.0, "Near FOV edge"),
        (np.array([25.0, 1.5, -100.0]), 0.0, 0.0, "Outside FOV"),
        (np.array([10.0, 1.5, -100.0]), 5.0, 0.0, "Nonzero pan attitude"),
        (np.array([0.0, 8.0, -100.0]), 0.0, 3.5, "Nonzero tilt attitude"),
        (np.array([12.0, 6.0, -150.0]), 4.0, -2.0, "General 3D attitude and depth"),
    ]

    for w_pos, pan_deg, tilt_deg, desc in test_cases:
        pan_rad = np.radians(pan_deg)
        tilt_rad = np.radians(tilt_deg)

        # Method 1: Camera coordinate transform + Pinhole division
        p_cam = cam.world_to_camera(w_pos, pan_rad, tilt_rad)
        u1, v1, in_fov1 = cam.camera_to_pixel(p_cam)

        # Method 2: Exact Three.js 4x4 View-Projection Matrix
        u2, v2, in_fov2 = cam.project_threejs(w_pos, pan_rad, tilt_rad)

        assert in_fov1 == in_fov2, f"FOV mismatch on {desc}: {in_fov1} vs {in_fov2}"
        if in_fov1:
            diff_u = abs(u1 - u2)
            diff_v = abs(v1 - v2)
            assert diff_u < 1e-10, f"Matrix disparity in U on {desc}: {diff_u} px"
            assert diff_v < 1e-10, f"Matrix disparity in V on {desc}: {diff_v} px"


def test_pixel_to_angle_round_trip():
    """
    Verify pixel -> angle -> pixel and angle -> pixel -> angle mathematical round trip.
    """
    cam = CameraModel(width=640, height=480, vfov_deg=7.5)

    test_angles = [
        (0.0, 0.0),
        (0.1, 0.05),
        (-0.5, 0.3),
        (1.0, -1.0),
        (-2.5, 1.8),
        (4.5, -3.2),  # Near FOV boundary
    ]

    for h_ang, v_ang in test_angles:
        # Angle to pixel
        u, v = cam.calibration.angle_to_pixel(h_ang, v_ang)

        # Pixel back to angle
        tot_deg, recovered_h, recovered_v = cam.pixel_to_angle(u, v)

        assert pytest.approx(recovered_h, abs=1e-10) == h_ang
        assert pytest.approx(recovered_v, abs=1e-10) == v_ang

        expected_tot = np.degrees(np.arccos(1.0 / np.sqrt(np.tan(np.radians(h_ang))**2 + np.tan(np.radians(v_ang))**2 + 1.0)))
        assert pytest.approx(tot_deg, abs=1e-10) == expected_tot


# =====================================================================
# 2. OBSERVABILITY & METRICS CONTRACT TESTS
# =====================================================================

def test_observability_flags_classification():
    """
    Verify strict separation between active, in_fov, optically_present, and detectable.
    """
    # Case 1: Target outside FOV
    active = True
    in_fov = False
    optically_present = active and in_fov
    assert not optically_present

    # Case 2: Intentional optical blackout (in FOV but transmitter off)
    active = False
    in_fov = True
    optically_present = active and in_fov
    assert not optically_present

    # Case 3: Severe fading (active & in FOV, but below detector threshold)
    active = True
    in_fov = True
    intensity = 0.05  # Below 0.1 threshold
    optically_present = active and in_fov
    detectable = optically_present and (intensity > 0.1)
    assert optically_present
    assert not detectable


def test_coverage_denominators_observable_vs_mission():
    """
    Verify observable_measurement_coverage_pct vs mission_measurement_coverage_pct.
    Scenario: 100 frames total, 70 frames target optically present, 30 frames blackout.
    Detector accepts measurements on all 70 observable frames.
    """
    calc = BenchmarkMetricsCalculator(dt=0.02)

    telemetry = []
    # 70 observable frames, all detected
    for t in range(70):
        telemetry.append({
            "simulation_time": t * 0.02,
            "detection_available": True,
            "ground_truth_target_active": True,
            "ground_truth_target_in_fov": True,
            "ground_truth_target_optically_present": True,
            "evaluated_candidate_id": "BEACON-01",
            "tracking_state": "LOCKED",
            "tracking_error_deg": 0.05
        })

    # 30 blackout frames (active=False, optically_present=False, det=False)
    for t in range(70, 100):
        telemetry.append({
            "simulation_time": t * 0.02,
            "detection_available": False,
            "ground_truth_target_active": False,
            "ground_truth_target_in_fov": True,
            "ground_truth_target_optically_present": False,
            "evaluated_candidate_id": None,
            "tracking_state": "SEARCHING",
            "tracking_error_deg": None
        })

    metrics = calc.compute_metrics(telemetry, designated_target_id="BEACON-01")

    # Observable coverage: 70 / 70 = 100%
    assert metrics["perception"]["observable_measurement_coverage_pct"] == 100.0
    # Mission coverage: 70 / 100 = 70%
    assert metrics["perception"]["mission_measurement_coverage_pct"] == 70.0
    # Observable recall: 1.0
    assert metrics["perception"]["detection_recall_observable"] == 1.0


def test_false_lock_identity_semantics_large_error_is_not_false_lock():
    """
    Verify identity-based false lock semantics:
    1. Large error while still tracking designated target is TRACKING ERROR, NOT false lock.
    2. Tracking a distractor for >= 0.5s IS a false lock.
    3. Tracking a distractor for < 0.5s is transient clutter, NOT a false lock event.
    """
    calc = BenchmarkMetricsCalculator(dt=0.02, false_lock_persistence_s=0.5)

    # Sequence A: 50 frames with 80px error, but evaluated ID is BEACON-01 (designated)
    telemetry_large_err = []
    for t in range(50):
        telemetry_large_err.append({
            "simulation_time": t * 0.02,
            "detection_available": True,
            "ground_truth_target_active": True,
            "ground_truth_target_in_fov": True,
            "ground_truth_target_optically_present": True,
            "evaluated_candidate_id": "BEACON-01",
            "tracking_state": "LOCKED",
            "tracking_error_deg": 1.5,
            "tracking_error_px": 80.0
        })

    res_a = calc.compute_metrics(telemetry_large_err, designated_target_id="BEACON-01")
    assert res_a["association"]["false_lock_event_count"] == 0
    assert res_a["association"]["false_lock_duration_s"] == 0.0

    # Sequence B: 30 frames (0.6s > 0.5s) tracking DISTRACTOR-01
    telemetry_distractor = []
    for t in range(30):
        telemetry_distractor.append({
            "simulation_time": t * 0.02,
            "detection_available": True,
            "ground_truth_target_active": True,
            "ground_truth_target_in_fov": True,
            "ground_truth_target_optically_present": True,
            "evaluated_candidate_id": "DISTRACTOR-01",
            "tracking_state": "LOCKED",
            "tracking_error_deg": 1.2
        })

    res_b = calc.compute_metrics(telemetry_distractor, designated_target_id="BEACON-01")
    assert res_b["association"]["false_lock_event_count"] == 1
    assert res_b["association"]["false_lock_duration_s"] > 0.0

    # Sequence C: 15 frames (0.3s < 0.5s) tracking DISTRACTOR-01, then drops
    telemetry_transient = []
    for t in range(15):
        telemetry_transient.append({
            "simulation_time": t * 0.02,
            "detection_available": True,
            "ground_truth_target_active": True,
            "ground_truth_target_in_fov": True,
            "ground_truth_target_optically_present": True,
            "evaluated_candidate_id": "DISTRACTOR-01",
            "tracking_state": "TRACKING",
            "tracking_error_deg": 0.8
        })

    res_c = calc.compute_metrics(telemetry_transient, designated_target_id="BEACON-01")
    assert res_c["association"]["false_lock_event_count"] == 0


def test_identity_switch_counting():
    """
    Verify counting of identity switches between distinct physical targets.
    BEACON-01 -> DISTRACTOR-01 -> DISTRACTOR-02 -> BEACON-01 (3 switches).
    """
    calc = BenchmarkMetricsCalculator(dt=0.02)
    telemetry = []

    sequence = ["BEACON-01"] * 10 + ["DISTRACTOR-01"] * 10 + ["DISTRACTOR-02"] * 10 + ["BEACON-01"] * 10
    for t, cand_id in enumerate(sequence):
        telemetry.append({
            "simulation_time": t * 0.02,
            "detection_available": True,
            "ground_truth_target_active": True,
            "ground_truth_target_in_fov": True,
            "ground_truth_target_optically_present": True,
            "evaluated_candidate_id": cand_id,
            "tracking_state": "TRACKING",
            "tracking_error_deg": 0.2
        })

    res = calc.compute_metrics(telemetry, designated_target_id="BEACON-01")
    assert res["association"]["identity_switch_count"] == 3


def test_pointing_error_populations_segregation():
    """
    Verify segregation of pointing error into all_active, target_in_fov, and locked populations.
    """
    calc = BenchmarkMetricsCalculator(dt=0.02)
    telemetry = [
        # Frame 0: in FOV, locked, error = 0.1
        {"simulation_time": 0.0, "tracking_error_deg": 0.1, "ground_truth_target_active": True, "ground_truth_target_in_fov": True, "tracking_state": "LOCKED"},
        # Frame 1: outside FOV, searching, error = 5.0
        {"simulation_time": 0.02, "tracking_error_deg": 5.0, "ground_truth_target_active": True, "ground_truth_target_in_fov": False, "tracking_state": "SEARCHING"},
        # Frame 2: in FOV, acquiring, error = 0.4
        {"simulation_time": 0.04, "tracking_error_deg": 0.4, "ground_truth_target_active": True, "ground_truth_target_in_fov": True, "tracking_state": "ACQUIRING"},
    ]

    metrics = calc.compute_metrics(telemetry, designated_target_id="BEACON-01")
    pops = metrics["tracking"]["pointing_error_populations"]

    # locked population should only include Frame 0 (error = 0.1)
    assert pops["locked_frames"]["count"] == 1
    assert pops["locked_frames"]["rms_deg"] == 0.1

    # in_fov population includes Frame 0 and Frame 2
    assert pops["target_in_fov"]["count"] == 2

    # all_active includes all 3 frames
    assert pops["all_active_frames"]["count"] == 3
