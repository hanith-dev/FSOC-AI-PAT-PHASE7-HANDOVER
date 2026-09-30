"""
Unit and Contract Test Suite for Phase 2 High-Precision Optical Beacon Localization.

Verifies:
1. Exact subpixel synthetic center recovery (< 0.1 px target on clean PSF).
2. Weighted centroid mathematics & background subtraction.
3. Robust local border background estimation.
4. Fallback on zero-weight, empty ROI, and edge ROIs.
5. Saturated beacon core handling.
6. Noise, blur, and fading robustness.
7. NaN/Inf protection and finite numerical bounds.
8. Uncertainty qualitative monotonic behavior (clean < degraded).
9. Strict ground-truth non-interference.
10. Preserved baseline localizer and dynamic mode switching.
"""
from typing import Tuple, List, Dict, Any
import pytest
import numpy as np
import cv2

from app.detection.localization import (
    LocalizationResult,
    BaseLocalizer,
    BaselineLocalizer,
    WeightedCentroidLocalizer,
    PSFRefinedLocalizer,
    get_localizer,
    estimate_local_background,
    extract_roi,
    LOCALIZER_REGISTRY
)
from app.detection.detector import OpenCVDetector
from app.camera.calibration import CameraCalibration


def _make_synthetic_spot(
    width: int,
    height: int,
    cx: float,
    cy: float,
    amplitude: float = 200.0,
    sigma: float = 2.5,
    bg: float = 20.0
) -> Tuple[np.ndarray, list]:
    img = np.full((height, width), bg, dtype=np.float64)
    ys, xs = np.indices((height, width), dtype=np.float64)
    spot = amplitude * np.exp(-0.5 * (((xs - cx) / sigma) ** 2 + ((ys - cy) / sigma) ** 2))
    img += spot
    mono = np.clip(img, 0, 255).astype(np.uint8)

    # Convert to 3-channel green optical beacon matching SyntheticFrameRenderer
    bgr = np.zeros((height, width, 3), dtype=np.uint8)
    bgr[:, :, 0] = (mono * (65.0 / 255.0)).astype(np.uint8)
    bgr[:, :, 1] = (mono * (225.0 / 255.0)).astype(np.uint8)
    bgr[:, :, 2] = (mono * (80.0 / 255.0)).astype(np.uint8)
    # Saturated emitter core reaches sensor saturation
    sat_mask = mono >= 240
    if np.any(sat_mask):
        bgr[sat_mask] = (180, 255, 190)

    r = int(round(sigma * 3.5))
    bbox = [max(0, int(round(cx - r))), max(0, int(round(cy - r))),
            min(width, int(round(cx + r))), min(height, int(round(cy + r)))]
    return bgr, bbox


def test_baseline_localizer_preserved():
    """Verify inherited baseline localizer computes exact integer bbox midpoint."""
    localizer = BaselineLocalizer()
    frame = np.zeros((480, 640, 3), dtype=np.uint8)
    bbox = [100, 150, 120, 180] # width=20, height=30
    res = localizer.localize(frame, bbox)

    assert res.success is True
    assert res.method == "baseline"
    assert res.x == 110.0 # 100 + 20/2
    assert res.y == 165.0 # 150 + 30/2
    assert "BASELINE_BBOX_MIDPOINT" in res.quality_flags


def test_exact_subpixel_synthetic_centers():
    """
    Verify subpixel localization error < 0.1 px on clean, well-sampled synthetic PSF frames.
    Tests specific fractional coordinates: (100.25, 150.75) and (320.50, 240.50).
    """
    wc = WeightedCentroidLocalizer(roi_margin=5)
    psf = PSFRefinedLocalizer(roi_margin=5)

    test_coords = [
        (100.25, 150.75),
        (220.10, 100.90),
        (320.50, 240.50),
        (500.33, 320.67)
    ]

    for tx, ty in test_coords:
        frame, bbox = _make_synthetic_spot(640, 480, tx, ty, amplitude=220.0, sigma=2.5, bg=16.0)

        # 1. Weighted Centroid
        res_wc = wc.localize(frame, bbox)
        assert res_wc.success is True
        err_wc = np.hypot(res_wc.x - tx, res_wc.y - ty)
        # Verify internal synthetic target: median/clean error < 0.1 px
        assert err_wc < 0.05, f"Weighted centroid error {err_wc:.4f}px exceeded 0.05px at ({tx}, {ty})"

        # 2. PSF Refined
        res_psf = psf.localize(frame, bbox)
        assert res_psf.success is True
        err_psf = np.hypot(res_psf.x - tx, res_psf.y - ty)
        assert err_psf < 0.02, f"PSF refined error {err_psf:.4f}px exceeded 0.02px at ({tx}, {ty})"


def test_weighted_centroid_mathematics():
    """
    Direct verification of the intensity-weighted centroid equation:
    w_i = max(I_i - I_bg, 0)
    x_c = sum(x_i * w_i) / sum(w_i)
    y_c = sum(y_i * w_i) / sum(w_i)
    """
    # 7x7 patch with background = 20.0
    patch = np.full((7, 7), 20.0, dtype=np.float64)
    # Put asymmetric weights above background at known coordinates
    # at (3, 3): +10.0, at (4, 3): +30.0 -> expected x_c = (3*10 + 4*30)/40 = 150/40 = 3.75
    patch[3, 3] += 10.0
    patch[3, 4] += 30.0

    bgr = np.repeat(patch[:, :, np.newaxis], 3, axis=2).astype(np.uint8)
    wc = WeightedCentroidLocalizer(roi_margin=0)
    res = wc.localize(bgr, [0, 0, 7, 7])

    assert res.success is True
    assert pytest.approx(res.x, abs=0.1) == 3.75
    assert pytest.approx(res.y, abs=0.1) == 3.0


def test_background_estimation():
    """Verify border-pixel median background estimation suppresses core spot and rejects hot pixels."""
    roi = np.full((21, 21), 25.0, dtype=np.float64)
    # Bright spot in center (should NOT bias border median)
    roi[8:13, 8:13] = 220.0
    # Single hot pixel on border
    roi[0, 5] = 255.0

    bg_level, bg_std = estimate_local_background(roi)
    assert pytest.approx(bg_level, abs=1.0) == 25.0
    assert bg_std >= 1.0


def test_zero_weight_fallback():
    """Verify localizer handles uniform/blank/zero signal gracefully without crashing."""
    frame = np.full((480, 640, 3), 30, dtype=np.uint8)
    bbox = [100, 100, 130, 130]

    wc = WeightedCentroidLocalizer()
    res = wc.localize(frame, bbox)

    assert res.success is False
    assert "ZERO_NET_WEIGHT_FALLBACK" in res.quality_flags
    # Must return bbox midpoint as safe fallback
    assert res.x == 115.0
    assert res.y == 115.0
    assert res.localization_uncertainty_x_px >= 3.0


def test_edge_roi_handling():
    """Verify beacons near or touching sensor boundaries are flagged and return safe results."""
    frame = np.zeros((480, 640, 3), dtype=np.uint8)
    # Spot on extreme left boundary
    cv2.circle(frame, (2, 200), 8, (65, 225, 80), -1)
    bbox = [0, 192, 10, 208]

    wc = WeightedCentroidLocalizer(roi_margin=4)
    res = wc.localize(frame, bbox)

    assert "TOUCHING_SENSOR_EDGE" in res.quality_flags
    assert 0.0 <= res.x <= 640.0
    assert 0.0 <= res.y <= 480.0


def test_saturated_core_handling():
    """Verify saturated optical spots (>= 250 in 8-bit) are flagged."""
    frame, bbox = _make_synthetic_spot(640, 480, 200.0, 200.0, amplitude=600.0, sigma=3.5, bg=20.0)

    wc = WeightedCentroidLocalizer()
    res_wc = wc.localize(frame, bbox)
    assert "SATURATED_CORE" in res_wc.quality_flags

    # PSF refiner should recognize saturation and gracefully fallback to weighted centroid
    psf = PSFRefinedLocalizer()
    res_psf = psf.localize(frame, bbox)
    assert res_psf.success is True
    assert "SATURATED_CORE" in res_psf.quality_flags
    assert "PSF_FALLBACK_CENTROID_PREFERRED" in res_psf.quality_flags or res_psf.method.startswith("psf")


def test_nan_inf_protection():
    """Verify localizer does not crash on empty, inverted, or invalid bounding boxes."""
    frame = np.zeros((480, 640, 3), dtype=np.uint8)
    wc = WeightedCentroidLocalizer()

    # Empty frame
    res1 = wc.localize(np.zeros((0, 0, 3), dtype=np.uint8), [10, 10, 20, 20])
    assert res1.success is False

    # Inverted bbox
    res2 = wc.localize(frame, [50, 50, 30, 30])
    assert res2.success is False


def test_uncertainty_bounds_and_qualitative_monotonicity():
    """
    Verify localization uncertainty proxy:
    1. Stays within safe finite bounds [0.015, 10.0] px.
    2. Clean bright PSF has lower uncertainty than noisy, blurred, or faded spot.
    """
    wc = WeightedCentroidLocalizer()

    # 1. Clean spot
    frame_clean, bbox_clean = _make_synthetic_spot(640, 480, 200.0, 200.0, amplitude=220.0, sigma=2.5, bg=16.0)
    res_clean = wc.localize(frame_clean, bbox_clean)

    # 2. Degraded spot (faded + noisy)
    frame_degraded, bbox_degraded = _make_synthetic_spot(640, 480, 200.0, 200.0, amplitude=35.0, sigma=5.0, bg=30.0)
    noise = np.random.RandomState(42).normal(0.0, 12.0, frame_degraded.shape).astype(np.int16)
    frame_degraded = np.clip(frame_degraded.astype(np.int16) + noise, 0, 255).astype(np.uint8)
    res_degraded = wc.localize(frame_degraded, bbox_degraded)

    assert 0.01 <= res_clean.localization_uncertainty_x_px <= 5.0
    assert 0.01 <= res_degraded.localization_uncertainty_x_px <= 10.0
    # Monotonic behavior: clean uncertainty must be strictly smaller than degraded uncertainty
    assert res_clean.localization_uncertainty_x_px < res_degraded.localization_uncertainty_x_px


def test_truth_non_interference():
    """Verify that the localization module contains zero dependencies on ground truth."""
    import inspect
    import app.detection.localization as loc_mod

    # Check function and method signatures in localization module
    for name, obj in inspect.getmembers(loc_mod):
        if inspect.isclass(obj) and issubclass(obj, BaseLocalizer):
            sig = inspect.signature(obj.localize)
            params = list(sig.parameters.keys())
            for forbidden in ["truth", "gt", "ground_truth", "target_world", "beacon_pos"]:
                assert forbidden not in params, f"Forbidden truth parameter '{forbidden}' found in {name}.localize!"


def test_mode_switching_and_detector_integration():
    """Verify dynamic mode switching in OpenCVDetector and LOCALIZER_REGISTRY."""
    assert "baseline" in LOCALIZER_REGISTRY
    assert "weighted_centroid" in LOCALIZER_REGISTRY
    assert "psf_refined" in LOCALIZER_REGISTRY

    det_base = OpenCVDetector(localization_mode="baseline")
    det_wc = OpenCVDetector(localization_mode="weighted_centroid")
    det_psf = OpenCVDetector(localization_mode="psf_refined")

    assert det_base.localizer.method == "baseline" if hasattr(det_base.localizer, "method") else True
    assert isinstance(det_base.localizer, BaselineLocalizer)
    assert isinstance(det_wc.localizer, WeightedCentroidLocalizer)
    assert isinstance(det_psf.localizer, PSFRefinedLocalizer)

    # Test detection with subpixel center and localization metadata attached
    frame, _ = _make_synthetic_spot(640, 480, 300.25, 200.75, amplitude=220.0, sigma=2.5)
    res = det_wc.detect(frame)
    assert res["detected"] is True
    assert "localization" in res["candidates"][0]
    loc_meta = res["candidates"][0]["localization"]
    assert loc_meta["method"] == "weighted_centroid"
    assert loc_meta["snr_proxy"] > 5.0
    assert pytest.approx(res["center"][0], abs=0.1) == 300.25
    assert pytest.approx(res["center"][1], abs=0.1) == 200.75
