"""
Unit and Contract Test Suite for Phase 3 Robust Spatial-Temporal Beacon Association.

Verifies:
1. Mahalanobis distance calculation and numerical conditioning.
2. Chi-square statistical gate rejection (99% confidence, chi2_2 = 9.21).
3. Dynamic incorporation of Phase 2 localization uncertainty into candidate covariance.
4. Multi-feature cost normalization in [0, 1].
5. Ambiguity detection on crossing candidates and safe coasting.
6. Track continuity and switching hysteresis.
7. Initial multi-candidate acquisition temporal confirmation.
8. Dropout protection (rejecting out-of-gate distractor during blackout).
9. All-candidates-rejected safe coasting behavior.
10. Strict ground-truth non-interference.
11. Legacy simple mode preservation.
"""
import pytest
import numpy as np

from app.tracking.kalman import KalmanFilter2D
from app.tracking.association import (
    CandidateMeasurement,
    AssociationResult,
    AssociationEngine
)


def _make_candidate(
    idx: int,
    x: float,
    y: float,
    confidence: float = 0.9,
    unc: float = 0.5,
    signal: float = 1000.0,
    shape_q: float = 1.0,
    timestamp: float = 0.0
) -> CandidateMeasurement:
    return CandidateMeasurement(
        candidate_index=idx,
        x=float(x),
        y=float(y),
        localization_method="weighted_centroid",
        localization_uncertainty_x_px=unc,
        localization_uncertainty_y_px=unc,
        confidence=confidence,
        area=100.0,
        peak_intensity=220.0,
        integrated_signal=signal,
        background_level=20.0,
        snr_proxy=10.0,
        sigma_x_px=2.5,
        sigma_y_px=2.5,
        shape_quality=shape_q,
        quality_flags=[],
        timestamp=timestamp
    )


def test_mahalanobis_distance_and_conditioning():
    """Verify Mahalanobis distance calculation against analytical matrix algebra."""
    kf = KalmanFilter2D(q_noise=0.5, r_noise=1.0)
    kf.initialize(100.0, 100.0, 0.0)

    engine = AssociationEngine(mode="robust_multifeature")
    cand = _make_candidate(0, 105.0, 100.0, unc=1.0)

    # State is [100, 100, 0, 0], at dt=0.02, prediction is [100, 100]
    res = engine.associate([cand], kf, 0.02)
    assert res.status == "ACCEPTED"
    assert res.mahalanobis_distance is not None
    # Distance is approximately 5px / sqrt(sigma_p^2 + sigma_m^2)
    assert 1.0 < res.mahalanobis_distance < 4.0


def test_chi2_statistical_gate_rejection():
    """Verify that distant candidates outside chi-square gate (D^2 > 9.21) are strictly rejected."""
    kf = KalmanFilter2D(q_noise=0.5, r_noise=1.0)
    kf.initialize(100.0, 100.0, 0.0)

    engine = AssociationEngine(mode="robust_multifeature", chi2_gate_level=9.210)
    # Candidate 100 pixels away from prediction
    far_cand = _make_candidate(0, 200.0, 100.0, unc=0.5)

    res = engine.associate([far_cand], kf, 0.02)
    assert res.status == "OUTSIDE_GATE"
    assert res.selected_candidate is None
    assert res.selected_measurement is None
    assert res.coasting_due_to_association_rejection is True
    assert res.gated_candidate_count == 0


def test_uncertainty_incorporation_into_covariance():
    """Verify that larger localization uncertainty expands innovation covariance and lowers Mahalanobis D^2."""
    kf = KalmanFilter2D(q_noise=0.5, r_noise=1.0)
    kf.initialize(100.0, 100.0, 0.0)

    engine = AssociationEngine(mode="robust_multifeature")

    # High-precision candidate (unc = 0.2px)
    cand_sharp = _make_candidate(0, 103.0, 100.0, unc=0.2)
    res_sharp = engine.associate([cand_sharp], kf, 0.02)

    # Noisy/faded candidate at identical coordinate (unc = 4.0px)
    cand_noisy = _make_candidate(0, 103.0, 100.0, unc=4.0)
    res_noisy = engine.associate([cand_noisy], kf, 0.02)

    assert res_sharp.mahalanobis_distance > res_noisy.mahalanobis_distance


def test_multi_feature_score_normalization():
    """Verify that all cost components and total score are strictly bounded in [0, 1]."""
    kf = KalmanFilter2D(q_noise=0.5, r_noise=1.0)
    kf.initialize(100.0, 100.0, 0.0)

    engine = AssociationEngine(mode="robust_multifeature")
    cand = _make_candidate(0, 101.0, 101.0, confidence=0.8, signal=1000.0)

    res = engine.associate([cand], kf, 0.02)
    assert res.status == "ACCEPTED"
    assert 0.0 <= res.association_cost <= 1.0


def test_ambiguity_detection_on_crossing_targets():
    """
    Verify that two candidates with near-identical costs in the gate
    trigger AMBIGUOUS status and cause safe coasting.
    """
    kf = KalmanFilter2D(q_noise=0.5, r_noise=1.0)
    kf.initialize(100.0, 100.0, 0.0)

    engine = AssociationEngine(mode="robust_multifeature", ambiguity_margin=0.10)

    # Two symmetric candidates equidistant from predicted position (100, 100)
    cand1 = _make_candidate(0, 102.0, 100.0, confidence=0.9, signal=1000.0)
    cand2 = _make_candidate(1, 98.0, 100.0, confidence=0.9, signal=1000.0)

    res = engine.associate([cand1, cand2], kf, 0.02)
    assert res.status == "AMBIGUOUS"
    assert res.selected_candidate is None
    assert res.coasting_due_to_association_rejection is True
    assert res.association_margin < 0.10


def test_track_continuity_and_switching_hysteresis():
    """Verify that tracker rejects an unconfirmed competing distractor due to track hysteresis."""
    kf = KalmanFilter2D(q_noise=0.5, r_noise=1.0)
    kf.initialize(100.0, 100.0, 0.0)

    engine = AssociationEngine(mode="robust_multifeature", switch_hysteresis_margin=0.15)

    # Establish track over several frames
    for t in np.arange(0.02, 0.16, 0.02):
        c = _make_candidate(0, 100.0 + t*10, 100.0, signal=1000.0)
        res = engine.associate([c], kf, t)
        assert res.status == "ACCEPTED"
        kf.update(np.array([c.x, c.y]), t)

    # Now introduce a distractor nearby with slightly higher raw confidence
    target_cand = _make_candidate(0, 101.8, 100.0, confidence=0.85, signal=1000.0)
    distractor_cand = _make_candidate(1, 101.4, 98.0, confidence=0.95, signal=3000.0)

    res = engine.associate([target_cand, distractor_cand], kf, 0.18)
    assert res.status == "ACCEPTED"
    # Target must be maintained due to signal continuity and hysteresis
    assert res.selected_candidate.candidate_index == 0


def test_initial_acquisition_temporal_confirmation():
    """Verify that initial acquisition requires persistence when multiple candidates exist."""
    kf = KalmanFilter2D() # Uninitialized
    engine = AssociationEngine(mode="robust_multifeature", acquisition_confirmation_frames=2)

    cand1 = _make_candidate(0, 320.0, 240.0, confidence=0.7)
    cand2 = _make_candidate(1, 400.0, 200.0, confidence=0.7)

    # Frame 1: pending confirmation
    res1 = engine.associate([cand1, cand2], kf, 0.0)
    assert res1.status == "UNCONFIRMED_ACQUISITION"
    assert res1.selected_candidate is None

    # Frame 2: confirmed at persistent location
    res2 = engine.associate([cand1, cand2], kf, 0.02)
    assert res2.status == "ACCEPTED"
    assert res2.selected_candidate is not None


def test_dropout_protection_no_distractor_latching():
    """
    Critical test: When designated target drops out (blackout),
    the crossing distractor outside gate must NOT be captured.
    """
    kf = KalmanFilter2D(q_noise=0.5, r_noise=1.0)
    kf.initialize(200.0, 200.0, 0.0)
    kf.update(np.array([200.0, 200.0]), 0.0)

    engine = AssociationEngine(mode="robust_multifeature", chi2_gate_level=9.210)

    # Target is blacked out; only a distractor is detected 40px away
    distractor = _make_candidate(1, 240.0, 200.0, confidence=0.95, signal=4000.0)

    res = engine.associate([distractor], kf, 0.02)
    assert res.status == "OUTSIDE_GATE"
    assert res.selected_candidate is None
    assert res.coasting_due_to_association_rejection is True


def test_all_candidates_rejected_coasting():
    """Verify that when all candidates are rejected, tracker reports safe coasting."""
    kf = KalmanFilter2D(q_noise=0.5, r_noise=1.0)
    kf.initialize(100.0, 100.0, 0.0)

    engine = AssociationEngine(mode="robust_multifeature")
    cands = [
        _make_candidate(0, 250.0, 250.0),
        _make_candidate(1, 300.0, 300.0)
    ]

    res = engine.associate(cands, kf, 0.02)
    assert res.status == "OUTSIDE_GATE"
    assert res.selected_candidate is None
    assert res.coasting_due_to_association_rejection is True


def test_truth_non_interference_association():
    """Verify that AssociationEngine and CandidateMeasurement have zero ground truth dependencies."""
    import inspect
    import app.tracking.association as assoc_mod

    for name, obj in inspect.getmembers(assoc_mod):
        if inspect.isclass(obj):
            if hasattr(obj, "associate"):
                sig = inspect.signature(obj.associate)
                for forbidden in ["truth", "gt", "target_id", "ground_truth"]:
                    assert forbidden not in sig.parameters, f"Forbidden parameter '{forbidden}' in {name}.associate!"


def test_legacy_simple_preservation():
    """Verify that legacy_simple mode preserves Phase 0D nearest-neighbor behavior without gating."""
    kf = KalmanFilter2D(q_noise=0.5, r_noise=1.0)
    kf.initialize(100.0, 100.0, 0.0)

    engine = AssociationEngine(mode="legacy_simple")
    # Candidate 80 pixels away (would be rejected by chi2 gate)
    far_cand = _make_candidate(0, 180.0, 100.0)

    res = engine.associate([far_cand], kf, 0.02)
    assert res.status == "ACCEPTED"
    assert res.selected_candidate is not None
    assert res.selected_candidate.candidate_index == 0
