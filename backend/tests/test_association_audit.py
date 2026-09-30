"""
Phase 3.5 Unit & Contract Tests: Association Safety, Coverage & Claim Consistency.

Verifies:
1. Distinction between CORRECT_ACCEPT, SAFE_REJECT, and WRONG_ACCEPT.
2. Safe rejection does NOT count as false lock.
3. Rapid recovery after ambiguity crossing.
4. Finite coast duration and timeout behavior.
5. Strict ground-truth firewall enforcement in association engine.
6. Deterministic association repeatability.
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
    confidence: float = 0.90,
    unc: float = 0.30,
    signal: float = 1000.0,
    snr: float = 15.0,
    shape_q: float = 0.90,
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
        area=45.0,
        peak_intensity=200.0,
        integrated_signal=signal,
        background_level=20.0,
        snr_proxy=snr,
        sigma_x_px=unc * 4.0,
        sigma_y_px=unc * 4.0,
        shape_quality=shape_q,
        quality_flags=[],
        timestamp=timestamp
    )


def test_three_outcome_classification_distinct():
    """
    Verify that CORRECT_ACCEPT, SAFE_REJECT, and WRONG_ACCEPT
    are distinct, mutually exclusive operational classifications.
    """
    engine = AssociationEngine(mode="robust_multifeature", ambiguity_margin=0.08)
    kf = KalmanFilter2D(q_noise=0.5, r_noise=1.0)
    kf.initialize(320.0, 240.0, 0.0)

    # 1. CORRECT_ACCEPT: Designated target in gate, alone
    target_cand = _make_candidate(0, 321.0, 240.0, confidence=0.90)
    res1 = engine.associate([target_cand], kf, 0.02)
    assert res1.accepted is True
    assert res1.status == "ACCEPTED"
    assert res1.selected_candidate.candidate_index == 0

    # 2. SAFE_REJECT: Ambiguous twin appears with near-identical cost
    twin_cand = _make_candidate(1, 321.5, 240.5, confidence=0.90)
    res2 = engine.associate([target_cand, twin_cand], kf, 0.04)
    assert res2.accepted is False
    assert res2.status == "AMBIGUOUS"
    assert res2.coasting_due_to_association_rejection is True
    assert res2.selected_candidate is None

    # 3. WRONG_ACCEPT: Legacy nearest-neighbor unconditionally accepts distractor
    legacy_engine = AssociationEngine(mode="legacy_simple")
    distractor_cand = _make_candidate(1, 400.0, 300.0, confidence=0.95)
    res3 = legacy_engine.associate([distractor_cand], kf, 0.06)
    assert res3.accepted is True
    assert res3.selected_candidate.candidate_index == 1  # Accepted distractor!


def test_safe_rejection_not_false_lock():
    """
    Verify that when association rejects an ambiguous candidate or out-of-gate distractor,
    it commands safe coasting without triggering false lock metrics.
    """
    engine = AssociationEngine(mode="robust_multifeature")
    kf = KalmanFilter2D(q_noise=0.5, r_noise=1.0)
    kf.initialize(200.0, 200.0, 0.0)

    # Distractor 50px away during blackout
    distractor = _make_candidate(0, 250.0, 200.0, confidence=0.95)
    res = engine.associate([distractor], kf, 0.02)

    assert res.accepted is False
    assert res.status == "OUTSIDE_GATE"
    assert res.selected_candidate is None
    # Must be marked as safe coasting rejection
    assert res.coasting_due_to_association_rejection is True


def test_recovery_after_ambiguity():
    """
    Verify that after candidates separate following an ambiguous crossing,
    the association engine immediately resumes tracking the true target with 0 delay.
    """
    engine = AssociationEngine(mode="robust_multifeature", ambiguity_margin=0.08)
    kf = KalmanFilter2D(q_noise=0.5, r_noise=1.0)
    kf.initialize(320.0, 240.0, 0.0)

    # Frame 1: Tracking target
    c_target_1 = _make_candidate(0, 322.0, 240.0, confidence=0.90)
    res1 = engine.associate([c_target_1], kf, 0.02)
    assert res1.accepted is True
    kf.update(np.array([c_target_1.x, c_target_1.y]), 0.02)

    # Frame 2: Crossing collision - ambiguous
    c_target_2 = _make_candidate(0, 324.0, 240.0, confidence=0.88)
    c_distractor_2 = _make_candidate(1, 324.2, 240.2, confidence=0.90)
    res2 = engine.associate([c_target_2, c_distractor_2], kf, 0.04)
    assert res2.status == "AMBIGUOUS"
    assert res2.accepted is False
    kf.predict_only(0.04)

    # Frame 3: Candidates separate - target remains near predicted (326, 240), distractor flies to (350, 280)
    c_target_3 = _make_candidate(0, 326.0, 240.0, confidence=0.90)
    c_distractor_3 = _make_candidate(1, 350.0, 280.0, confidence=0.90)
    res3 = engine.associate([c_target_3, c_distractor_3], kf, 0.06)
    assert res3.status == "ACCEPTED"
    assert res3.selected_candidate.candidate_index == 0


def test_finite_coast_duration_and_timeout():
    """
    Verify that consecutive coasting frames are tracked and do not coast forever without reset.
    """
    engine = AssociationEngine(mode="robust_multifeature")
    kf = KalmanFilter2D(q_noise=0.5, r_noise=1.0, max_prediction_seconds=0.10)
    kf.initialize(100.0, 100.0, 0.0)

    # Send 5 empty frames (0.10s)
    for f in range(5):
        t = (f + 1) * 0.02
        res = engine.associate([], kf, t)
        assert res.accepted is False
        assert res.status == "NO_CANDIDATE"
        kf.predict_only(t)

    assert engine.coasting_frames == 5

    # At t=0.12s, check timeout
    is_timeout = kf.check_occlusion_timeout(0.12)
    assert is_timeout is True
    assert kf.is_initialized is False


def test_truth_firewall_strictly_enforced_in_association():
    """
    Verify that CandidateMeasurement instances cannot accept ground truth attributes
    and AssociationEngine has zero truth dependencies.
    """
    cand = _make_candidate(0, 320.0, 240.0)
    assert not hasattr(cand, "truth_id")
    assert not hasattr(cand, "is_designated")
    assert not hasattr(cand, "world_position")

    engine = AssociationEngine()
    assert not hasattr(engine, "designated_target_id")
    assert not hasattr(engine, "truth_targets")


def test_association_determinism():
    """
    Verify that identical candidate sequences produce bit-for-bit identical association outputs.
    """
    def run_trial():
        engine = AssociationEngine(mode="robust_multifeature", ambiguity_margin=0.08)
        kf = KalmanFilter2D(q_noise=0.5, r_noise=1.0)
        kf.initialize(300.0, 200.0, 0.0)
        results = []
        for f in range(20):
            t = f * 0.02
            c1 = _make_candidate(0, 300.0 + f * 2.0, 200.0, confidence=0.88, timestamp=t)
            c2 = _make_candidate(1, 300.0 + f * 2.2, 200.0 + f * 0.5, confidence=0.85, timestamp=t)
            res = engine.associate([c1, c2], kf, t)
            results.append((res.status, res.selected_candidate_index, res.association_cost, res.association_margin))
            if res.accepted and res.selected_candidate:
                kf.update(np.array([res.selected_candidate.x, res.selected_candidate.y]), t)
            else:
                kf.predict_only(t)
        return results

    trial_1 = run_trial()
    trial_2 = run_trial()
    assert trial_1 == trial_2
