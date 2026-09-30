"""
Phase 5 Unit & Contract Tests: Intelligent PAT Lifecycle, Coasting, and Basic Autonomous Reacquisition.

Verifies:
1. Operational transitions F01-F18 across SEARCH, CANDIDATE, ACQUIRE, TRACK, LOCK, COAST, REACQUIRE, LOST.
2. Candidate multi-frame temporal confirmation.
3. Acquisition confirmation from credible consecutive updates.
4. Lock criteria (angular error, pos_sigma, consecutive updates, duration) and hysteresis exit.
5. Single-frame miss dropping to COAST without destroying track.
6. Short dropout coast recovery vs coast timeout to REACQUIRE.
7. Estimator health (ACTIVE -> DEGRADED -> EXPIRED) driving state transitions.
8. Basic autonomous search generator (spiral and raster) determinism and offset bounds.
9. REACQUIRE timeout escalating to LOST -> SEARCH.
10. Strict truth firewall compliance (zero truth variables in state machine).
11. Preservation of legacy FSM for ablation.
"""
import pytest
import numpy as np

from app.tracking.pat_state_machine import (
    PATState,
    TransitionReason,
    PATFSMConfig,
    PATLifecycleStateMachine
)
from app.tracking.search_pattern import (
    SearchCommandGenerator,
    SearchPatternConfig
)
from app.tracking.state_machine import (
    TrackingStateMachine,
    LegacyTrackingStateMachine,
    TrackingState
)


def test_legacy_fsm_preserved():
    """Verify that LegacyTrackingStateMachine alias exists and behaves identically to baseline."""
    legacy = LegacyTrackingStateMachine(lock_threshold_deg=0.15, lock_duration_sec=0.5)
    assert legacy.current_state == TrackingState.SEARCHING
    st = legacy.update(detected=True, is_occluded=False, angular_error_deg=0.1, current_time=0.0)
    assert st == TrackingState.DETECTED


def test_truth_firewall_pat_fsm():
    """Verify PATLifecycleStateMachine and SearchCommandGenerator take zero ground-truth parameters."""
    import inspect
    import app.tracking.pat_state_machine as pat_mod
    import app.tracking.search_pattern as sp_mod

    for mod in [pat_mod, sp_mod]:
        for name, obj in inspect.getmembers(mod):
            if inspect.isclass(obj):
                for method_name in ["update", "transition_to", "get_offset", "start"]:
                    if hasattr(obj, method_name):
                        sig = inspect.signature(getattr(obj, method_name))
                        for forbidden in ["truth", "gt", "target_id", "ground_truth", "designated_target"]:
                            assert forbidden not in sig.parameters, f"Forbidden param '{forbidden}' in {name}.{method_name}!"


def test_f01_clean_acquisition_tracking_lock():
    """
    F01: Clean acquisition -> candidate confirmation -> acquire -> track -> lock.
    """
    fsm = PATLifecycleStateMachine()
    t = 0.0
    dt = 0.02

    assert fsm.current_state == PATState.SEARCH

    # Frame 1: Candidate detected (count=1, needed=2)
    s = fsm.update(
        association_status="ACCEPTED",
        measurement_accepted=True,
        candidate_count=1,
        estimator_health="ACTIVE",
        position_sigma_px=3.0,
        angular_error_deg=0.8,
        current_time=t
    )
    assert s == PATState.SEARCH.value

    # Frame 2: Candidate confirmed -> CANDIDATE
    t += dt
    s = fsm.update(
        association_status="ACCEPTED",
        measurement_accepted=True,
        candidate_count=1,
        estimator_health="ACTIVE",
        position_sigma_px=3.0,
        angular_error_deg=0.8,
        current_time=t
    )
    assert s == PATState.CANDIDATE.value

    # Frame 3: Measurement accepted with track initialized -> ACQUIRE
    t += dt
    s = fsm.update(
        association_status="ACCEPTED",
        measurement_accepted=True,
        candidate_count=1,
        estimator_health="ACTIVE",
        position_sigma_px=2.5,
        angular_error_deg=0.4,
        current_time=t
    )
    assert s == PATState.ACQUIRE.value

    # Frame 4 & 5: Stable consecutive accepted measurements -> TRACK (after 3 frames)
    for _ in range(2):
        t += dt
        s = fsm.update(
            association_status="ACCEPTED",
            measurement_accepted=True,
            candidate_count=1,
            estimator_health="ACTIVE",
            position_sigma_px=2.0,
            angular_error_deg=0.3,
            current_time=t
        )
    assert s == PATState.TRACK.value

    # Pointing error drops into core (<= 0.15 deg) with pos_sigma <= 5.0 px
    # Needs sustained duration 0.40s to lock
    for _ in range(25):  # 0.50s > 0.40s
        t += dt
        s = fsm.update(
            association_status="ACCEPTED",
            measurement_accepted=True,
            candidate_count=1,
            estimator_health="ACTIVE",
            position_sigma_px=1.5,
            angular_error_deg=0.10,
            current_time=t
        )
    assert s == PATState.LOCK.value


def test_f02_single_frame_detector_miss():
    """
    F02: In LOCK state, a single frame detector miss enters COAST and returns directly to LOCK.
    """
    fsm = PATLifecycleStateMachine()
    # Fast forward into LOCK
    fsm.current_state = PATState.LOCK
    fsm.consecutive_accepted_measurements = 10
    t = 1.0

    # Single miss
    s = fsm.update(
        association_status="NO_CANDIDATE",
        measurement_accepted=False,
        candidate_count=0,
        estimator_health="ACTIVE",
        position_sigma_px=2.0,
        angular_error_deg=0.10,
        current_time=t
    )
    assert s == PATState.COAST.value
    assert fsm.transition_reason == TransitionReason.MEASUREMENT_LOST.value

    # Next frame: measurement returns immediately
    t += 0.02
    s = fsm.update(
        association_status="ACCEPTED",
        measurement_accepted=True,
        candidate_count=1,
        estimator_health="ACTIVE",
        position_sigma_px=2.0,
        angular_error_deg=0.10,
        current_time=t
    )
    assert s == PATState.LOCK.value
    assert fsm.transition_reason == TransitionReason.SHORT_COAST_RECOVERED.value


def test_f03_f04_short_dropout_coast_and_recovery():
    """
    F03/F04: 100ms - 180ms dropout stays in COAST; recovers to TRACK/LOCK.
    """
    fsm = PATLifecycleStateMachine()
    fsm.current_state = PATState.TRACK
    fsm.consecutive_accepted_measurements = 5
    t = 1.0

    # Dropout starts at t=1.0s
    for step in range(8):  # 8 * 0.02 = 0.16s (< 0.20s coast limit)
        t += 0.02
        s = fsm.update(
            association_status="NO_CANDIDATE",
            measurement_accepted=False,
            candidate_count=0,
            estimator_health="DEGRADED",
            position_sigma_px=3.0 + step * 0.5,
            angular_error_deg=0.20,
            current_time=t
        )
        assert s == PATState.COAST.value

    # Target returns at t=1.18s
    t += 0.02
    s = fsm.update(
        association_status="ACCEPTED",
        measurement_accepted=True,
        candidate_count=1,
        estimator_health="ACTIVE",
        position_sigma_px=2.0,
        angular_error_deg=0.20,
        current_time=t
    )
    assert s == PATState.TRACK.value


def test_f05_coast_timeout_to_reacquire():
    """
    F05: Dropout exceeding coast_timeout_s (0.20s) escalates COAST -> REACQUIRE.
    """
    fsm = PATLifecycleStateMachine()
    fsm.current_state = PATState.TRACK
    t = 1.0

    # Coast for 0.22s (11 frames)
    for _ in range(11):
        t += 0.02
        s = fsm.update(
            association_status="NO_CANDIDATE",
            measurement_accepted=False,
            candidate_count=0,
            estimator_health="DEGRADED",
            position_sigma_px=8.0,
            angular_error_deg=0.25,
            current_time=t
        )

    assert s == PATState.REACQUIRE.value
    assert fsm.transition_reason == TransitionReason.COAST_TIMEOUT.value


def test_f06_blackout_and_reacquire_timeout_to_lost():
    """
    F06: Extended blackout (1.5s) -> COAST -> REACQUIRE -> timeout -> LOST -> SEARCH.
    """
    cfg = PATFSMConfig()
    cfg.search_config.max_duration_s = 0.50  # 0.5s local reacquire timeout
    fsm = PATLifecycleStateMachine(cfg)
    fsm.current_state = PATState.TRACK
    t = 1.0

    # 1. Coast timeout (0.20s)
    for _ in range(11):
        t += 0.02
        fsm.update(
            association_status="NO_CANDIDATE",
            measurement_accepted=False,
            candidate_count=0,
            estimator_health="DEGRADED",
            position_sigma_px=10.0,
            angular_error_deg=0.3,
            current_time=t
        )
    assert fsm.current_state == PATState.REACQUIRE

    # 2. Local search runs for 0.52s (> 0.50s max_duration_s)
    for _ in range(27):
        t += 0.02
        fsm.update(
            association_status="NO_CANDIDATE",
            measurement_accepted=False,
            candidate_count=0,
            estimator_health="DEGRADED",
            position_sigma_px=20.0,
            angular_error_deg=0.5,
            current_time=t
        )

    # Escalates to LOST, then next cycle clears to SEARCH
    assert fsm.previous_state == PATState.LOST or fsm.current_state in (PATState.LOST, PATState.SEARCH)
    # Give one step if in LOST
    if fsm.current_state == PATState.LOST:
        fsm.update("NO_CANDIDATE", False, 0, "EXPIRED", 0.0, 0.0, t + 0.02)
    assert fsm.current_state == PATState.SEARCH


def test_f07_f08_f09_distractor_gate_and_ambiguity_rejection():
    """
    F07/F08/F09: Association rejection (OUTSIDE_GATE / AMBIGUOUS) commands COAST, not lock corruption.
    """
    fsm = PATLifecycleStateMachine()
    fsm.current_state = PATState.TRACK
    t = 2.0

    # Distractor outside gate
    s = fsm.update(
        association_status="OUTSIDE_GATE",
        measurement_accepted=False,
        candidate_count=1,
        estimator_health="ACTIVE",
        position_sigma_px=2.5,
        angular_error_deg=0.15,
        current_time=t
    )
    assert s == PATState.COAST.value
    assert fsm.transition_reason == TransitionReason.GATE_REJECTION.value

    # Re-enter track
    fsm.current_state = PATState.TRACK
    t += 0.02
    # Ambiguous crossing
    s = fsm.update(
        association_status="AMBIGUOUS",
        measurement_accepted=False,
        candidate_count=2,
        estimator_health="ACTIVE",
        position_sigma_px=2.5,
        angular_error_deg=0.15,
        current_time=t
    )
    assert s == PATState.COAST.value
    assert fsm.transition_reason == TransitionReason.AMBIGUOUS_ASSOCIATION.value


def test_f14_f15_estimator_degraded_and_expired():
    """
    F14/F15: Estimator covariance growth and expiration trigger transition out of COAST.
    """
    fsm = PATLifecycleStateMachine()
    fsm.current_state = PATState.COAST
    fsm.coast_start_time = 1.0
    t = 1.04

    # High covariance exceeds safe limit
    s = fsm.update(
        association_status="NO_CANDIDATE",
        measurement_accepted=False,
        candidate_count=0,
        estimator_health="DEGRADED",
        position_sigma_px=30.0,  # exceeds coast_max_pos_sigma_px (25.0)
        angular_error_deg=0.5,
        current_time=t
    )
    assert s == PATState.REACQUIRE.value
    assert fsm.transition_reason == TransitionReason.UNCERTAINTY_LIMIT_EXCEEDED.value

    # Estimator expiration forces immediate termination
    fsm.current_state = PATState.COAST
    fsm.coast_start_time = 2.0
    t = 2.02
    s = fsm.update(
        association_status="NO_CANDIDATE",
        measurement_accepted=False,
        candidate_count=0,
        estimator_health="EXPIRED",
        position_sigma_px=60.0,
        angular_error_deg=1.5,
        current_time=t
    )
    assert s == PATState.REACQUIRE.value
    assert fsm.transition_reason == TransitionReason.ESTIMATOR_EXPIRED.value


def test_f17_successful_local_reacquisition():
    """
    F17: During REACQUIRE, detector picks up target and confirms with N frames -> TRACK.
    """
    fsm = PATLifecycleStateMachine()
    fsm.current_state = PATState.REACQUIRE
    fsm.reacquire_start_time = 1.0
    t = 1.2

    # Frame 1: candidate found (reacquire confirmation requires 2)
    s = fsm.update(
        association_status="ACCEPTED",
        measurement_accepted=True,
        candidate_count=1,
        estimator_health="ACTIVE",
        position_sigma_px=3.0,
        angular_error_deg=0.4,
        current_time=t
    )
    assert s == PATState.REACQUIRE.value

    # Frame 2: confirmed! -> TRACK
    t += 0.02
    s = fsm.update(
        association_status="ACCEPTED",
        measurement_accepted=True,
        candidate_count=1,
        estimator_health="ACTIVE",
        position_sigma_px=2.5,
        angular_error_deg=0.3,
        current_time=t
    )
    assert s == PATState.TRACK.value
    assert fsm.transition_reason == TransitionReason.LOCAL_REACQUIRE_SUCCESS.value


def test_lock_hysteresis():
    """
    Verify that lock entry requires <= 0.15 deg, but exit allows up to 0.35 deg before dropping to TRACK.
    """
    fsm = PATLifecycleStateMachine()
    fsm.current_state = PATState.LOCK
    fsm.consecutive_accepted_measurements = 10
    t = 1.0

    # Error increases to 0.25 deg (> 0.15 deg entry threshold, but <= 0.35 deg exit threshold)
    s = fsm.update(
        association_status="ACCEPTED",
        measurement_accepted=True,
        candidate_count=1,
        estimator_health="ACTIVE",
        position_sigma_px=3.0,
        angular_error_deg=0.25,
        current_time=t
    )
    assert s == PATState.LOCK.value  # Remains in LOCK due to hysteresis!

    # Error exceeds exit threshold (0.40 deg > 0.35 deg)
    t += 0.02
    s = fsm.update(
        association_status="ACCEPTED",
        measurement_accepted=True,
        candidate_count=1,
        estimator_health="ACTIVE",
        position_sigma_px=3.0,
        angular_error_deg=0.40,
        current_time=t
    )
    assert s == PATState.TRACK.value
    assert fsm.transition_reason == TransitionReason.LOCK_DEGRADED_TO_TRACK.value


def test_search_command_generator_determinism():
    """Verify that SearchCommandGenerator produces deterministic, bounded scan patterns."""
    cfg = SearchPatternConfig(
        pattern_type="spiral",
        angular_step_deg=0.25,
        scan_rate_deg_s=1.0,
        max_local_radius_deg=2.0,
        max_duration_s=2.0
    )
    gen1 = SearchCommandGenerator(cfg)
    gen2 = SearchCommandGenerator(cfg)

    gen1.start(0.0)
    gen2.start(0.0)

    for step in range(50):
        t = step * 0.04
        p1, t1, tout1 = gen1.get_offset(t)
        p2, t2, tout2 = gen2.get_offset(t)
        assert p1 == p2
        assert t1 == t2
        assert tout1 == tout2
        radius = np.sqrt(p1**2 + t1**2)
        assert radius <= cfg.max_local_radius_deg + 1e-5


def test_f10_abrupt_maneuver_rejections():
    """F10: Abrupt maneuver causes validation gate rejection -> COAST -> REACQUIRE."""
    fsm = PATLifecycleStateMachine()
    fsm.current_state = PATState.TRACK
    t = 1.0

    # Gate rejection due to abrupt target maneuver
    s = fsm.update(
        association_status="OUTSIDE_GATE",
        measurement_accepted=False,
        candidate_count=1,
        estimator_health="ACTIVE",
        position_sigma_px=4.0,
        angular_error_deg=0.35,
        current_time=t
    )
    assert s == PATState.COAST.value
    assert fsm.transition_reason == TransitionReason.GATE_REJECTION.value


def test_f11_f12_operational_fov_exit_and_reappearance():
    """F11/F12: Target exits FOV (measurement loss) -> COAST -> REACQUIRE -> reappears -> confirmed TRACK."""
    fsm = PATLifecycleStateMachine()
    fsm.current_state = PATState.TRACK
    t = 2.0

    # FOV exit causes consecutive misses
    for _ in range(12):  # > 0.20s coast limit
        t += 0.02
        fsm.update("NO_CANDIDATE", False, 0, "DEGRADED", 6.0, 0.4, t)
    assert fsm.current_state == PATState.REACQUIRE

    # Reappearance during REACQUIRE (2 consecutive confirmations needed)
    t += 0.02
    s1 = fsm.update("ACCEPTED", True, 1, "ACTIVE", 3.0, 0.25, t)
    assert s1 == PATState.REACQUIRE.value

    t += 0.02
    s2 = fsm.update("ACCEPTED", True, 1, "ACTIVE", 2.0, 0.20, t)
    assert s2 == PATState.TRACK.value
    assert fsm.transition_reason == TransitionReason.LOCAL_REACQUIRE_SUCCESS.value


def test_f13_repeated_short_dropouts():
    """F13: Repeated short dropouts (< 100ms) with intermittent measurements remain stable without chatter."""
    fsm = PATLifecycleStateMachine()
    fsm.current_state = PATState.TRACK
    t = 0.0

    for cycle in range(5):
        # 3 frames miss (60 ms)
        for _ in range(3):
            t += 0.02
            s_miss = fsm.update("NO_CANDIDATE", False, 0, "ACTIVE", 2.5, 0.15, t)
            assert s_miss == PATState.COAST.value

        # 5 frames valid measurement (100 ms)
        for _ in range(5):
            t += 0.02
            s_hit = fsm.update("ACCEPTED", True, 1, "ACTIVE", 2.0, 0.15, t)
            assert s_hit == PATState.TRACK.value


def test_f16_failed_local_reacquisition_to_lost_to_search():
    """F16: Failed local reacquisition times out to LOST, then clears to global SEARCH."""
    cfg = PATFSMConfig()
    cfg.search_config.max_duration_s = 0.20  # Fast timeout for test
    fsm = PATLifecycleStateMachine(cfg)
    fsm.transition_to(PATState.REACQUIRE, TransitionReason.COAST_TIMEOUT, 1.0)

    # Run past max duration (0.24s > 0.20s)
    t = 1.0
    for _ in range(12):
        t += 0.02
        fsm.update("NO_CANDIDATE", False, 0, "DEGRADED", 15.0, 0.8, t)

    assert fsm.previous_state == PATState.LOST or fsm.current_state in (PATState.LOST, PATState.SEARCH)
    # Next cycle clears LOST into SEARCH
    t += 0.02
    s_search = fsm.update("NO_CANDIDATE", False, 0, "EXPIRED", 20.0, 1.0, t)
    assert s_search == PATState.SEARCH.value


def test_f18_noise_intermittent_candidate_detection():
    """F18: Single intermittent candidate detection in SEARCH does not trigger premature ACQUIRE."""
    fsm = PATLifecycleStateMachine()
    assert fsm.current_state == PATState.SEARCH
    t = 0.0

    # 1 frame noise detection (needs 2 for confirmation)
    s = fsm.update("NO_CANDIDATE", False, 1, "EXPIRED", 0.0, 0.0, t)
    assert s == PATState.SEARCH.value

    # Next frame noise vanishes
    t += 0.02
    s = fsm.update("NO_CANDIDATE", False, 0, "EXPIRED", 0.0, 0.0, t)
    assert s == PATState.SEARCH.value
    assert fsm.candidate_frame_count == 0

