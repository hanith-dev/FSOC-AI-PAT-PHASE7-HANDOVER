"""
Intelligent PAT Lifecycle State Machine (Phase 5).
Explicitly distinguishes:
    SEARCH
    CANDIDATE
    ACQUIRE
    TRACK
    LOCK
    COAST
    REACQUIRE
    LOST

Decides:
    WHEN to trust measurements
    WHEN to coast using estimation
    WHEN prediction has become unreliable
    WHEN to attempt reacquisition
    WHEN to declare the track lost
WITHOUT using simulation ground truth.
"""
from dataclasses import dataclass, field
from enum import Enum
from typing import Dict, Any, Optional, Tuple, List
import time
import logging
import numpy as np

from app.tracking.search_pattern import SearchCommandGenerator, SearchPatternConfig

logger = logging.getLogger("FSOC-PatStateMachine")


class PATState(str, Enum):
    SEARCH = "SEARCH"
    CANDIDATE = "CANDIDATE"
    ACQUIRE = "ACQUIRE"
    TRACK = "TRACK"
    LOCK = "LOCK"
    COAST = "COAST"
    REACQUIRE = "REACQUIRE"
    LOST = "LOST"


class TransitionReason(str, Enum):
    INITIAL_SEARCH = "INITIAL_SEARCH"
    CANDIDATE_DETECTED = "CANDIDATE_DETECTED"
    CANDIDATE_CONFIRMED = "CANDIDATE_CONFIRMED"
    CANDIDATE_LOST = "CANDIDATE_LOST"
    ACQUISITION_CONFIRMED = "ACQUISITION_CONFIRMED"
    ACQUISITION_FAILED = "ACQUISITION_FAILED"
    LOCK_CRITERIA_MET = "LOCK_CRITERIA_MET"
    LOCK_DEGRADED_TO_TRACK = "LOCK_DEGRADED_TO_TRACK"
    MEASUREMENT_LOST = "MEASUREMENT_LOST"
    AMBIGUOUS_ASSOCIATION = "AMBIGUOUS_ASSOCIATION"
    GATE_REJECTION = "GATE_REJECTION"
    SHORT_COAST_RECOVERED = "SHORT_COAST_RECOVERED"
    COAST_TIMEOUT = "COAST_TIMEOUT"
    ESTIMATOR_DEGRADED = "ESTIMATOR_DEGRADED"
    ESTIMATOR_EXPIRED = "ESTIMATOR_EXPIRED"
    UNCERTAINTY_LIMIT_EXCEEDED = "UNCERTAINTY_LIMIT_EXCEEDED"
    LOCAL_REACQUIRE_SUCCESS = "LOCAL_REACQUIRE_SUCCESS"
    LOCAL_REACQUIRE_TIMEOUT = "LOCAL_REACQUIRE_TIMEOUT"
    TRACK_CONFIDENCE_EXHAUSTED = "TRACK_CONFIDENCE_EXHAUSTED"
    SEARCH_CANDIDATE_FOUND = "SEARCH_CANDIDATE_FOUND"
    FORCED_RESET = "FORCED_RESET"


@dataclass
class PATFSMConfig:
    # Confirmation thresholds
    candidate_confirmation_frames: int = 2     # Frames with valid detection to leave SEARCH -> CANDIDATE -> ACQUIRE
    acquire_confirmation_frames: int = 3       # Consecutive accepted updates to transition ACQUIRE -> TRACK
    reacquire_confirmation_frames: int = 2     # Consecutive accepted updates to exit REACQUIRE -> TRACK
    candidate_timeout_frames: int = 3          # Missed frames before dropping CANDIDATE back to SEARCH

    # Lock entry thresholds (strict)
    lock_angular_error_entry_deg: float = 0.15 # Max pointing error for LOCK entry
    lock_duration_entry_s: float = 0.40        # Sustained duration (s) within core error before LOCK
    lock_max_pos_sigma_px: float = 5.0         # Max position sigma (px) for LOCK entry
    lock_min_consecutive_accepted: int = 5     # Minimum consecutive accepted measurements for LOCK entry

    # Lock exit thresholds (hysteresis)
    lock_angular_error_exit_deg: float = 0.35  # Error threshold to drop LOCK -> TRACK (hysteresis)
    lock_max_pos_sigma_exit_px: float = 12.0   # Sigma threshold to drop LOCK -> TRACK

    # Coasting policy (derived from Phase 4.5 20-200ms CA6 prediction window)
    coast_timeout_s: float = 0.20              # Max trusted coast duration before escalating to REACQUIRE
    coast_max_pos_sigma_px: float = 25.0       # Max position uncertainty allowed in COAST before REACQUIRE

    # Reacquisition search policy
    search_config: SearchPatternConfig = field(default_factory=SearchPatternConfig)
    max_reacquire_attempts: int = 1


class PATLifecycleStateMachine:
    """
    Production-grade 8-state PAT Lifecycle State Machine.
    Decoupled strictly from truth; operates solely on observable metrics.
    """

    def __init__(self, config: PATFSMConfig = None):
        self.config = config or PATFSMConfig()
        self.search_generator = SearchCommandGenerator(self.config.search_config)
        self.reset()

    def reset(self):
        """Reset state machine to initial SEARCH state."""
        self.current_state: PATState = PATState.SEARCH
        self.previous_state: PATState = PATState.SEARCH
        self.state_entry_time: float = 0.0
        self.transition_reason: str = TransitionReason.INITIAL_SEARCH.value

        # Timers & Counters
        self.lock_start_time: Optional[float] = None
        self.coast_start_time: Optional[float] = None
        self.reacquire_start_time: Optional[float] = None
        self.lost_start_time: Optional[float] = None

        self.consecutive_accepted_measurements: int = 0
        self.consecutive_rejected_measurements: int = 0
        self.candidate_frame_count: int = 0
        self.reacquire_attempts: int = 0

        # Operational telemetry snapshot
        self.last_estimated_error_deg: float = 0.0
        self.last_pos_sigma_px: float = 0.0
        self.last_estimator_health: str = "EXPIRED"
        self.last_association_status: str = "NO_CANDIDATE"
        self.lock_eligible: bool = False

        self.search_offset_pan_deg: float = 0.0
        self.search_offset_tilt_deg: float = 0.0
        self.search_generator.reset()

    def transition_to(
        self,
        new_state: PATState,
        reason: TransitionReason,
        current_time: float,
        kf_pos_mean: Optional[np.ndarray] = None,
        kf_pos_cov: Optional[np.ndarray] = None,
        time_since_measurement: float = 0.0
    ):
        """Execute explicit legal transition between PAT states."""
        old_state = self.current_state
        self.previous_state = old_state
        self.current_state = new_state
        self.state_entry_time = float(current_time)
        self.transition_reason = reason.value

        # State entry side effects
        if new_state == PATState.LOCK:
            pass
        else:
            self.lock_start_time = None

        if new_state == PATState.COAST:
            self.coast_start_time = float(current_time)
        else:
            self.coast_start_time = None

        if new_state == PATState.REACQUIRE:
            self.reacquire_start_time = float(current_time)
            self.reacquire_attempts += 1
            self.search_generator.start(
                current_time,
                kf_pos_mean=kf_pos_mean,
                kf_pos_cov=kf_pos_cov,
                time_since_measurement=time_since_measurement
            )
        else:
            self.search_generator.reset()
            self.search_offset_pan_deg = 0.0
            self.search_offset_tilt_deg = 0.0

        if new_state == PATState.LOST:
            self.lost_start_time = float(current_time)

        if new_state in (PATState.SEARCH, PATState.LOST):
            self.consecutive_accepted_measurements = 0

        logger.info(f"PAT FSM Transition: {old_state.value} -> {new_state.value} (Reason: {self.transition_reason})")

    def update(
        self,
        association_status: str,
        measurement_accepted: bool,
        candidate_count: int,
        estimator_health: str,
        position_sigma_px: float,
        angular_error_deg: float,
        current_time: float,
        kf_pos_mean: Optional[np.ndarray] = None,
        kf_pos_cov: Optional[np.ndarray] = None,
        time_since_measurement: Optional[float] = None
    ) -> str:
        """
        Evaluate operational tracking state transitions.
        
        Inputs:
            association_status: e.g. "ACCEPTED", "NO_CANDIDATE", "OUTSIDE_GATE", "AMBIGUOUS"
            measurement_accepted: bool
            candidate_count: int
            estimator_health: "ACTIVE", "DEGRADED", "EXPIRED"
            position_sigma_px: float
            angular_error_deg: float (derived from estimator output and camera calibration)
            current_time: float
            kf_pos_mean: Optional[np.ndarray] ([x, y] in pixels)
            kf_pos_cov: Optional[np.ndarray] (2x2 position covariance)
            time_since_measurement: Optional[float] (s)
        """
        t = float(current_time)
        self.last_estimated_error_deg = float(angular_error_deg)
        self.last_pos_sigma_px = float(position_sigma_px)
        self.last_estimator_health = str(estimator_health)
        self.last_association_status = str(association_status)
        time_since_meas = float(time_since_measurement) if time_since_measurement is not None else 0.0

        # Update measurement counters
        if measurement_accepted:
            self.consecutive_accepted_measurements += 1
            self.consecutive_rejected_measurements = 0
        else:
            self.consecutive_rejected_measurements += 1
            self.consecutive_accepted_measurements = 0

        # Check lock eligibility (strict entry conditions)
        self.lock_eligible = bool(
            self.consecutive_accepted_measurements >= self.config.lock_min_consecutive_accepted and
            self.last_pos_sigma_px <= self.config.lock_max_pos_sigma_px and
            self.last_estimated_error_deg <= self.config.lock_angular_error_entry_deg and
            self.last_estimator_health == "ACTIVE"
        )

        # -------------------------------------------------------------
        # State Transition Logic
        # -------------------------------------------------------------
        if self.current_state == PATState.SEARCH:
            if candidate_count > 0 or measurement_accepted:
                self.candidate_frame_count += 1
                if self.candidate_frame_count >= self.config.candidate_confirmation_frames:
                    self.transition_to(PATState.CANDIDATE, TransitionReason.CANDIDATE_DETECTED, t)
            else:
                self.candidate_frame_count = 0

        elif self.current_state == PATState.CANDIDATE:
            if candidate_count == 0 and not measurement_accepted:
                self.candidate_frame_count = max(0, self.candidate_frame_count - 1)
                if self.candidate_frame_count == 0:
                    self.transition_to(PATState.SEARCH, TransitionReason.CANDIDATE_LOST, t)
            else:
                # Observable candidate evidence confirmed -> initialize track
                if measurement_accepted or self.candidate_frame_count >= self.config.candidate_confirmation_frames:
                    self.transition_to(PATState.ACQUIRE, TransitionReason.CANDIDATE_CONFIRMED, t)

        elif self.current_state == PATState.ACQUIRE:
            if self.last_estimator_health == "EXPIRED":
                self.transition_to(PATState.LOST, TransitionReason.ESTIMATOR_EXPIRED, t)
            elif not measurement_accepted:
                # Early loss during acquisition: don't coast long, escalate to REACQUIRE or LOST
                if self.consecutive_rejected_measurements >= 2:
                    self.transition_to(
                        PATState.REACQUIRE,
                        TransitionReason.MEASUREMENT_LOST,
                        t,
                        kf_pos_mean=kf_pos_mean,
                        kf_pos_cov=kf_pos_cov,
                        time_since_measurement=time_since_meas
                    )
            else:
                if self.consecutive_accepted_measurements >= self.config.acquire_confirmation_frames:
                    self.transition_to(PATState.TRACK, TransitionReason.ACQUISITION_CONFIRMED, t)

        elif self.current_state == PATState.TRACK:
            if not measurement_accepted:
                # Measurement missing/rejected: enter COAST if estimator is healthy
                if self.last_estimator_health in ("ACTIVE", "DEGRADED"):
                    self.transition_to(PATState.COAST, self._get_rejection_reason(association_status), t)
                else:
                    self.transition_to(
                        PATState.REACQUIRE,
                        TransitionReason.ESTIMATOR_EXPIRED,
                        t,
                        kf_pos_mean=kf_pos_mean,
                        kf_pos_cov=kf_pos_cov,
                        time_since_measurement=time_since_meas
                    )
            else:
                # High confidence lock entry check
                if self.lock_eligible:
                    if self.lock_start_time is None:
                        self.lock_start_time = t
                    elif (t - self.lock_start_time) >= self.config.lock_duration_entry_s:
                        self.transition_to(PATState.LOCK, TransitionReason.LOCK_CRITERIA_MET, t)
                else:
                    self.lock_start_time = None

        elif self.current_state == PATState.LOCK:
            if not measurement_accepted:
                # Single frame miss in LOCK enters COAST to prevent immediate track destruction
                if self.last_estimator_health in ("ACTIVE", "DEGRADED"):
                    self.transition_to(PATState.COAST, self._get_rejection_reason(association_status), t)
                else:
                    self.transition_to(
                        PATState.REACQUIRE,
                        TransitionReason.ESTIMATOR_EXPIRED,
                        t,
                        kf_pos_mean=kf_pos_mean,
                        kf_pos_cov=kf_pos_cov,
                        time_since_measurement=time_since_meas
                    )
            else:
                # Hysteresis check for LOCK exit back to TRACK
                if (self.last_estimated_error_deg > self.config.lock_angular_error_exit_deg or
                    self.last_pos_sigma_px > self.config.lock_max_pos_sigma_exit_px or
                    self.last_estimator_health != "ACTIVE"):
                    self.transition_to(PATState.TRACK, TransitionReason.LOCK_DEGRADED_TO_TRACK, t)

        elif self.current_state == PATState.COAST:
            coast_duration = t - (self.coast_start_time if self.coast_start_time is not None else t)

            if measurement_accepted:
                # Quick recovery from short coast!
                # If previously in LOCK, return directly to LOCK if error and covariance are within entry limits
                can_restore_lock = (
                    self.previous_state == PATState.LOCK and
                    self.last_estimated_error_deg <= self.config.lock_angular_error_entry_deg and
                    self.last_pos_sigma_px <= self.config.lock_max_pos_sigma_px and
                    self.last_estimator_health == "ACTIVE"
                )
                if can_restore_lock:
                    self.transition_to(PATState.LOCK, TransitionReason.SHORT_COAST_RECOVERED, t)
                else:
                    self.transition_to(PATState.TRACK, TransitionReason.SHORT_COAST_RECOVERED, t)
            else:
                # Evaluate coast termination conditions
                if self.last_estimator_health == "EXPIRED":
                    self.transition_to(
                        PATState.REACQUIRE,
                        TransitionReason.ESTIMATOR_EXPIRED,
                        t,
                        kf_pos_mean=kf_pos_mean,
                        kf_pos_cov=kf_pos_cov,
                        time_since_measurement=time_since_meas
                    )
                elif self.last_pos_sigma_px > self.config.coast_max_pos_sigma_px:
                    self.transition_to(
                        PATState.REACQUIRE,
                        TransitionReason.UNCERTAINTY_LIMIT_EXCEEDED,
                        t,
                        kf_pos_mean=kf_pos_mean,
                        kf_pos_cov=kf_pos_cov,
                        time_since_measurement=time_since_meas
                    )
                elif coast_duration >= self.config.coast_timeout_s:
                    self.transition_to(
                        PATState.REACQUIRE,
                        TransitionReason.COAST_TIMEOUT,
                        t,
                        kf_pos_mean=kf_pos_mean,
                        kf_pos_cov=kf_pos_cov,
                        time_since_measurement=time_since_meas
                    )

        elif self.current_state == PATState.REACQUIRE:
            # Active local search pattern generator
            d_pan, d_tilt, search_timed_out = self.search_generator.get_offset(
                t,
                kf_pos_mean=kf_pos_mean,
                kf_pos_cov=kf_pos_cov,
                time_since_measurement=time_since_measurement
            )
            self.search_offset_pan_deg = d_pan
            self.search_offset_tilt_deg = d_tilt

            if measurement_accepted and self.last_estimator_health in ("ACTIVE", "DEGRADED"):
                if self.consecutive_accepted_measurements >= self.config.reacquire_confirmation_frames:
                    self.transition_to(PATState.TRACK, TransitionReason.LOCAL_REACQUIRE_SUCCESS, t)
            elif search_timed_out or self.last_estimator_health == "EXPIRED":
                self.transition_to(PATState.LOST, TransitionReason.LOCAL_REACQUIRE_TIMEOUT, t)

        elif self.current_state == PATState.LOST:
            # Exhausted track confidence: clear track and return to global acquisition SEARCH
            self.transition_to(PATState.SEARCH, TransitionReason.TRACK_CONFIDENCE_EXHAUSTED, t)

        return self.current_state.value

    def _get_rejection_reason(self, status: str) -> TransitionReason:
        if status == "AMBIGUOUS":
            return TransitionReason.AMBIGUOUS_ASSOCIATION
        elif status == "OUTSIDE_GATE":
            return TransitionReason.GATE_REJECTION
        return TransitionReason.MEASUREMENT_LOST

    def force_lost(self, current_time: float = None):
        """External signal (e.g. from Kalman occlusion timeout) forcing track termination."""
        t = float(current_time if current_time is not None else time.time())
        if self.current_state not in (PATState.SEARCH, PATState.LOST):
            self.transition_to(PATState.LOST, TransitionReason.ESTIMATOR_EXPIRED, t)

    def get_telemetry(self, current_time: float) -> Dict[str, Any]:
        """Return full operational telemetry dictionary including Phase 7 search metrics."""
        t = float(current_time)
        state_dur = t - self.state_entry_time
        coast_dur = (t - self.coast_start_time) if self.coast_start_time is not None else 0.0
        reacq_dur = (t - self.reacquire_start_time) if self.reacquire_start_time is not None else 0.0
        cmd = self.search_generator.get_command() if self.current_state == PATState.REACQUIRE else None

        return {
            "pat_state": self.current_state.value,
            "previous_pat_state": self.previous_state.value,
            "state_duration_s": round(max(0.0, state_dur), 4),
            "transition_reason": self.transition_reason,
            "consecutive_accepted_measurements": self.consecutive_accepted_measurements,
            "consecutive_rejected_measurements": self.consecutive_rejected_measurements,
            "coast_duration_s": round(max(0.0, coast_dur), 4),
            "reacquisition_duration_s": round(max(0.0, reacq_dur), 4),
            "estimator_health": self.last_estimator_health,
            "position_sigma_px": round(self.last_pos_sigma_px, 3),
            "association_status": self.last_association_status,
            "lock_eligible": bool(self.lock_eligible),
            "search_pattern": self.config.search_config.pattern_type if self.current_state == PATState.REACQUIRE else "none",
            "search_offset_pan_deg": round(self.search_offset_pan_deg, 4),
            "search_offset_tilt_deg": round(self.search_offset_tilt_deg, 4),
            "reacquisition_attempt_count": self.reacquire_attempts,
            "search_stage": cmd.search_stage if cmd else "none",
            "search_confidence_region": cmd.confidence_region if cmd else "none",
            "search_mahalanobis_radius": round(cmd.mahalanobis_radius, 3) if cmd else 0.0,
            "search_point_index": cmd.point_index if cmd else 0,
            "search_angular_travel_deg": round(self.search_generator.total_angular_travel_deg, 4),
            "search_fov_clipped": bool(cmd.fov_clipped) if cmd else False
        }
