import time
import logging

logger = logging.getLogger("FSOC-StateMachine")

class TrackingState:
    SEARCHING = "SEARCHING"
    DETECTED = "DETECTED"
    ACQUIRING = "ACQUIRING"
    TRACKING = "TRACKING"
    LOCKED = "LOCKED"
    TARGET_LOST = "TARGET_LOST"

class TrackingStateMachine:
    def __init__(self, lock_threshold_deg: float = 0.15, lock_duration_sec: float = 1.0):
        self.lock_threshold = lock_threshold_deg
        self.lock_duration = lock_duration_sec
        
        self.current_state = TrackingState.SEARCHING
        self.lock_start_time = None
        self.pre_occlusion_state = TrackingState.SEARCHING
        self.state_entry_time = time.time()

    def update(self, detected: bool, is_occluded: bool, angular_error_deg: float, current_time: float = None) -> str:
        old_state = self.current_state
        if current_time is None:
            current_time = time.time()
        
        # 1. State transition rules
        if self.current_state == TrackingState.SEARCHING:
            if detected and not is_occluded:
                self.transition_to(TrackingState.DETECTED, current_time)
                
        elif self.current_state == TrackingState.DETECTED:
            if is_occluded:
                self.pre_occlusion_state = TrackingState.DETECTED
                self.transition_to(TrackingState.TARGET_LOST, current_time)
            elif not detected:
                self.transition_to(TrackingState.SEARCHING, current_time)
            else:
                # If target center is far, we are acquiring, otherwise tracking
                if angular_error_deg > 0.5:
                    self.transition_to(TrackingState.ACQUIRING, current_time)
                else:
                    self.transition_to(TrackingState.TRACKING, current_time)
                    
        elif self.current_state == TrackingState.ACQUIRING:
            if is_occluded:
                self.pre_occlusion_state = TrackingState.ACQUIRING
                self.transition_to(TrackingState.TARGET_LOST, current_time)
            elif not detected:
                self.transition_to(TrackingState.SEARCHING, current_time)
            elif angular_error_deg <= 0.5:
                self.transition_to(TrackingState.TRACKING, current_time)
                
        elif self.current_state == TrackingState.TRACKING:
            if is_occluded:
                self.pre_occlusion_state = TrackingState.TRACKING
                self.transition_to(TrackingState.TARGET_LOST, current_time)
            elif not detected:
                self.transition_to(TrackingState.SEARCHING, current_time)
            elif angular_error_deg > 0.5:
                self.transition_to(TrackingState.ACQUIRING, current_time)
            elif angular_error_deg <= self.lock_threshold:
                # Check lock duration condition
                if self.lock_start_time is None:
                    self.lock_start_time = current_time
                elif current_time - self.lock_start_time >= self.lock_duration:
                    self.transition_to(TrackingState.LOCKED, current_time)
            else:
                # Error is in (lock_threshold, 0.5] range, reset timer
                self.lock_start_time = None
                
        elif self.current_state == TrackingState.LOCKED:
            if is_occluded:
                self.pre_occlusion_state = TrackingState.LOCKED
                self.transition_to(TrackingState.TARGET_LOST, current_time)
            elif not detected:
                self.transition_to(TrackingState.SEARCHING, current_time)
            elif angular_error_deg > self.lock_threshold:
                self.lock_start_time = None
                if angular_error_deg > 0.5:
                    self.transition_to(TrackingState.ACQUIRING, current_time)
                else:
                    self.transition_to(TrackingState.TRACKING, current_time)
                    
        elif self.current_state == TrackingState.TARGET_LOST:
            if not is_occluded:
                if detected:
                    # Re-acquired! Go back to matching state
                    if angular_error_deg > 0.5:
                        self.transition_to(TrackingState.ACQUIRING, current_time)
                    else:
                        self.transition_to(TrackingState.TRACKING, current_time)
                else:
                    self.transition_to(TrackingState.SEARCHING, current_time)
            # prediction-only timeout is managed by the Kalman filter. If the filter resets,
            # we will set is_occluded = False, and transition back to SEARCHING.
            
        if self.current_state != old_state:
            logger.info(f"State Machine transition: {old_state} -> {self.current_state}")
            
        return self.current_state

    def force_lost(self, current_time: float = None):
        """Used when Kalman filter times out"""
        if self.current_state != TrackingState.SEARCHING:
            self.transition_to(TrackingState.SEARCHING, current_time)
            self.lock_start_time = None

    def transition_to(self, new_state: str, current_time: float = None):
        self.current_state = new_state
        self.state_entry_time = current_time if current_time is not None else time.time()
        if new_state != TrackingState.LOCKED:
            self.lock_start_time = None
            
    def get_status(self) -> dict:
        return {
            "state": self.current_state,
            "lock_time_elapsed": time.time() - self.lock_start_time if self.lock_start_time is not None else 0.0,
            "locked": self.current_state == TrackingState.LOCKED
        }


# Explicit preservation of legacy FSM for Phase 5 ablation
LegacyTrackingStateMachine = TrackingStateMachine

# Re-export Phase 5 PAT Lifecycle FSM
from app.tracking.pat_state_machine import (
    PATState,
    TransitionReason,
    PATFSMConfig,
    PATLifecycleStateMachine
)

