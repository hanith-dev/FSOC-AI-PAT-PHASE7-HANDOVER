import os
import sys
import numpy as np
import base64
import json
import asyncio
import cv2

# Set path
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "backend")))

from app.tracking.pat_state_machine import PATLifecycleStateMachine, PATFSMConfig, PATState
from app.tracking.kalman import KalmanFilter2D

def test_fsm_logic():
    print("=== TESTING PAT LIFECYCLE FSM LOGIC ===")
    fsm = PATLifecycleStateMachine(config=PATFSMConfig())
    kf = KalmanFilter2D(mode="ca6_adaptive_r")

    print(f"Initial State: {fsm.current_state.value}")
    assert fsm.current_state == PATState.SEARCH

    # 1. Candidate detected (needs 2 frames to confirm CANDIDATE)
    t = 0.02
    s0 = fsm.update("ACCEPTED", True, 1, "EXPIRED", 999.0, 2.0, t)
    print(f"Frame 1 (Candidate seen once): {s0}")
    assert s0 == PATState.SEARCH
    
    t += 0.02
    s1 = fsm.update("ACCEPTED", True, 1, "EXPIRED", 999.0, 2.0, t)
    print(f"Frame 2 (Candidate confirmed): {s1}")
    assert s1 == PATState.CANDIDATE

    # 2. Candidate confirmed -> ACQUIRE
    t += 0.02
    center = np.array([320.0, 240.0], dtype=np.float32)
    kf.update(center, t)
    s2 = fsm.update("ACCEPTED", True, 1, "ACTIVE", 1.0, 1.2, t, kf_pos_mean=center, kf_pos_cov=kf.covariance[:2, :2])
    print(f"Frame 3 (Enter ACQUIRE): {s2}")
    assert s2 == PATState.ACQUIRE

    # 3. ACQUIRE confirmation (needs 3 accepted frames to transition to TRACK)
    for i in range(2):
        t += 0.02
        kf.update(center, t)
        s3 = fsm.update("ACCEPTED", True, 1, "ACTIVE", 1.0, 0.4, t, kf_pos_mean=center, kf_pos_cov=kf.covariance[:2, :2])
    print(f"Frame 5 (Enter TRACK): {s3}")
    assert s3 == PATState.TRACK

    # 4. Lock criteria met (sustained <= 0.15 deg for 0.4s)
    for i in range(25):
        t += 0.02
        kf.update(center, t)
        s4 = fsm.update("ACCEPTED", True, 1, "ACTIVE", 0.5, 0.05, t, kf_pos_mean=center, kf_pos_cov=kf.covariance[:2, :2], time_since_measurement=0.0)
    print(f"Frame 30 (Enter LOCK): {s4}")
    assert s4 == PATState.LOCK

    # 5. Target interrupted -> COAST
    t += 0.02
    kf.predict_only(t)
    s5 = fsm.update("NO_CANDIDATE", False, 0, "ACTIVE", 2.0, 0.05, t, kf_pos_mean=center, kf_pos_cov=kf.covariance[:2, :2], time_since_measurement=0.02)
    print(f"Step 5 (Target interrupted, enter COAST): {s5}")
    assert s5 == PATState.COAST

    # 6. Coast timeout -> REACQUIRE
    # Default coast_duration_sec is 0.5s -> 25 frames
    for i in range(30):
        t += 0.02
        kf.predict_only(t)
        s6 = fsm.update("NO_CANDIDATE", False, 0, "DEGRADED", 5.0, 0.05, t, kf_pos_mean=center, kf_pos_cov=kf.covariance[:2, :2], time_since_measurement=(i+2)*0.02)
    print(f"Step 6 (Coast timeout, enter REACQUIRE): {s6}")
    print(f"  Search generator pattern: {fsm.search_generator.config.pattern_type}")
    print(f"  Search generator active: {fsm.search_generator.active}")
    assert s6 == PATState.REACQUIRE
    assert fsm.search_generator.config.pattern_type == "covariance_ellipse_scan"
    assert fsm.search_generator.active is True

    # 7. Target reappears during reacquire (needs 2 frames to transition REACQUIRE -> TRACK)
    t += 0.02
    kf.update(center, t)
    s7_1 = fsm.update("ACCEPTED", True, 1, "ACTIVE", 1.0, 0.3, t, kf_pos_mean=center, kf_pos_cov=kf.covariance[:2, :2], time_since_measurement=0.0)
    print(f"Step 7.1 (First frame of re-detection): {s7_1}")
    assert s7_1 == PATState.REACQUIRE

    t += 0.02
    kf.update(center, t)
    s7_2 = fsm.update("ACCEPTED", True, 1, "ACTIVE", 1.0, 0.3, t, kf_pos_mean=center, kf_pos_cov=kf.covariance[:2, :2], time_since_measurement=0.0)
    print(f"Step 7.2 (Reacquisition confirmed, enter TRACK): {s7_2}")
    assert s7_2 == PATState.TRACK

    # 8. Re-enter LOCK
    for i in range(25):
        t += 0.02
        kf.update(center, t)
        s8 = fsm.update("ACCEPTED", True, 1, "ACTIVE", 0.5, 0.05, t, kf_pos_mean=center, kf_pos_cov=kf.covariance[:2, :2], time_since_measurement=0.0)
    print(f"Step 8 (Re-enter LOCK): {s8}")
    assert s8 == PATState.LOCK

    print("=== PAT LIFECYCLE FSM LOGIC VERIFIED SUCCESSFULLY ===")

if __name__ == "__main__":
    test_fsm_logic()
