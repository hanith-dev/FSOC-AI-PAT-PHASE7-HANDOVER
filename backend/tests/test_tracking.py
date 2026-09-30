import pytest
import numpy as np
import time
from app.detection.detector import OpenCVDetector
from app.tracking.kalman import KalmanFilter2D
from app.tracking.state_machine import TrackingStateMachine, TrackingState

def test_opencv_detector_target_found():
    # Initialize OpenCV detector
    detector = OpenCVDetector(hsv_lower=[35, 50, 50], hsv_upper=[95, 255, 255])
    
    # Create black image (640x480)
    img = np.zeros((480, 640, 3), dtype=np.uint8)
    
    # Draw a green circle (beacon representation) at (300, 200)
    # Green in BGR is (0, 255, 0). That will convert to HSV Green.
    cv2_green = (120, 230, 180) # RGB is 180, 230, 120 (a bright green)
    # BGR format
    cv2_green_bgr = (120, 230, 16)
    cv2.circle(img, (300, 200), 10, cv2_green_bgr, -1) # radius 10, filled
    
    res = detector.detect(img)
    
    assert res["detected"] is True
    # Center should be roughly (300, 200)
    assert pytest.approx(res["center"][0], abs=1.5) == 300.0
    assert pytest.approx(res["center"][1], abs=1.5) == 200.0
    assert res["candidates_count"] == 1
    assert res["confidence"] > 0.0

def test_kalman_filter_constant_velocity():
    kf = KalmanFilter2D(q_noise=0.5, r_noise=1.0)
    
    # Initial state
    t0 = 100.0
    kf.initialize(100.0, 100.0, t0)
    
    assert kf.is_initialized
    assert kf.state[0] == 100.0
    assert kf.state[1] == 100.0
    
    # Run updates with constant velocity (+10 pixels/frame, dt=1.0s)
    kf.update(np.array([110.0, 110.0]), t0 + 1.0)
    kf.update(np.array([120.0, 120.0]), t0 + 2.0)
    
    # State should reflect positive velocity
    assert kf.state[2] > 0.0 # vx
    assert kf.state[3] > 0.0 # vy
    
    # Occlusion predict-only step
    # Target goes missing, we update time
    kf.predict_only(t0 + 3.0)
    
    # Position should predict forward along velocity vector
    assert kf.state[0] > 120.0
    assert kf.state[1] > 120.0
    assert kf.missed_detections_count == 1

def test_state_machine_flow():
    sm = TrackingStateMachine(lock_threshold_deg=0.15, lock_duration_sec=0.5)
    
    assert sm.current_state == TrackingState.SEARCHING
    
    # 1. Target detected far away (large error)
    state = sm.update(detected=True, is_occluded=False, angular_error_deg=1.5)
    assert state == TrackingState.DETECTED
    
    # Next step updates to ACQUIRING due to large error
    state = sm.update(detected=True, is_occluded=False, angular_error_deg=1.5)
    assert state == TrackingState.ACQUIRING
    
    # 2. Camera drives target closer (moderate error)
    state = sm.update(detected=True, is_occluded=False, angular_error_deg=0.3)
    assert state == TrackingState.TRACKING
    
    # 3. Align target under lock threshold
    state = sm.update(detected=True, is_occluded=False, angular_error_deg=0.1)
    assert state == TrackingState.TRACKING # Needs 0.5s duration to lock
    
    # Wait 0.6 seconds and update again
    time.sleep(0.6)
    state = sm.update(detected=True, is_occluded=False, angular_error_deg=0.1)
    assert state == TrackingState.LOCKED
    
    # 4. Target becomes occluded
    state = sm.update(detected=True, is_occluded=True, angular_error_deg=0.1)
    assert state == TrackingState.TARGET_LOST
    
    # 5. Target re-acquired
    state = sm.update(detected=True, is_occluded=False, angular_error_deg=0.1)
    assert state == TrackingState.TRACKING # lock resets and tracking starts again

# Need cv2 for drawing dummy image in tests
import cv2
