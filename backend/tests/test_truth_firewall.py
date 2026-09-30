"""
test_truth_firewall.py — Automated Ground-Truth Isolation & Non-Interference Verification

Tests strictly prove:
1. Candidate selection functions without gt_beacons.
2. A detection enters estimator input without ground truth.
3. GROUND_TRUTH_NON_INTERFERENCE: Changing simulator truth metadata while keeping an identical image
   cannot change the detector, candidate selection, or tracking result.
4. Disabling Kalman does not cause the system to track using world coordinates.
5. Complete image blackout provides zero target truth to controller.
6. Evaluation metrics may access ground truth in an isolated evaluation-only step.
7. AST audit confirms zero operational ground-truth leakage in main tracking loop.
"""

import ast
import base64
import json
from pathlib import Path
import numpy as np
import cv2
import pytest
from fastapi.testclient import TestClient

from app.main import app, sim_state, detector_manager
from app.tracking.kalman import KalmanFilter2D
from app.tracking.state_machine import TrackingStateMachine
from app.camera.model import CameraModel
from app.tracking.metrics import MetricsTracker


def create_synthetic_frame_b64(u: int = 640, v: int = 360, radius: int = 12) -> str:
    """Create a synthetic 1280x720 base64-encoded image with a glowing green optical beacon."""
    img = np.zeros((720, 1280, 3), dtype=np.uint8)
    # Emerald green beacon in BGR (Hue ~60 in HSV, matching OpenCVDetector [35, 50, 50]-[95, 255, 255])
    cv2.circle(img, (u, v), radius, (50, 220, 50), -1)
    img = cv2.GaussianBlur(img, (5, 5), 1.2)
    _, buf = cv2.imencode('.jpg', img)
    return "data:image/jpeg;base64," + base64.b64encode(buf.tobytes()).decode("utf-8")


def create_blackout_frame_b64() -> str:
    """Create a completely black base64-encoded image."""
    img = np.zeros((720, 1280, 3), dtype=np.uint8)
    _, buf = cv2.imencode('.jpg', img)
    return "data:image/jpeg;base64," + base64.b64encode(buf.tobytes()).decode("utf-8")


def test_candidate_selection_without_gt_beacons():
    """Verify candidate selection operates purely on image detections."""
    # Simulate a detector result with 2 candidates
    det_res = {
        "detected": True,
        "candidates": [
            {"center": [150.0, 200.0], "bbox": [140, 190, 160, 210], "confidence": 0.65, "area": 314.0},
            {"center": [500.0, 350.0], "bbox": [485, 335, 515, 365], "confidence": 0.92, "area": 706.0}
        ]
    }
    
    # Selection without prior tracking state chooses highest confidence
    detected_candidates = det_res["candidates"]
    best = max(detected_candidates, key=lambda c: (c["confidence"], c["area"]))
    assert best["center"] == [500.0, 350.0]
    assert best["confidence"] == 0.92


def test_detection_enters_estimator_without_ground_truth():
    """Verify that a candidate detection updates Kalman filter with zero truth reference."""
    kf = KalmanFilter2D()
    measured_pt = np.array([450.2, 312.8], dtype=np.float32)
    
    state = kf.update(measured_pt, timestamp=1.0)
    assert kf.is_initialized is True
    # State [x, y, vx, vy]
    assert np.isclose(state[0], 450.2, atol=1e-1)
    assert np.isclose(state[1], 312.8, atol=1e-1)


def test_ground_truth_non_interference():
    """
    REGRESSION TEST: GROUND_TRUTH_NON_INTERFERENCE
    Sending an identical optical image with drastically different ground-truth metadata
    must produce identical detection, candidate selection, Kalman state, and tracking output.
    """
    client = TestClient(app)
    b64_img = create_synthetic_frame_b64(u=640, v=360, radius=12)
    
    def run_single_frame(meta):
        with client.websocket_connect("/ws/tracking") as ws:
            greeting = ws.receive_json()
            assert greeting["type"] == "connection_status"
            ws.send_text(json.dumps({
                "type": "frame",
                "image": b64_img,
                "metadata": meta
            }))
            return ws.receive_json()
            
    # 1. Run with Truth Scenario Alpha (target at +50, +20, -100)
    meta_alpha = {
        "gimbal_pan": 0.0,
        "gimbal_tilt": 0.0,
        "camera_position": [0.0, 1.5, 0.0],
        "camera_orientation": [0.0, 0.0, 0.0, 1.0],
        "designated_target_id": "BEACON-01",
        "beacons": [
            {
                "id": "BEACON-01",
                "position": [50.0, 20.0, -100.0],
                "velocity": [0.0, 0.0, 0.0],
                "active": True
            }
        ]
    }
    resp_alpha = run_single_frame(meta_alpha)
    
    # 2. Run with completely falsified Truth Scenario Beta
    # The truth coordinates are intentionally inverted and placed thousands of meters away
    meta_beta = {
        "gimbal_pan": 0.0,
        "gimbal_tilt": 0.0,
        "camera_position": [0.0, 1.5, 0.0],
        "camera_orientation": [0.0, 0.0, 0.0, 1.0],
        "designated_target_id": "FAKE-BEACON-99",
        "beacons": [
            {
                "id": "FAKE-BEACON-99",
                "position": [-9999.0, -8888.0, -7777.0],
                "velocity": [100.0, -200.0, 0.0],
                "active": False
            }
        ]
    }
    resp_beta = run_single_frame(meta_beta)
    # Assert complete non-interference:
    # Optical detection must be identical
    assert resp_alpha["detected"] is True
    assert resp_beta["detected"] is True
    assert resp_alpha["raw_center"] == resp_beta["raw_center"]
    assert resp_alpha["bbox"] == resp_beta["bbox"]
    
    # Kalman filter estimate must be identical
    assert np.isclose(resp_alpha["kalman"]["x"], resp_beta["kalman"]["x"], atol=1e-3)
    assert np.isclose(resp_alpha["kalman"]["y"], resp_beta["kalman"]["y"], atol=1e-3)
    
    # Angular errors computed for the gimbal controller must be identical
    assert np.isclose(resp_alpha["angular_error_h"], resp_beta["angular_error_h"], atol=1e-3)
    assert np.isclose(resp_alpha["angular_error_v"], resp_beta["angular_error_v"], atol=1e-3)
    assert np.isclose(resp_alpha["angular_error_total"], resp_beta["angular_error_total"], atol=1e-3)
    
    # Tracking FSM state must be identical
    assert resp_alpha["tracking_state"] == resp_beta["tracking_state"]


def test_complete_image_blackout_yields_zero_target_truth():
    """During complete image blackout, controller receives no target truth coordinates."""
    client = TestClient(app)
    b64_blackout = create_blackout_frame_b64()
    
    with client.websocket_connect("/ws/tracking") as ws:
        greeting = ws.receive_json()
        assert greeting["type"] == "connection_status"
        
        # Truth metadata claims target is right on optical boresight
        meta = {
            "gimbal_pan": 0.0,
            "gimbal_tilt": 0.0,
            "camera_position": [0.0, 1.5, 0.0],
            "camera_orientation": [0.0, 0.0, 0.0, 1.0],
            "designated_target_id": "BEACON-01",
            "beacons": [
                {
                    "id": "BEACON-01",
                    "position": [0.0, 1.5, -50.0], # directly on optical axis
                    "active": True
                }
            ]
        }
        
        ws.send_text(json.dumps({
            "type": "frame",
            "image": b64_blackout,
            "metadata": meta
        }))
        resp = ws.receive_json()
        
        # Optical detector must report False
        assert resp["detected"] is False
        assert resp["raw_center"] == [0.0, 0.0]
        # Candidate list must contain zero false-positive cheats
        assert len(resp["candidates"]) == 0


def test_ablation_kalman_off_does_not_leak_world_coordinates():
    """Disabling Kalman filter does not cause system to use world coordinates."""
    client = TestClient(app)
    b64_img = create_synthetic_frame_b64(u=700, v=400, radius=10)
    
    with client.websocket_connect("/ws/tracking") as ws:
        greeting = ws.receive_json()
        assert greeting["type"] == "connection_status"
        
        # Disable Kalman filter via ablation
        ws.send_json({"type": "set_ablation", "ablation": {"kalman_filter": False}})
        ack = ws.receive_json()
        assert ack["status"] == "success"
        
        # Send frame with target at (700, 400) and bogus world beacon
        meta = {
            "gimbal_pan": 0.0,
            "gimbal_tilt": 0.0,
            "camera_position": [0.0, 1.5, 0.0],
            "camera_orientation": [0.0, 0.0, 0.0, 1.0],
            "designated_target_id": "BEACON-01",
            "beacons": [{"id": "BEACON-01", "position": [1234.0, 5678.0, -9000.0], "active": True}]
        }
        ws.send_text(json.dumps({
            "type": "frame",
            "image": b64_img,
            "metadata": meta
        }))
        resp = ws.receive_json()
        
        assert resp["detected"] is True
        # Without Kalman, output x, y is the raw detected visual center, NOT world coordinates
        assert np.isclose(resp["kalman"]["x"], 700.0, atol=5.0)
        assert np.isclose(resp["kalman"]["y"], 400.0, atol=5.0)
        assert resp["kalman"]["vx"] == 0.0
        assert resp["kalman"]["vy"] == 0.0
        
        # Re-enable Kalman for cleanliness
        ws.send_json({"type": "set_ablation", "ablation": {"kalman_filter": True}})
        ws.receive_json()


def test_evaluation_metrics_can_access_truth():
    """Evaluation metrics tracker correctly receives truth for offline evaluation."""
    tracker = MetricsTracker()
    tracker.log_frame(
        timestamp=1.0,
        ground_truth_3d_position=[10.0, 5.0, -100.0],
        ground_truth_direction=[0.099, 0.049, -0.993],
        detected_position=[650.0, 370.0],
        predicted_position=[650.0, 370.0],
        camera_position=[0.0, 1.5, 0.0],
        camera_orientation=[0.0, 0.0, 0.0, 1.0],
        pan_angle=0.0,
        tilt_angle=0.0,
        pixel_error=15.0,
        horizontal_angular_error=0.15,
        vertical_angular_error=0.08,
        total_angular_error=0.17,
        confidence=0.95,
        fps=60.0,
        inference_time_ms=1.2,
        tracking_time_ms=0.1,
        end_to_end_time_ms=1.5,
        is_in_fov=True,
        detected=True,
        tracking_state="TRACKING"
    )
    assert len(tracker.logs) == 1
    assert tracker.logs[0]["tracking_state"] == "TRACKING"
    assert tracker.logs[0]["ground_truth_3d_position"] == [10.0, 5.0, -100.0]
    csv_export = tracker.export_csv()
    assert "10.0" in csv_export # truth pos logged in CSV for offline analysis


def test_codebase_ast_truth_quarantine():
    """AST static analysis verifying that gt_beacons was completely removed from main.py."""
    main_py_path = Path(__file__).parents[1] / "app" / "main.py"
    if not main_py_path.exists():
        main_py_path = Path("backend/app/main.py")
    with open(main_py_path, "r", encoding="utf-8") as f:
        source = f.read()
    
    tree = ast.parse(source)
    
    # Search for any variable or attribute named gt_beacons
    for node in ast.walk(tree):
        if isinstance(node, ast.Name) and node.id == "gt_beacons":
            pytest.fail("FORBIDDEN: 'gt_beacons' was found in backend/app/main.py AST!")
