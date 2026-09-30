import time
import pytest
from app.tracking.metrics import MetricsTracker

def test_metrics_tracker_initialization():
    tracker = MetricsTracker()
    metrics = tracker.get_live_metrics()
    assert metrics["fps"]["avg"] == "N/A"
    assert metrics["acquisition_time"] == "N/A"
    assert metrics["tracking_error"]["avg"] == "N/A"

def test_metrics_fps_and_errors():
    tracker = MetricsTracker()
    
    # Log 3 frames with different metrics
    tracker.log_frame(
        timestamp=100.0,
        ground_truth_3d_position=[10.0, 5.0, -100.0],
        ground_truth_direction=[0.1, 0.05, -0.99],
        detected_position=[320.0, 240.0],
        predicted_position=[320.0, 240.0],
        camera_position=[0.0, 1.5, 0.0],
        camera_orientation=[0.0, 0.0, 0.0, 1.0],
        pan_angle=0.0,
        tilt_angle=0.0,
        pixel_error=0.0,
        horizontal_angular_error=0.1,
        vertical_angular_error=0.2,
        total_angular_error=0.2236,
        confidence=0.9,
        fps=15.0,
        inference_time_ms=10.0,
        tracking_time_ms=1.0,
        end_to_end_time_ms=12.0,
        is_in_fov=True,
        detected=True,
        tracking_state="TRACKING"
    )
    
    tracker.log_frame(
        timestamp=101.0,
        ground_truth_3d_position=[10.0, 5.0, -100.0],
        ground_truth_direction=[0.1, 0.05, -0.99],
        detected_position=[325.0, 242.0],
        predicted_position=[324.0, 241.0],
        camera_position=[0.0, 1.5, 0.0],
        camera_orientation=[0.0, 0.0, 0.0, 1.0],
        pan_angle=0.0,
        tilt_angle=0.0,
        pixel_error=5.38,
        horizontal_angular_error=0.2,
        vertical_angular_error=0.3,
        total_angular_error=0.3606,
        confidence=0.85,
        fps=25.0,
        inference_time_ms=12.0,
        tracking_time_ms=1.2,
        end_to_end_time_ms=15.0,
        is_in_fov=True,
        detected=True,
        tracking_state="TRACKING"
    )
    
    metrics = tracker.get_live_metrics()
    
    # FPS check: avg(15, 25) = 20
    assert metrics["fps"]["current"] == 25.0
    assert metrics["fps"]["avg"] == 20.0
    assert metrics["fps"]["min"] == 15.0
    assert metrics["fps"]["max"] == 25.0
    
    # Tracking error check: avg(0.2236, 0.3606) = 0.2921
    assert pytest.approx(metrics["tracking_error"]["avg"], abs=1e-3) == 0.292
    assert pytest.approx(metrics["tracking_error"]["max"], abs=1e-3) == 0.361

def test_metrics_detection_precision_recall():
    tracker = MetricsTracker()
    
    # TP: visible & detected
    tracker.log_frame(
        timestamp=100.0, ground_truth_3d_position=[10.0, 5.0, -100.0], ground_truth_direction=[0.1, 0.05, -0.99],
        detected_position=[320.0, 240.0], predicted_position=None, camera_position=[0.0, 1.5, 0.0],
        camera_orientation=[0.0, 0.0, 0.0, 1.0], pan_angle=0.0, tilt_angle=0.0, pixel_error=0.0,
        horizontal_angular_error=0.0, vertical_angular_error=0.0, total_angular_error=0.0, confidence=0.9,
        fps=15.0, inference_time_ms=10.0, tracking_time_ms=1.0, end_to_end_time_ms=12.0,
        is_in_fov=True, detected=True, tracking_state="TRACKING"
    )
    
    # FN: visible & NOT detected
    tracker.log_frame(
        timestamp=101.0, ground_truth_3d_position=[10.0, 5.0, -100.0], ground_truth_direction=[0.1, 0.05, -0.99],
        detected_position=None, predicted_position=None, camera_position=[0.0, 1.5, 0.0],
        camera_orientation=[0.0, 0.0, 0.0, 1.0], pan_angle=0.0, tilt_angle=0.0, pixel_error=None,
        horizontal_angular_error=0.0, vertical_angular_error=0.0, total_angular_error=0.0, confidence=0.0,
        fps=15.0, inference_time_ms=10.0, tracking_time_ms=1.0, end_to_end_time_ms=12.0,
        is_in_fov=True, detected=False, tracking_state="TRACKING"
    )
    
    # FP: NOT visible & detected
    tracker.log_frame(
        timestamp=102.0, ground_truth_3d_position=[100.0, 50.0, -100.0], ground_truth_direction=[0.1, 0.05, -0.99],
        detected_position=[10.0, 10.0], predicted_position=None, camera_position=[0.0, 1.5, 0.0],
        camera_orientation=[0.0, 0.0, 0.0, 1.0], pan_angle=0.0, tilt_angle=0.0, pixel_error=100.0,
        horizontal_angular_error=0.0, vertical_angular_error=0.0, total_angular_error=0.0, confidence=0.8,
        fps=15.0, inference_time_ms=10.0, tracking_time_ms=1.0, end_to_end_time_ms=12.0,
        is_in_fov=False, detected=True, tracking_state="TRACKING"
    )
    
    metrics = tracker.get_live_metrics()
    
    # TP=1, FN=1, FP=1, TN=0
    # Precision = TP / (TP+FP) = 1 / 2 = 0.5
    # Recall = TP / (TP+FN) = 1 / 2 = 0.5
    assert metrics["detection"]["precision"] == 0.5
    assert metrics["detection"]["recall"] == 0.5
    assert pytest.approx(metrics["detection"]["fp_rate"], abs=1e-3) == 0.333
    assert pytest.approx(metrics["detection"]["fn_rate"], abs=1e-3) == 0.333

def test_export_csv():
    tracker = MetricsTracker()
    tracker.log_frame(
        timestamp=100.0, ground_truth_3d_position=[10.0, 5.0, -100.0], ground_truth_direction=[0.1, 0.05, -0.99],
        detected_position=[320.0, 240.0], predicted_position=[320.0, 240.0], camera_position=[0.0, 1.5, 0.0],
        camera_orientation=[0.0, 0.0, 0.0, 1.0], pan_angle=0.0, tilt_angle=0.0, pixel_error=0.0,
        horizontal_angular_error=0.1, vertical_angular_error=0.2, total_angular_error=0.2236, confidence=0.9,
        fps=15.0, inference_time_ms=10.0, tracking_time_ms=1.0, end_to_end_time_ms=12.0,
        is_in_fov=True, detected=True, tracking_state="TRACKING"
    )
    csv_data = tracker.export_csv()
    assert "timestamp,gt_x,gt_y,gt_z" in csv_data
    assert "100.0,10.0,5.0,-100.0" in csv_data
