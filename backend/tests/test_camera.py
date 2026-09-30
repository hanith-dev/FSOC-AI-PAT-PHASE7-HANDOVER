import pytest
import numpy as np
from app.camera.model import CameraModel

def test_camera_dead_center():
    # Camera at origin looking along -Z
    model = CameraModel(width=640, height=480, hfov_deg=10.0, vfov_deg=7.5)
    
    # Target directly in front at 100m
    target_world = np.array([0.0, 0.0, -100.0])
    pan = 0.0
    tilt = 0.0
    
    u, v, is_inside = model.camera_to_pixel(model.world_to_camera(target_world, pan, tilt))
    total_err, h_err, v_err = model.compute_angular_errors(target_world, pan, tilt)
    
    # Target should project to image center (320, 240)
    assert pytest.approx(u) == 320.0
    assert pytest.approx(v) == 240.0
    assert is_inside
    
    # Errors should be 0
    assert pytest.approx(total_err, abs=1e-5) == 0.0
    assert pytest.approx(h_err, abs=1e-5) == 0.0
    assert pytest.approx(v_err, abs=1e-5) == 0.0

def test_camera_horizontal_offset():
    # Camera at origin
    model = CameraModel(width=640, height=480, hfov_deg=10.0, vfov_deg=7.5)
    
    # Target shifted horizontally to the right
    # At 100m distance, 10m offset yields an angle of arctan(10/100) = 5.71059 degrees
    target_world = np.array([10.0, 0.0, -100.0])
    pan = 0.0
    tilt = 0.0
    
    u, v, is_inside = model.camera_to_pixel(model.world_to_camera(target_world, pan, tilt))
    total_err, h_err, v_err = model.compute_angular_errors(target_world, pan, tilt)
    
    # Target is to the right, so u > 320 (cx)
    assert u > 320.0
    assert pytest.approx(v) == 240.0
    
    # Check angles
    expected_angle = np.degrees(np.arctan(10.0 / 100.0))
    assert pytest.approx(total_err, abs=1e-3) == expected_angle
    assert pytest.approx(h_err, abs=1e-3) == expected_angle
    assert pytest.approx(v_err, abs=1e-3) == 0.0

def test_camera_vertical_offset():
    # Camera at origin
    model = CameraModel(width=640, height=480, hfov_deg=10.0, vfov_deg=7.5)
    
    # Target shifted vertically upwards (positive Y)
    target_world = np.array([0.0, 10.0, -100.0])
    pan = 0.0
    tilt = 0.0
    
    u, v, is_inside = model.camera_to_pixel(model.world_to_camera(target_world, pan, tilt))
    total_err, h_err, v_err = model.compute_angular_errors(target_world, pan, tilt)
    
    # Target is shifted up, so pixel coordinate v should be < 240 (cy) since pixel y-axis goes down
    assert v < 240.0
    assert pytest.approx(u) == 320.0
    
    expected_angle = np.degrees(np.arctan(10.0 / 100.0))
    assert pytest.approx(total_err, abs=1e-3) == expected_angle
    assert pytest.approx(h_err, abs=1e-3) == 0.0
    assert pytest.approx(v_err, abs=1e-3) == expected_angle

def test_camera_rotation_alignment():
    model = CameraModel(width=640, height=480, hfov_deg=10.0, vfov_deg=7.5)
    
    # Target at offset
    target_world = np.array([10.0, 0.0, -100.0])
    
    # To look directly at a target at +X (right), the camera must pan to the right
    # (negative pan direction in Three.js coordinates)
    pan = -np.arctan2(10.0, 100.0)
    tilt = 0.0
    
    u, v, is_inside = model.camera_to_pixel(model.world_to_camera(target_world, pan, tilt))
    total_err, h_err, v_err = model.compute_angular_errors(target_world, pan, tilt)
    
    # Target should now be at center of screen
    assert pytest.approx(u) == 320.0
    assert pytest.approx(v) == 240.0
    assert pytest.approx(total_err, abs=1e-3) == 0.0
