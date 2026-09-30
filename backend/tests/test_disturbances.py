import numpy as np
import pytest
from app.detection.disturbances import DisturbanceEngine

def test_disturbance_engine_initialization():
    engine = DisturbanceEngine()
    assert all(val == 0 for val in engine.intensities.values())

def test_illumination_variation():
    engine = DisturbanceEngine()
    img = np.ones((100, 100, 3), dtype=np.uint8) * 128
    
    # Enable illumination variation
    engine.set_intensity("illumination_variation", 100)
    out = engine.apply(img, timestamp=0.0)
    
    # Darker/degraded image should have different values
    assert not np.array_equal(img, out)

def test_image_noise():
    engine = DisturbanceEngine()
    img = np.ones((100, 100, 3), dtype=np.uint8) * 128
    
    engine.set_intensity("image_noise", 80)
    out = engine.apply(img, timestamp=0.0)
    
    assert not np.array_equal(img, out)

def test_blur():
    engine = DisturbanceEngine()
    img = np.zeros((100, 100, 3), dtype=np.uint8)
    img[40:60, 40:60, :] = 255 # draw a sharp square
    
    engine.set_intensity("blur", 100)
    out = engine.apply(img, timestamp=0.0)
    
    assert not np.array_equal(img, out)

def test_atmospheric_turbulence():
    engine = DisturbanceEngine()
    img = np.zeros((100, 100, 3), dtype=np.uint8)
    img[40:60, 40:60, :] = 255
    
    engine.set_intensity("atmospheric_turbulence", 100)
    out = engine.apply(img, timestamp=0.0)
    
    assert not np.array_equal(img, out)

def test_vibration_and_jitter():
    engine = DisturbanceEngine()
    img = np.zeros((100, 100, 3), dtype=np.uint8)
    img[40:60, 40:60, :] = 255
    
    # vibration
    engine.set_intensity("platform_vibration", 100)
    out_vib = engine.apply(img, timestamp=0.1)
    assert not np.array_equal(img, out_vib)
    
    # jitter
    engine.reset()
    engine.set_intensity("camera_jitter", 100)
    out_jit = None
    # Run multiple times to trigger the probabilistic shock jump
    for t in np.linspace(0.1, 2.0, 40):
        out_jit = engine.apply(img, timestamp=float(t))
        if not np.array_equal(img, out_jit):
            break
    assert out_jit is not None
    assert not np.array_equal(img, out_jit)

def test_reset():
    engine = DisturbanceEngine()
    engine.randomize()
    assert any(val > 0 for val in engine.intensities.values())
    
    engine.reset()
    assert all(val == 0 for val in engine.intensities.values())
