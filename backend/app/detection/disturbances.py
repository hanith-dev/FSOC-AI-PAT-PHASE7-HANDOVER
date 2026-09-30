import time
import logging
import numpy as np
import cv2
from typing import Dict, Any

logger = logging.getLogger("FSOC-Disturbances")

class DisturbanceEngine:
    def __init__(self, config: Dict[str, Any] = None):
        # Default intensities (0-100%)
        self.intensities = {
            "platform_vibration": 0,
            "camera_jitter": 0,
            "image_noise": 0,
            "blur": 0,
            "atmospheric_turbulence": 0,
            "illumination_variation": 0
        }
        
        # State variables for temporal continuity
        self.jitter_dx = 0.0
        self.jitter_dy = 0.0
        self.last_jitter_time = 0.0
        
        if config:
            self.load_from_config(config)

    def load_from_config(self, config: Dict[str, Any]):
        for key in self.intensities.keys():
            if key in config:
                val = config[key]
                if isinstance(val, dict):
                    self.intensities[key] = int(val.get("intensity", 0))
                else:
                    self.intensities[key] = int(val)
        logger.info(f"Loaded disturbances intensities: {self.intensities}")

    def reset(self):
        for key in self.intensities.keys():
            self.intensities[key] = 0
        self.jitter_dx = 0.0
        self.jitter_dy = 0.0
        logger.info("Reset all disturbances to 0")

    def randomize(self):
        for key in self.intensities.keys():
            self.intensities[key] = int(np.random.randint(10, 80)) # 10% to 80% range for random
        logger.info(f"Randomized disturbances intensities: {self.intensities}")

    def set_intensity(self, name: str, val: int):
        if name in self.intensities:
            self.intensities[name] = max(0, min(100, int(val)))
            logger.info(f"Disturbance {name} intensity set to {self.intensities[name]}")
        else:
            logger.warning(f"Unknown disturbance: {name}")

    def apply(self, img: np.ndarray, timestamp: float) -> np.ndarray:
        if img is None or img.size == 0:
            return img
            
        h, w = img.shape[:2]
        processed = img.copy()
        
        # 1. Illumination Variation (Brightness/Contrast degradation + temporal flicker)
        illum_intensity = self.intensities.get("illumination_variation", 0)
        if illum_intensity > 0:
            alpha_deg = 1.0 - 0.7 * (illum_intensity / 100.0) # Contrast scale (down to 30%)
            beta_deg = -40.0 * (illum_intensity / 100.0)      # Brightness offset
            # 3Hz sine wave illumination flicker
            flicker = 1.0 + 0.15 * (illum_intensity / 100.0) * np.sin(2.0 * np.pi * 3.0 * timestamp)
            alpha = max(0.1, alpha_deg * flicker)
            processed = cv2.convertScaleAbs(processed, alpha=alpha, beta=beta_deg)

        # 2. Platform Vibration (High-frequency harmonic jitter in X and Y)
        vib_intensity = self.intensities.get("platform_vibration", 0)
        if vib_intensity > 0:
            # Let amplitude be up to 10 pixels, frequency 15Hz
            amp = 10.0 * (vib_intensity / 100.0)
            freq = 15.0
            v_dx = amp * np.sin(2.0 * np.pi * freq * timestamp)
            v_dy = amp * np.cos(2.0 * np.pi * freq * timestamp)
            
            M = np.float32([[1, 0, v_dx], [0, 1, v_dy]])
            processed = cv2.warpAffine(processed, M, (w, h), borderMode=cv2.BORDER_REFLECT)

        # 3. Camera Jitter (Sudden, random coordinate displacement jumps with exponential decay)
        jit_intensity = self.intensities.get("camera_jitter", 0)
        if jit_intensity > 0:
            dt = timestamp - self.last_jitter_time
            if dt > 0.05: # limits update rate of jitter state
                self.last_jitter_time = timestamp
                # 10% chance of sudden shock jump
                if np.random.rand() < 0.15:
                    max_jump = 30.0 * (jit_intensity / 100.0)
                    self.jitter_dx = float(np.random.uniform(-max_jump, max_jump))
                    self.jitter_dy = float(np.random.uniform(-max_jump, max_jump))
                else:
                    # Exponential decay (spring-damper recovery simulation)
                    self.jitter_dx *= 0.75
                    self.jitter_dy *= 0.75
                    
            M = np.float32([[1, 0, self.jitter_dx], [0, 1, self.jitter_dy]])
            processed = cv2.warpAffine(processed, M, (w, h), borderMode=cv2.BORDER_REFLECT)

        # 4. Atmospheric Turbulence (Local coordinate warping + scintillation flicker + mild blur)
        turb_intensity = self.intensities.get("atmospheric_turbulence", 0)
        if turb_intensity > 0:
            # Scintillation (high-frequency intensity noise multiplier)
            scintillation = 1.0 + 0.12 * (turb_intensity / 100.0) * np.random.normal(0, 1)
            processed_f = processed.astype(np.float32) * max(0.1, scintillation)
            processed = np.clip(processed_f, 0, 255).astype(np.uint8)
            
            # Localized spatial wobble via remap coordinates
            amp = 8.0 * (turb_intensity / 100.0)
            grid_y, grid_x = np.indices((h, w), dtype=np.float32)
            # Spatial frequencies and time-varying phases
            dx = amp * np.sin(grid_y / 15.0 + 5.0 * timestamp)
            dy = amp * np.cos(grid_x / 15.0 + 4.0 * timestamp)
            
            processed = cv2.remap(processed, grid_x + dx, grid_y + dy, cv2.INTER_LINEAR, borderMode=cv2.BORDER_REFLECT)
            
            # Turbulence induced blur
            t_blur = int(5.0 * (turb_intensity / 100.0))
            t_blur = t_blur + 1 if t_blur % 2 == 0 else t_blur
            if t_blur >= 3:
                processed = cv2.GaussianBlur(processed, (t_blur, t_blur), 0)

        # 5. Blur (Gaussian blur + Motion blur)
        blur_intensity = self.intensities.get("blur", 0)
        if blur_intensity > 0:
            # Gaussian blur part
            g_blur = int(17.0 * (blur_intensity / 100.0))
            g_blur = g_blur + 1 if g_blur % 2 == 0 else g_blur
            if g_blur >= 3:
                processed = cv2.GaussianBlur(processed, (g_blur, g_blur), 0)
                
            # Motion blur part (horizontal smear)
            m_size = int(15.0 * (blur_intensity / 100.0)) + 1
            if m_size > 1:
                kernel = np.zeros((m_size, m_size))
                kernel[int((m_size - 1) / 2), :] = np.ones(m_size)
                kernel = kernel / m_size
                processed = cv2.filter2D(processed, -1, kernel)

        # 6. Image Noise (Additive Gaussian noise + Salt-and-Pepper noise)
        noise_intensity = self.intensities.get("image_noise", 0)
        if noise_intensity > 0:
            # Gaussian noise addition
            g_std = 35.0 * (noise_intensity / 100.0)
            if g_std > 0:
                noise = np.random.normal(0, g_std, processed.shape).astype(np.int16)
                processed = np.clip(processed.astype(np.int16) + noise, 0, 255).astype(np.uint8)
                
            # Salt-and-Pepper noise addition
            sp_prob = 0.06 * (noise_intensity / 100.0)
            if sp_prob > 0:
                # Random matrices for salt and pepper
                rand_matrix = np.random.rand(h, w)
                processed[rand_matrix < (sp_prob / 2.0)] = 255 # Salt
                processed[(rand_matrix >= (sp_prob / 2.0)) & (rand_matrix < sp_prob)] = 0 # Pepper
                
        return processed
