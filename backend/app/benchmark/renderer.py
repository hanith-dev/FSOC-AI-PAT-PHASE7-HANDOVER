"""
Headless Synthetic Camera Frame Generator.
Renders optical beacons directly to 640x480 BGR frames using the pinhole CameraModel.
"""
from typing import Dict, Any, List, Tuple, Optional
import numpy as np
import cv2

from app.camera.model import CameraModel


class SyntheticFrameRenderer:
    """
    Headless deterministic scene renderer for camera frame generation.
    Decoupled from Three.js and WebGL.
    """
    def __init__(
        self,
        width: int = 640,
        height: int = 480,
        hfov_deg: float = 10.0,
        vfov_deg: float = 7.5,
        camera_pos: np.ndarray = np.array([0.0, 1.5, 0.0])
    ):
        self.width = width
        self.height = height
        self.camera_pos = camera_pos
        self.camera_model = CameraModel(
            width=width,
            height=height,
            hfov_deg=hfov_deg,
            vfov_deg=vfov_deg,
            camera_pos=camera_pos
        )

    def render(
        self,
        beacons: List[Dict[str, Any]],
        gimbal_pan_deg: float,
        gimbal_tilt_deg: float
    ) -> Tuple[np.ndarray, List[Dict[str, Any]]]:
        """
        Render beacons into a 640x480 BGR frame.
        
        Args:
            beacons: List of beacon state dicts at current simulation epoch
            gimbal_pan_deg: Camera yaw angle in degrees
            gimbal_tilt_deg: Camera pitch angle in degrees
            
        Returns:
            Tuple of:
              - frame (np.ndarray): 640x480 uint8 BGR image
              - truth_projections (List[dict]): Evaluation-only ground-truth pixel & FOV projections
        """
        pan_rad = np.radians(gimbal_pan_deg)
        tilt_rad = np.radians(gimbal_tilt_deg)

        # Baseline dark ambient / sky background (deep charcoal space/night)
        frame = np.full((self.height, self.width, 3), (16, 16, 20), dtype=np.uint8)

        truth_projections = []

        for b in beacons:
            b_id = b.get("id", "BEACON-01")
            b_pos = np.array(b.get("position", [0.0, 5.0, -100.0]), dtype=np.float64)
            is_active = b.get("active", True)
            size = float(b.get("size", 1.5))
            intensity = float(b.get("intensity", 2.0))
            is_designated = b.get("designated", False)

            # Project 3D coordinate to camera frame and 2D pixel space
            target_camera = self.camera_model.world_to_camera(b_pos, pan_rad, tilt_rad)
            u, v, is_in_fov = self.camera_model.camera_to_pixel(target_camera)
            depth = -float(target_camera[2])

            is_optically_present = bool(is_active and is_in_fov and depth > 1.0)
            is_detectable = bool(is_optically_present and intensity > 0.1)

            truth_info = {
                "id": b_id,
                "world_position": b_pos.tolist(),
                "camera_position": target_camera.tolist(),
                "pixel_x": float(u),
                "pixel_y": float(v),
                "depth_m": depth,
                "is_in_fov": bool(is_in_fov),
                "is_active": bool(is_active),
                "is_optically_present": bool(is_optically_present),
                "is_detectable": bool(is_detectable),
                "designated": bool(is_designated)
            }
            truth_projections.append(truth_info)

            # Render photons onto sensor plane only if active, in front of camera, and near FOV
            if not is_active or depth <= 1.0:
                continue

            # Allow rendering if within margin of image bounds
            margin = 50
            if -margin <= u <= self.width + margin and -margin <= v <= self.height + margin:
                # Radius in pixels based on physical size and depth
                # At 100m with size 1.5, base radius is ~8px
                base_radius = max(3.0, (size * 500.0) / depth)
                spot_radius = base_radius * np.sqrt(max(0.5, intensity))
                
                # BGR color: default vibrant optical green #10B981 -> BGR (60, 220, 80)
                # Matches OpenCV detector HSV [35, 50, 50] to [95, 255, 255]
                color_hex = b.get("color", "#10B981")
                if color_hex == "#10B981" or "green" in str(color_hex).lower():
                    bgr_core = (65, 225, 80)
                    bgr_halo = (40, 160, 50)
                    bgr_bright = (180, 255, 190)
                elif color_hex == "#3B82F6" or "blue" in str(color_hex).lower():
                    bgr_core = (240, 130, 50)
                    bgr_halo = (180, 80, 20)
                    bgr_bright = (255, 210, 170)
                elif color_hex == "#F59E0B" or "yellow" in str(color_hex).lower() or "orange" in str(color_hex).lower():
                    bgr_core = (20, 160, 245)
                    bgr_halo = (10, 110, 180)
                    bgr_bright = (150, 220, 255)
                else:
                    # Default optical beacon green
                    bgr_core = (65, 225, 80)
                    bgr_halo = (40, 160, 50)
                    bgr_bright = (180, 255, 190)

                # If intensity is higher (e.g. S14 brighter distractor), scale core brightness
                if intensity > 3.0:
                    spot_radius *= 1.4

                pt = (int(round(u)), int(round(v)))
                r_int = int(max(3, round(spot_radius)))

                # 1. Outer soft halo (Gaussian glow)
                halo_radius = int(round(r_int * 2.0))
                cv2.circle(frame, pt, halo_radius, bgr_halo, -1, lineType=cv2.LINE_AA)

                # 2. Primary emitter core
                cv2.circle(frame, pt, r_int, bgr_core, -1, lineType=cv2.LINE_AA)

                # 3. Saturated emitter center
                inner_r = max(1, int(round(r_int * 0.4)))
                cv2.circle(frame, pt, inner_r, bgr_bright, -1, lineType=cv2.LINE_AA)

        return frame, truth_projections
