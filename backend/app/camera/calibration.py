"""
Authoritative Camera Calibration Contract for FSOC-AI-PAT.
Defines single source of truth for optical intrinsics, extrinsics, conventions,
and coordinate transformations between 3D world, camera reference frame, and 2D sensor pixels.
"""
from dataclasses import dataclass
from typing import Literal, Tuple
import numpy as np


@dataclass(frozen=True)
class CameraCalibration:
    """
    Authoritative pinhole camera calibration parameters.
    Default configuration models the SIH26169 FSOC optical receiver sensor.
    """
    resolution_width: int = 640
    resolution_height: int = 480

    # FOV convention: Three.js uses vertical FOV as authoritative
    fov_axis: Literal["vertical", "horizontal"] = "vertical"
    vfov_deg: float = 7.5
    hfov_deg: float = 9.988928  # Derived for square pixels: 2 * atan((W/H) * tan(VFOV/2))

    # Pinhole focal lengths (pixels)
    fx_px: float = 3661.723149
    fy_px: float = 3661.723149

    # Principal point (pixels)
    cx_px: float = 320.0
    cy_px: float = 240.0

    # Physical / Pixel properties
    pixel_aspect_ratio: float = 1.0  # 1.0 for square pixels (fx == fy)

    # Coordinate & Geometric Conventions
    coordinate_handedness: str = "right_handed"
    camera_forward_axis: str = "-Z"
    pixel_origin: str = "top_left"
    pixel_x_direction: str = "right"
    pixel_y_direction: str = "down"

    # Gimbal rotation signs
    pan_sign: str = "positive_clockwise_around_plus_Y"
    tilt_sign: str = "positive_upward_pitch_around_plus_X"

    @classmethod
    def from_vfov(
        cls,
        width: int = 640,
        height: int = 480,
        vfov_deg: float = 7.5
    ) -> "CameraCalibration":
        """
        Derive calibration where Vertical FOV (VFOV) is authoritative (Three.js convention).
        For square pixels:
            fy = (H / 2) / tan(VFOV / 2)
            fx = fy
            HFOV = 2 * atan((W / 2) / fx)
        """
        w = int(width)
        h = int(height)
        vfov_rad = np.radians(float(vfov_deg))

        fy = (h / 2.0) / np.tan(vfov_rad / 2.0)
        fx = fy  # Square pixels

        hfov_rad = 2.0 * np.arctan((w / 2.0) / fx)
        hfov_deg = float(np.degrees(hfov_rad))

        return cls(
            resolution_width=w,
            resolution_height=h,
            fov_axis="vertical",
            vfov_deg=float(vfov_deg),
            hfov_deg=hfov_deg,
            fx_px=float(fx),
            fy_px=float(fy),
            cx_px=w / 2.0,
            cy_px=h / 2.0,
            pixel_aspect_ratio=1.0
        )

    @classmethod
    def from_hfov(
        cls,
        width: int = 640,
        height: int = 480,
        hfov_deg: float = 10.0
    ) -> "CameraCalibration":
        """
        Derive calibration where Horizontal FOV (HFOV) is authoritative.
        For square pixels:
            fx = (W / 2) / tan(HFOV / 2)
            fy = fx
            VFOV = 2 * atan((H / 2) / fy)
        """
        w = int(width)
        h = int(height)
        hfov_rad = np.radians(float(hfov_deg))

        fx = (w / 2.0) / np.tan(hfov_rad / 2.0)
        fy = fx  # Square pixels

        vfov_rad = 2.0 * np.arctan((h / 2.0) / fy)
        vfov_deg = float(np.degrees(vfov_rad))

        return cls(
            resolution_width=w,
            resolution_height=h,
            fov_axis="horizontal",
            vfov_deg=vfov_deg,
            hfov_deg=float(hfov_deg),
            fx_px=float(fx),
            fy_px=float(fy),
            cx_px=w / 2.0,
            cy_px=h / 2.0,
            pixel_aspect_ratio=1.0
        )

    def pixel_to_angle(self, u: float, v: float) -> Tuple[float, float, float]:
        """
        Convert pixel coordinate (u, v) into optical angular deviation.
        Returns:
            (total_angle_deg, horizontal_angle_deg, vertical_angle_deg)
        where horizontal is positive rightward (+X) and vertical is positive upward (+Y).
        """
        # Normalized camera ray coordinates (Z = -1)
        x_norm = (float(u) - self.cx_px) / self.fx_px
        y_norm = (self.cy_px - float(v)) / self.fy_px  # Inverted because pixel v is down

        # Angular components
        h_angle_deg = float(np.degrees(np.arctan(x_norm)))
        v_angle_deg = float(np.degrees(np.arctan(y_norm)))

        # Total 3D angle from boresight [0, 0, -1]
        # ray = [x_norm, y_norm, -1.0]
        norm = np.sqrt(x_norm**2 + y_norm**2 + 1.0)
        tot_angle_deg = float(np.degrees(np.arccos(1.0 / norm)))

        return tot_angle_deg, h_angle_deg, v_angle_deg

    def angle_to_pixel(self, h_angle_deg: float, v_angle_deg: float) -> Tuple[float, float]:
        """
        Convert angular deviations (degrees) relative to boresight into pixel coordinate (u, v).
        """
        x_norm = np.tan(np.radians(float(h_angle_deg)))
        y_norm = np.tan(np.radians(float(v_angle_deg)))

        u = float(self.cx_px + self.fx_px * x_norm)
        v = float(self.cy_px - self.fy_px * y_norm)
        return u, v


# Authoritative default calibration instance
DEFAULT_CAMERA_CALIBRATION = CameraCalibration.from_vfov(
    width=640,
    height=480,
    vfov_deg=7.5
)
