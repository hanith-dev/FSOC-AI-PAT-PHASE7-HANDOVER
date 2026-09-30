from typing import Optional, Tuple
import numpy as np
from app.camera.calibration import CameraCalibration, DEFAULT_CAMERA_CALIBRATION


class CameraModel:
    """
    Pinhole Camera and Coordinate Transformation Model for FSOC tracking.
    Conforms to the authoritative CameraCalibration contract.
    """
    def __init__(
        self,
        width: int = 640,
        height: int = 480,
        hfov_deg: float = 10.0,
        vfov_deg: float = 7.5,
        camera_pos: np.ndarray = np.array([0.0, 0.0, 0.0]),
        calibration: Optional[CameraCalibration] = None
    ):
        if calibration is not None:
            self.calibration = calibration
        else:
            # Three.js uses VFOV as authoritative; derive square pixel calibration
            self.calibration = CameraCalibration.from_vfov(
                width=width,
                height=height,
                vfov_deg=vfov_deg
            )

        self.width = self.calibration.resolution_width
        self.height = self.calibration.resolution_height
        self.fx = self.calibration.fx_px
        self.fy = self.calibration.fy_px
        self.cx = self.calibration.cx_px
        self.cy = self.calibration.cy_px
        self.hfov_deg = self.calibration.hfov_deg
        self.vfov_deg = self.calibration.vfov_deg
        self.hfov_rad = np.radians(self.hfov_deg)
        self.vfov_rad = np.radians(self.vfov_deg)
        self.camera_pos = np.array(camera_pos, dtype=np.float64)

    def get_camera_rotation_matrix(self, pan_rad: float, tilt_rad: float) -> np.ndarray:
        """
        Compute camera rotation matrix R_c from pan (yaw, around Y) and tilt (pitch, around X).
        In Three.js, rotation order is Y then X (first yaw, then pitch).
        """
        cy = np.cos(pan_rad)
        sy = np.sin(pan_rad)
        R_y = np.array([
            [cy, 0.0, sy],
            [0.0, 1.0, 0.0],
            [-sy, 0.0, cy]
        ], dtype=np.float64)

        cp = np.cos(tilt_rad)
        sp = np.sin(tilt_rad)
        R_x = np.array([
            [1.0, 0.0, 0.0],
            [0.0, cp, -sp],
            [0.0, sp, cp]
        ], dtype=np.float64)

        return R_y @ R_x

    def world_to_camera(self, target_world: np.ndarray, pan_rad: float, tilt_rad: float) -> np.ndarray:
        """
        Transform 3D world target position to camera coordinates.
        P_c = R_c.T @ (P_w - C_w)
        """
        R_c = self.get_camera_rotation_matrix(pan_rad, tilt_rad)
        t = np.array(target_world, dtype=np.float64) - self.camera_pos
        return R_c.T @ t

    def camera_to_pixel(self, target_camera: np.ndarray) -> Tuple[float, float, bool]:
        """
        Project 3D camera coordinates to 2D pixel coordinates.
        Camera looks down -Z, +X is right, +Y is up.
        Pixel coordinates have (0, 0) top-left, +u right, +v down.
        """
        x_c, y_c, z_c = target_camera

        # Target must be in front of the camera (z_c < 0)
        if z_c >= 0:
            return -1.0, -1.0, False

        depth = -z_c
        u = self.fx * (x_c / depth) + self.cx
        v = self.cy - self.fy * (y_c / depth)

        is_inside = (0.0 <= u <= float(self.width)) and (0.0 <= v <= float(self.height))
        return float(u), float(v), bool(is_inside)

    def pixel_to_angle(self, u: float, v: float) -> Tuple[float, float, float]:
        """
        Convert pixel coordinate (u, v) into optical angular error components (deg).
        """
        return self.calibration.pixel_to_angle(u, v)

    def threejs_view_matrix(self, pan_rad: float, tilt_rad: float) -> np.ndarray:
        """
        Build exact 4x4 View Matrix matching Three.js camera transformation.
        """
        R_c = self.get_camera_rotation_matrix(pan_rad, tilt_rad)
        V = np.eye(4, dtype=np.float64)
        V[0:3, 0:3] = R_c.T
        V[0:3, 3] = -R_c.T @ self.camera_pos
        return V

    def threejs_projection_matrix(self, near: float = 0.1, far: float = 10000.0) -> np.ndarray:
        """
        Build exact 4x4 Perspective Projection Matrix matching Three.js PerspectiveCamera.
        """
        M = np.zeros((4, 4), dtype=np.float64)
        M[0, 0] = (2.0 * self.fx) / self.width
        M[1, 1] = (2.0 * self.fy) / self.height
        M[2, 2] = -(far + near) / (far - near)
        M[2, 3] = -(2.0 * far * near) / (far - near)
        M[3, 2] = -1.0
        return M

    def project_threejs(
        self,
        target_world: np.ndarray,
        pan_rad: float,
        tilt_rad: float,
        near: float = 0.1,
        far: float = 10000.0
    ) -> Tuple[float, float, bool]:
        """
        Perform matrix-level 4x4 projection equivalent to Three.js WebGL pipeline:
        P_clip = M_proj @ V_view @ P_world
        ndc = P_clip[0:2] / P_clip[3]
        pixel = ndc to screen mapping.
        """
        p_w = np.array([target_world[0], target_world[1], target_world[2], 1.0], dtype=np.float64)
        V = self.threejs_view_matrix(pan_rad, tilt_rad)
        P = self.threejs_projection_matrix(near=near, far=far)

        p_clip = P @ (V @ p_w)
        w = p_clip[3]

        if w <= 0.0:
            return -1.0, -1.0, False

        ndc_x = p_clip[0] / w
        ndc_y = p_clip[1] / w

        u = (ndc_x + 1.0) * (self.width / 2.0)
        v = (1.0 - ndc_y) * (self.height / 2.0)

        is_inside = (0.0 <= u <= float(self.width)) and (0.0 <= v <= float(self.height))
        return float(u), float(v), bool(is_inside)

    def compute_angular_errors(
        self,
        target_world: np.ndarray,
        pan_rad: float,
        tilt_rad: float
    ) -> Tuple[float, float, float]:
        """
        Compute total angular error, signed horizontal error, and signed vertical error (deg).
        """
        target_camera = self.world_to_camera(target_world, pan_rad, tilt_rad)
        x_c, y_c, z_c = target_camera

        optical_axis = np.array([0.0, 0.0, -1.0])
        norm = np.linalg.norm(target_camera)
        target_dir = target_camera / norm if norm > 0 else optical_axis

        dot_product = np.clip(np.dot(target_dir, optical_axis), -1.0, 1.0)
        total_error_rad = np.arccos(dot_product)
        total_error_deg = float(np.degrees(total_error_rad))

        h_error_deg = float(np.degrees(np.arctan2(x_c, -z_c)))
        v_error_deg = float(np.degrees(np.arctan2(y_c, -z_c)))

        return total_error_deg, h_error_deg, v_error_deg
