import time
import logging
from typing import Tuple, Optional, Dict, Any, List
import numpy as np

logger = logging.getLogger("FSOC-Kalman")

QUALITY_INFLATION_FACTORS = {
    "TOUCHING_SENSOR_EDGE": 2.0,
    "SATURATED_CORE": 1.5,
    "LOW_SNR": 1.5,
    "PSF_FALLBACK": 2.0
}


class KalmanFilter2D:
    """
    Unified 2D Kinematic Estimator for coarse Pointing, Acquisition, and Tracking (PAT).
    
    Supported Estimator Modes:
        - "cv4_fixed": 4-state Constant Velocity model with fixed measurement covariance (Phase 3.5 baseline).
        - "ca6_fixed": 6-state Constant Acceleration model with continuous white-noise jerk and fixed R.
        - "ca6_adaptive_r": 6-state Constant Acceleration model with Phase 2 localized uncertainty and quality flag inflation.
        - "ca6_adaptive_r_bounded_q": 6-state CA with adaptive R and bounded innovation-driven process noise.

    Canonical 6-State Vector:
        x = [x, y, vx, vy, ax, ay]^T
        Units:
            position: px
            velocity: px/s
            acceleration: px/s^2
            
    Canonical 4-State Vector (CV4 mode):
        x = [x, y, vx, vy]^T
    """

    def __init__(
        self,
        mode: str = "ca6_adaptive_r",
        q_noise: float = 0.5,
        q_jerk: float = 10000.0,
        r_noise: float = 1.0,
        max_prediction_seconds: float = 2.0,
        max_position_sigma_px: float = 50.0
    ):
        self.mode = mode
        self.q_noise = float(q_noise)          # Process noise scale for CV4
        self.q_jerk = float(q_jerk)            # Continuous white jerk spectral density (px^2 / s^5) for CA6
        self.r_noise = float(r_noise)          # Nominal measurement noise standard deviation (px)
        self.max_prediction_seconds = float(max_prediction_seconds)
        self.max_position_sigma_px = float(max_position_sigma_px)
        
        self.is_6state = self.mode in ("ca6_fixed", "ca6_adaptive_r", "ca6_adaptive_r_bounded_q")
        self.state_dim = 6 if self.is_6state else 4
        
        # Operational innovation statistics
        self.last_innovation = np.zeros(2, dtype=np.float32)
        self.last_innovation_covariance = np.eye(2, dtype=np.float32)
        self.last_nis = 0.0
        self.last_kalman_gain = np.zeros((self.state_dim, 2), dtype=np.float32)
        
        self.reset()

    def reset(self):
        self.state = np.zeros(self.state_dim, dtype=np.float32)
        if self.is_6state:
            # High initial uncertainty
            self.covariance = np.diag([10.0, 10.0, 2500.0, 2500.0, 10000.0, 10000.0]).astype(np.float32)
            self.H = np.array([
                [1, 0, 0, 0, 0, 0],
                [0, 1, 0, 0, 0, 0]
            ], dtype=np.float32)
        else:
            self.covariance = np.eye(4, dtype=np.float32) * 100.0
            self.H = np.array([
                [1, 0, 0, 0],
                [0, 1, 0, 0]
            ], dtype=np.float32)
            
        # Nominal measurement noise R
        self.R = np.eye(2, dtype=np.float32) * (self.r_noise ** 2)
        
        self.is_initialized = False
        self.last_update_time = 0.0
        self.last_detection_time = 0.0
        self.missed_detections_count = 0
        self.consecutive_updates = 0
        
        self.last_innovation = np.zeros(2, dtype=np.float32)
        self.last_innovation_covariance = np.eye(2, dtype=np.float32)
        self.last_nis = 0.0
        self.last_kalman_gain = np.zeros((self.state_dim, 2), dtype=np.float32)

    def initialize(self, x: float, y: float, timestamp: float):
        if self.is_6state:
            self.state = np.array([x, y, 0.0, 0.0, 0.0, 0.0], dtype=np.float32)
            # Initial covariance: position uncertain from R, velocity & acceleration high finite uncertainty
            self.covariance = np.diag([
                float(self.R[0, 0]), float(self.R[1, 1]),
                2500.0, 2500.0,
                10000.0, 10000.0
            ]).astype(np.float32)
        else:
            self.state = np.array([x, y, 0.0, 0.0], dtype=np.float32)
            self.covariance = np.diag([10.0, 10.0, 2500.0, 2500.0]).astype(np.float32)
            
        self.last_update_time = timestamp
        self.last_detection_time = timestamp
        self.is_initialized = True
        self.missed_detections_count = 0
        self.consecutive_updates = 1
        logger.info(f"Kalman filter ({self.mode}) initialized at position: [{x:.1f}, {y:.1f}]")

    def _get_F(self, dt: float) -> np.ndarray:
        """Construct state transition matrix F(dt)."""
        if self.is_6state:
            dt2_half = 0.5 * (dt ** 2)
            return np.array([
                [1.0, 0.0, dt,  0.0, dt2_half, 0.0],
                [0.0, 1.0, 0.0, dt,  0.0, dt2_half],
                [0.0, 0.0, 1.0, 0.0, dt,  0.0],
                [0.0, 0.0, 0.0, 1.0, 0.0, dt],
                [0.0, 0.0, 0.0, 0.0, 1.0, 0.0],
                [0.0, 0.0, 0.0, 0.0, 0.0, 1.0]
            ], dtype=np.float32)
        else:
            return np.array([
                [1.0, 0.0, dt,  0.0],
                [0.0, 1.0, 0.0, dt],
                [0.0, 0.0, 1.0, 0.0],
                [0.0, 0.0, 0.0, 1.0]
            ], dtype=np.float32)

    def _get_Q(self, dt: float) -> np.ndarray:
        """Construct process noise covariance Q(dt)."""
        if self.is_6state:
            # Continuous white jerk discretization for 1 axis: [p, v, a]
            # q11 = dt^5/20, q12 = dt^4/8, q13 = dt^3/6
            # q22 = dt^3/3,  q23 = dt^2/2, q33 = dt
            qj = self.q_jerk
            q11 = (dt ** 5) / 20.0
            q12 = (dt ** 4) / 8.0
            q13 = (dt ** 3) / 6.0
            q22 = (dt ** 3) / 3.0
            q23 = (dt ** 2) / 2.0
            q33 = dt

            Q = np.zeros((6, 6), dtype=np.float32)
            # x-axis block: indices (0, 2, 4)
            Q[0, 0] = q11 * qj
            Q[0, 2] = q12 * qj
            Q[0, 4] = q13 * qj
            Q[2, 0] = q12 * qj
            Q[2, 2] = q22 * qj
            Q[2, 4] = q23 * qj
            Q[4, 0] = q13 * qj
            Q[4, 2] = q23 * qj
            Q[4, 4] = q33 * qj

            # y-axis block: indices (1, 3, 5)
            Q[1, 1] = q11 * qj
            Q[1, 3] = q12 * qj
            Q[1, 5] = q13 * qj
            Q[3, 1] = q12 * qj
            Q[3, 3] = q22 * qj
            Q[3, 5] = q23 * qj
            Q[5, 1] = q13 * qj
            Q[5, 3] = q23 * qj
            Q[5, 5] = q33 * qj
            return Q
        else:
            q_scale = max(self.q_noise, 50.0)
            return np.array([
                [0.25 * (dt ** 4), 0.0, 0.5 * (dt ** 3), 0.0],
                [0.0, 0.25 * (dt ** 4), 0.0, 0.5 * (dt ** 3)],
                [0.5 * (dt ** 3), 0.0, dt ** 2, 0.0],
                [0.0, 0.5 * (dt ** 3), 0.0, dt ** 2]
            ], dtype=np.float32) * q_scale

    def _compute_R(
        self,
        R_custom: Optional[np.ndarray] = None,
        quality_flags: Optional[List[str]] = None
    ) -> np.ndarray:
        """Compute bounded measurement covariance with optional quality flag inflation."""
        if self.mode == "cv4_fixed" or self.mode == "ca6_fixed":
            return self.R.copy()

        # Adaptive R modes
        if R_custom is not None:
            R = np.array(R_custom, dtype=np.float32)
            diag_R = np.clip(np.diag(R), 0.04, 25.0)  # sigma in [0.2, 5.0] px
            R = np.diag(diag_R).astype(np.float32)
        else:
            R = self.R.copy()

        # Conservative inflation for observable quality conditions
        if quality_flags:
            factor = 1.0
            for flag in quality_flags:
                if flag in QUALITY_INFLATION_FACTORS:
                    factor *= QUALITY_INFLATION_FACTORS[flag]
            factor = min(4.0, factor)
            R = R * factor

        return R

    def get_prediction(self, timestamp: float) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        Calculate forward prediction at timestamp without modifying internal filter state.
        Returns:
            (x_pred, P_pred, z_pred)
        """
        if not self.is_initialized:
            return self.state.copy(), self.covariance.copy(), (self.H @ self.state).copy()

        dt = timestamp - self.last_update_time
        if dt <= 0:
            dt = 0.016

        F = self._get_F(dt)
        Q = self._get_Q(dt)

        x_pred = F @ self.state
        P_pred = (F @ self.covariance @ F.T) + Q
        z_pred = self.H @ x_pred
        return x_pred, P_pred, z_pred

    def predict_state(self, horizon_s: float) -> Tuple[np.ndarray, np.ndarray]:
        """
        Project state and covariance forward by horizon_s seconds without mutating internal state.
        Returns:
            (x_future, P_future)
        """
        if not self.is_initialized or horizon_s <= 0:
            return self.state.copy(), self.covariance.copy()

        F = self._get_F(horizon_s)
        Q = self._get_Q(horizon_s)
        x_future = F @ self.state
        P_future = (F @ self.covariance @ F.T) + Q
        return x_future, P_future

    def correct(
        self,
        z: np.ndarray,
        timestamp: float,
        R_custom: Optional[np.ndarray] = None,
        quality_flags: Optional[List[str]] = None
    ) -> np.ndarray:
        """
        Correct step: propagates state forward to timestamp (if dt > 0)
        and updates state with measurement z and adaptive covariance R.
        """
        if not self.is_initialized:
            self.initialize(float(z[0]), float(z[1]), timestamp)
            return self.state

        dt = timestamp - self.last_update_time

        # Determine measurement noise covariance R
        R = self._compute_R(R_custom=R_custom, quality_flags=quality_flags)

        # Two-stage initialization on second consecutive update:
        # Initialize velocity via finite difference; acceleration remains 0 with high variance
        if self.consecutive_updates == 1 and dt > 1e-4:
            vx = float((z[0] - self.state[0]) / dt)
            vy = float((z[1] - self.state[1]) / dt)
            var_r = float(max(R[0, 0], R[1, 1]))
            var_v = float(2.0 * var_r / (dt ** 2))
            cov_pv = float(var_r / dt)

            if self.is_6state:
                self.state = np.array([float(z[0]), float(z[1]), vx, vy, 0.0, 0.0], dtype=np.float32)
                self.covariance = np.array([
                    [R[0, 0], 0.0,      cov_pv, 0.0,    0.0,     0.0],
                    [0.0,      R[1, 1], 0.0,    cov_pv, 0.0,     0.0],
                    [cov_pv,   0.0,      var_v,  0.0,    0.0,     0.0],
                    [0.0,      cov_pv,   0.0,    var_v,  0.0,     0.0],
                    [0.0,      0.0,      0.0,    0.0,    10000.0, 0.0],
                    [0.0,      0.0,      0.0,    0.0,    0.0,     10000.0]
                ], dtype=np.float32)
            else:
                self.state = np.array([float(z[0]), float(z[1]), vx, vy], dtype=np.float32)
                self.covariance = np.array([
                    [R[0, 0], 0.0, cov_pv, 0.0],
                    [0.0, R[1, 1], 0.0, cov_pv],
                    [cov_pv, 0.0, var_v, 0.0],
                    [0.0, cov_pv, 0.0, var_v]
                ], dtype=np.float32)

            self.last_update_time = timestamp
            self.last_detection_time = timestamp
            self.missed_detections_count = 0
            self.consecutive_updates = 2
            return self.state

        # Prediction step
        if dt > 1e-5:
            self._predict_step(dt)
            self.last_update_time = timestamp

        # Innovation (measurement residual)
        z_meas = np.array(z, dtype=np.float32)
        y_residual = z_meas - (self.H @ self.state)
        self.last_innovation = y_residual.copy()

        # Innovation covariance S
        S = (self.H @ self.covariance @ self.H.T) + R
        self.last_innovation_covariance = S.copy()

        # Invert S with numerical conditioning safeguard
        try:
            S_inv = np.linalg.inv(S)
        except np.linalg.LinAlgError:
            S_inv = np.linalg.inv(S + 1e-4 * np.eye(2, dtype=np.float32))

        # Normalized Innovation Squared (NIS)
        self.last_nis = float(y_residual.T @ S_inv @ y_residual)

        # Kalman Gain K
        K = self.covariance @ self.H.T @ S_inv
        self.last_kalman_gain = K.copy()

        # Optional bounded innovation-driven Q adaptation for E4
        if self.mode == "ca6_adaptive_r_bounded_q" and self.last_nis > 9.21:
            # Slow bounded adjustment
            adapt_ratio = min(2.0, max(1.0, np.sqrt(self.last_nis / 9.21)))
            self.q_jerk = min(50000.0, self.q_jerk * adapt_ratio)

        # Update state and covariance (Joseph form for numerical symmetry)
        I_KH = np.eye(self.state_dim, dtype=np.float32) - (K @ self.H)
        self.state = self.state + (K @ y_residual)
        self.covariance = (I_KH @ self.covariance @ I_KH.T) + (K @ R @ K.T)

        self.last_detection_time = timestamp
        self.missed_detections_count = 0
        self.consecutive_updates += 1
        return self.state

    def update(
        self,
        z: np.ndarray,
        timestamp: float,
        R_custom: Optional[np.ndarray] = None,
        quality_flags: Optional[List[str]] = None
    ) -> np.ndarray:
        """Full step: predict then correct with new detection measurement z."""
        return self.correct(z, timestamp, R_custom=R_custom, quality_flags=quality_flags)

    def predict_only(self, timestamp: float) -> np.ndarray:
        """
        Predict step only (used when target is occluded, coasting, or rejected by association).
        Covariance grows naturally.
        """
        if not self.is_initialized:
            return self.state

        dt = timestamp - self.last_update_time
        if dt <= 0:
            dt = 0.016

        self._predict_step(dt)
        self.last_update_time = timestamp
        self.missed_detections_count += 1
        return self.state

    def _predict_step(self, dt: float):
        F = self._get_F(dt)
        Q = self._get_Q(dt)
        self.state = F @ self.state
        self.covariance = (F @ self.covariance @ F.T) + Q

    def get_uncertainty_ellipse(self, n_sigma: float = 1.0) -> Dict[str, float]:
        """
        Calculate error ellipse parameters for the 2D position covariance block.
        Returns:
            major_axis_px, minor_axis_px, orientation_deg
        """
        if not self.is_initialized:
            return {"major_axis_px": 0.0, "minor_axis_px": 0.0, "orientation_deg": 0.0}

        P_pos = self.covariance[0:2, 0:2]
        eigenvals, eigenvecs = np.linalg.eigh(P_pos)
        # Sort descending
        order = np.argsort(eigenvals)[::-1]
        eigenvals = np.maximum(0.0, eigenvals[order])
        eigenvecs = eigenvecs[:, order]

        major_axis = float(n_sigma * np.sqrt(eigenvals[0]))
        minor_axis = float(n_sigma * np.sqrt(eigenvals[1]))
        angle_rad = float(np.arctan2(eigenvecs[1, 0], eigenvecs[0, 0]))
        orientation_deg = float(np.degrees(angle_rad))

        return {
            "major_axis_px": round(major_axis, 3),
            "minor_axis_px": round(minor_axis, 3),
            "orientation_deg": round(orientation_deg, 2)
        }

    def get_health_status(self, current_time: float) -> str:
        """
        Evaluate estimator health contract:
            - ACTIVE: tracking nominal measurements, no coasting.
            - DEGRADED: coasting within allowable timeout or high position covariance.
            - EXPIRED: occlusion timeout exceeded or position uncertainty diverged.
        """
        if not self.is_initialized:
            return "EXPIRED"

        elapsed = current_time - self.last_detection_time
        pos_sigma = float(np.sqrt(max(0.0, self.covariance[0, 0] + self.covariance[1, 1])))

        if elapsed > self.max_prediction_seconds or pos_sigma > self.max_position_sigma_px:
            return "EXPIRED"
        elif self.missed_detections_count > 0 or pos_sigma > (self.max_position_sigma_px * 0.5):
            return "DEGRADED"
        return "ACTIVE"

    def check_occlusion_timeout(self, current_time: float) -> bool:
        """Returns True if the duration since the last valid detection exceeds the limit."""
        if not self.is_initialized:
            return False

        elapsed_seconds = current_time - self.last_detection_time
        pos_sigma = float(np.sqrt(max(0.0, self.covariance[0, 0] + self.covariance[1, 1])))
        if elapsed_seconds > self.max_prediction_seconds or pos_sigma > self.max_position_sigma_px:
            logger.warning(
                f"Kalman filter expired (elapsed={elapsed_seconds:.2f}s, pos_sigma={pos_sigma:.1f}px). Resetting."
            )
            self.reset()
            return True
        return False

    def get_state(self) -> dict:
        """Returns dictionary representation of state and health for telemetry."""
        if not self.is_initialized:
            return {
                "active": False,
                "mode": self.mode,
                "x": 0.0, "y": 0.0,
                "vx": 0.0, "vy": 0.0,
                "ax": 0.0, "ay": 0.0,
                "pos_sigma_px": 0.0,
                "health": "EXPIRED"
            }
        
        pos_sigma = float(np.sqrt(max(0.0, self.covariance[0, 0] + self.covariance[1, 1])))
        state_dict = {
            "active": True,
            "mode": self.mode,
            "x": float(self.state[0]),
            "y": float(self.state[1]),
            "vx": float(self.state[2]),
            "vy": float(self.state[3]),
            "ax": float(self.state[4]) if self.is_6state else 0.0,
            "ay": float(self.state[5]) if self.is_6state else 0.0,
            "pos_sigma_px": round(pos_sigma, 3),
            "nis": round(float(self.last_nis), 3),
            "health": self.get_health_status(self.last_update_time)
        }
        return state_dict
