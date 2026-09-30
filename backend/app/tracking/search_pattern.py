"""
Deterministic Autonomous Search Pattern Generator for PAT Lifecycle (Phases 5 & 7).
Produces scan offsets relative to the search center (pan/tilt in degrees).
Completely decoupled from Ground Truth: uses only configured scan geometry, scan rate,
operational Kalman filter state/covariance, camera calibration, and elapsed reacquisition time.

Supported Search Modes:
  - R0: fixed_spiral (Phase 5 baseline Archimedean spiral)
  - R1: covariance_ellipse_scan (Phase 7 uncertainty-aware 2D confidence ellipse search)
  - R2: probability_ordered_ellipse_scan (Discrete probability-ranked confidence region scan)
  - raster: bounded rectangular raster scan
"""
from dataclasses import dataclass, field
from typing import Literal, Tuple, Dict, Any, Optional, List
import numpy as np

from app.camera.calibration import DEFAULT_CAMERA_CALIBRATION


@dataclass
class SearchCommand:
    """Rich output contract for autonomous search command generator (Phase 7)."""
    pan_offset_deg: float
    tilt_offset_deg: float
    search_stage: str                  # e.g. "inner_ellipse", "middle_ellipse", "outer_ellipse", "extended_ellipse", "bounded_fallback"
    confidence_region: str             # e.g. "1_sigma", "2_sigma", "95_percent", "99_percent", "fallback"
    mahalanobis_radius: float          # normalized Mahalanobis radius k
    point_index: int                   # sequential point counter
    total_angular_travel_deg: float    # cumulative angular distance traveled (deg)
    fov_clipped: bool = False          # True if search command was clipped to sensor FOV
    is_timed_out: bool = False         # True if search duration exceeded max_duration_s
    u_commanded_px: float = 320.0      # Commanded image plane x
    v_commanded_px: float = 240.0      # Commanded image plane y


def chi2_scale_for_probability(p: float) -> float:
    """
    Compute 2-DOF chi-square scaling factor k for a cumulative probability mass p in [0, 1).
    CDF: p = 1 - exp(-s / 2)  =>  s = -2 * ln(1 - p)  =>  k = sqrt(s).
    """
    p_clamped = min(0.9999, max(1e-4, float(p)))
    s = -2.0 * np.log(1.0 - p_clamped)
    return float(np.sqrt(s))


def probability_for_chi2_scale(k: float) -> float:
    """
    Compute 2-DOF chi-square cumulative probability mass for Mahalanobis radius k.
    CDF: p = 1 - exp(-k^2 / 2).
    """
    k_val = max(0.0, float(k))
    return float(1.0 - np.exp(-0.5 * (k_val ** 2)))


def compute_covariance_ellipse(P_pos: np.ndarray, scale_k: float = 1.0) -> Dict[str, Any]:
    """
    Eigendecompose 2D position covariance P_pos into semi-major axis, semi-minor axis,
    and orientation angle.
    
    Args:
        P_pos: 2x2 covariance matrix [[Pxx, Pxy], [Pyx, Pyy]].
        scale_k: Mahalanobis distance multiplier (e.g. 1.0 for 1-sigma, 2.0 for 2-sigma,
                 2.4477 for 95%, 3.0349 for 99%).
                 
    Returns:
        Dictionary containing major_axis_px, minor_axis_px, orientation_deg, orientation_rad.
    """
    P = np.array(P_pos, dtype=np.float64)
    # Ensure symmetry
    P_sym = 0.5 * (P + P.T)
    
    try:
        eigenvals, eigenvecs = np.linalg.eigh(P_sym)
    except np.linalg.LinAlgError:
        eigenvals = np.array([25.0, 25.0], dtype=np.float64)
        eigenvecs = np.eye(2, dtype=np.float64)

    # Sort descending
    order = np.argsort(eigenvals)[::-1]
    lam1 = max(1e-6, float(eigenvals[order[0]]))
    lam2 = max(1e-6, float(eigenvals[order[1]]))
    vec1 = eigenvecs[:, order[0]]
    if vec1[0] < 0 or (abs(vec1[0]) < 1e-9 and vec1[1] < 0):
        vec1 = -vec1

    major_axis = float(scale_k * np.sqrt(lam1))
    minor_axis = float(scale_k * np.sqrt(lam2))
    orientation_rad = float(np.arctan2(vec1[1], vec1[0]))
    orientation_deg = float(np.degrees(orientation_rad))

    return {
        "major_axis_px": round(major_axis, 4),
        "minor_axis_px": round(minor_axis, 4),
        "orientation_deg": round(orientation_deg, 2),
        "orientation_rad": orientation_rad,
        "eigenvals": (lam1, lam2),
        "eigenvecs": eigenvecs[:, order],
        "scale_k": scale_k
    }


# Standard 2-D Chi-Square Confidence Regions
CONFIDENCE_STAGES = [
    # (stage_name, region_name, k_min, k_max, duration_s)
    ("inner_ellipse", "1_sigma", 0.0, 1.0, 0.35),               # ~39.3% mass
    ("middle_ellipse", "2_sigma", 1.0, 2.0, 0.50),              # ~86.5% mass
    ("outer_ellipse", "95_percent", 2.0, 2.4477, 0.50),         # 95.0% mass (-2*ln(0.05))
    ("extended_ellipse", "99_percent", 2.4477, 3.0349, 0.40),   # 99.0% mass (-2*ln(0.01))
    ("bounded_fallback", "fallback", 3.0349, 3.80, 0.25)        # 99.9% mass / fallback limit
]


@dataclass
class SearchPatternConfig:
    pattern_type: Literal["spiral", "fixed_spiral", "raster", "covariance_ellipse_scan", "probability_ordered_ellipse_scan"] = "covariance_ellipse_scan"
    angular_step_deg: float = 0.25         # Angular spacing between turns / scan rows (deg)
    scan_rate_deg_s: float = 1.0           # Actuator scanning speed (deg/s)
    max_local_radius_deg: float = 2.0      # Maximum search radius for local reacquisition (deg)
    dwell_time_s: float = 0.0              # Dwell time per discrete step if stepped (0.0 = continuous)
    max_duration_s: float = 2.0            # Timeout duration before escalating to LOST (s)
    
    # Phase 7 Covariance Search Parameters
    trust_horizon_s: float = 0.35          # Time since last measurement before freezing center propagation (s)
    min_axis_px: float = 3.0               # Minimum semi-axis length to prevent degenerate zero-area scan (px)
    max_axis_px: float = 350.0             # Maximum semi-axis length clamp (px)
    fov_clipping: bool = True              # Enable clipping to observable camera FOV
    fov_margin_px: float = 10.0            # Observable boundary margin from sensor edges (px)
    
    # Camera Calibration Parameters (Scientific Contract)
    camera_width: int = 640
    camera_height: int = 480
    fx: float = 3661.723149
    fy: float = 3661.723149
    cx: float = 320.0
    cy: float = 240.0


class SearchCommandGenerator:
    """
    Deterministic Autonomous Search Command Generator.
    Produces (delta_pan_deg, delta_tilt_deg) offsets relative to the search center,
    or uncertainty-aware ellipse scans respecting covariance and camera projection.
    Decoupled strictly from Ground Truth.
    """

    def __init__(self, config: Optional[SearchPatternConfig] = None):
        self.config = config or SearchPatternConfig()
        self.reset()

    def reset(self):
        """Reset search scan generator state."""
        self.start_time: float = 0.0
        self.active: bool = False
        self.last_offset_pan: float = 0.0
        self.last_offset_tilt: float = 0.0
        self.point_index: int = 0
        self.total_angular_travel_deg: float = 0.0
        self.last_command: Optional[SearchCommand] = None

        # Covariance state cache
        self.cached_pos_mean: np.ndarray = np.array([self.config.cx, self.config.cy], dtype=np.float64)
        self.cached_pos_cov: np.ndarray = np.eye(2, dtype=np.float64) * 25.0
        self.frozen_center: Tuple[float, float] = (self.config.cx, self.config.cy)
        self.center_frozen: bool = False
        self.time_since_measurement_at_start: float = 0.0

        # Eigendecomposition cache
        self.sigma1: float = 5.0
        self.sigma2: float = 5.0
        self.ellipse_theta_rad: float = 0.0
        self.ellipse_theta_deg: float = 0.0

    def start(
        self,
        start_time: float,
        kf_pos_mean: Optional[np.ndarray] = None,
        kf_pos_cov: Optional[np.ndarray] = None,
        time_since_measurement: float = 0.0
    ):
        """
        Start a new search pattern from t=start_time with optional initial estimator covariance.
        """
        self.start_time = float(start_time)
        self.active = True
        self.last_offset_pan = 0.0
        self.last_offset_tilt = 0.0
        self.point_index = 0
        self.total_angular_travel_deg = 0.0
        self.time_since_measurement_at_start = float(time_since_measurement)
        self.center_frozen = False

        if kf_pos_mean is not None and len(kf_pos_mean) >= 2:
            self.cached_pos_mean = np.array([float(kf_pos_mean[0]), float(kf_pos_mean[1])], dtype=np.float64)
        else:
            self.cached_pos_mean = np.array([self.config.cx, self.config.cy], dtype=np.float64)
        self.frozen_center = (float(self.cached_pos_mean[0]), float(self.cached_pos_mean[1]))

        if kf_pos_cov is not None and kf_pos_cov.shape == (2, 2):
            self.cached_pos_cov = np.array(kf_pos_cov, dtype=np.float64)
        else:
            self.cached_pos_cov = np.eye(2, dtype=np.float64) * 25.0

        self._update_ellipse_geometry(self.cached_pos_cov)

        self.last_command = SearchCommand(
            pan_offset_deg=0.0,
            tilt_offset_deg=0.0,
            search_stage="center",
            confidence_region="1_sigma",
            mahalanobis_radius=0.0,
            point_index=0,
            total_angular_travel_deg=0.0,
            fov_clipped=False,
            is_timed_out=False,
            u_commanded_px=self.cached_pos_mean[0],
            v_commanded_px=self.cached_pos_mean[1]
        )

    def _update_ellipse_geometry(self, P_pos: np.ndarray):
        """Decompose position covariance into semi-major, semi-minor, and orientation."""
        res = compute_covariance_ellipse(P_pos, scale_k=1.0)
        lam1, lam2 = res["eigenvals"]
        self.sigma1 = max(self.config.min_axis_px, float(np.sqrt(lam1)))
        self.sigma2 = max(self.config.min_axis_px, float(np.sqrt(lam2)))
        self.ellipse_theta_rad = float(res["orientation_rad"])
        self.ellipse_theta_deg = float(res["orientation_deg"])

    def get_offset(
        self,
        current_time: float,
        kf_pos_mean: Optional[np.ndarray] = None,
        kf_pos_cov: Optional[np.ndarray] = None,
        time_since_measurement: Optional[float] = None
    ) -> Tuple[float, float, bool]:
        """
        Compute search offset at current_time.
        
        Returns:
            (offset_pan_deg, offset_tilt_deg, is_timed_out)
        """
        if not self.active:
            return 0.0, 0.0, False

        t_elapsed = max(0.0, float(current_time) - self.start_time)
        timed_out = t_elapsed >= self.config.max_duration_s

        ptype = self.config.pattern_type

        if ptype in ("fixed_spiral", "spiral"):
            pan, tilt, cmd = self._step_fixed_spiral(t_elapsed, timed_out)
        elif ptype == "raster":
            pan, tilt, cmd = self._step_raster(t_elapsed, timed_out)
        elif ptype == "probability_ordered_ellipse_scan":
            pan, tilt, cmd = self._step_probability_ordered_ellipse(
                t_elapsed, timed_out, kf_pos_mean, kf_pos_cov, time_since_measurement
            )
        else:  # default "covariance_ellipse_scan"
            pan, tilt, cmd = self._step_covariance_ellipse(
                t_elapsed, timed_out, kf_pos_mean, kf_pos_cov, time_since_measurement
            )

        # Accumulate angular travel distance
        d_ang = float(np.hypot(pan - self.last_offset_pan, tilt - self.last_offset_tilt))
        self.total_angular_travel_deg += d_ang
        cmd.total_angular_travel_deg = self.total_angular_travel_deg
        self.point_index += 1
        cmd.point_index = self.point_index

        self.last_offset_pan = pan
        self.last_offset_tilt = tilt
        self.last_command = cmd

        return pan, tilt, timed_out

    def get_command(self) -> Optional[SearchCommand]:
        """Return the rich SearchCommand dataclass from the last update step."""
        return self.last_command

    # -----------------------------------------------------------------
    # R0: Phase 5 Fixed Archimedean Spiral Baseline
    # -----------------------------------------------------------------
    def _step_fixed_spiral(self, t_elapsed: float, timed_out: bool) -> Tuple[float, float, SearchCommand]:
        b = self.config.angular_step_deg / (2.0 * np.pi)
        if b <= 1e-6:
            b = 0.05
        v = self.config.scan_rate_deg_s
        theta = np.sqrt(2.0 * v * t_elapsed / b)
        r = min(self.config.max_local_radius_deg, b * theta)

        pan = float(r * np.cos(theta))
        tilt = float(r * np.sin(theta))

        # Project angular offset to pixel space for telemetry
        cx, cy = self.config.cx, self.config.cy
        fx, fy = self.config.fx, self.config.fy
        u_cmd = cx - fx * np.tan(np.radians(pan))
        v_cmd = cy - fy * np.tan(np.radians(tilt))

        cmd = SearchCommand(
            pan_offset_deg=pan,
            tilt_offset_deg=tilt,
            search_stage="fixed_spiral",
            confidence_region="none",
            mahalanobis_radius=round(float(r / max(1e-4, self.config.angular_step_deg)), 3),
            point_index=self.point_index,
            total_angular_travel_deg=self.total_angular_travel_deg,
            fov_clipped=False,
            is_timed_out=timed_out,
            u_commanded_px=float(u_cmd),
            v_commanded_px=float(v_cmd)
        )
        return pan, tilt, cmd

    # -----------------------------------------------------------------
    # Bounded Rectangular Raster Scan
    # -----------------------------------------------------------------
    def _step_raster(self, t_elapsed: float, timed_out: bool) -> Tuple[float, float, SearchCommand]:
        r_max = self.config.max_local_radius_deg
        row_height = self.config.angular_step_deg
        scan_width = 2.0 * r_max
        speed = self.config.scan_rate_deg_s

        time_per_row = scan_width / speed if speed > 1e-6 else 1.0
        row_idx = int(t_elapsed // time_per_row)
        row_t = t_elapsed % time_per_row

        fraction = row_t / time_per_row
        if row_idx % 2 == 0:
            pan = -r_max + fraction * scan_width
        else:
            pan = r_max - fraction * scan_width

        tilt = r_max - (row_idx * row_height)
        if abs(tilt) > r_max:
            tilt = float(np.sign(tilt) * r_max)

        cx, cy = self.config.cx, self.config.cy
        fx, fy = self.config.fx, self.config.fy
        u_cmd = cx - fx * np.tan(np.radians(pan))
        v_cmd = cy - fy * np.tan(np.radians(tilt))

        cmd = SearchCommand(
            pan_offset_deg=float(pan),
            tilt_offset_deg=float(tilt),
            search_stage="raster",
            confidence_region="none",
            mahalanobis_radius=round(float(np.hypot(pan, tilt)), 3),
            point_index=self.point_index,
            total_angular_travel_deg=self.total_angular_travel_deg,
            fov_clipped=False,
            is_timed_out=timed_out,
            u_commanded_px=float(u_cmd),
            v_commanded_px=float(v_cmd)
        )
        return float(pan), float(tilt), cmd

    # -----------------------------------------------------------------
    # R1: Covariance Ellipse Scan (Phase 7 Core Implementation)
    # -----------------------------------------------------------------
    def _step_covariance_ellipse(
        self,
        t_elapsed: float,
        timed_out: bool,
        kf_pos_mean: Optional[np.ndarray],
        kf_pos_cov: Optional[np.ndarray],
        time_since_measurement: Optional[float]
    ) -> Tuple[float, float, SearchCommand]:
        """
        Continuous, orientation-respecting covariance ellipse scanning trajectory.
        Traverses lowest Mahalanobis radius regions first (~1 sigma -> ~2 sigma -> 95% -> 99% -> fallback).
        Applies search-center trust horizon damping and FOV clipping.
        """
        # 1. Search Center Management & Trust Horizon
        age = self.time_since_measurement_at_start + t_elapsed if time_since_measurement is None else float(time_since_measurement)
        if age <= self.config.trust_horizon_s and kf_pos_mean is not None and len(kf_pos_mean) >= 2:
            self.cached_pos_mean = np.array([float(kf_pos_mean[0]), float(kf_pos_mean[1])], dtype=np.float64)
            self.frozen_center = (float(self.cached_pos_mean[0]), float(self.cached_pos_mean[1]))
            self.center_frozen = False
        else:
            self.center_frozen = True
            self.cached_pos_mean = np.array(self.frozen_center, dtype=np.float64)

        # 2. Covariance Update
        if kf_pos_cov is not None and kf_pos_cov.shape == (2, 2):
            self.cached_pos_cov = np.array(kf_pos_cov, dtype=np.float64)
            self._update_ellipse_geometry(self.cached_pos_cov)

        # 3. Determine Confidence Region Progression Stage
        # Cumulative timing:
        t_cum = 0.0
        current_stage = CONFIDENCE_STAGES[0]
        for stage in CONFIDENCE_STAGES:
            s_name, r_name, k_min, k_max, dur = stage
            if t_elapsed <= (t_cum + dur):
                current_stage = stage
                frac = (t_elapsed - t_cum) / dur if dur > 1e-6 else 0.0
                k_val = k_min + frac * (k_max - k_min)
                break
            t_cum += dur
        else:
            # Fallback tail
            current_stage = CONFIDENCE_STAGES[-1]
            k_val = current_stage[3]

        stage_name, region_name, _, _, _ = current_stage

        # 4. Parametric Angular Progression phi(t)
        # Scan frequency: ~2.0 Hz yields continuous unbroken trace at scan_rate_deg_s
        # Total phase winds smoothly across stages without discontinuity
        f_scan = 2.0  # Hz
        phi = 2.0 * np.pi * f_scan * t_elapsed

        # 5. Unrotated & Rotated Elliptical Coordinates
        a = np.clip(k_val * self.sigma1, self.config.min_axis_px, self.config.max_axis_px)
        b = np.clip(k_val * self.sigma2, self.config.min_axis_px, self.config.max_axis_px)

        x_ell = a * np.cos(phi)
        y_ell = b * np.sin(phi)

        cos_th = np.cos(self.ellipse_theta_rad)
        sin_th = np.sin(self.ellipse_theta_rad)
        delta_u = x_ell * cos_th - y_ell * sin_th
        delta_v = x_ell * sin_th + y_ell * cos_th

        # Unclipped image plane location
        u_raw = self.cached_pos_mean[0] + delta_u
        v_raw = self.cached_pos_mean[1] + delta_v

        # 6. FOV Clipping
        fov_clipped = False
        if self.config.fov_clipping:
            u_min = self.config.fov_margin_px
            u_max = self.config.camera_width - self.config.fov_margin_px
            v_min = self.config.fov_margin_px
            v_max = self.config.camera_height - self.config.fov_margin_px

            u_cmd = float(np.clip(u_raw, u_min, u_max))
            v_cmd = float(np.clip(v_raw, v_min, v_max))
            if u_cmd != u_raw or v_cmd != v_raw:
                fov_clipped = True
        else:
            u_cmd = float(u_raw)
            v_cmd = float(v_raw)

        # 7. Convert Commanded Pixel Position to Angular Pan / Tilt Offsets
        # Exact pinhole projection using certified scientific contract
        fx = self.config.fx
        fy = self.config.fy
        cx = self.config.cx
        cy = self.config.cy

        # Reference position currently targeted by controller (KF state if active, else center)
        if kf_pos_mean is not None and len(kf_pos_mean) >= 2:
            u_kf = float(kf_pos_mean[0])
            v_kf = float(kf_pos_mean[1])
        else:
            u_kf = self.cached_pos_mean[0]
            v_kf = self.cached_pos_mean[1]

        # Optical azimuth / elevation angles relative to camera optical axis
        # h_angle = atan((u - cx) / fx), v_angle = atan((cy - v) / fy)
        theta_h_cmd = float(np.degrees(np.arctan((u_cmd - cx) / fx)))
        theta_v_cmd = float(np.degrees(np.arctan((cy - v_cmd) / fy)))

        theta_h_kf = float(np.degrees(np.arctan((u_kf - cx) / fx)))
        theta_v_kf = float(np.degrees(np.arctan((cy - v_kf) / fy)))

        # Differential offset so that:
        # ctrl_h_err = -theta_h_kf + pan_offset  =>  ctrl_h_err = -theta_h_cmd
        # ctrl_v_err = +theta_v_kf + tilt_offset =>  ctrl_v_err = +theta_v_cmd
        pan_offset = -(theta_h_cmd - theta_h_kf)
        tilt_offset = (theta_v_cmd - theta_v_kf)

        # 8. Actuator Authority Boundary Check
        # Ensure commanded offset from initial search center does not exceed max_local_radius_deg
        r_ang = float(np.hypot(pan_offset, tilt_offset))
        if r_ang > self.config.max_local_radius_deg and r_ang > 1e-4:
            scale_ang = self.config.max_local_radius_deg / r_ang
            pan_offset *= scale_ang
            tilt_offset *= scale_ang

        cmd = SearchCommand(
            pan_offset_deg=round(pan_offset, 4),
            tilt_offset_deg=round(tilt_offset, 4),
            search_stage=stage_name,
            confidence_region=region_name,
            mahalanobis_radius=round(float(k_val), 3),
            point_index=self.point_index,
            total_angular_travel_deg=self.total_angular_travel_deg,
            fov_clipped=fov_clipped,
            is_timed_out=timed_out,
            u_commanded_px=round(u_cmd, 2),
            v_commanded_px=round(v_cmd, 2)
        )
        return pan_offset, tilt_offset, cmd

    # -----------------------------------------------------------------
    # R2: Probability-Ordered Ellipse Scan (Optional Mode)
    # -----------------------------------------------------------------
    def _step_probability_ordered_ellipse(
        self,
        t_elapsed: float,
        timed_out: bool,
        kf_pos_mean: Optional[np.ndarray],
        kf_pos_cov: Optional[np.ndarray],
        time_since_measurement: Optional[float]
    ) -> Tuple[float, float, SearchCommand]:
        """
        Probability-Ordered Ellipse Scan: explores discrete concentric probability rings
        (1-sigma, 2-sigma, 95%, 99%) ordered strictly by probability density.
        """
        # Shares the continuous covariance generator with discrete stage dwell steps
        pan, tilt, cmd = self._step_covariance_ellipse(
            t_elapsed, timed_out, kf_pos_mean, kf_pos_cov, time_since_measurement
        )
        cmd.search_stage = f"prob_ordered_{cmd.search_stage}"
        return pan, tilt, cmd

    def to_dict(self) -> Dict[str, Any]:
        """Serialize current search generator state for telemetry."""
        return {
            "pattern_type": self.config.pattern_type,
            "active": self.active,
            "last_offset_pan_deg": round(self.last_offset_pan, 4),
            "last_offset_tilt_deg": round(self.last_offset_tilt, 4),
            "search_stage": self.last_command.search_stage if self.last_command else "idle",
            "confidence_region": self.last_command.confidence_region if self.last_command else "none",
            "mahalanobis_radius": round(self.last_command.mahalanobis_radius, 3) if self.last_command else 0.0,
            "point_index": self.last_command.point_index if self.last_command else 0,
            "total_angular_travel_deg": round(self.total_angular_travel_deg, 4),
            "fov_clipped": self.last_command.fov_clipped if self.last_command else False,
            "max_local_radius_deg": self.config.max_local_radius_deg,
            "scan_rate_deg_s": self.config.scan_rate_deg_s,
            "max_duration_s": self.config.max_duration_s
        }
