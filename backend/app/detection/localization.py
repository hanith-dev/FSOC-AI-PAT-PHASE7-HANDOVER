"""
High-Precision Optical Beacon Localization Subsystem for FSOC-AI-PAT.
Decouples Candidate Detection from Precision Subpixel Localization.

Provides:
- LocalizationResult: Standard measurement contract with uncertainty and quality flags.
- BaselineLocalizer: Inherited bounding-box midpoint localizer (preserved for ablation).
- WeightedCentroidLocalizer: Background-corrected intensity-weighted subpixel centroid.
- PSFRefinedLocalizer: 2D Gaussian PSF least-squares refinement with graceful fallback.
"""
from dataclasses import dataclass, field
import time
from typing import Dict, Any, List, Optional, Tuple, Literal
import numpy as np
import cv2


@dataclass(frozen=True)
class LocalizationResult:
    """
    Standard measurement contract for optical beacon localization.
    Provides subpixel coordinates, physical quality metrics, and uncertainty proxy.
    """
    success: bool
    x: float
    y: float
    method: str
    confidence: float
    background_level: float
    peak_intensity: float
    integrated_signal: float
    snr_proxy: float
    sigma_x_px: float
    sigma_y_px: float
    localization_uncertainty_x_px: float
    localization_uncertainty_y_px: float
    roi_width: int
    roi_height: int
    processing_time_ms: float
    quality_flags: List[str] = field(default_factory=list)

    def to_dict(self) -> Dict[str, Any]:
        return {
            "success": self.success,
            "x": round(self.x, 4) if not np.isnan(self.x) else None,
            "y": round(self.y, 4) if not np.isnan(self.y) else None,
            "method": self.method,
            "confidence": round(self.confidence, 4),
            "background_level": round(self.background_level, 3),
            "peak_intensity": round(self.peak_intensity, 3),
            "integrated_signal": round(self.integrated_signal, 2),
            "snr_proxy": round(self.snr_proxy, 2),
            "sigma_x_px": round(self.sigma_x_px, 3),
            "sigma_y_px": round(self.sigma_y_px, 3),
            "localization_uncertainty_x_px": round(self.localization_uncertainty_x_px, 4),
            "localization_uncertainty_y_px": round(self.localization_uncertainty_y_px, 4),
            "roi_width": self.roi_width,
            "roi_height": self.roi_height,
            "processing_time_ms": round(self.processing_time_ms, 3),
            "quality_flags": list(self.quality_flags)
        }


def extract_roi(
    image: np.ndarray,
    bbox: List[int],
    margin: int = 4
) -> Tuple[np.ndarray, int, int, List[str]]:
    """
    Safely extract local ROI around candidate bounding box with margin.
    Returns:
        (roi, x_offset, y_offset, quality_flags)
    """
    h_img, w_img = image.shape[:2]
    flags = []

    x1, y1, x2, y2 = [int(v) for v in bbox]

    # Add margin
    x_start = max(0, x1 - margin)
    y_start = max(0, y1 - margin)
    x_end = min(w_img, x2 + margin)
    y_end = min(h_img, y2 + margin)

    if x_start == 0 or y_start == 0 or x_end == w_img or y_end == h_img:
        flags.append("TOUCHING_SENSOR_EDGE")

    if x_end <= x_start or y_end <= y_start:
        flags.append("EMPTY_ROI")
        return np.zeros((0, 0), dtype=image.dtype), x_start, y_start, flags

    roi = image[y_start:y_end, x_start:x_end]
    return roi, x_start, y_start, flags


def estimate_local_background(roi_gray: np.ndarray) -> Tuple[float, float]:
    """
    Robust local background estimation using border pixel median and standard deviation.
    Border pixels exclude the bright spot core located in the interior.
    Returns:
        (background_level, background_std)
    """
    h, w = roi_gray.shape
    if h < 3 or w < 3:
        # Fallback for tiny ROIs: lowest 20th percentile
        bg = float(np.percentile(roi_gray, 20))
        std = max(1.0, float(np.std(roi_gray)))
        return bg, std

    # Extract 1-pixel perimeter border
    top_border = roi_gray[0, :]
    bottom_border = roi_gray[-1, :]
    left_border = roi_gray[1:-1, 0]
    right_border = roi_gray[1:-1, -1]

    border_pixels = np.concatenate([top_border, bottom_border, left_border, right_border]).astype(np.float64)

    # Median is robust to isolated hot pixels
    bg_level = float(np.median(border_pixels))
    bg_std = float(np.std(border_pixels))
    bg_std = max(1.0, bg_std)  # Avoid division by zero

    return bg_level, bg_std


class BaseLocalizer:
    """Base interface for subpixel beacon localizers."""
    def localize(
        self,
        frame: np.ndarray,
        bbox: List[int],
        **kwargs
    ) -> LocalizationResult:
        raise NotImplementedError


# =====================================================================
# 1. BASELINE LOCALIZER (Inherited Phase 0/1 Method)
# =====================================================================

class BaselineLocalizer(BaseLocalizer):
    """
    Inherited baseline localization method: Bounding-box midpoint.
    Source: backend/app/detection/detector.py:OpenCVDetector (lines 81-83)
    cx = x + box_w / 2.0
    cy = y + box_h / 2.0
    Preserved strictly for ablation comparison.
    """
    def localize(
        self,
        frame: np.ndarray,
        bbox: List[int],
        **kwargs
    ) -> LocalizationResult:
        t_start = time.perf_counter()
        flags = ["BASELINE_BBOX_MIDPOINT"]

        x1, y1, x2, y2 = [int(v) for v in bbox]
        box_w = x2 - x1
        box_h = y2 - y1

        if box_w <= 0 or box_h <= 0 or frame is None or frame.size == 0:
            t_elapsed = (time.perf_counter() - t_start) * 1000.0
            return LocalizationResult(
                success=False,
                x=float("nan"),
                y=float("nan"),
                method="baseline",
                confidence=0.0,
                background_level=0.0,
                peak_intensity=0.0,
                integrated_signal=0.0,
                snr_proxy=0.0,
                sigma_x_px=0.0,
                sigma_y_px=0.0,
                localization_uncertainty_x_px=10.0,
                localization_uncertainty_y_px=10.0,
                roi_width=0,
                roi_height=0,
                processing_time_ms=t_elapsed,
                quality_flags=flags + ["INVALID_BOUNDS"]
            )

        cx = x1 + box_w / 2.0
        cy = y1 + box_h / 2.0

        # Extract basic ROI signal statistics
        roi, _, _, roi_flags = extract_roi(frame, bbox, margin=0)
        flags.extend(roi_flags)

        roi_gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY) if len(roi.shape) == 3 else roi
        peak_val = float(np.max(roi_gray)) if roi_gray.size > 0 else 0.0
        int_signal = float(np.sum(roi_gray)) if roi_gray.size > 0 else 0.0

        # Baseline quantization jitter uncertainty: uniform distribution over 1 px -> std ~ 1/sqrt(12) = 0.288px
        uncertainty = max(0.5, float(max(box_w, box_h) * 0.1))

        t_elapsed = (time.perf_counter() - t_start) * 1000.0
        return LocalizationResult(
            success=True,
            x=float(cx),
            y=float(cy),
            method="baseline",
            confidence=0.5,
            background_level=0.0,
            peak_intensity=peak_val,
            integrated_signal=int_signal,
            snr_proxy=1.0,
            sigma_x_px=box_w / 2.0,
            sigma_y_px=box_h / 2.0,
            localization_uncertainty_x_px=uncertainty,
            localization_uncertainty_y_px=uncertainty,
            roi_width=box_w,
            roi_height=box_h,
            processing_time_ms=t_elapsed,
            quality_flags=flags
        )


# =====================================================================
# 2. METHOD A: BACKGROUND-CORRECTED WEIGHTED CENTROID
# =====================================================================

class WeightedCentroidLocalizer(BaseLocalizer):
    """
    Background-corrected Intensity-Weighted Centroid Localizer.
    Calculates subpixel optical center by subtracting local border background,
    clamping negative residuals, and computing first and second intensity moments:
        w_i = max(I_i - I_bg, 0)
        x_c = sum(x_i * w_i) / sum(w_i)
        y_c = sum(y_i * w_i) / sum(w_i)
    """
    def __init__(self, roi_margin: int = 5, min_snr: float = 1.2):
        self.roi_margin = int(roi_margin)
        self.min_snr = float(min_snr)

    def localize(
        self,
        frame: np.ndarray,
        bbox: List[int],
        **kwargs
    ) -> LocalizationResult:
        t_start = time.perf_counter()
        quality_flags = []

        if frame is None or frame.size == 0:
            return self._make_failure_result("EMPTY_FRAME", t_start)

        # 1. Extract Local ROI with margin
        roi, x_offset, y_offset, roi_flags = extract_roi(frame, bbox, margin=self.roi_margin)
        quality_flags.extend(roi_flags)

        if roi.size == 0:
            return self._make_failure_result("EMPTY_ROI", t_start, quality_flags)

        h_roi, w_roi = roi.shape[:2]

        # Convert to single-channel luminance / grayscale if color
        if len(roi.shape) == 3:
            # High-fidelity optical green channel weighting or luminance
            # FSOC beacon is predominantly green channel #10B981
            roi_gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        else:
            roi_gray = roi

        raw_peak = float(np.max(roi)) if roi.size > 0 else 0.0
        roi_float = roi_gray.astype(np.float64)
        peak_intensity = float(np.max(roi_float)) if roi_float.size > 0 else 0.0

        # Check for core sensor well saturation (>= 250 in 8-bit raw sensor or luminance)
        if raw_peak >= 250.0 or peak_intensity >= 240.0:
            quality_flags.append("SATURATED_CORE")

        # 2. Robust Local Background Estimation
        bg_level, bg_std = estimate_local_background(roi_float)

        # Net peak signal above background
        net_peak = max(0.0, peak_intensity - bg_level)
        snr = net_peak / bg_std if bg_std > 0 else 0.0

        if snr < self.min_snr:
            quality_flags.append("LOW_SNR")

        # 3. Background Subtraction & Weight Calculation
        # w_i = max(I_i - I_background, 0)
        weights = np.maximum(0.0, roi_float - bg_level)

        # Optional threshold cut at 0.5 * bg_std to suppress background noise floor
        noise_threshold = 0.5 * bg_std
        weights[weights < noise_threshold] = 0.0

        sum_w = float(np.sum(weights))
        if sum_w <= 1e-6:
            # Zero signal above background: fallback to bbox midpoint
            quality_flags.append("ZERO_NET_WEIGHT_FALLBACK")
            x1, y1, x2, y2 = bbox
            cx = (x1 + x2) / 2.0
            cy = (y1 + y2) / 2.0
            t_elapsed = (time.perf_counter() - t_start) * 1000.0
            return LocalizationResult(
                success=False,
                x=float(cx),
                y=float(cy),
                method="weighted_centroid",
                confidence=0.1,
                background_level=bg_level,
                peak_intensity=peak_intensity,
                integrated_signal=0.0,
                snr_proxy=snr,
                sigma_x_px=float(w_roi) / 2.0,
                sigma_y_px=float(h_roi) / 2.0,
                localization_uncertainty_x_px=5.0,
                localization_uncertainty_y_px=5.0,
                roi_width=w_roi,
                roi_height=h_roi,
                processing_time_ms=t_elapsed,
                quality_flags=quality_flags
            )

        # 4. Intensity-Weighted First Moments (Subpixel Center)
        y_coords, x_coords = np.indices((h_roi, w_roi), dtype=np.float64)
        local_xc = float(np.sum(x_coords * weights) / sum_w)
        local_yc = float(np.sum(y_coords * weights) / sum_w)

        # Transform local ROI coordinates to global sensor coordinates
        global_xc = float(local_xc + x_offset)
        global_yc = float(local_yc + y_offset)

        # 5. Weighted Second Moments (Spot Spread / Size)
        dx = x_coords - local_xc
        dy = y_coords - local_yc
        var_x = float(np.sum(weights * (dx ** 2)) / sum_w)
        var_y = float(np.sum(weights * (dy ** 2)) / sum_w)
        sigma_x = float(np.sqrt(max(0.1, var_x)))
        sigma_y = float(np.sqrt(max(0.1, var_y)))

        # 6. Physical Uncertainty Proxy (Cramer-Rao Lower Bound approximation)
        # sigma_loc ~= sigma_PSF / SNR * sqrt(1 + 8*pi*sigma_PSF^2 * sigma_bg^2 / Net_Flux^2)
        safe_snr = max(1.0, snr)
        crlb_factor = np.sqrt(1.0 + (8.0 * np.pi * (sigma_x * sigma_y) * (bg_std ** 2)) / (sum_w ** 2))
        unc_x = float(np.clip((sigma_x / safe_snr) * crlb_factor, 0.02, 5.0))
        unc_y = float(np.clip((sigma_y / safe_snr) * crlb_factor, 0.02, 5.0))

        confidence = float(np.clip(snr / (snr + 5.0), 0.1, 0.99))
        if "TOUCHING_SENSOR_EDGE" in quality_flags:
            confidence *= 0.7
            unc_x *= 1.5
            unc_y *= 1.5

        t_elapsed = (time.perf_counter() - t_start) * 1000.0
        return LocalizationResult(
            success=True,
            x=global_xc,
            y=global_yc,
            method="weighted_centroid",
            confidence=confidence,
            background_level=bg_level,
            peak_intensity=peak_intensity,
            integrated_signal=sum_w,
            snr_proxy=snr,
            sigma_x_px=sigma_x,
            sigma_y_px=sigma_y,
            localization_uncertainty_x_px=unc_x,
            localization_uncertainty_y_px=unc_y,
            roi_width=w_roi,
            roi_height=h_roi,
            processing_time_ms=t_elapsed,
            quality_flags=quality_flags
        )

    def _make_failure_result(self, reason: str, t_start: float, flags: Optional[List[str]] = None) -> LocalizationResult:
        qf = list(flags or []) + [reason]
        t_elapsed = (time.perf_counter() - t_start) * 1000.0
        return LocalizationResult(
            success=False,
            x=float("nan"),
            y=float("nan"),
            method="weighted_centroid",
            confidence=0.0,
            background_level=0.0,
            peak_intensity=0.0,
            integrated_signal=0.0,
            snr_proxy=0.0,
            sigma_x_px=0.0,
            sigma_y_px=0.0,
            localization_uncertainty_x_px=10.0,
            localization_uncertainty_y_px=10.0,
            roi_width=0,
            roi_height=0,
            processing_time_ms=t_elapsed,
            quality_flags=qf
        )


# =====================================================================
# 3. METHOD B: GAUSSIAN PSF LEAST-SQUARES REFINEMENT
# =====================================================================

class PSFRefinedLocalizer(BaseLocalizer):
    """
    2D Elliptical Gaussian Point Spread Function (PSF) Refiner.
    Initializes from WeightedCentroidLocalizer.
    Fits:
        I(x, y) = bg + A * exp(-((x - x0)^2 / (2*sig_x^2) + (y - y0)^2 / (2*sig_y^2)))
    using bounded non-linear least squares (Levenberg-Marquardt / Trust Region Reflective).
    Gracefully falls back to weighted centroid on non-convergence or invalid fit.
    """
    def __init__(self, roi_margin: int = 5, max_nfev: int = 20):
        self.centroid_localizer = WeightedCentroidLocalizer(roi_margin=roi_margin)
        self.max_nfev = int(max_nfev)

    def localize(
        self,
        frame: np.ndarray,
        bbox: List[int],
        **kwargs
    ) -> LocalizationResult:
        t_start = time.perf_counter()

        # Step 1: Run Weighted Centroid to get robust initial guess
        init_res = self.centroid_localizer.localize(frame, bbox, **kwargs)
        if not init_res.success:
            return init_res

        flags = list(init_res.quality_flags)

        # Do not fit PSF if spot is saturated (flattop departs from Gaussian) or SNR is very low
        if "SATURATED_CORE" in flags or init_res.snr_proxy < 2.0 or init_res.roi_width < 4 or init_res.roi_height < 4:
            flags.append("PSF_FALLBACK_CENTROID_PREFERRED")
            t_elapsed = (time.perf_counter() - t_start) * 1000.0
            return LocalizationResult(
                success=True,
                x=init_res.x,
                y=init_res.y,
                method="psf_refined (fallback)",
                confidence=init_res.confidence,
                background_level=init_res.background_level,
                peak_intensity=init_res.peak_intensity,
                integrated_signal=init_res.integrated_signal,
                snr_proxy=init_res.snr_proxy,
                sigma_x_px=init_res.sigma_x_px,
                sigma_y_px=init_res.sigma_y_px,
                localization_uncertainty_x_px=init_res.localization_uncertainty_x_px,
                localization_uncertainty_y_px=init_res.localization_uncertainty_y_px,
                roi_width=init_res.roi_width,
                roi_height=init_res.roi_height,
                processing_time_ms=t_elapsed,
                quality_flags=flags
            )

        # Step 2: Extract Local ROI
        roi, x_offset, y_offset, _ = extract_roi(frame, bbox, margin=self.centroid_localizer.roi_margin)
        h_roi, w_roi = roi.shape[:2]
        roi_gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY) if len(roi.shape) == 3 else roi
        data = roi_gray.astype(np.float64)

        y_coords, x_coords = np.indices((h_roi, w_roi), dtype=np.float64)
        x_flat = x_coords.ravel()
        y_flat = y_coords.ravel()
        z_flat = data.ravel()

        # Initial parameter estimates [x0, y0, sigma_x, sigma_y, A, bg]
        init_local_x = init_res.x - x_offset
        init_local_y = init_res.y - y_offset
        init_sig_x = max(0.8, min(init_res.sigma_x_px, float(w_roi) / 2.0))
        init_sig_y = max(0.8, min(init_res.sigma_y_px, float(h_roi) / 2.0))
        init_A = max(5.0, init_res.peak_intensity - init_res.background_level)
        init_bg = init_res.background_level

        p0 = [init_local_x, init_local_y, init_sig_x, init_sig_y, init_A, init_bg]

        # Bounds to ensure physical stability
        # Center restricted to +/- 2.0 pixels from initial weighted centroid
        lb = [
            max(0.0, init_local_x - 2.5),
            max(0.0, init_local_y - 2.5),
            0.4,
            0.4,
            1.0,
            max(0.0, init_bg - 15.0)
        ]
        ub = [
            min(float(w_roi - 1), init_local_x + 2.5),
            min(float(h_roi - 1), init_local_y + 2.5),
            max(4.0, float(w_roi)),
            max(4.0, float(h_roi)),
            255.0,
            min(255.0, init_bg + 15.0)
        ]

        # Pure NumPy Levenberg-Marquardt Non-linear Least Squares Fit
        try:
            p = np.array(p0, dtype=np.float64)
            lb_arr = np.array(lb, dtype=np.float64)
            ub_arr = np.array(ub, dtype=np.float64)
            p = np.clip(p, lb_arr, ub_arr)

            lam = 1e-2
            converged = False

            def eval_gaussian(params):
                x0, y0, sx, sy, A, bg = params
                sx_safe = max(0.2, sx)
                sy_safe = max(0.2, sy)
                u = (x_flat - x0) / sx_safe
                v = (y_flat - y0) / sy_safe
                E = np.exp(-0.5 * (u * u + v * v))
                model = bg + A * E
                res = model - z_flat
                return model, res, E, u, v, sx_safe, sy_safe

            model, res, E, u, v, sx_safe, sy_safe = eval_gaussian(p)
            cost = float(np.sum(res ** 2))
            n_pts = len(z_flat)

            for _ in range(self.max_nfev):
                x0, y0, sx, sy, A, bg = p
                inv_sx = 1.0 / sx_safe
                inv_sy = 1.0 / sy_safe
                AE = A * E

                # Analytical Jacobian (n_pts x 6)
                J = np.empty((n_pts, 6), dtype=np.float64)
                J[:, 0] = AE * u * inv_sx
                J[:, 1] = AE * v * inv_sy
                J[:, 2] = AE * (u ** 2) * inv_sx
                J[:, 3] = AE * (v ** 2) * inv_sy
                J[:, 4] = E
                J[:, 5] = 1.0

                Jt = J.T
                JtJ = Jt @ J
                Jtr = Jt @ res

                diag_H = np.diag(np.diag(JtJ)) + 1e-6 * np.eye(6)
                H = JtJ + lam * diag_H

                try:
                    dp = -np.linalg.solve(H, Jtr)
                except np.linalg.LinAlgError:
                    break

                if np.max(np.abs(dp[:2])) < 1e-3:
                    converged = True
                    break

                p_cand = np.clip(p + dp, lb_arr, ub_arr)
                cand_model, cand_res, cand_E, cand_u, cand_v, cand_sx, cand_sy = eval_gaussian(p_cand)
                cand_cost = float(np.sum(cand_res ** 2))

                if cand_cost < cost:
                    lam = max(1e-5, lam * 0.5)
                    p = p_cand
                    cost = cand_cost
                    res = cand_res
                    E = cand_E
                    u = cand_u
                    v = cand_v
                    sx_safe = cand_sx
                    sy_safe = cand_sy
                    if (cost - cand_cost) / (cost + 1e-8) < 1e-4:
                        converged = True
                        break
                else:
                    lam = min(1e4, lam * 4.0)

            fit_x0, fit_y0, fit_sx, fit_sy, fit_A, fit_bg = p
            global_fit_x = float(fit_x0 + x_offset)
            global_fit_y = float(fit_y0 + y_offset)

            # Quality check: fit center must be reasonably close to weighted centroid (< 2.0 px)
            fit_dist = float(np.hypot(global_fit_x - init_res.x, global_fit_y - init_res.y))
            if fit_dist <= 2.0:
                flags.append("PSF_CONVERGED")
                res_std = float(np.std(res))
                fit_unc_x = float(np.clip((fit_sx / max(1.0, fit_A / max(1.0, res_std))), 0.015, 3.0))
                fit_unc_y = float(np.clip((fit_sy / max(1.0, fit_A / max(1.0, res_std))), 0.015, 3.0))

                t_elapsed = (time.perf_counter() - t_start) * 1000.0
                return LocalizationResult(
                    success=True,
                    x=global_fit_x,
                    y=global_fit_y,
                    method="psf_refined",
                    confidence=min(0.99, init_res.confidence * 1.05),
                    background_level=float(fit_bg),
                    peak_intensity=float(fit_bg + fit_A),
                    integrated_signal=float(2.0 * np.pi * fit_sx * fit_sy * fit_A),
                    snr_proxy=float(fit_A / max(1.0, res_std)),
                    sigma_x_px=float(fit_sx),
                    sigma_y_px=float(fit_sy),
                    localization_uncertainty_x_px=fit_unc_x,
                    localization_uncertainty_y_px=fit_unc_y,
                    roi_width=w_roi,
                    roi_height=h_roi,
                    processing_time_ms=t_elapsed,
                    quality_flags=flags
                )
        except Exception:
            pass

        # Graceful fallback to weighted centroid
        flags.append("PSF_DIVERGED_FALLBACK_CENTROID")
        t_elapsed = (time.perf_counter() - t_start) * 1000.0
        return LocalizationResult(
            success=True,
            x=init_res.x,
            y=init_res.y,
            method="psf_refined (fallback)",
            confidence=init_res.confidence,
            background_level=init_res.background_level,
            peak_intensity=init_res.peak_intensity,
            integrated_signal=init_res.integrated_signal,
            snr_proxy=init_res.snr_proxy,
            sigma_x_px=init_res.sigma_x_px,
            sigma_y_px=init_res.sigma_y_px,
            localization_uncertainty_x_px=init_res.localization_uncertainty_x_px,
            localization_uncertainty_y_px=init_res.localization_uncertainty_y_px,
            roi_width=init_res.roi_width,
            roi_height=init_res.roi_height,
            processing_time_ms=t_elapsed,
            quality_flags=flags
        )


# =====================================================================
# 4. LOCALIZER FACTORY / REGISTRY
# =====================================================================

LOCALIZER_REGISTRY: Dict[str, BaseLocalizer] = {
    "baseline": BaselineLocalizer(),
    "weighted_centroid": WeightedCentroidLocalizer(),
    "psf_refined": PSFRefinedLocalizer()
}


def get_localizer(mode: Literal["baseline", "weighted_centroid", "psf_refined"] = "weighted_centroid") -> BaseLocalizer:
    """Retrieve switchable localizer instance by mode name."""
    mode_key = str(mode).lower().strip()
    if mode_key in ("centroid", "weighted"):
        mode_key = "weighted_centroid"
    elif mode_key in ("psf", "gaussian", "refined"):
        mode_key = "psf_refined"
    elif mode_key in ("bbox", "default_baseline"):
        mode_key = "baseline"

    if mode_key not in LOCALIZER_REGISTRY:
        raise ValueError(f"Unknown localization mode: '{mode}'. Available: {list(LOCALIZER_REGISTRY.keys())}")
    return LOCALIZER_REGISTRY[mode_key]
