"""
Robust Spatial-Temporal Optical Beacon Association Subsystem for FSOC-AI-PAT.
Decouples Candidate Detection and Localization from Track-to-Measurement Association.

Provides:
- CandidateMeasurement: Standard operational candidate data contract.
- AssociationResult: Standard measurement selection contract with gating & cost telemetry.
- AssociationEngine: Configurable association engine supporting:
    1. legacy_simple (inherited Phase 0D baseline)
    2. euclidean_prediction (Euclidean distance to predicted measurement)
    3. mahalanobis_only (statistical chi-square gate with minimum Mahalanobis distance)
    4. robust_multifeature (full multi-feature cost, ambiguity rejection, track persistence)
"""
from dataclasses import dataclass, field
import time
from typing import Dict, Any, List, Optional, Tuple, Literal
import numpy as np

from app.tracking.kalman import KalmanFilter2D


@dataclass(frozen=True)
class CandidateMeasurement:
    """
    Standard operational candidate measurement representation.
    Guaranteed zero ground-truth dependencies.
    """
    candidate_index: int
    x: float
    y: float
    localization_method: str
    localization_uncertainty_x_px: float
    localization_uncertainty_y_px: float
    confidence: float
    area: float
    peak_intensity: float
    integrated_signal: float
    background_level: float
    snr_proxy: float
    sigma_x_px: float
    sigma_y_px: float
    shape_quality: float
    quality_flags: List[str] = field(default_factory=list)
    timestamp: float = 0.0

    @classmethod
    def from_detector_candidate(
        cls,
        cand: Dict[str, Any],
        candidate_index: int,
        timestamp: float = 0.0
    ) -> "CandidateMeasurement":
        """Factory constructor from OpenCVDetector / Localization candidate dict."""
        # Center coordinates: check candidate top-level or localization dict
        center = cand.get("center")
        loc = cand.get("localization", {})
        if center is None:
            center = loc.get("center", [0.0, 0.0])
        bbox = cand.get("bbox", [0, 0, 0, 0])
        bw = max(1, bbox[2] - bbox[0])
        bh = max(1, bbox[3] - bbox[1])

        # Measurement uncertainty from Phase 2 localizer or safe default
        unc_x = float(loc.get("uncertainty_x", loc.get("localization_uncertainty_x_px", 0.5)))
        unc_y = float(loc.get("uncertainty_y", loc.get("localization_uncertainty_y_px", 0.5)))
        sig_x = float(loc.get("sigma_x_px", bw / 2.0))
        sig_y = float(loc.get("sigma_y_px", bh / 2.0))

        # Optical shape quality (ratio of minor to major axis; 1.0 = circular PSF)
        max_sig = max(sig_x, sig_y, 0.1)
        min_sig = min(sig_x, sig_y, 0.1)
        shape_q = float(np.clip(min_sig / max_sig, 0.0, 1.0))

        return cls(
            candidate_index=int(candidate_index),
            x=float(center[0]),
            y=float(center[1]),
            localization_method=str(loc.get("method", "detector_bbox")),
            localization_uncertainty_x_px=unc_x,
            localization_uncertainty_y_px=unc_y,
            confidence=float(cand.get("confidence", 0.5)),
            area=float(cand.get("area", bw * bh)),
            peak_intensity=float(loc.get("peak_intensity", 200.0)),
            integrated_signal=float(loc.get("integrated_signal", 1000.0)),
            background_level=float(loc.get("background_level", 20.0)),
            snr_proxy=float(loc.get("snr_proxy", 5.0)),
            sigma_x_px=sig_x,
            sigma_y_px=sig_y,
            shape_quality=shape_q,
            quality_flags=list(loc.get("quality_flags", [])),
            timestamp=float(timestamp)
        )

    @classmethod
    def from_dict(
        cls,
        cand: Dict[str, Any],
        candidate_index: int = 0,
        candidate_id: Optional[str] = None,
        timestamp: float = 0.0
    ) -> "CandidateMeasurement":
        """Convenience alias for from_detector_candidate."""
        return cls.from_detector_candidate(cand, candidate_index=candidate_index, timestamp=timestamp)

    @property
    def uncertainty_x(self) -> float:
        return self.localization_uncertainty_x_px

    @property
    def uncertainty_y(self) -> float:
        return self.localization_uncertainty_y_px

    @property
    def bbox(self) -> List[int]:
        return [int(self.x - self.sigma_x_px), int(self.y - self.sigma_y_px),
                int(self.x + self.sigma_x_px), int(self.y + self.sigma_y_px)]


@dataclass(frozen=True)
class AssociationResult:
    """
    Standard operational association output.
    Fed to Kalman filter and recorded to telemetry.
    """
    selected_candidate: Optional[CandidateMeasurement]
    selected_candidate_index: Optional[int]
    selected_measurement: Optional[Tuple[float, float]]
    status: str  # "ACCEPTED", "NO_CANDIDATE", "OUTSIDE_GATE", "AMBIGUOUS", "UNCONFIRMED_ACQUISITION"
    mahalanobis_distance: Optional[float]
    mahalanobis_gate_threshold: float
    association_cost: Optional[float]
    second_best_cost: Optional[float]
    association_margin: Optional[float]
    gated_candidate_count: int
    total_candidate_count: int
    association_latency_ms: float
    association_reason: str
    coasting_due_to_association_rejection: bool

    @property
    def accepted(self) -> bool:
        return self.status == "ACCEPTED"

    @property
    def selected_index(self) -> Optional[int]:
        return self.selected_candidate_index

    @property
    def mahalanobis_threshold(self) -> float:
        return self.mahalanobis_gate_threshold

    @property
    def reason(self) -> str:
        return self.association_reason

    def to_dict(self) -> Dict[str, Any]:
        return {
            "selected_candidate_index": self.selected_candidate_index,
            "selected_x": round(self.selected_measurement[0], 3) if self.selected_measurement else None,
            "selected_y": round(self.selected_measurement[1], 3) if self.selected_measurement else None,
            "status": self.status,
            "mahalanobis_distance": round(self.mahalanobis_distance, 3) if self.mahalanobis_distance is not None else None,
            "mahalanobis_gate_threshold": round(self.mahalanobis_gate_threshold, 3),
            "association_cost": round(self.association_cost, 4) if self.association_cost is not None else None,
            "second_best_cost": round(self.second_best_cost, 4) if self.second_best_cost is not None else None,
            "association_margin": round(self.association_margin, 4) if self.association_margin is not None else None,
            "gated_candidate_count": self.gated_candidate_count,
            "total_candidate_count": self.total_candidate_count,
            "association_latency_ms": round(self.association_latency_ms, 3),
            "association_reason": self.association_reason,
            "coasting_due_to_association_rejection": self.coasting_due_to_association_rejection
        }


class AssociationEngine:
    """
    Spatial-Temporal Beacon Association and Distractor Rejection Engine.
    """
    def __init__(
        self,
        mode: Literal["legacy_simple", "euclidean_prediction", "mahalanobis_only", "robust_multifeature"] = "robust_multifeature",
        chi2_gate_level: float = 9.210,  # 99% confidence for 2 DOF (chi2_2 = 9.210)
        ambiguity_margin: float = 0.08,
        switch_hysteresis_margin: float = 0.12,
        min_acquisition_confidence: float = 0.20,
        acquisition_confirmation_frames: int = 1,
        w_motion: float = 0.40,
        w_signal: float = 0.25,
        w_quality: float = 0.15,
        w_shape: float = 0.10,
        w_uncert: float = 0.10,
        image_width: int = 640,
        image_height: int = 480
    ):
        self.mode = str(mode).lower().strip()
        self.chi2_gate_threshold = float(chi2_gate_level)
        self.ambiguity_margin = float(ambiguity_margin)
        self.switch_hysteresis_margin = float(switch_hysteresis_margin)
        self.min_acquisition_confidence = float(min_acquisition_confidence)
        self.acquisition_confirmation_frames = int(acquisition_confirmation_frames)

        # Multi-feature weights (normalized to sum to 1.0)
        weights_sum = w_motion + w_signal + w_quality + w_shape + w_uncert
        self.w_motion = float(w_motion / weights_sum)
        self.w_signal = float(w_signal / weights_sum)
        self.w_quality = float(w_quality / weights_sum)
        self.w_shape = float(w_shape / weights_sum)
        self.w_uncert = float(w_uncert / weights_sum)

        self.image_width = int(image_width)
        self.image_height = int(image_height)
        self.boresight_x = float(image_width / 2.0)
        self.boresight_y = float(image_height / 2.0)

        # Track history state
        self.reset()

    def reset(self):
        """Reset association track continuity history."""
        self.last_selected_pos: Optional[Tuple[float, float]] = None
        self.last_selected_signal: Optional[float] = None
        self.last_selected_index: Optional[int] = None
        self.consecutive_track_frames: int = 0
        self.coasting_frames: int = 0
        self.pending_acquisition_candidate: Optional[Tuple[float, float]] = None
        self.pending_acquisition_count: int = 0

    def associate(
        self,
        candidates: List[CandidateMeasurement],
        kalman: KalmanFilter2D,
        timestamp: float
    ) -> AssociationResult:
        """
        Main association entry point. Evaluates candidate list against track state.
        """
        t0 = time.perf_counter()
        total_candidates = len(candidates)

        if total_candidates == 0:
            self.coasting_frames += 1
            self.consecutive_track_frames = max(0, self.consecutive_track_frames - 1)
            t_elapsed = (time.perf_counter() - t0) * 1000.0
            return AssociationResult(
                selected_candidate=None,
                selected_candidate_index=None,
                selected_measurement=None,
                status="NO_CANDIDATE",
                mahalanobis_distance=None,
                mahalanobis_gate_threshold=self.chi2_gate_threshold,
                association_cost=None,
                second_best_cost=None,
                association_margin=None,
                gated_candidate_count=0,
                total_candidate_count=0,
                association_latency_ms=t_elapsed,
                association_reason="Detector returned zero candidates",
                coasting_due_to_association_rejection=False
            )

        # -------------------------------------------------------------
        # Mode A: Legacy Simple (Phase 0D Baseline preserved for ablation)
        # -------------------------------------------------------------
        if self.mode == "legacy_simple":
            return self._associate_legacy_simple(candidates, kalman, t0)

        # -------------------------------------------------------------
        # Initial Acquisition: Kalman filter is not yet initialized
        # -------------------------------------------------------------
        if not kalman.is_initialized:
            return self._associate_initial_acquisition(candidates, t0)

        # -------------------------------------------------------------
        # Track Established: Forward Prediction & Statistical Validation
        # -------------------------------------------------------------
        x_pred, P_pred, z_pred = kalman.get_prediction(timestamp)
        H = kalman.H  # 2x4 measurement matrix

        # Mode B: Euclidean Prediction (Ablation Mode)
        if self.mode == "euclidean_prediction":
            return self._associate_euclidean_prediction(candidates, z_pred, t0)

        # -------------------------------------------------------------
        # Step 1: Statistical Mahalanobis Gating for each candidate
        # -------------------------------------------------------------
        # Dynamic gate threshold:
        # 1. Track initiation transient (consecutive updates <= 2): expand gate to allow
        #    initial velocity convergence and gimbal controller step transient.
        # 2. Coasting: expand gate smoothly with coast duration to permit re-acquisition.
        kf_updates = getattr(kalman, "consecutive_updates", self.consecutive_track_frames)
        if kf_updates <= 3:
            eff_gate_threshold = self.chi2_gate_threshold * 4.0
        elif kf_updates <= 15:
            eff_gate_threshold = self.chi2_gate_threshold * 2.0
        elif self.coasting_frames > 0:
            coast_expansion = min(3.0, 1.0 + 0.25 * self.coasting_frames)
            eff_gate_threshold = self.chi2_gate_threshold * coast_expansion
        else:
            eff_gate_threshold = self.chi2_gate_threshold

        gated_candidates: List[Tuple[CandidateMeasurement, float, np.ndarray]] = []

        for cand in candidates:
            # Candidate-specific measurement noise covariance R_i
            # Enforce safe physical bounds: sigma in [0.2, 5.0] px -> variance in [0.04, 25.0] px^2
            var_x = float(np.clip(cand.localization_uncertainty_x_px ** 2, 0.04, 25.0))
            var_y = float(np.clip(cand.localization_uncertainty_y_px ** 2, 0.04, 25.0))
            R_i = np.diag([var_x, var_y]).astype(np.float32)

            # Innovation covariance S_i = H P_pred H^T + R_i
            S_i = (H @ P_pred @ H.T) + R_i
            # Physical innovation floor against sub-pixel covariance collapse (sigma_min = 2.0px)
            S_i[0, 0] = max(float(S_i[0, 0]), 4.0)
            S_i[1, 1] = max(float(S_i[1, 1]), 4.0)

            # S_i inversion with condition regularization
            try:
                S_inv = np.linalg.inv(S_i)
            except np.linalg.LinAlgError:
                S_inv = np.linalg.inv(S_i + 1e-4 * np.eye(2, dtype=np.float32))

            # Innovation vector nu_i = z_i - z_pred
            z_i = np.array([cand.x, cand.y], dtype=np.float32)
            nu_i = z_i - z_pred

            # Mahalanobis distance squared D_i^2 = nu_i^T S_inv nu_i
            D_sq = float(nu_i.T @ S_inv @ nu_i)

            # Validation Gate check: D_i^2 <= eff_gate_threshold
            if D_sq <= eff_gate_threshold:
                gated_candidates.append((cand, D_sq, R_i))

        gated_count = len(gated_candidates)

        # If zero candidates survived the gate, reject all and coast
        if gated_count == 0:
            self.coasting_frames += 1
            self.consecutive_track_frames = max(0, self.consecutive_track_frames - 1)
            t_elapsed = (time.perf_counter() - t0) * 1000.0
            # Find closest candidate Mahalanobis distance for diagnostic logging
            S_diag = (H @ P_pred @ H.T) + np.eye(2, dtype=np.float32)
            S_diag[0, 0] = max(float(S_diag[0, 0]), 4.0)
            S_diag[1, 1] = max(float(S_diag[1, 1]), 4.0)
            try:
                S_diag_inv = np.linalg.inv(S_diag)
            except np.linalg.LinAlgError:
                S_diag_inv = np.linalg.inv(S_diag + 1e-4 * np.eye(2, dtype=np.float32))

            quad_vals = [
                float((np.array([c.x, c.y], dtype=np.float32) - z_pred).T @
                      S_diag_inv @
                      (np.array([c.x, c.y], dtype=np.float32) - z_pred))
                for c in candidates
            ]
            min_d = float(np.sqrt(max(0.0, min(quad_vals)))) if quad_vals else 0.0
            return AssociationResult(
                selected_candidate=None,
                selected_candidate_index=None,
                selected_measurement=None,
                status="OUTSIDE_GATE",
                mahalanobis_distance=min_d,
                mahalanobis_gate_threshold=eff_gate_threshold,
                association_cost=None,
                second_best_cost=None,
                association_margin=None,
                gated_candidate_count=0,
                total_candidate_count=total_candidates,
                association_latency_ms=t_elapsed,
                association_reason="All candidates rejected by Mahalanobis validation gate",
                coasting_due_to_association_rejection=True
            )

        # -------------------------------------------------------------
        # Mode C: Mahalanobis Only (Ablation Mode)
        # -------------------------------------------------------------
        if self.mode == "mahalanobis_only":
            best_cand, best_d_sq, best_R = min(gated_candidates, key=lambda item: item[1])
            best_d = float(np.sqrt(best_d_sq))
            self._update_track_history(best_cand)
            t_elapsed = (time.perf_counter() - t0) * 1000.0
            return AssociationResult(
                selected_candidate=best_cand,
                selected_candidate_index=best_cand.candidate_index,
                selected_measurement=(best_cand.x, best_cand.y),
                status="ACCEPTED",
                mahalanobis_distance=best_d,
                mahalanobis_gate_threshold=eff_gate_threshold,
                association_cost=best_d_sq / self.chi2_gate_threshold,
                second_best_cost=None,
                association_margin=None,
                gated_candidate_count=gated_count,
                total_candidate_count=total_candidates,
                association_latency_ms=t_elapsed,
                association_reason="Selected minimum Mahalanobis distance inside gate",
                coasting_due_to_association_rejection=False
            )

        # -------------------------------------------------------------
        # Mode D: Robust Multi-Feature Association (Production Default)
        # -------------------------------------------------------------
        scored_candidates: List[Tuple[CandidateMeasurement, float, float, np.ndarray]] = []

        for cand, d_sq, R_i in gated_candidates:
            # 1. Motion cost (normalized Mahalanobis distance squared in [0, 1])
            c_motion = float(np.clip(d_sq / self.chi2_gate_threshold, 0.0, 1.0))

            # 2. Signal continuity cost (flux stability relative to established track)
            if self.last_selected_signal is not None and self.last_selected_signal > 10.0:
                sig_diff = abs(cand.integrated_signal - self.last_selected_signal)
                c_signal = float(np.clip(sig_diff / max(self.last_selected_signal, 100.0), 0.0, 1.0))
            else:
                c_signal = 0.0

            # 3. Detector confidence penalty in [0, 1]
            c_quality = float(np.clip(1.0 - cand.confidence, 0.0, 1.0))

            # 4. Shape quality penalty (ellipticity / departure from circular PSF)
            c_shape = float(np.clip(1.0 - cand.shape_quality, 0.0, 1.0))

            # 5. Localization uncertainty penalty in [0, 1]
            max_unc = max(cand.localization_uncertainty_x_px, cand.localization_uncertainty_y_px)
            c_uncert = float(np.clip(max_unc / 2.5, 0.0, 1.0))

            # Total multi-feature association cost
            total_cost = (
                self.w_motion * c_motion +
                self.w_signal * c_signal +
                self.w_quality * c_quality +
                self.w_shape * c_shape +
                self.w_uncert * c_uncert
            )

            # Apply switch hysteresis penalty if candidate is not the previously tracked candidate
            if self.last_selected_pos is not None and self.consecutive_track_frames >= 5:
                dist_to_prev = float(np.hypot(cand.x - self.last_selected_pos[0], cand.y - self.last_selected_pos[1]))
                if dist_to_prev > 15.0:
                    total_cost += self.switch_hysteresis_margin

            d_val = float(np.sqrt(d_sq))
            scored_candidates.append((cand, total_cost, d_val, R_i))

        # Sort by total cost ascending
        scored_candidates.sort(key=lambda item: item[1])

        best_cand, best_cost, best_d, best_R = scored_candidates[0]
        second_cost = scored_candidates[1][1] if len(scored_candidates) > 1 else None
        margin = float(second_cost - best_cost) if second_cost is not None else 1.0

        # Ambiguity check: if top 2 candidates are both inside gate with very similar cost
        if len(scored_candidates) > 1:
            if margin < self.ambiguity_margin:
                # Ambiguous crossing detected: coast rather than latching onto wrong target
                self.coasting_frames += 1
                self.consecutive_track_frames = max(0, self.consecutive_track_frames - 1)
                t_elapsed = (time.perf_counter() - t0) * 1000.0
                return AssociationResult(
                    selected_candidate=None,
                    selected_candidate_index=None,
                    selected_measurement=None,
                    status="AMBIGUOUS",
                    mahalanobis_distance=best_d,
                    mahalanobis_gate_threshold=eff_gate_threshold,
                    association_cost=best_cost,
                    second_best_cost=second_cost,
                    association_margin=margin,
                    gated_candidate_count=gated_count,
                    total_candidate_count=total_candidates,
                    association_latency_ms=t_elapsed,
                    association_reason=f"Ambiguous association: cost margin {margin:.4f} < {self.ambiguity_margin}",
                    coasting_due_to_association_rejection=True
                )

        # Accept best candidate
        self._update_track_history(best_cand)
        t_elapsed = (time.perf_counter() - t0) * 1000.0
        return AssociationResult(
            selected_candidate=best_cand,
            selected_candidate_index=best_cand.candidate_index,
            selected_measurement=(best_cand.x, best_cand.y),
            status="ACCEPTED",
            mahalanobis_distance=best_d,
            mahalanobis_gate_threshold=eff_gate_threshold,
            association_cost=best_cost,
            second_best_cost=second_cost,
            association_margin=margin,
            gated_candidate_count=gated_count,
            total_candidate_count=total_candidates,
            association_latency_ms=t_elapsed,
            association_reason="Accepted valid candidate meeting statistical gate & multi-feature criteria",
            coasting_due_to_association_rejection=False
        )

    def _associate_initial_acquisition(
        self,
        candidates: List[CandidateMeasurement],
        t_start: float
    ) -> AssociationResult:
        """
        Initial acquisition policy before Kalman filter initialization.
        Ranks by optical quality, confidence, and distance from sensor boresight.
        Requires temporal confirmation when multiple candidates exist.
        """
        # Filter candidates meeting minimum confidence
        valid_acq = [c for c in candidates if c.confidence >= self.min_acquisition_confidence]
        if not valid_acq:
            valid_acq = candidates

        # If only a single candidate exists, initialize immediately
        if len(candidates) == 1:
            cand = candidates[0]
            self._update_track_history(cand)
            t_elapsed = (time.perf_counter() - t_start) * 1000.0
            return AssociationResult(
                selected_candidate=cand,
                selected_candidate_index=cand.candidate_index,
                selected_measurement=(cand.x, cand.y),
                status="ACCEPTED",
                mahalanobis_distance=0.0,
                mahalanobis_gate_threshold=self.chi2_gate_threshold,
                association_cost=0.0,
                second_best_cost=None,
                association_margin=1.0,
                gated_candidate_count=1,
                total_candidate_count=1,
                association_latency_ms=t_elapsed,
                association_reason="Initial acquisition accepted single candidate",
                coasting_due_to_association_rejection=False
            )

        # Multi-candidate acquisition scoring:
        # Prefer higher confidence, higher SNR, and closer to boresight center
        max_diag = float(np.hypot(self.image_width, self.image_height))

        def acq_score(c: CandidateMeasurement) -> float:
            d_boresight = float(np.hypot(c.x - self.boresight_x, c.y - self.boresight_y)) / max_diag
            score = (
                0.40 * c.confidence +
                0.35 * min(1.0, c.snr_proxy / 20.0) +
                0.25 * (1.0 - d_boresight)
            )
            return score

        scored = sorted(valid_acq, key=acq_score, reverse=True)
        best_cand = scored[0]

        # Check acquisition confirmation frames
        if self.acquisition_confirmation_frames <= 1:
            self._update_track_history(best_cand)
            t_elapsed = (time.perf_counter() - t_start) * 1000.0
            return AssociationResult(
                selected_candidate=best_cand,
                selected_candidate_index=best_cand.candidate_index,
                selected_measurement=(best_cand.x, best_cand.y),
                status="ACCEPTED",
                mahalanobis_distance=0.0,
                mahalanobis_gate_threshold=self.chi2_gate_threshold,
                association_cost=0.0,
                second_best_cost=None,
                association_margin=1.0,
                gated_candidate_count=len(valid_acq),
                total_candidate_count=len(candidates),
                association_latency_ms=t_elapsed,
                association_reason="Initial acquisition accepted best candidate",
                coasting_due_to_association_rejection=False
            )

        # Require consecutive confirmation across frames before initiating track lock
        if self.pending_acquisition_candidate is not None:
            dist = float(np.hypot(best_cand.x - self.pending_acquisition_candidate[0], best_cand.y - self.pending_acquisition_candidate[1]))
            if dist <= 30.0:
                self.pending_acquisition_count += 1
            else:
                self.pending_acquisition_candidate = (best_cand.x, best_cand.y)
                self.pending_acquisition_count = 1
        else:
            self.pending_acquisition_candidate = (best_cand.x, best_cand.y)
            self.pending_acquisition_count = 1

        if self.pending_acquisition_count >= self.acquisition_confirmation_frames:
            self._update_track_history(best_cand)
            t_elapsed = (time.perf_counter() - t_start) * 1000.0
            return AssociationResult(
                selected_candidate=best_cand,
                selected_candidate_index=best_cand.candidate_index,
                selected_measurement=(best_cand.x, best_cand.y),
                status="ACCEPTED",
                mahalanobis_distance=0.0,
                mahalanobis_gate_threshold=self.chi2_gate_threshold,
                association_cost=0.0,
                second_best_cost=None,
                association_margin=1.0,
                gated_candidate_count=len(valid_acq),
                total_candidate_count=len(candidates),
                association_latency_ms=t_elapsed,
                association_reason="Initial acquisition confirmed after multi-frame persistence",
                coasting_due_to_association_rejection=False
            )

        t_elapsed = (time.perf_counter() - t_start) * 1000.0
        return AssociationResult(
            selected_candidate=None,
            selected_candidate_index=None,
            selected_measurement=None,
            status="UNCONFIRMED_ACQUISITION",
            mahalanobis_distance=None,
            mahalanobis_gate_threshold=self.chi2_gate_threshold,
            association_cost=None,
            second_best_cost=None,
            association_margin=None,
            gated_candidate_count=len(valid_acq),
            total_candidate_count=len(candidates),
            association_latency_ms=t_elapsed,
            association_reason=f"Acquisition pending temporal confirmation ({self.pending_acquisition_count}/{self.acquisition_confirmation_frames})",
            coasting_due_to_association_rejection=False
        )

    def _associate_legacy_simple(
        self,
        candidates: List[CandidateMeasurement],
        kalman: KalmanFilter2D,
        t_start: float
    ) -> AssociationResult:
        """Exact Phase 0D inherited candidate selection preserved for ablation."""
        if len(candidates) == 1:
            best_cand = candidates[0]
        else:
            if kalman.is_initialized:
                prev_pos = kalman.state[:2]
                best_cand = min(
                    candidates,
                    key=lambda c: (
                        float(np.hypot(c.x - prev_pos[0], c.y - prev_pos[1])),
                        -float(c.confidence),
                        -float(c.area)
                    )
                )
            else:
                best_cand = max(
                    candidates,
                    key=lambda c: (float(c.confidence), float(c.area))
                )

        t_elapsed = (time.perf_counter() - t_start) * 1000.0
        return AssociationResult(
            selected_candidate=best_cand,
            selected_candidate_index=best_cand.candidate_index,
            selected_measurement=(best_cand.x, best_cand.y),
            status="ACCEPTED",
            mahalanobis_distance=None,
            mahalanobis_gate_threshold=self.chi2_gate_threshold,
            association_cost=0.0,
            second_best_cost=None,
            association_margin=None,
            gated_candidate_count=len(candidates),
            total_candidate_count=len(candidates),
            association_latency_ms=t_elapsed,
            association_reason="Legacy simple nearest-neighbor / max-confidence selection",
            coasting_due_to_association_rejection=False
        )

    def _associate_euclidean_prediction(
        self,
        candidates: List[CandidateMeasurement],
        z_pred: np.ndarray,
        t_start: float
    ) -> AssociationResult:
        """Euclidean distance to forward prediction z_pred with fixed radius gate."""
        gate_radius_px = 35.0
        in_gate = []
        for c in candidates:
            dist = float(np.hypot(c.x - z_pred[0], c.y - z_pred[1]))
            if dist <= gate_radius_px:
                in_gate.append((c, dist))

        if not in_gate:
            t_elapsed = (time.perf_counter() - t_start) * 1000.0
            return AssociationResult(
                selected_candidate=None,
                selected_candidate_index=None,
                selected_measurement=None,
                status="OUTSIDE_GATE",
                mahalanobis_distance=None,
                mahalanobis_gate_threshold=self.chi2_gate_threshold,
                association_cost=None,
                second_best_cost=None,
                association_margin=None,
                gated_candidate_count=0,
                total_candidate_count=len(candidates),
                association_latency_ms=t_elapsed,
                association_reason=f"All candidates outside Euclidean gate radius ({gate_radius_px}px)",
                coasting_due_to_association_rejection=True
            )

        in_gate.sort(key=lambda item: item[1])
        best_cand, best_dist = in_gate[0]
        self._update_track_history(best_cand)
        t_elapsed = (time.perf_counter() - t_start) * 1000.0
        return AssociationResult(
            selected_candidate=best_cand,
            selected_candidate_index=best_cand.candidate_index,
            selected_measurement=(best_cand.x, best_cand.y),
            status="ACCEPTED",
            mahalanobis_distance=None,
            mahalanobis_gate_threshold=self.chi2_gate_threshold,
            association_cost=best_dist / gate_radius_px,
            second_best_cost=in_gate[1][1] / gate_radius_px if len(in_gate) > 1 else None,
            association_margin=None,
            gated_candidate_count=len(in_gate),
            total_candidate_count=len(candidates),
            association_latency_ms=t_elapsed,
            association_reason="Selected nearest candidate to predicted position within Euclidean gate",
            coasting_due_to_association_rejection=False
        )

    def _update_track_history(self, cand: CandidateMeasurement):
        """Updates internal track statistics for signal and motion continuity."""
        self.last_selected_pos = (cand.x, cand.y)
        self.last_selected_index = cand.candidate_index
        # Exponential moving average of integrated signal
        if self.last_selected_signal is None:
            self.last_selected_signal = cand.integrated_signal
        else:
            self.last_selected_signal = 0.85 * self.last_selected_signal + 0.15 * cand.integrated_signal
        self.consecutive_track_frames += 1
        self.coasting_frames = 0
        self.pending_acquisition_candidate = None
        self.pending_acquisition_count = 0
