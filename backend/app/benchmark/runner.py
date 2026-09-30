"""
Headless Deterministic Scientific Benchmark Engine.
Executes fixed-timestep simulation independent of browser, WebSocket, and Three.js.
Authoritative source for algorithm verification and baseline measurement.
"""
import os
import sys
import json
import time
import random
import platform
import subprocess
import dataclasses
import csv
from typing import Dict, Any, List, Optional, Tuple

import numpy as np

from app.camera.model import CameraModel
from app.camera.calibration import CameraCalibration
from app.detection.detector import OpenCVDetector
from app.detection.disturbances import DisturbanceEngine
from app.tracking.kalman import KalmanFilter2D
from app.tracking.state_machine import TrackingStateMachine, LegacyTrackingStateMachine
from app.tracking.pat_state_machine import PATLifecycleStateMachine, PATFSMConfig, PATState
from app.tracking.pid import PIDController, LegacyPIDController
from app.tracking.controller import GimbalController, GimbalControllerConfig, ControlMode
from app.tracking.association import AssociationEngine, CandidateMeasurement
from app.benchmark.renderer import SyntheticFrameRenderer
from app.benchmark.scenarios import BenchmarkScenario, SCENARIO_REGISTRY
from app.benchmark.metrics import BenchmarkMetricsCalculator
from app.benchmark.plot import generate_benchmark_plots


@dataclasses.dataclass
class BenchmarkRunResult:
    experiment_id: str
    scenario_id: str
    seed: int
    dt: float
    duration: float
    output_dir: str
    metrics: Dict[str, Any]
    telemetry: List[Dict[str, Any]]
    events: List[Dict[str, Any]]
    metadata: Dict[str, Any]


class BenchmarkEngine:
    """
    Fixed-timestep deterministic simulation and evaluation runner.
    """
    def __init__(
        self,
        dt: float = 0.02,
        camera_hfov: float = 10.0,
        camera_vfov: float = 7.5,
        width: int = 640,
        height: int = 480,
        localization_mode: str = "weighted_centroid",
        association_mode: str = "robust_multifeature",
        estimator_mode: str = "ca6_adaptive_r",
        fsm_mode: str = "new_pat_fsm",
        pat_fsm_config: Optional[PATFSMConfig] = None,
        controller_mode: str = "filtered_pid",
        controller_config: Optional[GimbalControllerConfig] = None,
        injected_latency_s: float = 0.0,
        search_pattern_type: Optional[str] = None
    ):
        self.dt = float(dt)
        self.width = int(width)
        self.height = int(height)
        self.localization_mode = str(localization_mode)
        self.association_mode = str(association_mode)
        self.estimator_mode = str(estimator_mode)
        self.fsm_mode = str(fsm_mode)
        self.pat_fsm_config = pat_fsm_config or PATFSMConfig()
        if search_pattern_type:
            self.pat_fsm_config.search_config.pattern_type = str(search_pattern_type)
        self.controller_mode = str(controller_mode)
        self.controller_config = controller_config or GimbalControllerConfig()
        self.injected_latency_s = float(injected_latency_s)
        self.calibration = CameraCalibration.from_vfov(
            width=self.width,
            height=self.height,
            vfov_deg=camera_vfov
        )
        self.camera_hfov = self.calibration.hfov_deg
        self.camera_vfov = self.calibration.vfov_deg

    @staticmethod
    def get_git_commit() -> str:
        """Fetch current git commit SHA if available."""
        try:
            res = subprocess.run(
                ["git", "rev-parse", "HEAD"],
                capture_output=True,
                text=True,
                check=False
            )
            if res.returncode == 0:
                return res.stdout.strip()
        except Exception:
            pass
        return "UNKNOWN_COMMIT"

    def run_scenario(
        self,
        scenario: BenchmarkScenario,
        seed: int = 42,
        duration: Optional[float] = None,
        output_base_dir: str = "results",
        save_plots: bool = True
    ) -> BenchmarkRunResult:
        """
        Execute a complete deterministic headless benchmark run for a scenario.
        """
        sim_duration = float(duration) if duration is not None else float(scenario.duration)
        n_steps = int(round(sim_duration / self.dt))

        # -------------------------------------------------------------
        # 1. Deterministic Seed Initialization
        # -------------------------------------------------------------
        random.seed(seed)
        np.random.seed(seed)

        # Unique experiment ID
        timestamp_str = time.strftime("%Y%m%d_%H%M%S")
        exp_id = f"BENCH_{scenario.id}_seed{seed}_{timestamp_str}"
        exp_dir = os.path.join(output_base_dir, exp_id)
        os.makedirs(exp_dir, exist_ok=True)

        # -------------------------------------------------------------
        # 2. Instantiate Phase 0D Baseline Hardware & Tracking Objects
        # -------------------------------------------------------------
        camera_model = CameraModel(
            width=self.width,
            height=self.height,
            calibration=self.calibration,
            camera_pos=np.array([0.0, 1.5, 0.0])
        )
        renderer = SyntheticFrameRenderer(
            width=self.width,
            height=self.height,
            hfov_deg=self.camera_hfov,
            vfov_deg=self.camera_vfov,
            camera_pos=np.array([0.0, 1.5, 0.0])
        )
        detector = OpenCVDetector(
            hsv_lower=[35, 50, 50],
            hsv_upper=[95, 255, 255],
            localization_mode=self.localization_mode
        )
        disturbance_engine = DisturbanceEngine()
        disturbance_engine.load_from_config(scenario.disturbances)

        kalman = KalmanFilter2D(
            mode=self.estimator_mode,
            q_noise=0.5,
            q_jerk=10000.0,
            r_noise=1.0,
            max_prediction_seconds=2.0
        )
        if self.fsm_mode == "legacy_fsm":
            state_machine = LegacyTrackingStateMachine(
                lock_threshold_deg=0.15,
                lock_duration_sec=1.0
            )
        else:
            state_machine = PATLifecycleStateMachine(
                config=self.pat_fsm_config
            )

        ctrl_mode_enum = ControlMode(self.controller_mode) if self.controller_mode in [m.value for m in ControlMode] else ControlMode.PREDICTIVE_SMOOTH
        gimbal_controller = GimbalController(config=self.controller_config, mode=ctrl_mode_enum)
        association_engine = AssociationEngine(
            mode=self.association_mode,
            image_width=self.width,
            image_height=self.height
        )

        # Tracking loop state variables
        camera_pan = float(scenario.initial_pan_deg)
        camera_tilt = float(scenario.initial_tilt_deg)
        pan_rate_cmd = 0.0
        tilt_rate_cmd = 0.0
        telemetry_rows: List[Dict[str, Any]] = []
        events: List[Dict[str, Any]] = []
        prev_evaluated_candidate_id: Optional[str] = None
        prev_sm_state = "SEARCHING" if self.fsm_mode == "legacy_fsm" else "SEARCH"

        # -------------------------------------------------------------
        # 3. Deterministic Headless Step Loop
        # -------------------------------------------------------------
        for step_idx in range(n_steps):
            sim_time = step_idx * self.dt

            # 3.1 Physics & Beacon Propagation
            beacons = scenario.get_beacons(sim_time)

            # 3.2 Headless Photon Rendering onto Image Sensor
            raw_frame, truth_projections = renderer.render(beacons, camera_pan, camera_tilt)

            # 3.3 Apply Atmospheric / Optical Disturbances
            frame = disturbance_engine.apply(raw_frame, sim_time)

            # ---------------------------------------------------------
            # Operational Pipeline: Perception -> Tracking -> Control
            # ---------------------------------------------------------
            t_pipeline_start = time.perf_counter()

            # --- DETECTOR EXECUTION ---
            t_det_start = time.perf_counter()
            det_res = detector.detect(frame)
            det_latency_ms = (time.perf_counter() - t_det_start) * 1000.0

            detected_candidates = det_res.get("candidates", [])
            candidate_measurements = [
                CandidateMeasurement.from_detector_candidate(c, idx, sim_time)
                for idx, c in enumerate(detected_candidates)
            ]

            # --- ASSOCIATION ENGINE UPDATE ---
            t_assoc_start = time.perf_counter()
            assoc_res = association_engine.associate(candidate_measurements, kalman, sim_time)
            assoc_latency_ms = (time.perf_counter() - t_assoc_start) * 1000.0

            target_detected = assoc_res.accepted
            if target_detected and assoc_res.selected_candidate is not None:
                target_center = [assoc_res.selected_candidate.x, assoc_res.selected_candidate.y]
                target_confidence = float(assoc_res.selected_candidate.confidence)
                cand_ux = assoc_res.selected_candidate.uncertainty_x
                cand_uy = assoc_res.selected_candidate.uncertainty_y
                R_candidate = np.diag([cand_ux ** 2, cand_uy ** 2])
            else:
                target_center = [0.0, 0.0]
                target_confidence = 0.0
                cand_ux = None
                cand_uy = None
                R_candidate = None

            coasting_due_to_rejection = bool(
                kalman.is_initialized and not target_detected and len(detected_candidates) > 0
            )

            # --- KALMAN FILTER UPDATE ---
            t_est_start = time.perf_counter()
            if target_detected:
                center_pt = np.array(target_center, dtype=np.float32)
                q_flags = assoc_res.selected_candidate.quality_flags if assoc_res.selected_candidate else None
                kf_state = kalman.update(center_pt, sim_time, R_custom=R_candidate, quality_flags=q_flags)
            else:
                kf_state = kalman.predict_only(sim_time)

            is_sm_detected = kalman.is_initialized
            is_occluded = not target_detected and is_sm_detected

            if kalman.is_initialized:
                timeout = kalman.check_occlusion_timeout(sim_time)
                if timeout:
                    state_machine.force_lost(sim_time)
                    is_occluded = False

            # --- ANGULAR ERROR FROM ESTIMATOR ---
            h_err = 0.0
            v_err = 0.0
            total_err = 0.0

            if is_sm_detected or target_detected:
                kx, ky = kf_state[0], kf_state[1]
                dx = kx - camera_model.cx
                dy = camera_model.cy - ky
                h_err = float(np.degrees(np.arctan2(dx, camera_model.fx)))
                v_err = float(np.degrees(np.arctan2(dy, camera_model.fy)))
                total_err = float(np.sqrt(h_err ** 2 + v_err ** 2))

            # --- STATE MACHINE UPDATE ---
            if self.fsm_mode == "legacy_fsm":
                sm_state = state_machine.update(
                    detected=is_sm_detected,
                    is_occluded=is_occluded,
                    angular_error_deg=total_err,
                    current_time=sim_time
                )
                search_pan_offset = 0.0
                search_tilt_offset = 0.0
                pat_telemetry = {
                    "pat_state": sm_state,
                    "previous_pat_state": prev_sm_state,
                    "state_duration_s": 0.0,
                    "transition_reason": "LEGACY_UPDATE",
                    "consecutive_accepted_measurements": 0,
                    "consecutive_rejected_measurements": 0,
                    "coast_duration_s": 0.0,
                    "reacquisition_duration_s": 0.0,
                    "estimator_health": kalman.get_health_status(sim_time),
                    "position_sigma_px": round(float(np.sqrt(max(0.0, kalman.covariance[0, 0] + kalman.covariance[1, 1]))), 3) if kalman.is_initialized else 0.0,
                    "association_status": str(assoc_res.status),
                    "lock_eligible": False,
                    "search_pattern": "none",
                    "search_offset_pan_deg": 0.0,
                    "search_offset_tilt_deg": 0.0,
                    "reacquisition_attempt_count": 0
                }
            else:
                pos_sigma = float(np.sqrt(max(0.0, kalman.covariance[0, 0] + kalman.covariance[1, 1]))) if kalman.is_initialized else 10.0
                est_health = kalman.get_health_status(sim_time)
                kf_mean = kalman.state[:2] if kalman.is_initialized else None
                kf_cov = kalman.covariance[:2, :2] if kalman.is_initialized else None
                time_since_meas = (sim_time - kalman.last_detection_time) if kalman.is_initialized else 0.0

                sm_state = state_machine.update(
                    association_status=str(assoc_res.status),
                    measurement_accepted=bool(target_detected),
                    candidate_count=int(len(detected_candidates)),
                    estimator_health=est_health,
                    position_sigma_px=pos_sigma,
                    angular_error_deg=total_err,
                    current_time=sim_time,
                    kf_pos_mean=kf_mean,
                    kf_pos_cov=kf_cov,
                    time_since_measurement=time_since_meas
                )
                pat_telemetry = state_machine.get_telemetry(sim_time)
                search_pan_offset = pat_telemetry["search_offset_pan_deg"]
                search_tilt_offset = pat_telemetry["search_offset_tilt_deg"]

            # --- CLOSED-LOOP GIMBAL CONTROLLER ACTUATION ---
            # Compute commands using authoritative GimbalController (C0, C1, C2, C3)
            # In REACQUIRE state, search scan commands overlay on top of pointing control
            pan_rate_cmd, tilt_rate_cmd, ctrl_telem = gimbal_controller.compute_control(
                kf_state=kf_state if (kalman.is_initialized and (is_sm_detected or target_detected)) else None,
                camera_cx=camera_model.cx,
                camera_cy=camera_model.cy,
                camera_fx=camera_model.fx,
                camera_fy=camera_model.fy,
                dt=self.dt,
                pat_state=sm_state,
                search_pan_offset_deg=search_pan_offset,
                search_tilt_offset_deg=search_tilt_offset,
                injected_latency_s=self.injected_latency_s
            )
            is_saturated = ctrl_telem.get("rate_saturated", False) or ctrl_telem.get("acceleration_saturated", False)

            # Update gimbal angles for next step (Euler integration with physical angle bounds matching Mode B)
            camera_pan = max(-360.0, min(360.0, camera_pan + pan_rate_cmd * self.dt))
            camera_tilt = max(-90.0, min(90.0, camera_tilt + tilt_rate_cmd * self.dt))

            est_latency_ms = (time.perf_counter() - t_est_start) * 1000.0
            pipeline_latency_ms = (time.perf_counter() - t_pipeline_start) * 1000.0

            # Record FSM State Transition Event
            if sm_state != prev_sm_state:
                events.append({
                    "simulation_time": round(sim_time, 4),
                    "event_type": "FSM_TRANSITION",
                    "from_state": prev_sm_state,
                    "to_state": sm_state,
                    "angular_error_deg": round(total_err, 4)
                })
                prev_sm_state = sm_state

            # ---------------------------------------------------------
            # 3.5 Ground Truth Evaluation (STRICT FIREWALL: EVALUATION ONLY)
            # ---------------------------------------------------------
            designated_gt = next((p for p in truth_projections if p["id"] == scenario.designated_target_id), None)
            
            gt_px_x = designated_gt["pixel_x"] if designated_gt else None
            gt_px_y = designated_gt["pixel_y"] if designated_gt else None
            gt_in_fov = designated_gt["is_in_fov"] if designated_gt else False
            gt_active = designated_gt["is_active"] if designated_gt else False
            gt_optically_present = designated_gt.get("is_optically_present", gt_active and gt_in_fov) if designated_gt else False
            gt_detectable = designated_gt.get("is_detectable", gt_optically_present) if designated_gt else False

            # True pointing error between optical axis and true target
            if designated_gt:
                true_tot_err_deg, _, _ = camera_model.compute_angular_errors(
                    np.array(designated_gt["world_position"]),
                    np.radians(camera_pan),
                    np.radians(camera_tilt)
                )
                true_px_err = float(np.hypot(gt_px_x - camera_model.cx, gt_px_y - camera_model.cy)) if gt_in_fov else None
            else:
                true_tot_err_deg = None
                true_px_err = None

            # Evaluation-Only Candidate Identity Association
            # Matches selected candidate with ground truth to detect false locks
            evaluated_cand_id = None
            if target_detected:
                min_dist = float("inf")
                closest_id = None
                for tp in truth_projections:
                    if tp["is_active"] and tp["is_in_fov"]:
                        dist = float(np.hypot(target_center[0] - tp["pixel_x"], target_center[1] - tp["pixel_y"]))
                        if dist < min_dist:
                            min_dist = dist
                            closest_id = tp["id"]
                if min_dist <= 25.0:
                    evaluated_cand_id = closest_id
                else:
                    evaluated_cand_id = "FALSE_ARTIFACT"

            is_identity_switch = False
            if evaluated_cand_id is not None and prev_evaluated_candidate_id is not None:
                if evaluated_cand_id != prev_evaluated_candidate_id:
                    is_identity_switch = True
            if evaluated_cand_id is not None:
                prev_evaluated_candidate_id = evaluated_cand_id

            # Evaluation-Only Reacquisition Covariance Consistency Check (Strict Truth Firewall)
            target_in_1sigma = None
            target_in_2sigma = None
            target_in_95pct = None
            target_in_99pct = None
            reacq_d2 = None
            if sm_state == "REACQUIRE" and designated_gt and gt_active and gt_px_x is not None:
                if kalman.is_initialized:
                    dx_err = gt_px_x - kalman.state[0]
                    dy_err = gt_px_y - kalman.state[1]
                    P_sub = kalman.covariance[0:2, 0:2]
                    try:
                        P_inv = np.linalg.inv(P_sub)
                        d2_val = float(np.dot([dx_err, dy_err], P_inv @ [dx_err, dy_err]))
                        reacq_d2 = round(d2_val, 4)
                        target_in_1sigma = bool(d2_val <= 1.0)
                        target_in_2sigma = bool(d2_val <= 4.0)
                        target_in_95pct = bool(d2_val <= 5.991465)
                        target_in_99pct = bool(d2_val <= 9.210340)
                    except Exception:
                        pass

            # ---------------------------------------------------------
            # 3.6 Frame-by-Frame Telemetry Log
            # ---------------------------------------------------------
            telemetry_row = {
                "simulation_time": round(sim_time, 4),
                "ground_truth_target_x": round(gt_px_x, 2) if gt_px_x is not None else None,
                "ground_truth_target_y": round(gt_px_y, 2) if gt_px_y is not None else None,
                "ground_truth_target_in_fov": bool(gt_in_fov),
                "ground_truth_target_active": bool(gt_active),
                "ground_truth_target_optically_present": bool(gt_optically_present),
                "ground_truth_target_detectable": bool(gt_detectable),
                "detection_available": bool(target_detected),
                "raw_measurement_x": round(float(target_center[0]), 2) if target_detected else None,
                "raw_measurement_y": round(float(target_center[1]), 2) if target_detected else None,
                "measurement_confidence": round(float(target_confidence), 4),
                "selected_candidate_x": round(float(target_center[0]), 2) if target_detected else None,
                "selected_candidate_y": round(float(target_center[1]), 2) if target_detected else None,
                "candidate_count": int(len(detected_candidates)),
                "gated_candidate_count": int(assoc_res.gated_candidate_count),
                "selected_candidate_index": assoc_res.selected_index,
                "selected_candidate_uncertainty_x": round(float(cand_ux), 4) if cand_ux is not None else None,
                "selected_candidate_uncertainty_y": round(float(cand_uy), 4) if cand_uy is not None else None,
                "mahalanobis_distance": round(float(assoc_res.mahalanobis_distance), 4) if assoc_res.mahalanobis_distance is not None else None,
                "mahalanobis_gate_threshold": round(float(assoc_res.mahalanobis_threshold), 4) if assoc_res.mahalanobis_threshold is not None else None,
                "association_cost": round(float(assoc_res.association_cost), 4) if assoc_res.association_cost is not None else None,
                "second_best_cost": round(float(assoc_res.second_best_cost), 4) if assoc_res.second_best_cost is not None else None,
                "association_margin": round(float(assoc_res.association_margin), 4) if assoc_res.association_margin is not None else None,
                "association_status": str(assoc_res.status),
                "association_reason": str(assoc_res.reason),
                "association_latency_ms": round(float(assoc_latency_ms), 3),
                "coasting_due_to_association_rejection": bool(coasting_due_to_rejection),
                "estimated_x": round(float(kf_state[0]), 2) if kalman.is_initialized else None,
                "estimated_y": round(float(kf_state[1]), 2) if kalman.is_initialized else None,
                "estimated_vx": round(float(kf_state[2]), 2) if kalman.is_initialized else None,
                "estimated_vy": round(float(kf_state[3]), 2) if kalman.is_initialized else None,
                "tracking_state": sm_state,
                "camera_pan": round(camera_pan, 4),
                "camera_tilt": round(camera_tilt, 4),
                "command_pan_rate": round(pan_rate_cmd, 4),
                "command_tilt_rate": round(tilt_rate_cmd, 4),
                "tracking_error_px": round(true_px_err, 3) if true_px_err is not None else None,
                "tracking_error_deg": round(true_tot_err_deg, 4) if true_tot_err_deg is not None else None,
                "detector_latency_ms": round(det_latency_ms, 3),
                "estimator_latency_ms": round(est_latency_ms, 3),
                "pipeline_latency_ms": round(pipeline_latency_ms, 3),
                "controller_saturated": bool(is_saturated),
                "evaluated_candidate_id": evaluated_cand_id,
                "identity_switch": bool(is_identity_switch),
                "pat_state": pat_telemetry["pat_state"],
                "previous_pat_state": pat_telemetry["previous_pat_state"],
                "state_duration_s": pat_telemetry["state_duration_s"],
                "transition_reason": pat_telemetry["transition_reason"],
                "consecutive_accepted_measurements": pat_telemetry["consecutive_accepted_measurements"],
                "consecutive_rejected_measurements": pat_telemetry["consecutive_rejected_measurements"],
                "coast_duration_s": pat_telemetry["coast_duration_s"],
                "reacquisition_duration_s": pat_telemetry["reacquisition_duration_s"],
                "estimator_health": pat_telemetry["estimator_health"],
                "position_sigma_px": pat_telemetry["position_sigma_px"],
                "lock_eligible": pat_telemetry["lock_eligible"],
                "search_pattern": pat_telemetry["search_pattern"],
                "search_offset_pan_deg": pat_telemetry["search_offset_pan_deg"],
                "search_offset_tilt_deg": pat_telemetry["search_offset_tilt_deg"],
                "reacquisition_attempt_count": pat_telemetry["reacquisition_attempt_count"],
                "search_stage": pat_telemetry.get("search_stage", "none"),
                "search_confidence_region": pat_telemetry.get("search_confidence_region", "none"),
                "search_mahalanobis_radius": pat_telemetry.get("search_mahalanobis_radius", 0.0),
                "search_point_index": pat_telemetry.get("search_point_index", 0),
                "search_angular_travel_deg": pat_telemetry.get("search_angular_travel_deg", 0.0),
                "search_fov_clipped": pat_telemetry.get("search_fov_clipped", False),
                "target_in_predicted_1sigma": target_in_1sigma,
                "target_in_predicted_2sigma": target_in_2sigma,
                "target_in_predicted_95pct": target_in_95pct,
                "target_in_predicted_99pct": target_in_99pct,
                "reacq_mahalanobis_d2": reacq_d2,
                # Controller Telemetry (Phase 6)
                "control_mode": ctrl_telem.get("control_mode", self.controller_mode),
                "predicted_target_x": ctrl_telem.get("predicted_target_x"),
                "predicted_target_y": ctrl_telem.get("predicted_target_y"),
                "prediction_horizon_ms": ctrl_telem.get("prediction_horizon_ms", 0.0),
                "feedforward_pan_rate": ctrl_telem.get("feedforward_pan_rate", 0.0),
                "feedforward_tilt_rate": ctrl_telem.get("feedforward_tilt_rate", 0.0),
                "pid_pan_component": ctrl_telem.get("pid_pan_component", 0.0),
                "pid_tilt_component": ctrl_telem.get("pid_tilt_component", 0.0),
                "raw_command_pan_rate": ctrl_telem.get("raw_command_pan_rate", 0.0),
                "raw_command_tilt_rate": ctrl_telem.get("raw_command_tilt_rate", 0.0),
                "limited_command_pan_rate": ctrl_telem.get("limited_command_pan_rate", 0.0),
                "limited_command_tilt_rate": ctrl_telem.get("limited_command_tilt_rate", 0.0),
                "pan_acceleration": ctrl_telem.get("pan_acceleration", 0.0),
                "tilt_acceleration": ctrl_telem.get("tilt_acceleration", 0.0),
                "pan_jerk": ctrl_telem.get("pan_jerk", 0.0),
                "tilt_jerk": ctrl_telem.get("tilt_jerk", 0.0),
                "rate_saturated": ctrl_telem.get("rate_saturated", False),
                "acceleration_saturated": ctrl_telem.get("acceleration_saturated", False),
                "jerk_saturated": ctrl_telem.get("jerk_saturated", False),
                "deadband_active": ctrl_telem.get("deadband_active", False),
                "integral_pan": ctrl_telem.get("integral_pan", 0.0),
                "integral_tilt": ctrl_telem.get("integral_tilt", 0.0),
                "controller_state_reset_event": ctrl_telem.get("controller_state_reset_event", False)
            }
            telemetry_rows.append(telemetry_row)

        # -------------------------------------------------------------
        # 4. Compute Metrics Summary
        # -------------------------------------------------------------
        metrics_calc = BenchmarkMetricsCalculator(dt=self.dt, lock_threshold_deg=0.15)
        metrics = metrics_calc.compute_metrics(telemetry_rows, designated_target_id=scenario.designated_target_id)

        # -------------------------------------------------------------
        # 5. Build Metadata Snapshot
        # -------------------------------------------------------------
        metadata = {
            "experiment_id": exp_id,
            "scenario": scenario.id,
            "scenario_name": scenario.name,
            "seed": int(seed),
            "git_commit": self.get_git_commit(),
            "timestamp": timestamp_str,
            "simulation_dt": self.dt,
            "simulation_duration": sim_duration,
            "total_frames": n_steps,
            "algorithm_configuration": {
                "detector": "OpenCVDetector (HSV)",
                "localization_mode": self.localization_mode,
                "association_mode": self.association_mode,
                "association": f"AssociationEngine ({self.association_mode})",
                "estimator": f"KalmanFilter2D ({self.estimator_mode})",
                "controller": f"GimbalController ({self.controller_mode})",
                "controller_mode": self.controller_mode,
                "injected_latency_s": self.injected_latency_s,
                "fsm": f"PATLifecycleStateMachine ({self.fsm_mode})" if self.fsm_mode != "legacy_fsm" else "TrackingStateMachine (legacy_fsm)",
                "reacquisition_search": self.pat_fsm_config.search_config.pattern_type
            },
            "runtime_environment": {
                "os": f"{platform.system()} {platform.release()}",
                "python_version": platform.python_version(),
                "processor": platform.processor(),
                "machine": platform.machine()
            }
        }

        # -------------------------------------------------------------
        # 6. Save Artifacts to Disk
        # -------------------------------------------------------------
        # 6.1 config.json
        config_snapshot = {
            "scenario": {
                "id": scenario.id,
                "name": scenario.name,
                "duration": sim_duration,
                "disturbances": scenario.disturbances,
                "initial_pan_deg": scenario.initial_pan_deg,
                "initial_tilt_deg": scenario.initial_tilt_deg
            },
            "dt": self.dt,
            "seed": seed,
            "localization_mode": self.localization_mode,
            "association_mode": self.association_mode,
            "camera": {
                "hfov_deg": self.camera_hfov,
                "vfov_deg": self.camera_vfov,
                "width": self.width,
                "height": self.height
            }
        }
        with open(os.path.join(exp_dir, "config.json"), "w") as f:
            json.dump(config_snapshot, f, indent=2)

        # 6.2 metadata.json
        with open(os.path.join(exp_dir, "metadata.json"), "w") as f:
            json.dump(metadata, f, indent=2)

        # 6.3 telemetry.csv
        telemetry_path = os.path.join(exp_dir, "telemetry.csv")
        if telemetry_rows:
            fieldnames = list(telemetry_rows[0].keys())
            with open(telemetry_path, "w", newline="") as f:
                writer = csv.DictWriter(f, fieldnames=fieldnames)
                writer.writeheader()
                writer.writerows(telemetry_rows)

        # 6.4 summary.json
        with open(os.path.join(exp_dir, "summary.json"), "w") as f:
            json.dump(metrics, f, indent=2)

        # 6.5 events.json
        with open(os.path.join(exp_dir, "events.json"), "w") as f:
            json.dump(events, f, indent=2)

        # 6.6 plots/
        if save_plots:
            plots_dir = os.path.join(exp_dir, "plots")
            generate_benchmark_plots(telemetry_rows, plots_dir, scenario.name, seed)

        return BenchmarkRunResult(
            experiment_id=exp_id,
            scenario_id=scenario.id,
            seed=seed,
            dt=self.dt,
            duration=sim_duration,
            output_dir=exp_dir,
            metrics=metrics,
            telemetry=telemetry_rows,
            events=events,
            metadata=metadata
        )
