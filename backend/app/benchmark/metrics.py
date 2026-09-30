"""
Scientific Metric Evaluation Engine for FSOC-AI-PAT Benchmark.
Computes 26 rigorous performance metrics across Perception, Tracking, Lock,
Recovery, Association, Control, and Computational Performance domains.
Enforces mandatory Error + Coverage pairing, clear observability denominators,
and evaluation-only identity-based false-lock detection.
"""
from typing import Dict, Any, List, Optional
import numpy as np


class BenchmarkMetricsCalculator:
    """
    Post-run evaluation engine. Blindly analyzes recorded telemetry time series.
    Does not interact with or feedback into the operational tracking loop.
    """
    def __init__(self, dt: float = 0.02, lock_threshold_deg: float = 0.15, false_lock_persistence_s: float = 0.5):
        self.dt = dt
        self.lock_threshold_deg = lock_threshold_deg
        self.false_lock_persistence_s = false_lock_persistence_s
        self.false_lock_persistence_frames = max(1, int(round(false_lock_persistence_s / dt)))

    def compute_metrics(
        self,
        telemetry: List[Dict[str, Any]],
        designated_target_id: str = "BEACON-01"
    ) -> Dict[str, Any]:
        """
        Compute comprehensive benchmark summary metrics from telemetry log.
        """
        if not telemetry:
            return {"status": "EMPTY_TELEMETRY"}

        n_frames = len(telemetry)
        sim_duration = n_frames * self.dt

        # -------------------------------------------------------------
        # 1. Telemetry Vector Extraction & Observability Flags
        # -------------------------------------------------------------
        sim_times = np.array([row["simulation_time"] for row in telemetry])
        det_avail = np.array([bool(row.get("detection_available", False)) for row in telemetry])
        target_in_fov = np.array([bool(row.get("ground_truth_target_in_fov", True)) for row in telemetry])
        target_active = np.array([bool(row.get("ground_truth_target_active", True)) for row in telemetry])

        # Part B Observability Flags
        target_optically_present = np.array([
            bool(row.get("ground_truth_target_optically_present", target_active[i] and target_in_fov[i]))
            for i, row in enumerate(telemetry)
        ])
        target_detectable = np.array([
            bool(row.get("ground_truth_target_detectable", target_optically_present[i]))
            for i, row in enumerate(telemetry)
        ])

        # Operational tracking states (normalize to upper-case string)
        states = [str(row.get("tracking_state", "SEARCHING")).upper() for row in telemetry]

        # Gimbal commands
        pan_rates = np.array([float(row.get("command_pan_rate", 0.0)) for row in telemetry])
        tilt_rates = np.array([float(row.get("command_tilt_rate", 0.0)) for row in telemetry])
        sat_flags = np.array([bool(row.get("controller_saturated", False)) for row in telemetry])

        # Latencies
        det_latencies = np.array([float(row.get("detector_latency_ms", 0.0)) for row in telemetry])
        pipe_latencies = np.array([float(row.get("pipeline_latency_ms", 0.0)) for row in telemetry])

        # -------------------------------------------------------------
        # 2. PERCEPTION & OBSERVABILITY METRICS
        # -------------------------------------------------------------
        tp_count = 0
        fp_count = 0
        fn_count = 0

        for i, row in enumerate(telemetry):
            detected = det_avail[i]
            optically_observable = target_optically_present[i]
            cand_id = row.get("evaluated_candidate_id")

            if detected:
                if cand_id == designated_target_id:
                    tp_count += 1
                elif cand_id in ("FALSE_ARTIFACT", "CLUTTER") or (not optically_observable):
                    fp_count += 1
                else:
                    fp_count += 1
            else:
                if optically_observable:
                    fn_count += 1

        precision = float(tp_count / (tp_count + fp_count)) if (tp_count + fp_count) > 0 else 1.0

        observable_frames_count = int(np.sum(target_optically_present))
        active_frames_count = int(np.sum(target_active))

        detector_recall_observable = float(tp_count / observable_frames_count) if observable_frames_count > 0 else 1.0
        detector_recall_mission = float(tp_count / active_frames_count) if active_frames_count > 0 else 1.0

        observable_accepted_frames = int(np.sum(det_avail & target_optically_present))
        observable_coverage_pct = float(100.0 * observable_accepted_frames / observable_frames_count) if observable_frames_count > 0 else 100.0
        mission_coverage_pct = float(100.0 * np.sum(det_avail) / n_frames) if n_frames > 0 else 0.0

        # -------------------------------------------------------------
        # 3. TRACKING ACCURACY & POPULATION SEGREGATION
        # -------------------------------------------------------------
        errs_all_active = [
            row["tracking_error_deg"] for i, row in enumerate(telemetry)
            if row.get("tracking_error_deg") is not None and target_active[i]
        ]
        errs_in_fov = [
            row["tracking_error_deg"] for i, row in enumerate(telemetry)
            if row.get("tracking_error_deg") is not None and target_in_fov[i]
        ]
        errs_locked = [
            row["tracking_error_deg"] for i, row in enumerate(telemetry)
            if row.get("tracking_error_deg") is not None and states[i] in ("LOCKED", "LOCK")
        ]

        def compute_pop_stats(err_list: List[float]) -> Dict[str, Optional[float]]:
            if not err_list:
                return {"rms_deg": None, "p95_deg": None, "max_deg": None, "count": 0}
            arr = np.array(err_list, dtype=np.float64)
            return {
                "rms_deg": round(float(np.sqrt(np.mean(arr ** 2))), 4),
                "p95_deg": round(float(np.percentile(arr, 95)), 4),
                "max_deg": round(float(np.max(arr)), 4),
                "count": len(err_list)
            }

        pop_active = compute_pop_stats(errs_all_active)
        pop_in_fov = compute_pop_stats(errs_in_fov)
        pop_locked = compute_pop_stats(errs_locked)

        # Pixel errors (when target in FOV)
        pixel_errors = [
            row["tracking_error_px"] for i, row in enumerate(telemetry)
            if row.get("tracking_error_px") is not None and target_in_fov[i]
        ]
        if pixel_errors:
            px_arr = np.array(pixel_errors, dtype=np.float64)
            rms_px_error = float(np.sqrt(np.mean(px_arr ** 2)))
            p95_px_error = float(np.percentile(px_arr, 95))
            max_px_error = float(np.max(px_arr))
            outlier_count_5px = int(np.sum(px_arr > 5.0))
        else:
            rms_px_error = float("nan")
            p95_px_error = float("nan")
            max_px_error = float("nan")
            outlier_count_5px = 0

        # -------------------------------------------------------------
        # 4. LOCK RETENTION & FSM DYNAMICS
        # -------------------------------------------------------------
        acq_time = None
        for i, s in enumerate(states):
            if s in ("LOCKED", "LOCK"):
                acq_time = float(sim_times[i])
                break

        # Lock retention when target is observable vs over mission duration
        locked_states = {"LOCKED", "LOCK", "TRACKING", "TRACK"}
        locked_observable_frames = sum(
            1 for i in range(n_frames)
            if states[i] in locked_states and target_optically_present[i]
        )
        total_locked_frames = sum(
            1 for i in range(n_frames)
            if states[i] in locked_states
        )

        lock_retention_observable_pct = float(100.0 * locked_observable_frames / observable_frames_count) if observable_frames_count > 0 else 100.0
        mission_lock_retention_pct = float(100.0 * total_locked_frames / n_frames) if n_frames > 0 else 0.0

        # Causal Target Loss Accounting
        target_loss_count = 0
        loss_reasons = {
            "OUTSIDE_FOV": 0,
            "INTENTIONAL_BLACKOUT": 0,
            "DETECTOR_MISS": 0,
            "ASSOCIATION_REJECTION": 0,
            "ESTIMATOR_EXPIRATION": 0
        }
        was_locked = False

        for i, s in enumerate(states):
            if s in locked_states:
                was_locked = True
            elif was_locked and s in ("TARGET_LOST", "SEARCHING", "SEARCH", "LOST", "REACQUIRE"):
                target_loss_count += 1
                was_locked = False
                # Diagnose cause
                if not target_in_fov[i]:
                    loss_reasons["OUTSIDE_FOV"] += 1
                elif not target_active[i] or not target_optically_present[i]:
                    loss_reasons["INTENTIONAL_BLACKOUT"] += 1
                elif telemetry[i].get("candidate_count", 0) == 0:
                    loss_reasons["DETECTOR_MISS"] += 1
                elif not det_avail[i]:
                    loss_reasons["ASSOCIATION_REJECTION"] += 1
                else:
                    loss_reasons["ESTIMATOR_EXPIRATION"] += 1

        # -------------------------------------------------------------
        # 4.5 REACQUISITION EPISODE ANALYSIS (Phase 7)
        # -------------------------------------------------------------
        reacq_episodes = []
        in_reacq = False
        reacq_start_idx = None
        reacq_start_time = None

        for i, s in enumerate(states):
            if s == "REACQUIRE" and not in_reacq:
                in_reacq = True
                reacq_start_idx = i
                reacq_start_time = float(sim_times[i])
            elif in_reacq and s != "REACQUIRE":
                in_reacq = False
                reacq_end_idx = i
                reacq_end_time = float(sim_times[i])
                succeeded = s in ("TRACK", "TRACKING", "LOCK", "LOCKED")

                # Observable return timestamp: first frame target became optically present
                obs_return_time = None
                for j in range(reacq_start_idx, min(reacq_end_idx + 1, n_frames)):
                    if target_optically_present[j]:
                        obs_return_time = float(sim_times[j])
                        break

                latency_after_obs = max(0.0, reacq_end_time - obs_return_time) if (obs_return_time is not None and succeeded) else None
                search_dur = reacq_end_time - reacq_start_time
                pts_count = reacq_end_idx - reacq_start_idx

                sub_pan = pan_rates[reacq_start_idx:reacq_end_idx] * self.dt
                sub_tilt = tilt_rates[reacq_start_idx:reacq_end_idx] * self.dt
                ang_travel = float(np.sum(np.sqrt(sub_pan**2 + sub_tilt**2)))
                last_region = telemetry[reacq_end_idx - 1].get("search_confidence_region", "none")

                reacq_episodes.append({
                    "start_time_s": round(reacq_start_time, 4),
                    "end_time_s": round(reacq_end_time, 4),
                    "success": succeeded,
                    "final_state": s,
                    "search_duration_s": round(search_dur, 4),
                    "observable_return_time_s": round(obs_return_time, 4) if obs_return_time is not None else None,
                    "reacquisition_latency_s": round(latency_after_obs, 4) if latency_after_obs is not None else None,
                    "search_angular_travel_deg": round(ang_travel, 4),
                    "points_commanded": pts_count,
                    "confidence_region_at_exit": last_region
                })

        # Summary reacquisition metrics
        total_reacq_attempts = len(reacq_episodes)
        successful_reacqs = [ep for ep in reacq_episodes if ep["success"]]
        reacq_success_rate = float(len(successful_reacqs) / total_reacq_attempts) if total_reacq_attempts > 0 else 1.0
        valid_latencies = [ep["reacquisition_latency_s"] for ep in successful_reacqs if ep["reacquisition_latency_s"] is not None]
        mean_reacq_latency = float(np.mean(valid_latencies)) if valid_latencies else None
        total_search_travel = float(np.sum([ep["search_angular_travel_deg"] for ep in reacq_episodes])) if reacq_episodes else 0.0
        total_search_time = float(np.sum([ep["search_duration_s"] for ep in reacq_episodes])) if reacq_episodes else 0.0
        total_search_points = int(np.sum([ep["points_commanded"] for ep in reacq_episodes])) if reacq_episodes else 0
        success_conf_regions = [ep["confidence_region_at_exit"] for ep in successful_reacqs]
        primary_success_region = max(set(success_conf_regions), key=success_conf_regions.count) if success_conf_regions else "none"

        # Covariance coverage calibration (Evaluation-Only)
        reacq_cov_rows = [r for r in telemetry if r.get("reacq_mahalanobis_d2") is not None]
        n_cov_eval = len(reacq_cov_rows)
        cov_1sigma_frac = float(sum(1 for r in reacq_cov_rows if r.get("target_in_predicted_1sigma")) / n_cov_eval) if n_cov_eval > 0 else None
        cov_2sigma_frac = float(sum(1 for r in reacq_cov_rows if r.get("target_in_predicted_2sigma")) / n_cov_eval) if n_cov_eval > 0 else None
        cov_95pct_frac = float(sum(1 for r in reacq_cov_rows if r.get("target_in_predicted_95pct")) / n_cov_eval) if n_cov_eval > 0 else None
        cov_99pct_frac = float(sum(1 for r in reacq_cov_rows if r.get("target_in_predicted_99pct")) / n_cov_eval) if n_cov_eval > 0 else None

        # -------------------------------------------------------------
        # 5. ASSOCIATION, IDENTITY-BASED FALSE LOCK & SWITCHES
        # -------------------------------------------------------------
        false_lock_frames = 0
        current_streak = 0
        false_lock_events = 0
        identity_switches = 0
        prev_cand_id = None

        tracking_active_states = {"LOCKED", "LOCK", "TRACKING", "TRACK", "ACQUIRING", "ACQUIRE", "COAST"}

        for row in telemetry:
            cand_id = row.get("evaluated_candidate_id")
            s_curr = str(row.get("tracking_state", "")).upper()
            is_tracking = s_curr in tracking_active_states

            # Identity switch detection
            if cand_id and prev_cand_id and cand_id != prev_cand_id:
                identity_switches += 1
            if cand_id:
                prev_cand_id = cand_id

            # Identity-based false lock: operational candidate is a non-designated beacon/artifact
            if is_tracking and cand_id and (cand_id != designated_target_id):
                current_streak += 1
                if current_streak == self.false_lock_persistence_frames:
                    false_lock_events += 1
                if current_streak >= self.false_lock_persistence_frames:
                    false_lock_frames += 1
            else:
                current_streak = 0

        false_lock_duration_s = float(false_lock_frames * self.dt)

        # -------------------------------------------------------------
        # 6. MECHANICAL & ACTUATOR CONTROL DYNAMICS
        # -------------------------------------------------------------
        ang_vel = np.sqrt(pan_rates ** 2 + tilt_rates ** 2)
        rms_angular_velocity = float(np.sqrt(np.mean(ang_vel ** 2)))

        if n_frames > 1:
            dpan_dt = np.diff(pan_rates) / self.dt
            dtilt_dt = np.diff(tilt_rates) / self.dt
            ang_accel = np.sqrt(dpan_dt ** 2 + dtilt_dt ** 2)
            rms_angular_accel = float(np.sqrt(np.mean(ang_accel ** 2)))
        else:
            rms_angular_accel = 0.0

        if n_frames > 2:
            ddpan_dt2 = np.diff(dpan_dt) / self.dt
            ddtilt_dt2 = np.diff(dtilt_dt) / self.dt
            ang_jerk = np.sqrt(ddpan_dt2 ** 2 + ddtilt_dt2 ** 2)
            rms_angular_jerk = float(np.sqrt(np.mean(ang_jerk ** 2)))
        else:
            rms_angular_jerk = 0.0

        actuator_saturation_pct = float(100.0 * np.sum(sat_flags) / n_frames) if n_frames > 0 else 0.0

        # -------------------------------------------------------------
        # 7. COMPUTATIONAL LATENCY & THROUGHPUT
        # -------------------------------------------------------------
        mean_det_latency_ms = float(np.mean(det_latencies)) if len(det_latencies) > 0 else 0.0
        p95_det_latency_ms = float(np.percentile(det_latencies, 95)) if len(det_latencies) > 0 else 0.0
        mean_pipe_latency_ms = float(np.mean(pipe_latencies)) if len(pipe_latencies) > 0 else 0.0
        p95_pipe_latency_ms = float(np.percentile(pipe_latencies, 95)) if len(pipe_latencies) > 0 else 0.0

        # Pointing error values for summary pairing
        rms_ang_summary = pop_active["rms_deg"] if pop_active["rms_deg"] is not None else pop_in_fov["rms_deg"]

        return {
            "summary_pairing": {
                "rms_angular_error_deg": rms_ang_summary,
                "rms_pixel_error_px": round(rms_px_error, 3) if not np.isnan(rms_px_error) else None,
                "accepted_measurement_coverage_pct": round(observable_coverage_pct, 2),
                "observable_measurement_coverage_pct": round(observable_coverage_pct, 2),
                "mission_measurement_coverage_pct": round(mission_coverage_pct, 2),
                "paired_metric_note": "CRITICAL: Tracking error must always be paired with coverage percentage."
            },
            "perception": {
                "detection_precision": round(precision, 4),
                "detection_recall_observable": round(detector_recall_observable, 4),
                "detection_recall_mission": round(detector_recall_mission, 4),
                "observable_measurement_coverage_pct": round(observable_coverage_pct, 2),
                "mission_measurement_coverage_pct": round(mission_coverage_pct, 2),
                "accepted_measurement_coverage_pct": round(observable_coverage_pct, 2),  # Compatibility alias
                "false_positive_count": int(fp_count),
                "false_negative_count": int(fn_count),
                "total_detections": int(np.sum(det_avail)),
                "observable_frames_count": observable_frames_count,
                "active_frames_count": active_frames_count
            },
            "tracking": {
                "rms_pixel_error_px": round(rms_px_error, 3) if not np.isnan(rms_px_error) else None,
                "p95_pixel_error_px": round(p95_px_error, 3) if not np.isnan(p95_px_error) else None,
                "maximum_pixel_error_px": round(max_px_error, 3) if not np.isnan(max_px_error) else None,
                "rms_angular_error_deg": rms_ang_summary,
                "severe_outlier_count_gt_5px": int(outlier_count_5px),
                "pointing_error_populations": {
                    "all_active_frames": pop_active,
                    "target_in_fov": pop_in_fov,
                    "locked_frames": pop_locked
                }
            },
            "lock": {
                "acquisition_time_s": round(acq_time, 3) if acq_time is not None else None,
                "lock_retention_when_observable_pct": round(lock_retention_observable_pct, 2),
                "mission_lock_retention_pct": round(mission_lock_retention_pct, 2),
                "lock_retention_pct": round(lock_retention_observable_pct, 2),  # Compatibility alias
                "target_loss_count": int(target_loss_count),
                "target_loss_reasons": loss_reasons
            },
            "recovery": {
                "reacquisition_time_s": round(mean_reacq_latency, 3) if mean_reacq_latency is not None else None,
                "reacquisition_success_rate": round(reacq_success_rate, 4),
                "reacquisition_attempts_count": total_reacq_attempts,
                "successful_reacquisitions_count": len(successful_reacqs),
                "mean_reacquisition_latency_s": round(mean_reacq_latency, 4) if mean_reacq_latency is not None else None,
                "total_search_time_s": round(total_search_time, 4),
                "total_search_angular_travel_deg": round(total_search_travel, 4),
                "total_search_points_commanded": total_search_points,
                "confidence_region_at_success": primary_success_region,
                "episodes": reacq_episodes,
                "covariance_coverage": {
                    "evaluation_frames_count": n_cov_eval,
                    "fraction_inside_1sigma": round(cov_1sigma_frac, 4) if cov_1sigma_frac is not None else None,
                    "fraction_inside_2sigma": round(cov_2sigma_frac, 4) if cov_2sigma_frac is not None else None,
                    "fraction_inside_95pct": round(cov_95pct_frac, 4) if cov_95pct_frac is not None else None,
                    "fraction_inside_99pct": round(cov_99pct_frac, 4) if cov_99pct_frac is not None else None,
                    "theoretical_targets": {"1sigma": 0.3935, "2sigma": 0.8647, "95pct": 0.9500, "99pct": 0.9900}
                }
            },
            "association": {
                "false_lock_event_count": int(false_lock_events),
                "false_lock_duration_s": round(false_lock_duration_s, 3),
                "identity_switch_count": int(identity_switches),
                "false_lock_definition": "Tracking non-designated target continuously for >= 0.5s"
            },
            "control": {
                "rms_commanded_angular_velocity_deg_s": round(rms_angular_velocity, 3),
                "rms_angular_acceleration_deg_s2": round(rms_angular_accel, 3),
                "rms_angular_jerk_deg_s3": round(rms_angular_jerk, 3),
                "actuator_saturation_pct": round(actuator_saturation_pct, 2)
            },
            "performance": {
                "detector_mean_latency_ms": round(mean_det_latency_ms, 3),
                "detector_p95_latency_ms": round(p95_det_latency_ms, 3),
                "total_pipeline_mean_latency_ms": round(mean_pipe_latency_ms, 3),
                "total_pipeline_p95_latency_ms": round(p95_pipe_latency_ms, 3),
                "benchmark_throughput_fps": round(1000.0 / mean_pipe_latency_ms, 1) if mean_pipe_latency_ms > 0 else 0.0
            }
        }
