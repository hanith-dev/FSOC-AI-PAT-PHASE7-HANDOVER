import time
import numpy as np
from typing import Dict, Any, List, Optional

class MetricsTracker:
    def __init__(self):
        self.reset()

    def reset(self):
        self.logs: List[Dict[str, Any]] = []
        
        # State tracking for metrics calculations
        self.search_start_time: Optional[float] = None
        self.acquisition_times: List[float] = []
        
        # Recovery tracking
        self.loss_start_time: Optional[float] = None
        self.successful_recoveries = 0
        self.failed_recoveries = 0
        self.reacquisition_times: List[float] = []

    def log_frame(
        self,
        timestamp: float,
        ground_truth_3d_position: Optional[List[float]],
        ground_truth_direction: Optional[List[float]],
        detected_position: Optional[List[float]],
        predicted_position: Optional[List[float]],
        camera_position: Optional[List[float]],
        camera_orientation: Optional[List[float]],
        pan_angle: float,
        tilt_angle: float,
        pixel_error: Optional[float],
        horizontal_angular_error: float,
        vertical_angular_error: float,
        total_angular_error: float,
        confidence: float,
        fps: float,
        inference_time_ms: float,
        tracking_time_ms: float,
        end_to_end_time_ms: float,
        is_in_fov: bool,
        detected: bool,
        tracking_state: str
    ):
        lock_status = tracking_state in ("LOCKED", "LOCK")
        
        # 1. State machine transition monitoring
        # Capture search start
        if tracking_state in ("SEARCHING", "SEARCH"):
            if self.search_start_time is None:
                self.search_start_time = timestamp
        else:
            # Transitioned from search to tracking state
            if self.search_start_time is not None:
                if tracking_state in ("TRACKING", "LOCKED", "ACQUIRING", "DETECTED", "TRACK", "LOCK", "ACQUIRE", "CANDIDATE"):
                    acq_time = timestamp - self.search_start_time
                    self.acquisition_times.append(acq_time)
                    self.search_start_time = None
                
        # Capture recovery transitions
        if tracking_state in ("TARGET_LOST", "LOST", "REACQUIRE"):
            if self.loss_start_time is None:
                self.loss_start_time = timestamp
        else:
            # We are not in lost state
            if self.loss_start_time is not None:
                if tracking_state in ["TRACKING", "LOCKED", "ACQUIRING", "DETECTED", "TRACK", "LOCK", "ACQUIRE"]:
                    # Successful recovery!
                    reacq_time = timestamp - self.loss_start_time
                    self.reacquisition_times.append(reacq_time)
                    self.successful_recoveries += 1
                elif tracking_state in ("SEARCHING", "SEARCH"):
                    # Failed recovery (timed out to search)
                    self.failed_recoveries += 1
                self.loss_start_time = None

        # 2. Determine Detection TP/FP/FN/TN
        tp = False
        fp = False
        fn = False
        tn = False
        
        if is_in_fov:
            if detected:
                tp = True
            else:
                fn = True
        else:
            if detected:
                fp = True
            else:
                tn = True

        # Append to log list
        log_entry = {
            "timestamp": timestamp,
            "ground_truth_3d_position": ground_truth_3d_position,
            "ground_truth_direction": ground_truth_direction,
            "detected_position": detected_position,
            "predicted_position": predicted_position,
            "camera_position": camera_position,
            "camera_orientation": camera_orientation,
            "pan_angle": pan_angle,
            "tilt_angle": tilt_angle,
            "pixel_error": pixel_error,
            "horizontal_angular_error": horizontal_angular_error,
            "vertical_angular_error": vertical_angular_error,
            "total_angular_error": total_angular_error,
            "confidence": confidence,
            "fps": fps,
            "inference_time_ms": inference_time_ms,
            "tracking_time_ms": tracking_time_ms,
            "end_to_end_time_ms": end_to_end_time_ms,
            "tp": tp,
            "fp": fp,
            "fn": fn,
            "tn": tn,
            "tracking_state": tracking_state,
            "lock_status": lock_status
        }
        self.logs.append(log_entry)

    def get_live_metrics(self) -> Dict[str, Any]:
        if not self.logs:
            return {
                "fps": {"current": 0.0, "avg": "N/A", "min": "N/A", "max": "N/A"},
                "acquisition_time": "N/A",
                "tracking_error": {"avg": "N/A", "max": "N/A", "rms": "N/A"},
                "lock_retention_rate": "N/A",
                "detection": {"precision": "N/A", "recall": "N/A", "fp_rate": "N/A", "fn_rate": "N/A", "avg_confidence": "N/A"},
                "processing_time": {"inference_avg": "N/A", "tracking_avg": "N/A", "end_to_end_avg": "N/A"},
                "recovery": {"avg_loss_duration": "N/A", "avg_reacquisition_time": "N/A", "success_rate": "N/A"}
            }

        # FPS metrics
        fps_values = [log["fps"] for log in self.logs if log["fps"] > 0]
        curr_fps = self.logs[-1]["fps"]
        fps_avg = float(np.mean(fps_values)) if fps_values else 0.0
        fps_min = float(np.min(fps_values)) if fps_values else 0.0
        fps_max = float(np.max(fps_values)) if fps_values else 0.0

        # Acquisition Time
        avg_acq_time = float(np.mean(self.acquisition_times)) if self.acquisition_times else "N/A"

        # Tracking Errors (calculated only when tracking/locked/acquiring)
        active_error_states = ["TRACKING", "LOCKED", "ACQUIRING", "DETECTED", "TRACK", "LOCK", "ACQUIRE", "CANDIDATE"]
        tracking_errors = [
            log["total_angular_error"]
            for log in self.logs
            if log["tracking_state"] in active_error_states
        ]
        
        if tracking_errors:
            err_avg = float(np.mean(tracking_errors))
            err_max = float(np.max(tracking_errors))
            err_rms = float(np.sqrt(np.mean(np.square(tracking_errors))))
        else:
            err_avg = "N/A"
            err_max = "N/A"
            err_rms = "N/A"

        # Lock retention rate
        locked_frames = sum(1 for log in self.logs if log["tracking_state"] in ("LOCKED", "LOCK"))
        tracking_frames = sum(1 for log in self.logs if log["tracking_state"] in active_error_states)
        lock_retention = (locked_frames / tracking_frames * 100.0) if tracking_frames > 0 else "N/A"

        # Detection stats
        tp_count = sum(1 for log in self.logs if log["tp"])
        fp_count = sum(1 for log in self.logs if log["fp"])
        fn_count = sum(1 for log in self.logs if log["fn"])
        total_frames = len(self.logs)

        precision = (tp_count / (tp_count + fp_count)) if (tp_count + fp_count) > 0 else "N/A"
        recall = (tp_count / (tp_count + fn_count)) if (tp_count + fn_count) > 0 else "N/A"
        fp_rate = fp_count / total_frames if total_frames > 0 else 0.0
        fn_rate = fn_count / total_frames if total_frames > 0 else 0.0
        
        confidences = [log["confidence"] for log in self.logs if log["tp"] and log["confidence"] > 0]
        avg_conf = float(np.mean(confidences)) if confidences else "N/A"

        # Processing times
        inf_times = [log["inference_time_ms"] for log in self.logs]
        trk_times = [log["tracking_time_ms"] for log in self.logs]
        e2e_times = [log["end_to_end_time_ms"] for log in self.logs]
        
        inf_avg = float(np.mean(inf_times))
        trk_avg = float(np.mean(trk_times))
        e2e_avg = float(np.mean(e2e_times))

        # Recovery stats
        # Total loss duration
        loss_durations = []
        for i, log in enumerate(self.logs):
            is_lost_now = log["tracking_state"] in ("TARGET_LOST", "LOST", "REACQUIRE")
            is_lost_prev = self.logs[i-1]["tracking_state"] in ("TARGET_LOST", "LOST", "REACQUIRE") if i > 0 else False
            is_lost_next = self.logs[i+1]["tracking_state"] in ("TARGET_LOST", "LOST", "REACQUIRE") if i < len(self.logs) - 1 else False
            if is_lost_now:
                # Find consecutive lost segments
                if i == 0 or not is_lost_prev:
                    start_t = log["timestamp"]
                if i == len(self.logs) - 1 or not is_lost_next:
                    end_t = log["timestamp"]
                    loss_durations.append(end_t - start_t)
                    
        avg_loss_dur = float(np.mean(loss_durations)) if loss_durations else "N/A"
        avg_reacq = float(np.mean(self.reacquisition_times)) if self.reacquisition_times else "N/A"
        
        tot_recoveries = self.successful_recoveries + self.failed_recoveries
        rec_success_rate = (self.successful_recoveries / tot_recoveries * 100.0) if tot_recoveries > 0 else "N/A"

        return {
            "fps": {
                "current": round(curr_fps, 1),
                "avg": round(fps_avg, 1) if isinstance(fps_avg, float) else "N/A",
                "min": round(fps_min, 1) if isinstance(fps_min, float) else "N/A",
                "max": round(fps_max, 1) if isinstance(fps_max, float) else "N/A"
            },
            "acquisition_time": round(avg_acq_time, 2) if isinstance(avg_acq_time, float) else "N/A",
            "tracking_error": {
                "avg": round(err_avg, 3) if isinstance(err_avg, float) else "N/A",
                "max": round(err_max, 3) if isinstance(err_max, float) else "N/A",
                "rms": round(err_rms, 3) if isinstance(err_rms, float) else "N/A"
            },
            "lock_retention_rate": round(lock_retention, 1) if isinstance(lock_retention, float) else "N/A",
            "detection": {
                "precision": round(precision, 3) if isinstance(precision, float) else "N/A",
                "recall": round(recall, 3) if isinstance(recall, float) else "N/A",
                "fp_rate": round(fp_rate, 3),
                "fn_rate": round(fn_rate, 3),
                "avg_confidence": round(avg_conf, 3) if isinstance(avg_conf, float) else "N/A"
            },
            "processing_time": {
                "inference_avg": round(inf_avg, 2),
                "tracking_avg": round(trk_avg, 2),
                "end_to_end_avg": round(e2e_avg, 2)
            },
            "recovery": {
                "avg_loss_duration": round(avg_loss_dur, 2) if isinstance(avg_loss_dur, float) else "N/A",
                "avg_reacquisition_time": round(avg_reacq, 2) if isinstance(avg_reacq, float) else "N/A",
                "success_rate": round(rec_success_rate, 1) if isinstance(rec_success_rate, float) else "N/A"
            }
        }
        
    def export_csv(self) -> str:
        if not self.logs:
            return ""
            
        header = [
            "timestamp", "gt_x", "gt_y", "gt_z", "gt_dx", "gt_dy", "gt_dz",
            "det_u", "det_v", "pred_u", "pred_v", "cam_x", "cam_y", "cam_z",
            "cam_q0", "cam_q1", "cam_q2", "cam_q3", "pan", "tilt", "pixel_err",
            "h_ang_err", "v_ang_err", "total_ang_err", "confidence", "fps",
            "inference_time_ms", "tracking_time_ms", "end_to_end_time_ms",
            "tp", "fp", "fn", "tn", "state", "locked"
        ]
        
        lines = [",".join(header)]
        for log in self.logs:
            gt_pos = log["ground_truth_3d_position"] or ["N/A", "N/A", "N/A"]
            gt_dir = log["ground_truth_direction"] or ["N/A", "N/A", "N/A"]
            det_pos = log["detected_position"] or ["N/A", "N/A"]
            pred_pos = log["predicted_position"] or ["N/A", "N/A"]
            cam_pos = log["camera_position"] or ["N/A", "N/A", "N/A"]
            cam_ori = log["camera_orientation"] or ["N/A", "N/A", "N/A", "N/A"]
            
            row = [
                str(log["timestamp"]),
                *[str(x) for x in gt_pos],
                *[str(x) for x in gt_dir],
                *[str(x) for x in det_pos],
                *[str(x) for x in pred_pos],
                *[str(x) for x in cam_pos],
                *[str(x) for x in cam_ori],
                str(log["pan_angle"]),
                str(log["tilt_angle"]),
                str(log["pixel_error"]) if log["pixel_error"] is not None else "N/A",
                str(log["horizontal_angular_error"]),
                str(log["vertical_angular_error"]),
                str(log["total_angular_error"]),
                str(log["confidence"]),
                str(log["fps"]),
                str(log["inference_time_ms"]),
                str(log["tracking_time_ms"]),
                str(log["end_to_end_time_ms"]),
                str(int(log["tp"])),
                str(int(log["fp"])),
                str(int(log["fn"])),
                str(int(log["tn"])),
                log["tracking_state"],
                str(int(log["lock_status"]))
            ]
            lines.append(",".join(row))
            
        return "\n".join(lines)
