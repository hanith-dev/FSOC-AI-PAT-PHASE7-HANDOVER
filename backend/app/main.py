import os
import json
import logging
import time
import base64
from typing import Dict, Any, List
from fastapi import FastAPI, WebSocket, WebSocketDisconnect, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import FileResponse, PlainTextResponse
from fastapi.staticfiles import StaticFiles
import numpy as np
import cv2

# Import custom tracking and detection components
from app.detection.detector import DetectorInterface
from app.tracking.kalman import KalmanFilter2D
from app.tracking.state_machine import TrackingStateMachine, TrackingState
from app.tracking.pat_state_machine import PATLifecycleStateMachine, PATFSMConfig
from app.tracking.association import AssociationEngine, CandidateMeasurement
from app.tracking.controller import GimbalController, GimbalControllerConfig, ControlMode
from app.camera.model import CameraModel
from app.detection.disturbances import DisturbanceEngine
from app.tracking.metrics import MetricsTracker

# Set up logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger("FSOC-Backend")

app = FastAPI(title="FSOC Virtual Camera Tracking Simulator")

# Frontend distribution static mount
FRONTEND_DIST = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "frontend", "dist"))
ASSETS_DIR = os.path.join(FRONTEND_DIST, "assets")

if os.path.exists(ASSETS_DIR):
    app.mount("/assets", StaticFiles(directory=ASSETS_DIR), name="assets")

# CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Global configurations path
CONFIG_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "configs"))

def load_config(filename: str, default: Dict) -> Dict:
    path = os.path.join(CONFIG_DIR, filename)
    if os.path.exists(path):
        try:
            with open(path, 'r') as f:
                return json.load(f)
        except Exception as e:
            logger.error(f"Error reading config {filename}: {e}")
    return default

# Load configs or use defaults
ai_config = load_config("ai.json", {
    "model": {"weights_path": "models/beacon_detector.pt"},
    "hsv_thresholds": {
        "lower": [35, 50, 50],
        "upper": [95, 255, 255]
    }
})

camera_config = load_config("camera.json", {
    "camera": {
        "resolution": {"width": 640, "height": 480},
        "fov": {"horizontal_deg": 10.0, "vertical_deg": 7.5}
    }
})

tracking_config = load_config("tracking.json", {
    "tracker": {
        "process_noise_std": 0.5,
        "measurement_noise_std": 1.0,
        "max_occlusion_seconds": 2.0
    }
})

# Load other configs for Batch 3
search_config = load_config("search.json", {
    "pattern": "raster",
    "speed": 10.0,
    "range_h": 20.0,
    "range_v": 15.0,
    "step_size": 3.0
})
disturbances_config = load_config("disturbances.json", {
    "platform_vibration": {"intensity": 0, "frequency_hz": 10.0, "amplitude_px": 5.0},
    "camera_jitter": {"intensity": 0, "probability": 0.1, "max_jump_px": 20.0},
    "image_noise": {"intensity": 0},
    "blur": {"intensity": 0},
    "atmospheric_turbulence": {"intensity": 0},
    "illumination_variation": {"intensity": 0}
})
simulation_config = load_config("simulation.json", {
    "time_step_dt": 0.016,
    "speed_factor": 1.0,
    "max_duration": 60.0,
    "update_rate_hz": 60,
    "beacons": []
})

# Global state / managers
disturbance_engine = DisturbanceEngine(disturbances_config)
metrics_tracker = MetricsTracker()

# Initialize Experiment Manager
from app.experiments.manager import ExperimentManager
RESULTS_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "experiments", "results"))
SCENARIOS_PATH = os.path.join(CONFIG_DIR, "scenarios.json")
experiment_manager = ExperimentManager(RESULTS_DIR, SCENARIOS_PATH)

class SimulationState:
    def __init__(self):
        self.designated_target_id = "BEACON-01"
        self.beacons = simulation_config.get("beacons", [])
        self.search_config = search_config
        self.disturbances_config = disturbances_config
        self.ablation = {
            "ai_detection": True,
            "kalman_filter": True,
            "pid_controller": True,
            "disturbances": True
        }

sim_state = SimulationState()

# Initialize AI & tracking instances
hsv_lower = ai_config.get("hsv_thresholds", {}).get("lower", [35, 50, 50])
hsv_upper = ai_config.get("hsv_thresholds", {}).get("upper", [95, 255, 255])
yolo_path = ai_config.get("model", {}).get("weights_path", "models/beacon_detector.pt")

detector_manager = DetectorInterface(
    mode="opencv", # Default to opencv, can toggle via client command
    yolo_path=yolo_path,
    hsv_lower=hsv_lower,
    hsv_upper=hsv_upper
)

# Resolution setup
cam_res = camera_config.get("camera", {}).get("resolution", {"width": 640, "height": 480})
cam_fov = camera_config.get("camera", {}).get("fov", {"horizontal_deg": 10.0, "vertical_deg": 7.5})
width = cam_res.get("width", 640)
height = cam_res.get("height", 480)

# Instantiate a CameraModel helper for backend error metrics
camera_model = CameraModel(
    width=width,
    height=height,
    hfov_deg=cam_fov.get("horizontal_deg", 10.0),
    vfov_deg=cam_fov.get("vertical_deg", 7.5)
)

class TrackingConnectionManager:
    def __init__(self):
        self.active_connections: list[WebSocket] = []

    async def connect(self, websocket: WebSocket):
        await websocket.accept()
        self.active_connections.append(websocket)
        logger.info(f"WebSocket connected. Total active connections: {len(self.active_connections)}")

    def disconnect(self, websocket: WebSocket):
        if websocket in self.active_connections:
            self.active_connections.remove(websocket)
        logger.info(f"WebSocket disconnected. Remaining connections: {len(self.active_connections)}")

manager = TrackingConnectionManager()

@app.get("/api/status")
async def get_status():
    return {
        "status": "online",
        "message": "FSOC Simulator Backend is running",
        "timestamp": time.time(),
        "detector_mode": detector_manager.mode,
        "yolo_available": detector_manager.yolo_detector is not None
    }

@app.post("/api/detector/mode")
async def set_detector_mode(payload: Dict[str, str]):
    mode = payload.get("mode", "opencv")
    detector_manager.set_mode(mode)
    return {
        "status": "success",
        "mode": detector_manager.mode,
        "fallback_active": detector_manager.fallback_warning_active
    }

@app.get("/api/metrics")
async def get_metrics():
    return metrics_tracker.get_live_metrics()

@app.post("/api/metrics/reset")
async def reset_metrics_api():
    metrics_tracker.reset()
    return {"status": "success", "message": "Metrics reset successfully"}

@app.get("/api/metrics/csv")
async def get_metrics_csv():
    from fastapi.responses import PlainTextResponse
    csv_data = metrics_tracker.export_csv()
    return PlainTextResponse(csv_data, media_type="text/csv", headers={"Content-Disposition": "attachment; filename=fsoc_metrics.csv"})

@app.post("/api/disturbances/set")
async def set_disturbances_api(payload: Dict[str, Any]):
    disturbances = payload.get("disturbances", {})
    for k, v in disturbances.items():
        disturbance_engine.set_intensity(k, v)
    return {"status": "success", "disturbances": disturbance_engine.intensities}

@app.post("/api/disturbances/reset")
async def reset_disturbances_api():
    disturbance_engine.reset()
    return {"status": "success", "disturbances": disturbance_engine.intensities}

@app.get("/api/scenarios")
async def get_scenarios():
    return experiment_manager.get_scenarios()

@app.post("/api/experiments/start")
async def start_experiment_api(payload: Dict[str, Any]):
    scenario_id = payload.get("scenario_id")
    client_config = payload.get("client_config", {})
    
    # Reset metrics tracker
    metrics_tracker.reset()
    
    try:
        result = experiment_manager.start_experiment(scenario_id, client_config)
        
        # Load scenario settings to sync state
        scenario = result["config"]["scenario"]
        sim_state.designated_target_id = scenario["beacons"][0]["id"] if scenario["beacons"] else "BEACON-01"
        sim_state.beacons = scenario.get("beacons", [])
        
        # Update disturbances
        dist_intensities = scenario.get("disturbances", {})
        for k, v in dist_intensities.items():
            disturbance_engine.set_intensity(k, v)
            
        # Update ablation settings
        ablation = client_config.get("ablation", {})
        sim_state.ablation.update(ablation)
        
        # Update detector mode
        det_mode = client_config.get("detector_mode", "opencv")
        detector_manager.set_mode(det_mode)
        
        # Update search pattern if in scenario
        if "search" in scenario:
            sim_state.search_config.update(scenario["search"])
            
        return {
            "status": "success",
            "experiment": result
        }
    except Exception as e:
        logger.error(f"Error starting experiment: {e}")
        return {"status": "error", "message": str(e)}

@app.post("/api/experiments/stop")
async def stop_experiment_api():
    if not experiment_manager.active_experiment_id:
        return {"status": "error", "message": "No active experiment"}
    results = experiment_manager.stop_experiment(metrics_tracker.get_live_metrics(), metrics_tracker.export_csv())
    return {"status": "success", "experiment": results}

@app.get("/api/experiments/results/{exp_id}/pdf")
async def get_experiment_pdf(exp_id: str):
    pdf_path = os.path.join(RESULTS_DIR, f"{exp_id}.pdf")
    if os.path.exists(pdf_path):
        from fastapi.responses import FileResponse
        return FileResponse(pdf_path, media_type="application/pdf", filename=f"{exp_id}.pdf")
    return {"status": "error", "message": "PDF report not found"}

@app.get("/api/experiments/results/{exp_id}/csv")
async def get_experiment_csv(exp_id: str):
    csv_path = os.path.join(RESULTS_DIR, f"{exp_id}.csv")
    if os.path.exists(csv_path):
        from fastapi.responses import FileResponse
        return FileResponse(csv_path, media_type="text/csv", filename=f"{exp_id}.csv")
    return {"status": "error", "message": "CSV log not found"}

@app.get("/api/experiments/results")
async def get_experiments_results():
    return experiment_manager.get_results()

@app.get("/api/experiments/compare")
async def compare_experiments(ids: str):
    id_list = [i.strip() for i in ids.split(",") if i.strip()]
    return experiment_manager.get_comparison(id_list)

@app.delete("/api/experiments/results/{exp_id}")
async def delete_experiment_result(exp_id: str):
    success = experiment_manager.delete_result(exp_id)
    if success:
        return {"status": "success", "message": "Experiment deleted"}
    return {"status": "error", "message": "Experiment not found"}

@app.websocket("/ws/tracking")
async def websocket_endpoint(websocket: WebSocket):
    await manager.connect(websocket)
    
    # Initialize trackers for this socket session
    kalman_filter = KalmanFilter2D(
        mode=tracking_config.get("tracker", {}).get("mode", "ca6_adaptive_r"),
        q_noise=tracking_config.get("tracker", {}).get("process_noise_std", 0.5),
        q_jerk=tracking_config.get("tracker", {}).get("q_jerk", 10000.0),
        r_noise=tracking_config.get("tracker", {}).get("measurement_noise_std", 1.0),
        max_prediction_seconds=tracking_config.get("tracker", {}).get("max_occlusion_seconds", 2.0)
    )
    
    association_engine = AssociationEngine(
        mode="robust_multifeature",
        image_width=width,
        image_height=height
    )
    
    state_machine = TrackingStateMachine(
        lock_threshold_deg=0.15,
        lock_duration_sec=1.0
    )
    pat_fsm = PATLifecycleStateMachine(
        config=PATFSMConfig(
            lock_angular_error_entry_deg=0.15,
            lock_duration_entry_s=0.40
        )
    )
    gimbal_controller = GimbalController(
        config=GimbalControllerConfig(),
        mode=ControlMode.FILTERED_PID
    )
    
    # Send connection greeting
    await websocket.send_json({
        "type": "connection_status",
        "status": "connected",
        "message": "FSOC Telemetry and Frame channel active."
    })
    
    frame_count = 0
    start_time = time.time()
    
    try:
        while True:
            # Check connection type or receive command messages (JSON vs Binary)
            message = await websocket.receive()
            if message.get("type") == "websocket.disconnect":
                raise WebSocketDisconnect()
            
            if "bytes" in message:
                # Handle old binary fallback frames
                data = message["bytes"]
                frame_count += 1
                current_time = time.time()
                
                # 1. Parse Image Frame
                try:
                    np_arr = np.frombuffer(data, np.uint8)
                    img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
                except Exception as e:
                    logger.error(f"Error decoding image frame: {e}")
                    continue
                    
                if img is None:
                    continue
                
                start_processing_time = time.time()
                
                # 2. Run Detector
                start_det_time = time.time()
                det_res, active_detector = detector_manager.detect(img)
                detected = det_res["detected"]
                det_time = (time.time() - start_det_time) * 1000.0
                
                # 3. Update Kalman Filter
                start_track_time = time.time()
                if detected:
                    center = np.array(det_res["center"], dtype=np.float32)
                    kf_state = kalman_filter.update(center, current_time)
                else:
                    kf_state = kalman_filter.predict_only(current_time)
                track_time = (time.time() - start_track_time) * 1000.0
                    
                # 4. Check for occlusion timeouts
                is_occluded = not detected and kalman_filter.is_initialized
                if kalman_filter.is_initialized:
                    timeout = kalman_filter.check_occlusion_timeout(current_time)
                    if timeout:
                        state_machine.force_lost()
                        is_occluded = False
                
                # 5. Compute Angular Errors from Kalman Filter Output
                total_err = 0.0
                h_err = 0.0
                v_err = 0.0
                
                if kalman_filter.is_initialized:
                    kx, ky = kf_state[0], kf_state[1]
                    dx = kx - camera_model.cx
                    dy = camera_model.cy - ky
                    h_err = float(np.degrees(np.arctan2(dx, camera_model.fx)))
                    v_err = float(np.degrees(np.arctan2(dy, camera_model.fy)))
                    total_err = float(np.sqrt(h_err**2 + v_err**2))
                
                # 6. Evaluate State Machine transitions (Phase 5/7 PAT Lifecycle FSM)
                if kalman_filter.is_initialized:
                    time_since_m = current_time - kalman_filter.last_update_time
                    est_health = "ACTIVE" if time_since_m <= 0.5 else ("DEGRADED" if time_since_m <= kalman_filter.max_prediction_seconds else "EXPIRED")
                    pos_sigma = float(np.sqrt(np.trace(kalman_filter.covariance[:2, :2])))
                    kf_mean = kf_state[:2]
                    kf_cov = kalman_filter.covariance[:2, :2]
                else:
                    time_since_m = None
                    est_health = "EXPIRED"
                    pos_sigma = 999.0
                    kf_mean = None
                    kf_cov = None

                pat_state = pat_fsm.update(
                    association_status="ACCEPTED" if detected else "NO_CANDIDATE",
                    measurement_accepted=detected,
                    candidate_count=1 if detected else 0,
                    estimator_health=est_health,
                    position_sigma_px=pos_sigma,
                    angular_error_deg=total_err,
                    current_time=current_time,
                    kf_pos_mean=kf_mean,
                    kf_pos_cov=kf_cov,
                    time_since_measurement=time_since_m
                )
                sm_state = pat_state.value if hasattr(pat_state, "value") else str(pat_state)
                
                # Phase 6/6.5 Certified Gimbal Controller
                pan_rate_cmd, tilt_rate_cmd, ctrl_telem = gimbal_controller.compute_control(
                    kf_state=kf_state if kalman_filter.is_initialized else None,
                    camera_cx=camera_model.cx,
                    camera_cy=camera_model.cy,
                    camera_fx=camera_model.fx,
                    camera_fy=camera_model.fy,
                    dt=0.016,
                    pat_state=sm_state,
                    search_pan_offset_deg=pat_fsm.search_offset_pan_deg,
                    search_tilt_offset_deg=pat_fsm.search_offset_tilt_deg
                )

                end_processing_time = time.time()
                e2e_time = (end_processing_time - start_processing_time) * 1000.0
                fps = frame_count / (current_time - start_time) if current_time > start_time else 0.0
                
                # Update metrics
                metrics_tracker.log_frame(
                    timestamp=current_time,
                    ground_truth_3d_position=None,
                    ground_truth_direction=None,
                    detected_position=det_res["center"] if detected else None,
                    predicted_position=[float(kf_state[0]), float(kf_state[1])] if kalman_filter.is_initialized else None,
                    camera_position=None,
                    camera_orientation=None,
                    pan_angle=0.0,
                    tilt_angle=0.0,
                    pixel_error=float(np.sqrt((det_res["center"][0] - camera_model.cx)**2 + (det_res["center"][1] - camera_model.cy)**2)) if detected else None,
                    horizontal_angular_error=h_err,
                    vertical_angular_error=v_err,
                    total_angular_error=total_err,
                    confidence=det_res["confidence"],
                    fps=fps,
                    inference_time_ms=det_time,
                    tracking_time_ms=track_time,
                    end_to_end_time_ms=e2e_time,
                    is_in_fov=False,
                    detected=detected,
                    tracking_state=sm_state
                )
                
                response = {
                    "type": "telemetry",
                    "frame_index": frame_count,
                    "timestamp": current_time,
                    "tracking_state": sm_state,
                    "detected": detected,
                    "active_detector": active_detector,
                    "fallback_warning": detector_manager.fallback_warning_active,
                    "inference_time_ms": round(det_time, 2),
                    "fps": round(fps, 1),
                    "kalman": {
                        "active": kalman_filter.is_initialized,
                        "x": float(kf_state[0]) if kalman_filter.is_initialized else 0.0,
                        "y": float(kf_state[1]) if kalman_filter.is_initialized else 0.0,
                        "vx": float(kf_state[2]) if kalman_filter.is_initialized else 0.0,
                        "vy": float(kf_state[3]) if kalman_filter.is_initialized else 0.0,
                        "timestamp": kalman_filter.last_update_time
                    },
                    "confidence": round(det_res["confidence"], 2),
                    "bbox": det_res["bbox"],
                    "raw_center": det_res["center"],
                    "angular_error_total": total_err,
                    "angular_error_h": h_err,
                    "angular_error_v": v_err,
                    "pan_command": round(pan_rate_cmd, 4),
                    "tilt_command": round(tilt_rate_cmd, 4),
                    "controller_telemetry": ctrl_telem,
                    "search_offset_pan_deg": round(pat_fsm.search_offset_pan_deg, 4),
                    "search_offset_tilt_deg": round(pat_fsm.search_offset_tilt_deg, 4),
                    "pat_telemetry": pat_fsm.get_telemetry(current_time),
                    "candidates": [],
                    "live_metrics": metrics_tracker.get_live_metrics()
                }
                await websocket.send_json(response)
                
            elif "text" in message:
                try:
                    cmd_data = json.loads(message["text"])
                    cmd_type = cmd_data.get("type")
                    
                    if cmd_type == "frame":
                        frame_count += 1
                        current_time = time.time()
                        
                        # 1. Parse Image Frame and Metadata
                        image_str = cmd_data.get("image") or cmd_data.get("frame_data") or ""
                        if "," in image_str:
                            image_str = image_str.split(",", 1)[1]
                        try:
                            img_bytes = base64.b64decode(image_str)
                            np_arr = np.frombuffer(img_bytes, np.uint8)
                            img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
                        except Exception as e:
                            logger.error(f"Error decoding base64 image: {e}")
                            continue
                            
                        if img is None:
                            continue
                            
                        meta = cmd_data.get("metadata", {})
                        gimbal_pan = float(meta.get("gimbal_pan", 0.0))
                        gimbal_tilt = float(meta.get("gimbal_tilt", 0.0))
                        camera_pos = np.array(meta.get("camera_position", [0.0, 1.5, 0.0]))
                        camera_ori = meta.get("camera_orientation", [0.0, 0.0, 0.0, 1.0])
                        designated_target_id = meta.get("designated_target_id", sim_state.designated_target_id)
                        beacons_list = meta.get("beacons", [])
                        
                        # Sync sim state designated target
                        sim_state.designated_target_id = designated_target_id
                        
                        # 2. Apply Disturbances to Image Frame (unless ablated)
                        start_processing_time = time.time()
                        if sim_state.ablation.get("disturbances", True):
                            img = disturbance_engine.apply(img, current_time)
                        
                        # 3. Update Camera Extrinsics
                        pan_rad = np.radians(gimbal_pan)
                        tilt_rad = np.radians(gimbal_tilt)
                        camera_model.camera_pos = camera_pos
                        
                        # 4. Run Detector on the Disturbed Image (unless ablated)
                        start_det_time = time.time()
                        if sim_state.ablation.get("ai_detection", True):
                            det_res, active_detector = detector_manager.detect(img)
                        else:
                            det_res = {
                                "detected": False,
                                "confidence": 0.0,
                                "bbox": [0, 0, 0, 0],
                                "center": [0.0, 0.0],
                                "candidates": [],
                                "inference_time_ms": 0.0
                            }
                            active_detector = "ablation_off"
                        det_time = (time.time() - start_det_time) * 1000.0
                        
                        # 5. Autonomous Candidate Selection / Association (Phase 3)
                        detected_candidates = det_res.get("candidates", [])
                        candidate_measurements = [
                            CandidateMeasurement.from_dict(c, candidate_index=idx, candidate_id=f"c_{idx}", timestamp=current_time)
                            for idx, c in enumerate(detected_candidates)
                        ]
                        assoc_res = association_engine.associate(candidate_measurements, kalman_filter, current_time)

                        target_detected = assoc_res.accepted
                        if target_detected and assoc_res.selected_candidate is not None:
                            target_center = [assoc_res.selected_candidate.x, assoc_res.selected_candidate.y]
                            target_bbox = assoc_res.selected_candidate.bbox
                            target_confidence = float(assoc_res.selected_candidate.confidence)
                            cand_ux = assoc_res.selected_candidate.uncertainty_x
                            cand_uy = assoc_res.selected_candidate.uncertainty_y
                            R_candidate = np.diag([cand_ux ** 2, cand_uy ** 2])
                        else:
                            target_center = [0.0, 0.0]
                            target_bbox = [0, 0, 0, 0]
                            target_confidence = 0.0
                            R_candidate = None

                        # Construct candidate list for telemetry purely from observable detections
                        matched_candidates = []
                        for idx, cand in enumerate(detected_candidates):
                            is_selected = target_detected and (cand["center"] == target_center)
                            matched_candidates.append({
                                "id": f"CANDIDATE-{idx+1:02d}",
                                "confidence": float(cand.get("confidence", 0.0)),
                                "pixel_position": [float(cand["center"][0]), float(cand["center"][1])],
                                "bbox": [int(x) for x in cand.get("bbox", [0, 0, 0, 0])],
                                "area": float(cand.get("area", 0.0)),
                                "status": "tracked" if is_selected else "candidate"
                            })
                        
                        # 6. Update Kalman Filter (unless ablated)
                        start_track_time = time.time()
                        if sim_state.ablation.get("kalman_filter", True):
                            if target_detected:
                                center_pt = np.array(target_center, dtype=np.float32)
                                q_flags = assoc_res.selected_candidate.quality_flags if assoc_res.selected_candidate else None
                                kf_state = kalman_filter.update(center_pt, current_time, R_custom=R_candidate, quality_flags=q_flags)
                            else:
                                kf_state = kalman_filter.predict_only(current_time)
                            is_sm_detected = kalman_filter.is_initialized
                        else:
                            kalman_filter.reset()
                            is_sm_detected = target_detected
                            if target_detected:
                                kf_state = np.array([target_center[0], target_center[1], 0.0, 0.0])
                            else:
                                kf_state = np.array([camera_model.cx, camera_model.cy, 0.0, 0.0])
                        track_time = (time.time() - start_track_time) * 1000.0
                            
                        # Check for occlusion timeouts (only if Kalman is enabled)
                        is_occluded = not target_detected and is_sm_detected if sim_state.ablation.get("kalman_filter", True) else False
                        if sim_state.ablation.get("kalman_filter", True) and kalman_filter.is_initialized:
                            timeout = kalman_filter.check_occlusion_timeout(current_time)
                            if timeout:
                                state_machine.force_lost()
                                is_occluded = False
                                
                        # 7. Compute Angular Errors
                        total_err = 0.0
                        h_err = 0.0
                        v_err = 0.0
                        
                        if is_sm_detected or target_detected:
                            kx, ky = kf_state[0], kf_state[1]
                            dx = kx - camera_model.cx
                            dy = camera_model.cy - ky
                            h_err = float(np.degrees(np.arctan2(dx, camera_model.fx)))
                            v_err = float(np.degrees(np.arctan2(dy, camera_model.fy)))
                            total_err = float(np.sqrt(h_err**2 + v_err**2))
                        
                        # 8. Evaluate State Machine Transitions (Phase 5/7 PAT Lifecycle FSM)
                        if target_detected:
                            assoc_status = "ACCEPTED" if assoc_res.selected_candidate is not None else "NO_CANDIDATE"
                        else:
                            assoc_status = assoc_res.status if hasattr(assoc_res, "status") and assoc_res.status else "NO_CANDIDATE"
                        
                        if kalman_filter.is_initialized:
                            time_since_m = current_time - kalman_filter.last_update_time
                            if time_since_m <= 0.5:
                                est_health = "ACTIVE"
                            elif time_since_m <= kalman_filter.max_prediction_seconds:
                                est_health = "DEGRADED"
                            else:
                                est_health = "EXPIRED"
                            pos_sigma = float(np.sqrt(np.trace(kalman_filter.covariance[:2, :2])))
                            kf_mean = kf_state[:2]
                            kf_cov = kalman_filter.covariance[:2, :2]
                        else:
                            time_since_m = None
                            est_health = "EXPIRED"
                            pos_sigma = 999.0
                            kf_mean = None
                            kf_cov = None

                        pat_state = pat_fsm.update(
                            association_status=assoc_status,
                            measurement_accepted=target_detected,
                            candidate_count=len(detected_candidates),
                            estimator_health=est_health,
                            position_sigma_px=pos_sigma,
                            angular_error_deg=total_err,
                            current_time=current_time,
                            kf_pos_mean=kf_mean,
                            kf_pos_cov=kf_cov,
                            time_since_measurement=time_since_m
                        )
                        sm_state = pat_state.value if hasattr(pat_state, "value") else str(pat_state)
                        
                        # Phase 6/6.5 Certified Gimbal Controller
                        pan_rate_cmd, tilt_rate_cmd, ctrl_telem = gimbal_controller.compute_control(
                            kf_state=kf_state if (kalman_filter.is_initialized and (is_sm_detected or target_detected)) else None,
                            camera_cx=camera_model.cx,
                            camera_cy=camera_model.cy,
                            camera_fx=camera_model.fx,
                            camera_fy=camera_model.fy,
                            dt=0.016,
                            pat_state=sm_state,
                            search_pan_offset_deg=pat_fsm.search_offset_pan_deg,
                            search_tilt_offset_deg=pat_fsm.search_offset_tilt_deg
                        )

                        # End-to-end processing time
                        end_processing_time = time.time()
                        e2e_time = (end_processing_time - start_processing_time) * 1000.0
                        fps = frame_count / (current_time - start_time) if current_time > start_time else 0.0
                        
                        # 9. Offline Evaluation Metrics Logging (EVALUATION-ONLY)
                        designated_gt_3d = None
                        designated_gt_dir = None
                        designated_gt_in_fov = False
                        
                        for b in beacons_list:
                            if b.get("id") == designated_target_id:
                                b_pos = np.array(b["position"])
                                cam_to_target = b_pos - camera_pos
                                dist = np.linalg.norm(cam_to_target)
                                direction = (cam_to_target / dist).tolist() if dist > 0 else [0.0, 0.0, -1.0]
                                designated_gt_3d = b_pos.tolist()
                                designated_gt_dir = direction
                                
                                target_camera = camera_model.world_to_camera(b_pos, pan_rad, tilt_rad)
                                _, _, is_in_fov = camera_model.camera_to_pixel(target_camera)
                                designated_gt_in_fov = is_in_fov
                                break
                                
                        pixel_err = float(np.sqrt((target_center[0] - camera_model.cx)**2 + (target_center[1] - camera_model.cy)**2)) if target_detected else None
                        metrics_tracker.log_frame(
                            timestamp=current_time,
                            ground_truth_3d_position=designated_gt_3d,
                            ground_truth_direction=designated_gt_dir,
                            detected_position=target_center if target_detected else None,
                            predicted_position=[float(kf_state[0]), float(kf_state[1])] if is_sm_detected else None,
                            camera_position=camera_pos.tolist(),
                            camera_orientation=camera_ori,
                            pan_angle=gimbal_pan,
                            tilt_angle=gimbal_tilt,
                            pixel_error=pixel_err,
                            horizontal_angular_error=h_err,
                            vertical_angular_error=v_err,
                            total_angular_error=total_err,
                            confidence=target_confidence,
                            fps=fps,
                            inference_time_ms=det_time,
                            tracking_time_ms=track_time,
                            end_to_end_time_ms=e2e_time,
                            is_in_fov=designated_gt_in_fov,
                            detected=target_detected,
                            tracking_state=sm_state
                        )
                        
                        # Get live metrics for streaming
                        live_metrics = metrics_tracker.get_live_metrics()
                        
                        # 10. Construct and Stream Telemetry Response
                        response = {
                            "type": "telemetry",
                            "frame_index": frame_count,
                            "timestamp": current_time,
                            "tracking_state": sm_state,
                            "detected": target_detected,
                            "active_detector": active_detector,
                            "fallback_warning": detector_manager.fallback_warning_active,
                            "inference_time_ms": round(det_time, 2),
                            "fps": round(fps, 1),
                            "kalman": {
                                "active": is_sm_detected,
                                "x": float(kf_state[0]),
                                "y": float(kf_state[1]),
                                "vx": float(kf_state[2]) if sim_state.ablation.get("kalman_filter", True) else 0.0,
                                "vy": float(kf_state[3]) if sim_state.ablation.get("kalman_filter", True) else 0.0,
                                "timestamp": current_time
                            },
                            "confidence": round(target_confidence, 2) if target_detected else 0.0,
                            "bbox": target_bbox,
                            "raw_center": target_center,
                            "angular_error_total": total_err,
                            "angular_error_h": h_err,
                            "angular_error_v": v_err,
                            "pan_command": round(pan_rate_cmd, 4),
                            "tilt_command": round(tilt_rate_cmd, 4),
                            "controller_telemetry": ctrl_telem,
                            "search_offset_pan_deg": round(pat_fsm.search_offset_pan_deg, 4),
                            "search_offset_tilt_deg": round(pat_fsm.search_offset_tilt_deg, 4),
                            "pat_telemetry": pat_fsm.get_telemetry(current_time),
                            "candidates": matched_candidates,
                            "live_metrics": live_metrics,
                            "ablation": sim_state.ablation,
                            "pat_fsm": {
                                "state": sm_state,
                                "transition_reason": pat_fsm.transition_reason,
                                "lock_eligible": pat_fsm.lock_eligible,
                                "reacquire_attempts": pat_fsm.reacquire_attempts,
                                "search_active": pat_fsm.search_generator.active,
                                "search_pattern": pat_fsm.search_generator.config.pattern_type,
                                "search_offsets": [pat_fsm.search_offset_pan_deg, pat_fsm.search_offset_tilt_deg]
                            }
                        }
                        
                        if frame_count < 5 or frame_count % 30 == 0:
                            response["config_sync"] = {
                                "beacons": sim_state.beacons,
                                "search": sim_state.search_config,
                                "disturbances": disturbance_engine.intensities
                            }
                            
                        # Check experiment timeout
                        if experiment_manager.active_experiment_id:
                            if experiment_manager.check_timeout():
                                record = experiment_manager.stop_experiment(live_metrics, metrics_tracker.export_csv())
                                response["type"] = "experiment_complete"
                                response["experiment_result"] = record
                            else:
                                response["experiment_active"] = True
                                response["experiment_id"] = experiment_manager.active_experiment_id
                                response["experiment_time_remaining"] = round(experiment_manager.experiment_duration - (time.time() - experiment_manager.experiment_start_time), 1)
                                
                        await websocket.send_json(response)
                        
                    elif cmd_type == "set_detector":
                        mode = cmd_data.get("mode", "opencv")
                        detector_manager.set_mode(mode)
                        await websocket.send_json({
                            "type": "command_ack",
                            "status": "success",
                            "message": f"Detector mode set to {detector_manager.mode}"
                        })
                    elif cmd_type == "set_designated_target":
                        target_id = cmd_data.get("target_id", "BEACON-01")
                        sim_state.designated_target_id = target_id
                        for b in sim_state.beacons:
                            b["designated"] = b["id"] == target_id
                        await websocket.send_json({
                            "type": "command_ack",
                            "status": "success",
                            "message": f"Designated target set to {target_id}"
                        })
                    elif cmd_type == "set_trajectory":
                        b_id = cmd_data.get("beacon_id")
                        traj = cmd_data.get("trajectory")
                        speed = cmd_data.get("speed", 1.0)
                        start_pos = cmd_data.get("start_position")
                        for b in sim_state.beacons:
                            if b["id"] == b_id:
                                b["trajectory"] = traj
                                b["speed"] = speed
                                if start_pos:
                                    b["start_position"] = start_pos
                        await websocket.send_json({
                            "type": "command_ack",
                            "status": "success",
                            "message": f"Trajectory updated for {b_id}"
                        })
                    elif cmd_type == "set_search_config":
                        config = cmd_data.get("config", {})
                        sim_state.search_config.update(config)
                        await websocket.send_json({
                            "type": "command_ack",
                            "status": "success",
                            "message": "Search configuration updated"
                        })
                    elif cmd_type == "set_disturbances":
                        d_config = cmd_data.get("disturbances", {})
                        for k, v in d_config.items():
                            disturbance_engine.set_intensity(k, v)
                        await websocket.send_json({
                            "type": "command_ack",
                            "status": "success",
                            "message": "Disturbances configuration updated"
                        })
                    elif cmd_type == "reset_disturbances":
                        disturbance_engine.reset()
                        await websocket.send_json({
                            "type": "command_ack",
                            "status": "success",
                            "message": "Disturbances reset to zero"
                        })
                    elif cmd_type == "reset_metrics":
                        metrics_tracker.reset()
                        await websocket.send_json({
                            "type": "command_ack",
                            "status": "success",
                            "message": "Metrics tracker reset"
                        })
                    elif cmd_type == "set_association_mode":
                        assoc_m = cmd_data.get("mode", "robust_multifeature")
                        association_engine.mode = assoc_m
                        await websocket.send_json({
                            "type": "command_ack",
                            "status": "success",
                            "message": f"Association mode set to {association_engine.mode}"
                        })
                    elif cmd_type == "set_ablation":
                        toggles = cmd_data.get("toggles", {})
                        sim_state.ablation.update(toggles)
                        await websocket.send_json({
                            "type": "command_ack",
                            "status": "success",
                            "message": f"Ablation settings updated: {sim_state.ablation}"
                        })
                except Exception as e:
                    logger.error(f"Error parsing client text command: {e}")
                    
    except WebSocketDisconnect:
        manager.disconnect(websocket)
    except Exception as e:
        logger.error(f"Error in websocket endpoint: {e}")
        manager.disconnect(websocket)


@app.get("/")
async def serve_index():
    index_file = os.path.join(FRONTEND_DIST, "index.html")
    if os.path.exists(index_file):
        return FileResponse(index_file)
    return {
        "status": "online",
        "message": "FSOC Virtual Camera Tracking Simulator Backend Active. Build frontend to view GUI.",
        "api_docs": "/docs",
        "ws_endpoint": "/ws/tracking"
    }


@app.get("/{full_path:path}")
async def serve_spa_fallback(full_path: str):
    # Guard against intercepting unhandled API or WS routes
    if full_path.startswith("api/") or full_path.startswith("ws"):
        raise HTTPException(status_code=404, detail="Not Found")
    
    file_path = os.path.join(FRONTEND_DIST, full_path)
    if os.path.exists(file_path) and os.path.isfile(file_path):
        return FileResponse(file_path)
    
    index_file = os.path.join(FRONTEND_DIST, "index.html")
    if os.path.exists(index_file):
        return FileResponse(index_file)
    
    raise HTTPException(status_code=404, detail="Not Found")
