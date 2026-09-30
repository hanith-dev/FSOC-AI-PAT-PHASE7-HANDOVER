import os
import json
import time
import logging
from typing import Dict, Any, List, Optional

logger = logging.getLogger("FSOC-ExperimentManager")

class ExperimentManager:
    def __init__(self, results_dir: str, scenarios_config_path: str):
        self.results_dir = results_dir
        self.scenarios_config_path = scenarios_config_path
        
        # Ensure directories exist
        os.makedirs(self.results_dir, exist_ok=True)
        
        # Active experiment state
        self.active_experiment_id: Optional[str] = None
        self.active_scenario_id: Optional[str] = None
        self.experiment_start_time: float = 0.0
        self.experiment_duration: float = 20.0
        self.config_snapshot: Dict[str, Any] = {}
        
        # Load predefined scenarios
        self.scenarios = self._load_scenarios()

    def _load_scenarios(self) -> List[Dict[str, Any]]:
        if os.path.exists(self.scenarios_config_path):
            try:
                with open(self.scenarios_config_path, 'r') as f:
                    data = json.load(f)
                    return data.get("scenarios", [])
            except Exception as e:
                logger.error(f"Error loading scenarios config: {e}")
        
        # Fallback default predefined scenarios
        return [
            {
                "id": "baseline",
                "name": "Baseline Performance",
                "description": "Slow target, no disturbances.",
                "duration": 20.0,
                "disturbances": {
                    "platform_vibration": 0,
                    "camera_jitter": 0,
                    "image_noise": 0,
                    "blur": 0,
                    "atmospheric_turbulence": 0,
                    "illumination_variation": 0
                },
                "beacons": [
                    {
                        "id": "BEACON-01",
                        "trajectory": "linear",
                        "start_position": [10.0, 5.0, -100.0],
                        "velocity": [-2.0, 0.2, 5.0],
                        "speed": 5.0,
                        "size": 1.5,
                        "intensity": 2.0,
                        "color": "#10B981",
                        "designated": True,
                        "active": True
                    }
                ]
            }
        ]

    def get_scenarios(self) -> List[Dict[str, Any]]:
        return self.scenarios

    def start_experiment(self, scenario_id: str, client_config: Dict[str, Any]) -> Dict[str, Any]:
        # Find scenario
        scenario = next((s for s in self.scenarios if s["id"] == scenario_id), None)
        if not scenario:
            raise ValueError(f"Scenario {scenario_id} not found.")
            
        timestamp = int(time.time())
        self.active_experiment_id = f"EXP-{scenario_id}-{timestamp}"
        self.active_scenario_id = scenario_id
        self.experiment_start_time = time.time()
        self.experiment_duration = float(scenario.get("duration", 20.0))
        
        # Snapshot of the experiment setup
        self.config_snapshot = {
            "scenario": scenario,
            "ablation": client_config.get("ablation", {
                "ai_detection": True,
                "kalman_filter": True,
                "pid_controller": True,
                "disturbances": True
            }),
            "pid_config": client_config.get("pid_config", {
                "pan": {"kp": 1.5, "ki": 0.05, "kd": 0.1},
                "tilt": {"kp": 1.5, "ki": 0.05, "kd": 0.1}
            }),
            "detector_mode": client_config.get("detector_mode", "opencv")
        }
        
        logger.info(f"Started experiment: {self.active_experiment_id} for scenario {scenario_id}")
        return {
            "experiment_id": self.active_experiment_id,
            "scenario_id": self.active_scenario_id,
            "duration": self.experiment_duration,
            "config": self.config_snapshot
        }

    def stop_experiment(self, computed_metrics: Dict[str, Any], raw_csv_data: str) -> Dict[str, Any]:
        if not self.active_experiment_id:
            return {"status": "error", "message": "No active experiment running"}
            
        exp_id = self.active_experiment_id
        scen_id = self.active_scenario_id
        duration_run = time.time() - self.experiment_start_time
        
        # Save experiment record
        record = {
            "id": exp_id,
            "scenario_id": scen_id,
            "timestamp": time.time(),
            "duration_planned": self.experiment_duration,
            "duration_run": round(duration_run, 2),
            "config_snapshot": self.config_snapshot,
            "metrics": computed_metrics
        }
        
        filepath = os.path.join(self.results_dir, f"{exp_id}.json")
        try:
            with open(filepath, 'w') as f:
                json.dump(record, f, indent=2)
            logger.info(f"Saved experiment results to {filepath}")
        except Exception as e:
            logger.error(f"Error saving experiment results to {filepath}: {e}")
            
        # Save raw frame-level CSV log
        csv_filepath = os.path.join(self.results_dir, f"{exp_id}.csv")
        try:
            with open(csv_filepath, 'w') as f:
                f.write(raw_csv_data)
            logger.info(f"Saved raw CSV log to {csv_filepath}")
        except Exception as e:
            logger.error(f"Error saving raw CSV log to {csv_filepath}: {e}")

        # Automatically generate PDF report
        plot_filepath = os.path.join(self.results_dir, f"{exp_id}_plots.png")
        pdf_filepath = os.path.join(self.results_dir, f"{exp_id}.pdf")
        try:
            from app.experiments.reports import generate_plots, compile_pdf
            # Plot graphs to temporary png
            plots_success = generate_plots(csv_filepath, plot_filepath)
            if plots_success:
                compile_pdf(record, plot_filepath, pdf_filepath)
                # Cleanup temporary image
                if os.path.exists(plot_filepath):
                    os.remove(plot_filepath)
                logger.info(f"Successfully generated PDF report at {pdf_filepath}")
        except Exception as e:
            logger.error(f"Error generating PDF report: {e}")
            
        # Reset state
        self.active_experiment_id = None
        self.active_scenario_id = None
        
        return record

    def check_timeout(self) -> bool:
        if not self.active_experiment_id:
            return False
        return (time.time() - self.experiment_start_time) >= self.experiment_duration

    def get_results(self) -> List[Dict[str, Any]]:
        results = []
        if not os.path.exists(self.results_dir):
            return results
            
        for filename in os.listdir(self.results_dir):
            if filename.endswith(".json"):
                filepath = os.path.join(self.results_dir, filename)
                try:
                    with open(filepath, 'r') as f:
                        results.append(json.load(f))
                except Exception as e:
                    logger.error(f"Error loading result file {filename}: {e}")
                    
        # Sort by timestamp descending (newest first)
        results.sort(key=lambda x: x.get("timestamp", 0.0), reverse=True)
        return results

    def get_comparison(self, ids: List[str]) -> List[Dict[str, Any]]:
        comparison = []
        all_results = self.get_results()
        
        for exp_id in ids:
            res = next((r for r in all_results if r["id"] == exp_id), None)
            if res:
                metrics = res.get("metrics", {})
                config = res.get("config_snapshot", {})
                comparison.append({
                    "id": res["id"],
                    "scenario": res["scenario_id"],
                    "duration": res["duration_run"],
                    "precision": metrics.get("detection", {}).get("precision", "N/A"),
                    "recall": metrics.get("detection", {}).get("recall", "N/A"),
                    "avg_fps": metrics.get("fps", {}).get("avg", "N/A"),
                    "avg_error": metrics.get("tracking_error", {}).get("avg", "N/A"),
                    "max_error": metrics.get("tracking_error", {}).get("max", "N/A"),
                    "rms_error": metrics.get("tracking_error", {}).get("rms", "N/A"),
                    "acquisition_time": metrics.get("acquisition_time", "N/A"),
                    "recovery_time": metrics.get("recovery", {}).get("avg_reacquisition_time", "N/A"),
                    "lock_retention": metrics.get("lock_retention_rate", "N/A"),
                    "e2e_latency_ms": metrics.get("processing_time", {}).get("end_to_end_avg", "N/A"),
                    "ablation": config.get("ablation", {})
                })
        return comparison

    def delete_result(self, id: str) -> bool:
        filepath = os.path.join(self.results_dir, f"{id}.json")
        if os.path.exists(filepath):
            try:
                os.remove(filepath)
                logger.info(f"Deleted experiment result file {filepath}")
                return True
            except Exception as e:
                logger.error(f"Error deleting result file {filepath}: {e}")
        return False
