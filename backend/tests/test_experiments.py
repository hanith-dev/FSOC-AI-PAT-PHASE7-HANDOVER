import os
import shutil
import pytest
from app.experiments.manager import ExperimentManager

@pytest.fixture
def temp_results_dir(tmp_path):
    results_dir = tmp_path / "results"
    return str(results_dir)

@pytest.fixture
def dummy_scenarios_path(tmp_path):
    scenarios_path = tmp_path / "scenarios.json"
    import json
    data = {
        "scenarios": [
            {
                "id": "test_scenario",
                "name": "Test Scenario",
                "description": "A dummy test scenario.",
                "duration": 5.0,
                "disturbances": {
                    "platform_vibration": 10
                },
                "beacons": [
                    {
                        "id": "BEACON-01",
                        "trajectory": "linear",
                        "designated": True
                    }
                ]
            }
        ]
    }
    with open(scenarios_path, 'w') as f:
        json.dump(data, f)
    return str(scenarios_path)

def test_experiment_manager_init(temp_results_dir, dummy_scenarios_path):
    manager = ExperimentManager(temp_results_dir, dummy_scenarios_path)
    scenarios = manager.get_scenarios()
    assert len(scenarios) == 1
    assert scenarios[0]["id"] == "test_scenario"

def test_start_and_stop_experiment(temp_results_dir, dummy_scenarios_path):
    manager = ExperimentManager(temp_results_dir, dummy_scenarios_path)
    
    # Start
    config = {
        "ablation": {"ai_detection": True, "kalman_filter": False},
        "pid_config": {"pan": {"kp": 1.0}}
    }
    start_info = manager.start_experiment("test_scenario", config)
    assert start_info["scenario_id"] == "test_scenario"
    assert manager.active_experiment_id is not None
    assert manager.active_scenario_id == "test_scenario"
    
    # Check timeout (should not be timed out immediately)
    assert not manager.check_timeout()
    
    # Stop
    metrics = {
        "fps": {"avg": 20.0},
        "tracking_error": {"avg": 0.1},
        "lock_retention_rate": 85.0
    }
    dummy_csv = "timestamp,total_ang_err,fps,confidence,pan,tilt,state\n100.0,0.1,20.0,0.9,0.0,0.0,LOCKED\n"
    record = manager.stop_experiment(metrics, dummy_csv)
    assert record["id"] == start_info["experiment_id"]
    assert record["duration_planned"] == 5.0
    assert record["metrics"]["fps"]["avg"] == 20.0
    assert manager.active_experiment_id is None
    
    # Results directory should contain the file
    results = manager.get_results()
    assert len(results) == 1
    assert results[0]["id"] == record["id"]

def test_comparison(temp_results_dir, dummy_scenarios_path):
    manager = ExperimentManager(temp_results_dir, dummy_scenarios_path)
    
    start_info = manager.start_experiment("test_scenario", {})
    dummy_csv = "timestamp,total_ang_err,fps,confidence,pan,tilt,state\n100.0,0.1,15.0,0.9,0.0,0.0,LOCKED\n"
    record = manager.stop_experiment({"fps": {"avg": 15.0}}, dummy_csv)
    
    comparison = manager.get_comparison([record["id"]])
    assert len(comparison) == 1
    assert comparison[0]["id"] == record["id"]
    assert comparison[0]["avg_fps"] == 15.0

def test_delete(temp_results_dir, dummy_scenarios_path):
    manager = ExperimentManager(temp_results_dir, dummy_scenarios_path)
    
    start_info = manager.start_experiment("test_scenario", {})
    dummy_csv = "timestamp,total_ang_err,fps,confidence,pan,tilt,state\n100.0,0.1,20.0,0.9,0.0,0.0,LOCKED\n"
    record = manager.stop_experiment({}, dummy_csv)
    
    assert len(manager.get_results()) == 1
    manager.delete_result(record["id"])
    assert len(manager.get_results()) == 0
