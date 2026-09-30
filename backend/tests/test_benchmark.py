"""
Automated Regression and Determinism Tests for the Scientific Benchmark Engine.
Verifies bitwise determinism across runs, ground-truth firewall non-interference,
and metric calculation integrity.
"""
import pytest
import numpy as np

from app.benchmark.runner import BenchmarkEngine
from app.benchmark.scenarios import SCENARIO_REGISTRY
from app.tracking.pid import PIDController


def test_pid_controller_saturation():
    """Verify PID controller output clamping and saturation flag."""
    pid = PIDController(kp=10.0, output_limit=5.0)
    out = pid.update(100.0, 0.1)
    assert out == 5.0
    assert pid.is_saturated is True

    out_zero = pid.update(0.001, 0.1)  # within deadband
    assert out_zero == 0.0
    assert pid.is_saturated is False


def test_bitwise_determinism_nominal():
    """
    Run S00_nominal twice with the exact same seed (42).
    Assert that all operational simulation states and control decisions are bit-for-bit identical.
    """
    engine = BenchmarkEngine(dt=0.02)
    scenario = SCENARIO_REGISTRY["S00_nominal"]

    run1 = engine.run_scenario(scenario=scenario, seed=42, duration=3.0, output_base_dir="tests/artifacts/bench_det1")
    run2 = engine.run_scenario(scenario=scenario, seed=42, duration=3.0, output_base_dir="tests/artifacts/bench_det2")

    t1 = run1.telemetry
    t2 = run2.telemetry

    assert len(t1) == len(t2)
    n_frames = len(t1)

    operational_fields = [
        "simulation_time",
        "detection_available",
        "raw_measurement_x",
        "raw_measurement_y",
        "selected_candidate_x",
        "selected_candidate_y",
        "candidate_count",
        "estimated_x",
        "estimated_y",
        "estimated_vx",
        "estimated_vy",
        "tracking_state",
        "camera_pan",
        "camera_tilt",
        "command_pan_rate",
        "command_tilt_rate",
        "tracking_error_px",
        "tracking_error_deg",
        "controller_saturated"
    ]

    for i in range(n_frames):
        row1 = t1[i]
        row2 = t2[i]
        for field in operational_fields:
            v1 = row1.get(field)
            v2 = row2.get(field)
            assert v1 == v2, f"Determinism mismatch at frame {i} in field '{field}': {v1} vs {v2}"

    # Verify metrics summary match
    assert run1.metrics["summary_pairing"]["rms_angular_error_deg"] == run2.metrics["summary_pairing"]["rms_angular_error_deg"]
    assert run1.metrics["summary_pairing"]["accepted_measurement_coverage_pct"] == run2.metrics["summary_pairing"]["accepted_measurement_coverage_pct"]


def test_bitwise_determinism_with_disturbances():
    """
    Run S06_vibration twice with the exact same seed (101).
    Assert that stochastic vibration sequences produce identical operational trajectories.
    """
    engine = BenchmarkEngine(dt=0.02)
    scenario = SCENARIO_REGISTRY["S06_vibration"]

    run1 = engine.run_scenario(scenario=scenario, seed=101, duration=2.5, output_base_dir="tests/artifacts/bench_vib1")
    run2 = engine.run_scenario(scenario=scenario, seed=101, duration=2.5, output_base_dir="tests/artifacts/bench_vib2")

    t1 = run1.telemetry
    t2 = run2.telemetry

    assert len(t1) == len(t2)
    for i in range(len(t1)):
        assert t1[i]["camera_pan"] == t2[i]["camera_pan"]
        assert t1[i]["camera_tilt"] == t2[i]["camera_tilt"]
        assert t1[i]["tracking_state"] == t2[i]["tracking_state"]
        assert t1[i]["command_pan_rate"] == t2[i]["command_pan_rate"]


def test_benchmark_truth_firewall_non_interference():
    """
    Verify that altering post-hoc ground-truth metadata cannot change operational outputs:
    selected candidate, Kalman state, FSM state, or controller commands.
    """
    engine = BenchmarkEngine(dt=0.02)
    scenario = SCENARIO_REGISTRY["S00_nominal"]

    # Run standard scenario
    run_normal = engine.run_scenario(scenario=scenario, seed=42, duration=2.0, output_base_dir="tests/artifacts/bench_fw_norm")

    # Now run modified scenario where evaluation target ID is renamed to 'BOGUS-ID'
    modified_scenario = SCENARIO_REGISTRY["S00_nominal"]
    modified_scenario.designated_target_id = "BOGUS-TARGET-XYZ"

    run_modified = engine.run_scenario(scenario=modified_scenario, seed=42, duration=2.0, output_base_dir="tests/artifacts/bench_fw_mod")

    # Operational tracker fields must remain 100% IDENTICAL
    t_norm = run_normal.telemetry
    t_mod = run_modified.telemetry

    for i in range(len(t_norm)):
        assert t_norm[i]["selected_candidate_x"] == t_mod[i]["selected_candidate_x"]
        assert t_norm[i]["selected_candidate_y"] == t_mod[i]["selected_candidate_y"]
        assert t_norm[i]["estimated_x"] == t_mod[i]["estimated_x"]
        assert t_norm[i]["estimated_y"] == t_mod[i]["estimated_y"]
        assert t_norm[i]["tracking_state"] == t_mod[i]["tracking_state"]
        assert t_norm[i]["command_pan_rate"] == t_mod[i]["command_pan_rate"]
        assert t_norm[i]["command_tilt_rate"] == t_mod[i]["command_tilt_rate"]

    # Reset back to nominal
    modified_scenario.designated_target_id = "BEACON-01"


def test_moving_distractor_temporal_continuity():
    """
    Verify S12 moving distractor moves along continuous path, approaches target,
    crosses path, and remains visible across multiple frames.
    """
    scenario = SCENARIO_REGISTRY["S12_moving_distractor"]
    dt = 0.02
    n_steps = int(scenario.duration / dt)

    target_positions = []
    distractor_positions = []

    for step in range(n_steps):
        t = step * dt
        beacons = scenario.get_beacons(t)
        assert len(beacons) == 2, f"Frame {step} must have both target and distractor"

        target = next(b for b in beacons if b["id"] == "BEACON-01")
        distractor = next(b for b in beacons if b["id"] == "DISTRACTOR-01")

        target_positions.append(target["position"])
        distractor_positions.append(distractor["position"])

    # Verify positions change continuously with no discontinuous teleportation
    dist_arr = np.array(distractor_positions)
    deltas = np.diff(dist_arr, axis=0)
    max_step_dist = np.max(np.linalg.norm(deltas, axis=1))

    # At 50Hz, a 2.5m/s motion has dt*v = 0.05m step size
    assert max_step_dist < 0.2, f"Distractor motion must be smooth and continuous, found step: {max_step_dist}"

    # Verify distractor approaches target
    tgt_arr = np.array(target_positions)
    separations = np.linalg.norm(tgt_arr - dist_arr, axis=1)
    min_sep = np.min(separations)
    assert min_sep < 3.0, f"Distractor must pass close to designated target, min separation: {min_sep}m"


def test_metric_pairing_requirement():
    """
    Verify that tracking error is strictly paired with accepted measurement coverage.
    """
    engine = BenchmarkEngine(dt=0.02)
    scenario = SCENARIO_REGISTRY["S00_nominal"]
    res = engine.run_scenario(scenario=scenario, seed=42, duration=2.0, output_base_dir="tests/artifacts/bench_pair")

    pairing = res.metrics.get("summary_pairing")
    assert pairing is not None
    assert "rms_angular_error_deg" in pairing
    assert "accepted_measurement_coverage_pct" in pairing
    assert pairing["accepted_measurement_coverage_pct"] is not None
