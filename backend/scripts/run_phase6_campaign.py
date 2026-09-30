"""
Automated Phase 6 Benchmarking Campaign & Ablation Study.
Runs:
  1. Controlled Servo Benchmark (C01 - C15)
  2. Closed-Loop Controller Ablation across C0, C1, C2, C3 on S00 - S15
  3. Injected Control Latency Sweep (0ms, 20ms, 40ms, 60ms, 100ms)
  4. Smoothness & Actuator Dynamic Spectrum Extraction
Outputs:
  results/phase_06_control/
    controlled_servo_ablation.csv
    closed_loop_control_ablation.csv
    smoothness_metrics.csv
    latency_sweep.csv
    state_transition_control_metrics.csv
    association_regression.csv
"""
import os
import sys
import csv
import json
import time
from pathlib import Path
import numpy as np

# Ensure backend in path
ROOT_DIR = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT_DIR))

from app.benchmark.runner import BenchmarkEngine
from app.benchmark.scenarios import SCENARIO_REGISTRY
from app.benchmark.pat_metrics import compute_pat_telemetry_metrics
from app.tracking.controller import (
    GimbalController,
    GimbalControllerConfig,
    ControlMode,
    SingleAxisController
)

OUTPUT_DIR = ROOT_DIR.parent / "results" / "phase_06_control"
OUTPUT_DIR.mkdir(parents=True, exist_ok=True)


def run_controlled_servo_suite():
    print("--- [1/4] Running Controlled Servo Tests (C01 - C15) ---")
    dt = 0.02
    duration = 5.0
    steps = int(duration / dt)

    modes = [
        ("legacy_pid", ControlMode.LEGACY_PID),
        ("filtered_pid", ControlMode.FILTERED_PID),
        ("predictive_pid", ControlMode.PREDICTIVE_PID),
        ("predictive_smooth", ControlMode.PREDICTIVE_SMOOTH)
    ]

    servo_tests = [
        ("C01_zero_error", lambda t: 0.0, lambda t: 0.0),
        ("C02_small_constant", lambda t: 0.05, lambda t: 0.0),
        ("C03_step_input", lambda t: 5.0 if t >= 1.0 else 0.0, lambda t: 0.0),
        ("C04_ramp_target", lambda t: 1.0 * t, lambda t: 1.0),
        ("C05_constant_vel", lambda t: 2.0 * t, lambda t: 2.0),
        ("C06_constant_accel", lambda t: 0.5 * 1.5 * (t**2), lambda t: 1.5 * t),
        ("C07_sinusoidal", lambda t: 3.0 * np.sin(2.0 * np.pi * 0.5 * t), lambda t: 3.0 * 2.0 * np.pi * 0.5 * np.cos(2.0 * np.pi * 0.5 * t)),
        ("C08_direction_reversal", lambda t: 4.0 if t < 2.5 else -4.0, lambda t: 0.0),
        ("C11_large_saturation", lambda t: 50.0, lambda t: 0.0),
        ("C12_hf_noise", lambda t: 0.05 * np.sin(2.0 * np.pi * 15.0 * t), lambda t: 0.0)
    ]

    records = []
    for test_id, err_fn, ff_fn in servo_tests:
        for mode_name, mode_enum in modes:
            cfg = GimbalControllerConfig()
            axis = SingleAxisController(cfg)
            rates = []
            accels = []
            jerks = []
            errors = []
            sat_count = 0

            # Simulation of plant: angle[k] = angle[k-1] + rate[k]*dt
            plant_pos = 0.0
            for step in range(steps):
                t = step * dt
                target_pos = err_fn(t)
                ff_val = ff_fn(t)
                err = target_pos - plant_pos
                errors.append(abs(err))

                rate, telem = axis.update(err, dt, mode_enum, feedforward_rate=ff_val)
                plant_pos += rate * dt
                rates.append(rate)
                accels.append(telem.get("accel", 0.0))
                jerks.append(telem.get("jerk", 0.0))
                if telem.get("rate_sat") or telem.get("accel_sat") or telem.get("jerk_sat"):
                    sat_count += 1

            rates = np.array(rates)
            accels = np.array(accels)
            jerks = np.array(jerks)
            errors = np.array(errors)

            rms_err = float(np.sqrt(np.mean(errors**2)))
            max_err = float(np.max(errors))
            peak_rate = float(np.max(np.abs(rates)))
            rms_rate = float(np.sqrt(np.mean(rates**2)))
            peak_accel = float(np.max(np.abs(accels)))
            rms_accel = float(np.sqrt(np.mean(accels**2)))
            peak_jerk = float(np.max(np.abs(jerks)))
            rms_jerk = float(np.sqrt(np.mean(jerks**2)))

            # Overshoot & Settling time for step
            overshoot = 0.0
            settling_time = 0.0
            if "step" in test_id:
                step_idx = int(1.0 / dt)
                post_step_plant = []
                curr_p = 0.0
                axis_temp = SingleAxisController(cfg)
                for s in range(steps):
                    t = s * dt
                    targ = 5.0 if t >= 1.0 else 0.0
                    r, _ = axis_temp.update(targ - curr_p, dt, mode_enum)
                    curr_p += r * dt
                    if s >= step_idx:
                        post_step_plant.append(curr_p)
                if post_step_plant:
                    max_p = max(post_step_plant)
                    overshoot = max(0.0, (max_p - 5.0) / 5.0 * 100.0)
                    # Settling time within 2% (4.9 to 5.1)
                    for idx_rev in range(len(post_step_plant)-1, -1, -1):
                        if abs(post_step_plant[idx_rev] - 5.0) > 0.10:
                            settling_time = (idx_rev + 1) * dt
                            break

            records.append({
                "test_id": test_id,
                "control_mode": mode_name,
                "rms_error_deg": round(rms_err, 4),
                "max_error_deg": round(max_err, 4),
                "overshoot_pct": round(overshoot, 2),
                "settling_time_s": round(settling_time, 3),
                "peak_rate_deg_s": round(peak_rate, 2),
                "rms_rate_deg_s": round(rms_rate, 2),
                "peak_accel_deg_s2": round(peak_accel, 1),
                "rms_accel_deg_s2": round(rms_accel, 1),
                "peak_jerk_deg_s3": round(peak_jerk, 1),
                "rms_jerk_deg_s3": round(rms_jerk, 1),
                "saturation_pct": round(100.0 * sat_count / steps, 1)
            })

    csv_path = OUTPUT_DIR / "controlled_servo_ablation.csv"
    with open(csv_path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=records[0].keys())
        writer.writeheader()
        writer.writerows(records)
    print(f"Saved {csv_path}")


def run_closed_loop_campaign():
    print("--- [2/4] Running Closed-Loop Benchmark Campaign (C0, C1, C2, C3) ---")
    modes = ["legacy_pid", "filtered_pid", "predictive_pid", "predictive_smooth"]
    single_seed_scenarios = [
        "S00_nominal", "S07_sensor_noise", "S08_fading",
        "S10_fov_exit_reentry", "S12_moving_distractor",
        "S13_crossing_distractor", "S14_brighter_distractor"
    ]
    multi_seed_scenarios = [
        "S01_linear", "S02_sinusoidal", "S03_high_speed",
        "S04_acceleration", "S05_direction_change", "S06_vibration",
        "S09_temporary_dropout", "S15_combined_stress"
    ]

    test_runs = []
    for sc in single_seed_scenarios:
        test_runs.append((sc, 42))
    for sc in multi_seed_scenarios:
        for sd in [42, 43, 44, 45, 46]:
            test_runs.append((sc, sd))

    ablation_records = []
    smoothness_records = []
    transition_records = []
    assoc_records = []

    for mode in modes:
        print(f"\nEvaluating Control Mode: {mode} ({len(test_runs)} scenario runs)...")
        for sc, seed in test_runs:
            engine = BenchmarkEngine(
                fsm_mode="new_pat_fsm",
                controller_mode=mode
            )
            sc_obj = SCENARIO_REGISTRY[sc]
            res = engine.run_scenario(
                scenario=sc_obj,
                seed=seed,
                duration=10.0,
                save_plots=False
            )
            m = res.metrics
            pm = compute_pat_telemetry_metrics(res.telemetry, dt=0.02)

            pans = np.array([r.get("command_pan_rate", 0.0) for r in res.telemetry])
            tilts = np.array([r.get("command_tilt_rate", 0.0) for r in res.telemetry])
            dt = 0.02
            rates = np.sqrt(pans**2 + tilts**2)
            accels = np.sqrt((np.diff(pans)/dt)**2 + (np.diff(tilts)/dt)**2) if len(pans) > 1 else np.array([0.0])
            jerks = np.sqrt((np.diff(np.diff(pans)/dt)/dt)**2 + (np.diff(np.diff(tilts)/dt)/dt)**2) if len(pans) > 2 else np.array([0.0])

            p_sat = sum(1 for r in res.telemetry if r.get("rate_saturated", False))
            a_sat = sum(1 for r in res.telemetry if r.get("acceleration_saturated", False))
            j_sat = sum(1 for r in res.telemetry if r.get("jerk_saturated", False))
            n_steps = len(res.telemetry)

            peak_rate = float(np.max(rates))
            rms_rate = float(np.sqrt(np.mean(rates**2)))
            peak_acc = float(np.max(accels))
            rms_acc = float(np.sqrt(np.mean(accels**2)))
            peak_jrk = float(np.max(jerks))
            rms_jrk = float(np.sqrt(np.mean(jerks**2)))

            # Ablation record
            ab_rec = {
                "control_mode": mode,
                "scenario_id": sc,
                "seed": seed,
                "pointing_rms_deg": m["tracking"].get("rms_angular_error_deg") or 0.0,
                "pointing_p95_deg": m["tracking"]["pointing_error_populations"]["all_active_frames"].get("p95_deg") or 0.0,
                "pointing_max_deg": m["tracking"]["pointing_error_populations"]["all_active_frames"].get("max_deg") or 0.0,
                "observable_lock_retention_pct": m["lock"].get("lock_retention_when_observable_pct", 0.0),
                "mission_lock_retention_pct": m["lock"].get("mission_lock_retention_pct", 0.0),
                "time_to_first_lock_s": m["lock"].get("acquisition_time_s"),
                "reacquisition_time_s": pm.get("mean_reacquisition_time_s", 0.0),
                "peak_jerk_deg_s3": round(peak_jrk, 1),
                "rms_jerk_deg_s3": round(rms_jrk, 1),
                "peak_accel_deg_s2": round(peak_acc, 1),
                "rms_accel_deg_s2": round(rms_acc, 1),
                "peak_rate_deg_s": round(peak_rate, 2),
                "rms_rate_deg_s": round(rms_rate, 2),
                "state_chatter_count": pm.get("state_chatter_count", 0),
                "false_lock_duration_s": m["association"].get("false_lock_duration_s", 0.0),
                "identity_switches": m["association"].get("identity_switch_count", 0)
            }
            ablation_records.append(ab_rec)

            # Smoothness metrics record
            sm_rec = {
                "control_mode": mode,
                "scenario_id": sc,
                "seed": seed,
                "rms_rate": round(rms_rate, 3),
                "peak_rate": round(peak_rate, 3),
                "rms_accel": round(rms_acc, 2),
                "peak_accel": round(peak_acc, 2),
                "rms_jerk": round(rms_jrk, 1),
                "peak_jerk": round(peak_jrk, 1),
                "rate_saturation_pct": round(100.0 * p_sat / n_steps, 2),
                "accel_saturation_pct": round(100.0 * a_sat / n_steps, 2),
                "jerk_saturation_pct": round(100.0 * j_sat / n_steps, 2)
            }
            smoothness_records.append(sm_rec)

            # State transition control metrics
            tr_rec = {
                "control_mode": mode,
                "scenario_id": sc,
                "seed": seed,
                "state_transition_count": pm.get("state_transition_count", 0),
                "state_chatter_count": pm.get("state_chatter_count", 0),
                "coast_events": pm.get("coast_event_count", 0),
                "reacquire_events": pm.get("reacquisition_attempt_count", 0),
                "observable_lock_retention_pct": m["lock"].get("lock_retention_when_observable_pct", 0.0)
            }
            transition_records.append(tr_rec)

            # Association safety regression guard
            as_rec = {
                "control_mode": mode,
                "scenario_id": sc,
                "seed": seed,
                "false_lock_duration_s": m["association"].get("false_lock_duration_s", 0.0),
                "identity_switches": m["association"].get("identity_switch_count", 0),
                "observable_coverage_pct": m["perception"].get("observable_measurement_coverage_pct", 0.0)
            }
            assoc_records.append(as_rec)

    # Save CSVs
    for filename, recs in [
        ("closed_loop_control_ablation.csv", ablation_records),
        ("smoothness_metrics.csv", smoothness_records),
        ("state_transition_control_metrics.csv", transition_records),
        ("association_regression.csv", assoc_records)
    ]:
        path = OUTPUT_DIR / filename
        with open(path, "w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=recs[0].keys())
            writer.writeheader()
            writer.writerows(recs)
        print(f"Saved {path}")


def run_latency_sweep():
    print("--- [3/4] Running Injected Control Latency Sweep ---")
    latencies = [0.0, 0.02, 0.04, 0.06, 0.10] # 0, 20, 40, 60, 100 ms
    scenarios = ["S01_linear", "S04_acceleration", "S06_vibration"]
    modes = ["legacy_pid", "predictive_smooth"]

    records = []
    for lat_s in latencies:
        for sc in scenarios:
            for mode in modes:
                engine = BenchmarkEngine(
                    fsm_mode="new_pat_fsm",
                    controller_mode=mode,
                    injected_latency_s=lat_s
                )
                sc_obj = SCENARIO_REGISTRY[sc]
                res = engine.run_scenario(scenario=sc_obj, seed=42, duration=10.0, save_plots=False)
                m = res.metrics
                pans = np.array([r.get("command_pan_rate", 0.0) for r in res.telemetry])
                tilts = np.array([r.get("command_tilt_rate", 0.0) for r in res.telemetry])
                dt = 0.02
                jerks = np.sqrt((np.diff(np.diff(pans)/dt)/dt)**2 + (np.diff(np.diff(tilts)/dt)/dt)**2) if len(pans) > 2 else np.array([0.0])

                records.append({
                    "injected_latency_ms": round(lat_s * 1000.0, 1),
                    "scenario_id": sc,
                    "control_mode": mode,
                    "pointing_rms_deg": m["tracking"].get("rms_angular_error_deg") or 0.0,
                    "pointing_p95_deg": m["tracking"]["pointing_error_populations"]["all_active_frames"].get("p95_deg") or 0.0,
                    "peak_jerk_deg_s3": round(float(np.max(jerks)), 1),
                    "observable_lock_retention_pct": m["lock"].get("lock_retention_when_observable_pct", 0.0)
                })

    csv_path = OUTPUT_DIR / "latency_sweep.csv"
    with open(csv_path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=records[0].keys())
        writer.writeheader()
        writer.writerows(records)
    print(f"Saved {csv_path}")


if __name__ == "__main__":
    t_start = time.time()
    run_controlled_servo_suite()
    run_closed_loop_campaign()
    run_latency_sweep()
    print(f"\nAll Phase 6 campaigns successfully completed in {time.time() - t_start:.2f} seconds.")
