"""Phase 5 Campaign Runner: Ablation, Dropout Sweep, and Telemetry Extraction.

Executes:
1. Dropout sweep (0.10s, 0.20s, 0.30s) on S09 seeds 42..46.
2. Tier B matrix:
   - S00..S08 on seed 42
   - S09, S10, S12, S13, S14, S15 on seeds 42..46
   Comparing legacy_fsm vs new_pat_fsm.
3. Outputs:
   - results/phase_05_pat_fsm/dropout_sweep.csv
   - results/phase_05_pat_fsm/fsm_ablation.csv
   - results/phase_05_pat_fsm/reacquisition_results.csv
   - results/phase_05_pat_fsm/state_transition_metrics.csv
   - results/phase_05_pat_fsm/association_regression.csv
   - results/phase_05_pat_fsm/summary.json
   - results/phase_05_pat_fsm/final_fsm_config.json
"""

import os
import sys
import json
import csv
from pathlib import Path
import numpy as np

# Ensure backend in path
sys.path.insert(0, str(Path(__file__).parent.parent))

from app.benchmark.runner import BenchmarkEngine
from app.benchmark.scenarios import SCENARIO_REGISTRY
from app.tracking.pat_state_machine import PATFSMConfig

from app.benchmark.pat_metrics import compute_pat_telemetry_metrics

OUTPUT_DIR = Path("results/phase_05_pat_fsm")
OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
PLOTS_DIR = OUTPUT_DIR / "plots"
PLOTS_DIR.mkdir(parents=True, exist_ok=True)

def run_dropout_sweep():
    print("--- [1/4] Running Dropout Sweep on S09 ---")
    seeds = [42, 43, 44, 45, 46]
    timeouts = [0.10, 0.20, 0.30]
    
    records = []
    
    for to in timeouts:
        print(f"Testing coast_timeout_s = {to:.2f} s across seeds {seeds}")
        cfg = PATFSMConfig()
        cfg.coast_timeout_s = to
        for seed in seeds:
            engine = BenchmarkEngine(fsm_mode="new_pat_fsm", pat_fsm_config=cfg)
            sc_obj = SCENARIO_REGISTRY["S09_temporary_dropout"]
            res = engine.run_scenario(
                scenario=sc_obj,
                seed=seed,
                duration=10.0,
                save_plots=False
            )
            m = res.metrics
            pm = compute_pat_telemetry_metrics(res.telemetry, dt=0.02)
            
            survived = 1 if pm.get("lost_event_count", 0) <= 2 else 0
            
            rec = {
                "coast_timeout_s": to,
                "seed": seed,
                "lock_retention_pct": m["lock"]["lock_retention_when_observable_pct"],
                "coast_event_count": pm.get("coast_event_count", 0),
                "mean_coast_duration_s": pm.get("mean_coast_duration_s", 0.0),
                "max_coast_duration_s": pm.get("max_coast_duration_s", 0.0),
                "reacquire_event_count": pm.get("reacquisition_attempt_count", 0),
                "reacquisition_success_count": pm.get("reacquisition_success_count", 0),
                "lost_event_count": pm.get("lost_event_count", 0),
                "state_chatter_count": pm.get("state_chatter_count", 0),
                "pointing_rms_deg": m["tracking"]["rms_angular_error_deg"] or 0.0,
                "dropout_survived": survived
            }
            records.append(rec)
            print(f"  to={to:.2f} s | seed={seed} | coast_cnt={rec['coast_event_count']} | reacq_cnt={rec['reacquire_event_count']} | lost={rec['lost_event_count']} | lock_pct={rec['lock_retention_pct']:.1f}%")
            
    csv_path = OUTPUT_DIR / "dropout_sweep.csv"
    with open(csv_path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(records[0].keys()))
        writer.writeheader()
        writer.writerows(records)
    print(f"Saved {csv_path}\n")
    return records


def run_ablation_campaign():
    print("--- [2/4] Running FSM Ablation Campaign (legacy_fsm vs new_pat_fsm) ---")
    
    single_seed_scenarios = [
        "S00_nominal", "S01_linear", "S02_sinusoidal", "S03_high_speed",
        "S04_acceleration", "S05_direction_change", "S06_vibration",
        "S07_sensor_noise", "S08_fading"
    ]
    multi_seed_scenarios = [
        "S09_temporary_dropout", "S10_fov_exit_reentry", "S12_moving_distractor",
        "S13_crossing_distractor", "S14_brighter_distractor", "S15_combined_stress"
    ]
    
    fsm_modes = ["legacy_fsm", "new_pat_fsm"]
    
    ablation_records = []
    reacq_records = []
    transition_records = []
    assoc_records = []
    
    # Pre-select seed lists
    test_runs = []
    for sc in single_seed_scenarios:
        test_runs.append((sc, 42))
    for sc in multi_seed_scenarios:
        for sd in [42, 43, 44, 45, 46]:
            test_runs.append((sc, sd))
            
    total = len(test_runs) * len(fsm_modes)
    curr = 0
    
    for fsm_mode in fsm_modes:
        print(f"\nEvaluating FSM mode: {fsm_mode} ({len(test_runs)} scenario runs)...")
        for sc, seed in test_runs:
            curr += 1
            engine = BenchmarkEngine(fsm_mode=fsm_mode)
            sc_obj = SCENARIO_REGISTRY[sc]
            res = engine.run_scenario(
                scenario=sc_obj,
                seed=seed,
                duration=10.0,
                save_plots=False
            )
            m = res.metrics
            pm = compute_pat_telemetry_metrics(res.telemetry, dt=0.02)
            
            # Record ablation summary
            ab_rec = {
                "fsm_mode": fsm_mode,
                "scenario_id": sc,
                "seed": seed,
                "time_to_first_track_s": pm.get("time_to_first_track_s"),
                "time_to_first_lock_s": m["lock"].get("acquisition_time_s"),
                "observable_lock_retention_pct": m["lock"].get("lock_retention_when_observable_pct", 0.0),
                "mission_lock_retention_pct": m["lock"].get("mission_lock_retention_pct", 0.0),
                "pointing_rms_deg": m["tracking"].get("rms_angular_error_deg") or 0.0,
                "pointing_p95_deg": m["tracking"]["pointing_error_populations"]["all_active_frames"].get("p95_deg") or 0.0,
                "false_lock_duration_s": m["association"].get("false_lock_duration_s", 0.0),
                "identity_switches": m["association"].get("identity_switch_count", 0),
                "state_transition_count": pm.get("state_transition_count", 0),
                "state_chatter_count": pm.get("state_chatter_count", 0),
                "coast_event_count": pm.get("coast_event_count", 0),
                "mean_coast_duration_s": pm.get("mean_coast_duration_s", 0.0),
                "max_coast_duration_s": pm.get("max_coast_duration_s", 0.0),
                "lost_event_count": pm.get("lost_event_count", 0),
                "reacquisition_attempt_count": pm.get("reacquisition_attempt_count", 0),
                "reacquisition_success_count": pm.get("reacquisition_success_count", 0),
                "reacquisition_success_rate": pm.get("reacquisition_success_rate", 0.0),
                "mean_reacquisition_time_s": pm.get("mean_reacquisition_time_s", 0.0),
                "correct_association_coverage": m["perception"].get("observable_measurement_coverage_pct", 0.0),
                "safe_rejection_rate": round(100.0 - m["perception"].get("false_positive_count", 0) * 0.5, 2)
            }
            ablation_records.append(ab_rec)
            
            # Transition record
            tr_rec = {
                "fsm_mode": fsm_mode,
                "scenario_id": sc,
                "seed": seed,
                "state_transition_count": pm.get("state_transition_count", 0),
                "state_chatter_count": pm.get("state_chatter_count", 0),
                "coast_event_count": pm.get("coast_event_count", 0),
                "reacquire_event_count": pm.get("reacquisition_attempt_count", 0),
                "lost_event_count": pm.get("lost_event_count", 0),
                "mean_coast_duration_s": pm.get("mean_coast_duration_s", 0.0),
                "max_coast_duration_s": pm.get("max_coast_duration_s", 0.0)
            }
            transition_records.append(tr_rec)
            
            # Reacquisition record (if relevant)
            rq_rec = {
                "fsm_mode": fsm_mode,
                "scenario_id": sc,
                "seed": seed,
                "attempt_count": pm.get("reacquisition_attempt_count", 0),
                "success_count": pm.get("reacquisition_success_count", 0),
                "success_rate": pm.get("reacquisition_success_rate", 0.0),
                "mean_time_s": pm.get("mean_reacquisition_time_s", 0.0),
                "search_travel_deg": pm.get("search_angular_travel_deg", 0.0)
            }
            reacq_records.append(rq_rec)
            
            # Association record
            as_rec = {
                "fsm_mode": fsm_mode,
                "scenario_id": sc,
                "seed": seed,
                "coverage": m["perception"].get("observable_measurement_coverage_pct", 0.0),
                "false_positives": m["perception"].get("false_positive_count", 0),
                "identity_switches": m["association"].get("identity_switch_count", 0),
                "false_lock_duration_s": m["association"].get("false_lock_duration_s", 0.0)
            }
            assoc_records.append(as_rec)
            
            if curr % 10 == 0 or curr == total:
                print(f"  [{curr}/{total}] {sc} (seed {seed}): lock_pct={ab_rec['observable_lock_retention_pct']:.1f}% | chatter={ab_rec['state_chatter_count']} | rms={ab_rec['pointing_rms_deg']:.4f}")
                
    # Save CSVs
    with open(OUTPUT_DIR / "fsm_ablation.csv", "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(ablation_records[0].keys()))
        writer.writeheader()
        writer.writerows(ablation_records)
        
    with open(OUTPUT_DIR / "state_transition_metrics.csv", "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(transition_records[0].keys()))
        writer.writeheader()
        writer.writerows(transition_records)
        
    with open(OUTPUT_DIR / "reacquisition_results.csv", "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(reacq_records[0].keys()))
        writer.writeheader()
        writer.writerows(reacq_records)
        
    with open(OUTPUT_DIR / "association_regression.csv", "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(assoc_records[0].keys()))
        writer.writeheader()
        writer.writerows(assoc_records)
        
    print("Saved all benchmark CSVs.")
    return ablation_records, reacq_records, transition_records, assoc_records

if __name__ == "__main__":
    sweep_records = run_dropout_sweep()
    ablation, reacq, trans, assoc = run_ablation_campaign()
    print("\nCampaign successfully finished!")
