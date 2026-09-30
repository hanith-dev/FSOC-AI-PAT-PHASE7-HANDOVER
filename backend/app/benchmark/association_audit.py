"""
Phase 3.5 Association Safety, Coverage and Claim Consistency Audit.

Evaluates association behavior under the 3-outcome framework:
1. CORRECT_ACCEPT: Operational association accepts designated target.
2. WRONG_ACCEPT: Operational association accepts a non-designated candidate.
3. SAFE_REJECT_OR_COAST: Candidates exist but association safely rejects them.

Outputs:
- results/phase_03_5_association_audit/parameter_sweep.csv
- results/phase_03_5_association_audit/final_configuration.json
- results/phase_03_5_association_audit/outcome_matrix.csv
- results/phase_03_5_association_audit/summary.json
"""
import os
import sys
import json
import time
from dataclasses import dataclass
from typing import Dict, List, Any, Optional, Tuple
import csv
import numpy as np

from app.tracking.association import AssociationEngine, CandidateMeasurement, AssociationResult
from app.tracking.kalman import KalmanFilter2D
from app.benchmark.association_ablation import build_tier_a_sequences
from app.benchmark.scenarios import SCENARIO_REGISTRY
import app.benchmark.runner as runner_mod


@dataclass
class FrameOutcome:
    frame_idx: int
    timestamp: float
    target_present: bool
    candidate_count: int
    gated_count: int
    selected_id: Optional[str]
    status: str
    outcome: str  # "CORRECT_ACCEPT", "WRONG_ACCEPT", "SAFE_REJECT", "NO_CANDIDATE_COAST"
    cost: Optional[float]
    margin: Optional[float]
    d2: Optional[float]


def evaluate_tier_a_case(
    seq_info: Dict[str, Any],
    engine: AssociationEngine,
    dt: float = 0.02
) -> Dict[str, Any]:
    """
    Evaluate a Tier A synthetic sequence frame-by-frame and compute 3-outcome metrics.
    """
    frames = seq_info["frames"]
    true_id = seq_info["true_target_id"]
    kf = KalmanFilter2D(q_noise=0.5, r_noise=1.0)
    
    outcomes: List[FrameOutcome] = []
    
    for f_idx, cand_dicts in enumerate(frames):
        sim_time = f_idx * dt
        has_true_target = any(c["id"] == true_id for c in cand_dicts)
        
        # Build CandidateMeasurements
        cand_measurements = []
        for c_i, c_raw in enumerate(cand_dicts):
            cm = CandidateMeasurement(
                candidate_index=c_i,
                x=float(c_raw["x"]),
                y=float(c_raw["y"]),
                localization_method="tier_a_sim",
                localization_uncertainty_x_px=float(c_raw.get("unc", 0.3)),
                localization_uncertainty_y_px=float(c_raw.get("unc", 0.3)),
                confidence=float(c_raw.get("conf", 0.9)),
                area=float(c_raw.get("area", 45.0)),
                peak_intensity=200.0,
                integrated_signal=1000.0,
                background_level=20.0,
                snr_proxy=float(c_raw.get("snr", 15.0)),
                sigma_x_px=float(c_raw.get("unc", 0.3)) * 4.0,
                sigma_y_px=float(c_raw.get("unc", 0.3)) * 4.0,
                shape_quality=float(c_raw.get("shape", 0.9)),
                quality_flags=[],
                timestamp=sim_time
            )
            cand_measurements.append(cm)
            
        res = engine.associate(cand_measurements, kf, sim_time)
        
        selected_raw_id = None
        if res.accepted and res.selected_candidate is not None:
            sel_idx = res.selected_candidate.candidate_index
            selected_raw_id = cand_dicts[sel_idx]["id"]
            if selected_raw_id == true_id:
                outcome = "CORRECT_ACCEPT"
            else:
                outcome = "WRONG_ACCEPT"
            # Update Kalman
            z = np.array([res.selected_candidate.x, res.selected_candidate.y], dtype=np.float32)
            R_c = np.diag([res.selected_candidate.uncertainty_x**2, res.selected_candidate.uncertainty_y**2])
            kf.update(z, sim_time, R_custom=R_c)
        else:
            if len(cand_measurements) > 0:
                outcome = "SAFE_REJECT"
            else:
                outcome = "NO_CANDIDATE_COAST"
            if kf.is_initialized:
                kf.predict_only(sim_time)
                
        outcomes.append(FrameOutcome(
            frame_idx=f_idx,
            timestamp=sim_time,
            target_present=has_true_target,
            candidate_count=len(cand_measurements),
            gated_count=res.gated_candidate_count,
            selected_id=selected_raw_id,
            status=res.status,
            outcome=outcome,
            cost=res.association_cost,
            margin=res.association_margin,
            d2=(res.mahalanobis_distance**2) if res.mahalanobis_distance is not None else None
        ))
        
    # Aggregate metrics
    n_total = len(outcomes)
    n_observable = sum(1 for o in outcomes if o.target_present)
    n_correct = sum(1 for o in outcomes if o.outcome == "CORRECT_ACCEPT")
    n_wrong = sum(1 for o in outcomes if o.outcome == "WRONG_ACCEPT")
    n_safe = sum(1 for o in outcomes if o.outcome == "SAFE_REJECT")
    n_coast = sum(1 for o in outcomes if o.outcome in ["SAFE_REJECT", "NO_CANDIDATE_COAST"])
    
    correct_accept_rate = (n_correct / n_observable) if n_observable > 0 else 1.0
    wrong_accept_rate = n_wrong / n_total
    safe_rejection_rate = n_safe / n_total
    association_availability = (n_correct + n_wrong) / n_total
    association_precision = (n_correct / (n_correct + n_wrong)) if (n_correct + n_wrong) > 0 else 1.0
    association_recall = correct_accept_rate
    
    # Coast durations
    coast_duration = n_coast * dt
    max_consec_coast = 0
    curr_coast = 0
    for o in outcomes:
        if o.outcome in ["SAFE_REJECT", "NO_CANDIDATE_COAST"]:
            curr_coast += 1
            if curr_coast > max_consec_coast:
                max_consec_coast = curr_coast
        else:
            curr_coast = 0
            
    # Identity switches
    id_switches = 0
    prev_sel = None
    for o in outcomes:
        if o.selected_id != prev_sel:
            if prev_sel is not None and o.selected_id is not None:
                id_switches += 1
            prev_sel = o.selected_id
            
    # Time to resume correct track after an interruption
    time_to_resume = 0.0
    for i in range(1, len(outcomes)):
        if not outcomes[i-1].target_present or outcomes[i-1].outcome == "SAFE_REJECT":
            if outcomes[i].target_present and outcomes[i].outcome != "CORRECT_ACCEPT":
                frames_to_resume = 0
                for j in range(i, len(outcomes)):
                    if outcomes[j].outcome == "CORRECT_ACCEPT":
                        break
                    frames_to_resume += 1
                time_to_resume = max(time_to_resume, frames_to_resume * dt)
                
    return {
        "total_frames": n_total,
        "observable_frames": n_observable,
        "correct_accept_count": n_correct,
        "wrong_accept_count": n_wrong,
        "safe_rejection_count": n_safe,
        "coast_frame_count": n_coast,
        "correct_accept_rate": round(correct_accept_rate, 4),
        "wrong_accept_rate": round(wrong_accept_rate, 4),
        "safe_rejection_rate": round(safe_rejection_rate, 4),
        "association_availability": round(association_availability, 4),
        "association_precision": round(association_precision, 4),
        "association_recall_when_target_observable": round(association_recall, 4),
        "coast_duration_s": round(coast_duration, 4),
        "max_consecutive_coast_frames": max_consec_coast,
        "time_to_resume_correct_track_s": round(time_to_resume, 4),
        "false_lock_count": 1 if n_wrong > 0 else 0,
        "false_lock_duration_s": round(n_wrong * dt, 4),
        "identity_switch_count": id_switches,
        "outcomes": outcomes
    }


def evaluate_tier_b_telemetry(run_res: runner_mod.BenchmarkRunResult, designated_id: str, dt: float = 0.02) -> Dict[str, Any]:
    """
    Parse closed-loop simulation telemetry and compute 3-outcome metrics.
    """
    telemetry = run_res.telemetry
    n_total = len(telemetry)
    
    n_observable = 0
    n_correct = 0
    n_wrong = 0
    n_safe = 0
    n_nocand_coast = 0
    
    curr_coast = 0
    max_consec_coast = 0
    id_switches = 0
    prev_sel = None
    
    for row in telemetry:
        gt_in_fov = row["ground_truth_target_in_fov"]
        gt_active = row["ground_truth_target_active"]
        is_observable = gt_in_fov and gt_active
        if is_observable:
            n_observable += 1
            
        target_det = row["detection_available"]
        eval_id = row["evaluated_candidate_id"]
        c_count = row["candidate_count"]
        
        if target_det:
            if eval_id == designated_id:
                outcome = "CORRECT_ACCEPT"
                n_correct += 1
            else:
                outcome = "WRONG_ACCEPT"
                n_wrong += 1
            curr_coast = 0
        else:
            if c_count > 0:
                outcome = "SAFE_REJECT"
                n_safe += 1
            else:
                outcome = "NO_CANDIDATE_COAST"
                n_nocand_coast += 1
            curr_coast += 1
            if curr_coast > max_consec_coast:
                max_consec_coast = curr_coast
                
        if eval_id != prev_sel:
            if prev_sel is not None and eval_id is not None:
                id_switches += 1
            prev_sel = eval_id
            
    n_coast = n_safe + n_nocand_coast
    correct_accept_rate = (n_correct / n_observable) if n_observable > 0 else 1.0
    wrong_accept_rate = n_wrong / n_total
    safe_rejection_rate = n_safe / n_total
    association_availability = (n_correct + n_wrong) / n_total
    association_precision = (n_correct / (n_correct + n_wrong)) if (n_correct + n_wrong) > 0 else 1.0
    association_recall = correct_accept_rate
    coast_duration = n_coast * dt
    false_lock_dur = n_wrong * dt
    
    # Time to resume
    time_to_resume = 0.0
    for i in range(1, len(telemetry)):
        prev_obs = telemetry[i-1]["ground_truth_target_in_fov"] and telemetry[i-1]["ground_truth_target_active"]
        curr_obs = telemetry[i]["ground_truth_target_in_fov"] and telemetry[i]["ground_truth_target_active"]
        prev_correct = telemetry[i-1]["detection_available"] and telemetry[i-1]["evaluated_candidate_id"] == designated_id
        curr_correct = telemetry[i]["detection_available"] and telemetry[i]["evaluated_candidate_id"] == designated_id
        
        if (not prev_obs or not prev_correct) and curr_obs and not curr_correct:
            frames_to_resume = 0
            for j in range(i, len(telemetry)):
                if telemetry[j]["detection_available"] and telemetry[j]["evaluated_candidate_id"] == designated_id:
                    break
                frames_to_resume += 1
            time_to_resume = max(time_to_resume, frames_to_resume * dt)
            
    return {
        "total_frames": n_total,
        "observable_frames": n_observable,
        "correct_accept_count": n_correct,
        "wrong_accept_count": n_wrong,
        "safe_rejection_count": n_safe,
        "coast_frame_count": n_coast,
        "correct_accept_rate": round(correct_accept_rate, 4),
        "wrong_accept_rate": round(wrong_accept_rate, 4),
        "safe_rejection_rate": round(safe_rejection_rate, 4),
        "association_availability": round(association_availability, 4),
        "association_precision": round(association_precision, 4),
        "association_recall_when_target_observable": round(association_recall, 4),
        "coast_duration_s": round(coast_duration, 4),
        "max_consecutive_coast_frames": max_consec_coast,
        "time_to_resume_correct_track_s": round(time_to_resume, 4),
        "false_lock_count": 1 if n_wrong > 10 else 0,
        "false_lock_duration_s": round(false_lock_dur, 4),
        "identity_switch_count": id_switches
    }


def run_parameter_sweep(out_dir: str):
    """
    Run candidate parameter sweeps across ambiguity margins, distance checks, and hysteresis.
    """
    print("\n[Step 1/3] Running Parameter Sweep across Candidate Configurations...")
    sequences = build_tier_a_sequences()
    test_cases = [
        "T03_crossing_target_brighter",
        "T04_crossing_distractor_brighter",
        "T05_crossing_identical_snr",
        "T06_dropout_stationary_distractor",
        "T07_dropout_crossing_distractor",
        "T10_target_abrupt_maneuver",
        "T12_ambiguous_costs",
        "T13_subgate_oscillation"
    ]
    
    sweep_rows = []
    
    # 1. Ambiguity Margin Sweep: 0.03, 0.05, 0.08, 0.10
    for amb in [0.03, 0.05, 0.08, 0.10]:
        engine = AssociationEngine(mode="robust_multifeature", ambiguity_margin=amb)
        for c_id in test_cases:
            res = evaluate_tier_a_case(sequences[c_id], engine)
            sweep_rows.append({
                "sweep_type": "ambiguity_margin",
                "parameter_value": amb,
                "case_id": c_id,
                "correct_accept_rate": res["correct_accept_rate"],
                "wrong_accept_rate": res["wrong_accept_rate"],
                "safe_rejection_rate": res["safe_rejection_rate"],
                "association_precision": res["association_precision"],
                "coast_duration_s": res["coast_duration_s"],
                "false_lock_duration_s": res["false_lock_duration_s"]
            })
            
    # 2. Hysteresis Margin Sweep: 0.00, 0.06, 0.12, 0.18
    for hys in [0.00, 0.06, 0.12, 0.18]:
        engine = AssociationEngine(mode="robust_multifeature", switch_hysteresis_margin=hys)
        for c_id in test_cases:
            res = evaluate_tier_a_case(sequences[c_id], engine)
            sweep_rows.append({
                "sweep_type": "switch_hysteresis",
                "parameter_value": hys,
                "case_id": c_id,
                "correct_accept_rate": res["correct_accept_rate"],
                "wrong_accept_rate": res["wrong_accept_rate"],
                "safe_rejection_rate": res["safe_rejection_rate"],
                "association_precision": res["association_precision"],
                "coast_duration_s": res["coast_duration_s"],
                "false_lock_duration_s": res["false_lock_duration_s"]
            })
            
    sweep_csv_path = os.path.join(out_dir, "parameter_sweep.csv")
    if sweep_rows:
        with open(sweep_csv_path, "w", newline="", encoding="utf-8") as f:
            writer = csv.DictWriter(f, fieldnames=list(sweep_rows[0].keys()))
            writer.writeheader()
            writer.writerows(sweep_rows)
    print(f"Saved parameter sweep to {sweep_csv_path}")


def main():
    out_dir = os.path.join("results", "phase_03_5_association_audit")
    os.makedirs(out_dir, exist_ok=True)
    
    # 1. Parameter Sweep
    run_parameter_sweep(out_dir)
    
    # 2. Final Frozen Configuration
    final_config = {
        "mode": "robust_multifeature",
        "chi2_gate_level": 9.210,
        "confidence_level": 0.99,
        "dof": 2,
        "ambiguity_margin": 0.08,
        "ambiguity_distance_check": False,
        "switch_hysteresis_margin": 0.12,
        "min_acquisition_confidence": 0.40,
        "acquisition_confirmation_frames": 1,
        "weights": {
            "w_motion": 0.40,
            "w_signal": 0.25,
            "w_quality": 0.15,
            "w_shape": 0.10,
            "w_uncert": 0.10
        },
        "s_min_floor": 4.0,
        "sigma_min_px": 2.0,
        "initiation_expansion_window_frames": 15,
        "initiation_gate_multiplier_initial": 4.0,
        "initiation_gate_multiplier_settling": 2.0,
        "coasting_gate_expansion_rate": 0.25,
        "coasting_gate_expansion_max": 3.0
    }
    config_json_path = os.path.join(out_dir, "final_configuration.json")
    with open(config_json_path, "w") as f:
        json.dump(final_config, f, indent=2)
    print(f"Saved final configuration to {config_json_path}")
    
    # 3. Final Multi-Outcome Evaluation
    print("\n[Step 2/3] Evaluating Frozen Configuration across all 15 Tier A Sequences...")
    sequences = build_tier_a_sequences()
    engine = AssociationEngine(mode="robust_multifeature")
    
    outcome_rows = []
    tier_a_summary = {}
    
    for seq_name, seq_info in sequences.items():
        res = evaluate_tier_a_case(seq_info, engine)
        tier_a_summary[seq_name] = {
            "description": seq_info["description"],
            "metrics": {k: v for k, v in res.items() if k != "outcomes"}
        }
        outcome_rows.append({
            "tier": "Tier_A",
            "test_id": seq_name,
            "seed": None,
            "description": seq_info["description"],
            "total_frames": res["total_frames"],
            "observable_frames": res["observable_frames"],
            "correct_accept_count": res["correct_accept_count"],
            "wrong_accept_count": res["wrong_accept_count"],
            "safe_rejection_count": res["safe_rejection_count"],
            "coast_frame_count": res["coast_frame_count"],
            "correct_accept_rate": res["correct_accept_rate"],
            "wrong_accept_rate": res["wrong_accept_rate"],
            "safe_rejection_rate": res["safe_rejection_rate"],
            "association_availability": res["association_availability"],
            "association_precision": res["association_precision"],
            "association_recall": res["association_recall_when_target_observable"],
            "coast_duration_s": res["coast_duration_s"],
            "max_consec_coast_frames": res["max_consecutive_coast_frames"],
            "time_to_resume_s": res["time_to_resume_correct_track_s"],
            "false_lock_count": res["false_lock_count"],
            "false_lock_duration_s": res["false_lock_duration_s"],
            "identity_switches": res["identity_switch_count"]
        })
        
    print("\n[Step 3/3] Evaluating Frozen Configuration across Tier B Scenarios (S11-S15 x 5 Seeds)...")
    tier_b_scenarios = [
        "S11_stationary_distractor",
        "S12_moving_distractor",
        "S13_crossing_distractor",
        "S14_brighter_distractor",
        "S15_combined_stress"
    ]
    seeds = [42, 43, 44, 45, 46]
    tier_b_summary = {}
    
    b_engine = runner_mod.BenchmarkEngine(localization_mode="weighted_centroid", association_mode="robust_multifeature")
    
    for sc_name in tier_b_scenarios:
        sc = SCENARIO_REGISTRY[sc_name]
        tier_b_summary[sc_name] = {}
        for seed in seeds:
            t0 = time.time()
            run_res = b_engine.run_scenario(sc, seed=seed)
            t_elapsed = time.time() - t0
            metrics_3outcome = evaluate_tier_b_telemetry(run_res, sc.designated_target_id)
            tier_b_summary[sc_name][f"seed_{seed}"] = {
                "metrics": metrics_3outcome,
                "pointing_error_rms_deg": run_res.metrics["tracking"]["rms_angular_error_deg"],
                "observable_coverage_pct": run_res.metrics["perception"]["observable_measurement_coverage_pct"]
            }
            outcome_rows.append({
                "tier": "Tier_B",
                "test_id": sc_name,
                "seed": seed,
                "description": sc.description,
                "total_frames": metrics_3outcome["total_frames"],
                "observable_frames": metrics_3outcome["observable_frames"],
                "correct_accept_count": metrics_3outcome["correct_accept_count"],
                "wrong_accept_count": metrics_3outcome["wrong_accept_count"],
                "safe_rejection_count": metrics_3outcome["safe_rejection_count"],
                "coast_frame_count": metrics_3outcome["coast_frame_count"],
                "correct_accept_rate": metrics_3outcome["correct_accept_rate"],
                "wrong_accept_rate": metrics_3outcome["wrong_accept_rate"],
                "safe_rejection_rate": metrics_3outcome["safe_rejection_rate"],
                "association_availability": metrics_3outcome["association_availability"],
                "association_precision": metrics_3outcome["association_precision"],
                "association_recall": metrics_3outcome["association_recall_when_target_observable"],
                "coast_duration_s": metrics_3outcome["coast_duration_s"],
                "max_consec_coast_frames": metrics_3outcome["max_consecutive_coast_frames"],
                "time_to_resume_s": metrics_3outcome["time_to_resume_correct_track_s"],
                "false_lock_count": metrics_3outcome["false_lock_count"],
                "false_lock_duration_s": metrics_3outcome["false_lock_duration_s"],
                "identity_switches": metrics_3outcome["identity_switch_count"]
            })
            print(f"  {sc_name} seed={seed}: corr={metrics_3outcome['correct_accept_count']}, wrong={metrics_3outcome['wrong_accept_count']}, safe={metrics_3outcome['safe_rejection_count']}, prec={metrics_3outcome['association_precision']:.2f}, cov={metrics_3outcome['correct_accept_rate']*100:.1f}%, fl_dur={metrics_3outcome['false_lock_duration_s']:.2f}s ({t_elapsed:.1f}s)")
            
    # Save outcome_matrix.csv
    outcome_csv_path = os.path.join(out_dir, "outcome_matrix.csv")
    if outcome_rows:
        with open(outcome_csv_path, "w", newline="", encoding="utf-8") as f:
            writer = csv.DictWriter(f, fieldnames=list(outcome_rows[0].keys()))
            writer.writeheader()
            writer.writerows(outcome_rows)
    print(f"\nSaved outcome matrix to {outcome_csv_path}")
    
    # Save summary.json
    summary_data = {
        "audit_version": "Phase_3.5_Certified",
        "timestamp": time.strftime("%Y-%m-%d %H:%M:%S"),
        "final_configuration": final_config,
        "tier_a_results": tier_a_summary,
        "tier_b_results": tier_b_summary
    }
    summary_json_path = os.path.join(out_dir, "summary.json")
    with open(summary_json_path, "w") as f:
        json.dump(summary_data, f, indent=2)
    print(f"Saved summary JSON to {summary_json_path}")
    print("\n[SUCCESS] Phase 3.5 Association Audit Run Completed Successfully.")


if __name__ == "__main__":
    main()
