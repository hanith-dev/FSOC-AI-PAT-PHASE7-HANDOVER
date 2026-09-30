"""
Tier B: Closed-Loop System Benchmark Engine for FSOC-AI-PAT Phase 2.
Executes end-to-end closed-loop benchmark runs across required scenarios
(S00, S01, S04, S06, S07, S08, S09, S14, S15) for:
1. baseline
2. weighted_centroid
3. psf_refined
Using the Phase 1.6 certified metric contract and camera calibration.
"""
import os
import sys
import json
import csv
import time
from typing import Dict, Any, List
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from app.benchmark.runner import BenchmarkEngine
from app.benchmark.scenarios import SCENARIO_REGISTRY


def run_tier_b_closed_loop_ablation(
    output_dir: str = "results/phase_02_localization",
    seeds: List[int] = None
) -> Dict[str, Any]:
    """
    Runs Tier B closed-loop benchmark comparisons across localization modes.
    """
    if seeds is None:
        seeds = [42]

    os.makedirs(output_dir, exist_ok=True)
    plots_dir = os.path.join(output_dir, "plots")
    os.makedirs(plots_dir, exist_ok=True)
    runs_dir = os.path.join(output_dir, "runs")
    os.makedirs(runs_dir, exist_ok=True)

    target_scenarios = [
        "S00_nominal",
        "S01_linear",
        "S04_acceleration",
        "S06_vibration",
        "S07_sensor_noise",
        "S08_fading",
        "S09_temporary_dropout",
        "S14_brighter_distractor",
        "S15_combined_stress"
    ]

    modes = ["baseline", "weighted_centroid", "psf_refined"]

    closed_loop_records: List[Dict[str, Any]] = []

    print("[INFO] Starting Tier B Closed-Loop System Benchmark...")
    print(f"  Scenarios: {target_scenarios}")
    print(f"  Modes: {modes}")
    print(f"  Seeds: {seeds}")

    for mode in modes:
        engine = BenchmarkEngine(dt=0.02, localization_mode=mode)
        mode_runs_dir = os.path.join(runs_dir, mode)
        os.makedirs(mode_runs_dir, exist_ok=True)

        for sc_id in target_scenarios:
            scenario = SCENARIO_REGISTRY[sc_id]
            for seed in seeds:
                # Check if existing run directory already contains summary.json
                existing_runs = [d for d in os.listdir(mode_runs_dir) if d.startswith(f"BENCH_{sc_id}_seed{seed}_")] if os.path.exists(mode_runs_dir) else []
                summary_file = os.path.join(mode_runs_dir, existing_runs[0], "summary.json") if existing_runs else None

                if summary_file and os.path.exists(summary_file):
                    print(f"  --> Loading existing run: {existing_runs[0]}")
                    with open(summary_file, "r") as f:
                        m = json.load(f)
                    exp_id = existing_runs[0]
                    dur = 0.0
                else:
                    print(f"  --> Running mode={mode}, scenario={sc_id}, seed={seed}...")
                    t0 = time.time()
                    res = engine.run_scenario(
                        scenario=scenario,
                        seed=seed,
                        output_base_dir=mode_runs_dir
                    )
                    dur = time.time() - t0
                    m = res.metrics
                    exp_id = res.experiment_id

                sp = m.get("summary_pairing", {})
                tr = m.get("tracking", {})
                pop = tr.get("pointing_error_populations", {}).get("all_active_frames", {})
                asc = m.get("association", {})
                lk = m.get("lock", {})
                pf = m.get("performance", {})

                rec = {
                    "mode": mode,
                    "scenario": sc_id,
                    "seed": seed,
                    "rms_angular_error_deg": sp.get("rms_angular_error_deg"),
                    "p50_angular_error_deg": pop.get("p50_deg"),
                    "p95_angular_error_deg": pop.get("p95_deg"),
                    "max_angular_error_deg": pop.get("max_deg"),
                    "rms_pixel_error_px": sp.get("rms_pixel_error_px"),
                    "observable_coverage_pct": sp.get("observable_measurement_coverage_pct"),
                    "mission_coverage_pct": sp.get("mission_measurement_coverage_pct"),
                    "accepted_coverage_pct": sp.get("accepted_measurement_coverage_pct"),
                    "lock_time_s": lk.get("acquisition_time_s"),
                    "false_lock_events": asc.get("false_lock_event_count"),
                    "false_lock_duration_s": asc.get("false_lock_duration_s"),
                    "identity_switches": asc.get("identity_switch_count"),
                    "pipeline_latency_ms": pf.get("total_pipeline_mean_latency_ms"),
                    "throughput_fps": pf.get("benchmark_throughput_fps"),
                    "experiment_id": exp_id,
                    "wall_time_s": round(dur, 2)
                }
                closed_loop_records.append(rec)

    # -----------------------------------------------------------------
    # Save closed_loop_ablation.csv
    # -----------------------------------------------------------------
    csv_path = os.path.join(output_dir, "closed_loop_ablation.csv")
    if closed_loop_records:
        fieldnames = list(closed_loop_records[0].keys())
        with open(csv_path, "w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(closed_loop_records)
        print(f"[INFO] Saved closed loop ablation CSV: {csv_path}")

    # -----------------------------------------------------------------
    # Update method_comparison.json and summary.json
    # -----------------------------------------------------------------
    comp_json_path = os.path.join(output_dir, "method_comparison.json")
    if os.path.exists(comp_json_path):
        with open(comp_json_path, "r") as f:
            comp_data = json.load(f)
    else:
        comp_data = {}

    comp_data["closed_loop_by_scenario"] = {}
    for sc_id in target_scenarios:
        comp_data["closed_loop_by_scenario"][sc_id] = {}
        for mode in modes:
            sub = [r for r in closed_loop_records if r["scenario"] == sc_id and r["mode"] == mode and r["seed"] == 42]
            if sub:
                r0 = sub[0]
                comp_data["closed_loop_by_scenario"][sc_id][mode] = {
                    "rms_angular_error_deg": r0["rms_angular_error_deg"],
                    "rms_pixel_error_px": r0["rms_pixel_error_px"],
                    "accepted_coverage_pct": r0["accepted_coverage_pct"],
                    "false_lock_events": r0["false_lock_events"],
                    "false_lock_duration_s": r0["false_lock_duration_s"],
                    "identity_switches": r0["identity_switches"],
                    "pipeline_latency_ms": r0["pipeline_latency_ms"]
                }

    with open(comp_json_path, "w") as f:
        json.dump(comp_data, f, indent=2)

    # -----------------------------------------------------------------
    # Generate closed_loop_pointing_comparison.png
    # -----------------------------------------------------------------
    _generate_closed_loop_plots(closed_loop_records, target_scenarios, plots_dir)

    print("[INFO] Tier B Closed-Loop Benchmark Complete!")
    return comp_data


def _generate_closed_loop_plots(
    records: List[Dict[str, Any]],
    scenarios: List[str],
    plots_dir: str
):
    """Generates closed-loop pointing error comparison bar chart."""
    colors = {
        "baseline": "#EF4444",
        "weighted_centroid": "#10B981",
        "psf_refined": "#3B82F6"
    }

    plt.figure(figsize=(12, 6))
    x_indices = np.arange(len(scenarios))
    width = 0.26

    for i, (mode, color) in enumerate(colors.items()):
        vals = []
        for sc in scenarios:
            sub = [r for r in records if r["scenario"] == sc and r["mode"] == mode and r["seed"] == 42]
            val = sub[0].get("rms_angular_error_deg") if sub else 0.0
            vals.append(float(val) if val is not None else 0.0)
        plt.bar(x_indices + (i - 1) * width, vals, width=width, label=f"Mode: {mode}", color=color, alpha=0.85)

    short_names = [s.replace("S", "S").replace("_", "\n") for s in scenarios]
    plt.xticks(x_indices, short_names, fontsize=9)
    plt.ylabel("RMS Angular Pointing Error (deg)", fontsize=11)
    plt.title("Closed-Loop PAT Tracking Performance: Localization Mode Comparison (Seed 42)", fontsize=12)
    plt.grid(True, alpha=0.3, axis="y")
    plt.legend(fontsize=10)
    plt.tight_layout()
    plt.savefig(os.path.join(plots_dir, "closed_loop_pointing_comparison.png"), dpi=200)
    plt.close()


if __name__ == "__main__":
    run_tier_b_closed_loop_ablation()
