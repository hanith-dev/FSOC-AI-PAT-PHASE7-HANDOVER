"""
Command-Line Interface for the FSOC-AI-PAT Scientific Benchmark Engine.
"""
import os
import sys
import json
import argparse
from typing import List

from app.benchmark.runner import BenchmarkEngine
from app.benchmark.scenarios import SCENARIO_REGISTRY


def parse_args():
    parser = argparse.ArgumentParser(
        description="FSOC-AI-PAT Scientific Headless Benchmark Runner (Mode A)"
    )
    parser.add_argument(
        "--scenario",
        type=str,
        default=None,
        help="Scenario ID to run (e.g. S00_nominal, S12_moving_distractor), or omit when using --all"
    )
    parser.add_argument(
        "--all",
        action="store_true",
        help="Run all 16 benchmark scenarios in sequence"
    )
    parser.add_argument(
        "--seed",
        type=int,
        default=42,
        help="Random seed for deterministic disturbance & trajectory generation (default: 42)"
    )
    parser.add_argument(
        "--seeds",
        type=str,
        default=None,
        help="Comma-separated list of seeds (e.g. 42,43,44) or integer count (e.g. 5)"
    )
    parser.add_argument(
        "--dt",
        type=float,
        default=0.02,
        help="Fixed integration timestep in seconds (default: 0.02s -> 50 Hz)"
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=None,
        help="Override scenario duration in seconds"
    )
    parser.add_argument(
        "--output-dir",
        type=str,
        default="results/baseline_truth_isolated",
        help="Target results directory (default: results/baseline_truth_isolated)"
    )
    parser.add_argument(
        "--localization-mode",
        type=str,
        choices=["baseline", "weighted_centroid", "psf_refined"],
        default="weighted_centroid",
        help="Localization algorithm mode: baseline, weighted_centroid, or psf_refined (default: weighted_centroid)"
    )
    parser.add_argument(
        "--association-mode",
        type=str,
        choices=["legacy_simple", "euclidean_prediction", "mahalanobis_only", "robust_multifeature"],
        default="robust_multifeature",
        help="Association algorithm mode: legacy_simple, euclidean_prediction, mahalanobis_only, or robust_multifeature (default: robust_multifeature)"
    )
    return parser.parse_args()


def get_seed_list(seed_arg: int, seeds_arg: str = None) -> List[int]:
    if not seeds_arg:
        return [seed_arg]
    if "," in seeds_arg:
        return [int(s.strip()) for s in seeds_arg.split(",") if s.strip()]
    try:
        count = int(seeds_arg)
        return [seed_arg + i for i in range(count)]
    except ValueError:
        return [seed_arg]


def main():
    args = parse_args()

    engine = BenchmarkEngine(
        dt=args.dt,
        localization_mode=args.localization_mode,
        association_mode=args.association_mode
    )
    seeds = get_seed_list(args.seed, args.seeds)

    if args.all:
        scenarios_to_run = list(SCENARIO_REGISTRY.keys())
    elif args.scenario:
        if args.scenario not in SCENARIO_REGISTRY:
            print(f"[ERROR] Unknown scenario: '{args.scenario}'. Available scenarios:")
            for sc_id in SCENARIO_REGISTRY:
                print(f"  - {sc_id}")
            sys.exit(1)
        scenarios_to_run = [args.scenario]
    else:
        print("[ERROR] Please specify either --scenario <id> or --all. Use --help for usage.")
        sys.exit(1)

    print("=" * 70)
    print("FSOC-AI-PAT DETERMINISTIC SCIENTIFIC BENCHMARK ENGINE (MODE A)")
    print("=" * 70)
    print(f"Fixed Timestep (dt): {args.dt}s ({1.0/args.dt:.1f} Hz)")
    print(f"Scenarios to Run:   {len(scenarios_to_run)} ({', '.join(scenarios_to_run)})")
    print(f"Seeds:              {seeds}")
    print(f"Destination:        {args.output_dir}")
    print("=" * 70)

    overall_results = []

    for sc_id in scenarios_to_run:
        scenario = SCENARIO_REGISTRY[sc_id]
        for s in seeds:
            print(f"\n>> Running [{scenario.id}] '{scenario.name}' (Seed: {s})...")
            res = engine.run_scenario(
                scenario=scenario,
                seed=s,
                duration=args.duration,
                output_base_dir=args.output_dir
            )

            p = res.metrics["summary_pairing"]
            c = res.metrics["control"]
            a = res.metrics["association"]
            perf = res.metrics["performance"]

            print(f"   [DONE] Artifacts: {res.output_dir}")
            print(f"   RMS Pointing Error:     {p['rms_angular_error_deg']} deg")
            print(f"   RMS Pixel Error:        {p['rms_pixel_error_px']} px")
            print(f"   Accepted Coverage:      {p['accepted_measurement_coverage_pct']} % (ERROR MUST BE PAIRED WITH COVERAGE)")
            print(f"   False-Lock Events:      {a['false_lock_event_count']} (Duration: {a['false_lock_duration_s']}s)")
            print(f"   Actuator Jerk:          {c['rms_angular_jerk_deg_s3']} deg/s^3 (Saturation: {c['actuator_saturation_pct']}%)")
            print(f"   Pipeline Mean Latency:  {perf['total_pipeline_mean_latency_ms']} ms ({perf['benchmark_throughput_fps']} FPS)")

            overall_results.append({
                "scenario": scenario.id,
                "seed": s,
                "rms_angular_error_deg": p["rms_angular_error_deg"],
                "rms_pixel_error_px": p["rms_pixel_error_px"],
                "accepted_coverage_pct": p["accepted_measurement_coverage_pct"],
                "false_lock_events": a["false_lock_event_count"],
                "false_lock_duration_s": a["false_lock_duration_s"],
                "identity_switches": a["identity_switch_count"],
                "actuator_saturation_pct": c["actuator_saturation_pct"],
                "pipeline_latency_ms": perf["total_pipeline_mean_latency_ms"],
                "throughput_fps": perf["benchmark_throughput_fps"],
                "experiment_id": res.experiment_id
            })

    # Summary table across executed scenarios
    print("\n" + "=" * 105)
    print(f"{'SCENARIO':<24} | {'SEED':<4} | {'RMS ANG (deg)':<13} | {'COVERAGE (%)':<12} | {'FALSE LOCKS':<11} | {'JERK (deg/s3)':<13} | {'FPS':<6}")
    print("-" * 105)
    for r in overall_results:
        ang_str = f"{r['rms_angular_error_deg']:.4f}" if r['rms_angular_error_deg'] is not None else "N/A"
        print(f"{r['scenario']:<24} | {r['seed']:<4} | {ang_str:<13} | {r['accepted_coverage_pct']:<12.1f} | {r['false_lock_events']:<11} | {r.get('actuator_saturation_pct', 0.0):<13.1f} | {r['throughput_fps']:<6.1f}")
    print("=" * 105)

    # Save aggregated summary
    summary_path = os.path.join(args.output_dir, "benchmark_campaign_summary.json")
    with open(summary_path, "w") as f:
        json.dump(overall_results, f, indent=2)
    print(f"\n[OK] Campaign summary saved to: {summary_path}\n")


if __name__ == "__main__":
    main()
