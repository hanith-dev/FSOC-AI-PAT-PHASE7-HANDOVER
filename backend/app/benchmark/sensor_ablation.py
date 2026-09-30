"""
Tier A: Sensor-Level Localization Ablation Engine for FSOC-AI-PAT.
Evaluates BaselineLocalizer, WeightedCentroidLocalizer, and PSFRefinedLocalizer
on identical synthetic optical frames across 12 rigorous sensor conditions.
Calculates RMSE, MAE, P50, P95, Max error, bias, jitter, coverage %, and latency.
Generates evaluation CSV, JSON summary, and comparative analytical plots.
"""
import os
import sys
import time
import json
import csv
from typing import Dict, Any, List, Tuple
import numpy as np
import cv2
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from app.detection.localization import (
    LocalizationResult,
    BaselineLocalizer,
    WeightedCentroidLocalizer,
    PSFRefinedLocalizer,
    LOCALIZER_REGISTRY
)


def render_synthetic_beacon_frame(
    width: int,
    height: int,
    true_x: float,
    true_y: float,
    condition: str,
    seed: int = 42
) -> Tuple[np.ndarray, List[int]]:
    """
    Renders a 640x480 frame containing a synthetic optical beacon at exact float (true_x, true_y)
    with physical continuous Gaussian PSF sampling, plus disturbance conditions.
    Returns:
        (frame_bgr, candidate_bbox)
    """
    rng = np.random.RandomState(seed)

    # Base ambient background (space / dark sky)
    base_bg = 18.0
    bg_img = np.full((height, width), base_bg, dtype=np.float64)

    # Optical spot parameters
    amplitude = 220.0
    sigma_x = 2.5
    sigma_y = 2.5

    # Modify parameters based on condition
    if condition == "clean_psf":
        pass
    elif condition == "low_gaussian_noise":
        pass
    elif condition == "high_gaussian_noise":
        pass
    elif condition == "gaussian_blur":
        sigma_x = 5.5
        sigma_y = 5.5
    elif condition == "motion_blur":
        sigma_x = 6.0
        sigma_y = 2.0
    elif condition == "brightness_fading":
        amplitude = 45.0
    elif condition == "background_gradient":
        # Linear background ramp across width
        ramp = np.linspace(5.0, 45.0, width)
        bg_img = np.tile(ramp, (height, 1))
    elif condition == "low_snr":
        amplitude = 40.0
    elif condition == "near_threshold_beacon":
        amplitude = 28.0
    elif condition == "sensor_edge_beacon":
        # Handled by true_x, true_y near edge
        pass
    elif condition == "saturated_core":
        amplitude = 600.0  # Will saturate at 255.0
        sigma_x = 3.5
        sigma_y = 3.5
    elif condition == "combined_stress":
        amplitude = 50.0
        sigma_x = 4.5
        sigma_y = 4.5
        ramp = np.linspace(10.0, 35.0, width)
        bg_img = np.tile(ramp, (height, 1))

    # Evaluate continuous 2D Gaussian PSF on sensor grid
    # To be fast and accurate, render within local bounding patch
    patch_radius = int(np.ceil(max(sigma_x, sigma_y) * 4.5 + 4))
    x0 = max(0, int(np.floor(true_x - patch_radius)))
    x1 = min(width, int(np.ceil(true_x + patch_radius + 1)))
    y0 = max(0, int(np.floor(true_y - patch_radius)))
    y1 = min(height, int(np.ceil(true_y + patch_radius + 1)))

    if x1 > x0 and y1 > y0:
        ys, xs = np.indices((y1 - y0, x1 - x0), dtype=np.float64)
        xs = xs + x0
        ys = ys + y0
        u = (xs - true_x) / sigma_x
        v = (ys - true_y) / sigma_y
        spot = amplitude * np.exp(-0.5 * (u * u + v * v))
        bg_img[y0:y1, x0:x1] += spot

    # Add noise disturbances
    if condition in ("low_gaussian_noise", "combined_stress"):
        noise = rng.normal(0.0, 4.0, (height, width))
        bg_img += noise
    elif condition in ("high_gaussian_noise", "low_snr"):
        noise = rng.normal(0.0, 16.0, (height, width))
        bg_img += noise

    # Clip to uint8 dynamic range [0, 255]
    mono = np.clip(bg_img, 0.0, 255.0).astype(np.uint8)

    # If motion blur requested, apply directional filter
    if condition == "motion_blur":
        kernel = np.zeros((9, 9), dtype=np.float32)
        kernel[4, :] = 1.0 / 9.0
        mono = cv2.filter2D(mono, -1, kernel)

    # Convert to 3-channel optical green BGR
    # Beacon is #10B981 -> BGR (65, 225, 80)
    frame_bgr = np.zeros((height, width, 3), dtype=np.uint8)
    frame_bgr[:, :, 0] = (mono * (65.0 / 255.0)).astype(np.uint8)
    frame_bgr[:, :, 1] = (mono * (225.0 / 255.0)).astype(np.uint8)
    frame_bgr[:, :, 2] = (mono * (80.0 / 255.0)).astype(np.uint8)

    # Calculate realistic candidate detector bounding box from thresholded contour
    # HSV thresholding matching OpenCVDetector
    hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
    lower = np.array([35, 50, 40])
    upper = np.array([95, 255, 255])
    mask = cv2.inRange(hsv, lower, upper)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if contours:
        # Find contour enclosing the spot
        best_c = max(contours, key=cv2.contourArea)
        bx, by, bw, bh = cv2.boundingRect(best_c)
        bbox = [bx, by, bx + bw, by + bh]
    else:
        # Fallback candidate region if contrast is near-threshold
        est_r = int(max(4, patch_radius // 2))
        bx = max(0, int(round(true_x - est_r)))
        by = max(0, int(round(true_y - est_r)))
        bbox = [bx, by, min(width, bx + 2 * est_r), min(height, by + 2 * est_r)]

    return frame_bgr, bbox


def run_tier_a_sensor_ablation(
    output_dir: str = "results/phase_02_localization"
) -> Dict[str, Any]:
    """
    Executes Tier A sensor-level ablation across 12 conditions and 3 localization algorithms.
    """
    os.makedirs(output_dir, exist_ok=True)
    plots_dir = os.path.join(output_dir, "plots")
    os.makedirs(plots_dir, exist_ok=True)

    conditions = [
        "clean_psf",
        "low_gaussian_noise",
        "high_gaussian_noise",
        "gaussian_blur",
        "motion_blur",
        "brightness_fading",
        "background_gradient",
        "low_snr",
        "near_threshold_beacon",
        "sensor_edge_beacon",
        "saturated_core",
        "combined_stress"
    ]

    # Required controlled fractional positions
    fixed_positions = [
        (100.25, 150.75),
        (220.10, 100.90),
        (320.50, 240.50),
        (500.33, 320.67),
        (15.20, 20.40),    # near corner/edge
        (625.80, 465.10),  # near opposite edge
        (280.45, 180.55),
        (380.12, 310.88),
        (180.70, 390.25),
        (450.60, 120.35)
    ]

    localizers = {
        "baseline": BaselineLocalizer(),
        "weighted_centroid": WeightedCentroidLocalizer(roi_margin=5),
        "psf_refined": PSFRefinedLocalizer(roi_margin=5)
    }

    raw_records: List[Dict[str, Any]] = []

    width, height = 640, 480
    base_seed = 42

    print("[INFO] Starting Tier A Sensor-Level Localization Ablation...")
    print(f"  Conditions: {len(conditions)}")
    print(f"  Positions per condition: {len(fixed_positions)}")
    print(f"  Total test frames: {len(conditions) * len(fixed_positions)}")
    print(f"  Localizers: {list(localizers.keys())}")

    for cond_idx, condition in enumerate(conditions):
        for pos_idx, (base_x, base_y) in enumerate(fixed_positions):
            sample_seed = base_seed + cond_idx * 100 + pos_idx

            if condition == "sensor_edge_beacon":
                true_x = 6.25 if pos_idx % 2 == 0 else 633.75
                true_y = 6.80 if pos_idx % 2 == 0 else 473.20
            else:
                true_x = base_x
                true_y = base_y

            # Render single deterministic frame
            frame, bbox = render_synthetic_beacon_frame(
                width=width,
                height=height,
                true_x=true_x,
                true_y=true_y,
                condition=condition,
                seed=sample_seed
            )

            # Evaluate EVERY localizer on the EXACT SAME image and bounding box
            for method_name, loc_instance in localizers.items():
                t0 = time.perf_counter()
                res = loc_instance.localize(frame, bbox)
                t_latency = (time.perf_counter() - t0) * 1000.0

                if res.success and not np.isnan(res.x) and not np.isnan(res.y):
                    err_x = float(res.x - true_x)
                    err_y = float(res.y - true_y)
                    err_euc = float(np.hypot(err_x, err_y))
                    success = True
                else:
                    err_x = None
                    err_y = None
                    err_euc = None
                    success = False

                rec = {
                    "condition": condition,
                    "sample_idx": pos_idx,
                    "seed": sample_seed,
                    "true_x": round(true_x, 4),
                    "true_y": round(true_y, 4),
                    "bbox": bbox,
                    "method": method_name,
                    "success": success,
                    "meas_x": round(res.x, 4) if not np.isnan(res.x) else None,
                    "meas_y": round(res.y, 4) if not np.isnan(res.y) else None,
                    "error_x_px": round(err_x, 4) if err_x is not None else None,
                    "error_y_px": round(err_y, 4) if err_y is not None else None,
                    "error_px": round(err_euc, 4) if err_euc is not None else None,
                    "confidence": round(res.confidence, 4),
                    "snr_proxy": round(res.snr_proxy, 2),
                    "uncertainty_x": round(res.localization_uncertainty_x_px, 4),
                    "uncertainty_y": round(res.localization_uncertainty_y_px, 4),
                    "latency_ms": round(t_latency, 4),
                    "quality_flags": ";".join(res.quality_flags)
                }
                raw_records.append(rec)

    # -----------------------------------------------------------------
    # Save sensor_ablation.csv
    # -----------------------------------------------------------------
    csv_path = os.path.join(output_dir, "sensor_ablation.csv")
    if raw_records:
        fieldnames = list(raw_records[0].keys())
        with open(csv_path, "w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(raw_records)
        print(f"[INFO] Saved sensor ablation records: {csv_path}")

    # -----------------------------------------------------------------
    # Compute Aggregate Metrics per Method and Condition
    # -----------------------------------------------------------------
    summary_by_method: Dict[str, Any] = {}
    condition_comparison: Dict[str, Any] = {}

    for method_name in localizers.keys():
        m_recs = [r for r in raw_records if r["method"] == method_name]
        valid_recs = [r for r in m_recs if r["success"] and r["error_px"] is not None]

        errors = np.array([r["error_px"] for r in valid_recs], dtype=np.float64)
        errors_x = np.array([r["error_x_px"] for r in valid_recs], dtype=np.float64)
        errors_y = np.array([r["error_y_px"] for r in valid_recs], dtype=np.float64)
        latencies = np.array([r["latency_ms"] for r in m_recs], dtype=np.float64)
        uncs = np.array([0.5 * (r["uncertainty_x"] + r["uncertainty_y"]) for r in valid_recs], dtype=np.float64)

        n_total = len(m_recs)
        n_valid = len(valid_recs)
        coverage_pct = (n_valid / n_total * 100.0) if n_total > 0 else 0.0

        rmse = float(np.sqrt(np.mean(errors ** 2))) if n_valid > 0 else 0.0
        mae = float(np.mean(errors)) if n_valid > 0 else 0.0
        p50 = float(np.median(errors)) if n_valid > 0 else 0.0
        p95 = float(np.percentile(errors, 95)) if n_valid > 0 else 0.0
        max_err = float(np.max(errors)) if n_valid > 0 else 0.0
        bias_x = float(np.mean(errors_x)) if n_valid > 0 else 0.0
        bias_y = float(np.mean(errors_y)) if n_valid > 0 else 0.0
        jitter_std = float(np.std(errors)) if n_valid > 0 else 0.0

        mean_lat = float(np.mean(latencies)) if len(latencies) > 0 else 0.0
        p95_lat = float(np.percentile(latencies, 95)) if len(latencies) > 0 else 0.0

        # Correlation between uncertainty proxy and actual error
        if n_valid > 5 and np.std(uncs) > 1e-4 and np.std(errors) > 1e-4:
            corr_matrix = np.corrcoef(uncs, errors)
            unc_corr = float(corr_matrix[0, 1])
        else:
            unc_corr = 0.0

        summary_by_method[method_name] = {
            "total_samples": n_total,
            "valid_samples": n_valid,
            "coverage_pct": round(coverage_pct, 2),
            "failure_count": n_total - n_valid,
            "rmse_px": round(rmse, 4),
            "mae_px": round(mae, 4),
            "p50_px": round(p50, 4),
            "p95_px": round(p95, 4),
            "max_px": round(max_err, 4),
            "bias_x_px": round(bias_x, 4),
            "bias_y_px": round(bias_y, 4),
            "jitter_std_px": round(jitter_std, 4),
            "mean_latency_ms": round(mean_lat, 4),
            "p95_latency_ms": round(p95_lat, 4),
            "uncertainty_error_correlation": round(unc_corr, 4)
        }

    # Per condition breakdown
    for cond in conditions:
        condition_comparison[cond] = {}
        for method_name in localizers.keys():
            c_recs = [r for r in raw_records if r["condition"] == cond and r["method"] == method_name and r["success"]]
            if c_recs:
                errs = [r["error_px"] for r in c_recs]
                condition_comparison[cond][method_name] = {
                    "rmse_px": round(float(np.sqrt(np.mean(np.array(errs)**2))), 4),
                    "p50_px": round(float(np.median(errs)), 4),
                    "p95_px": round(float(np.percentile(errs, 95)), 4),
                    "max_px": round(float(np.max(errs)), 4)
                }
            else:
                condition_comparison[cond][method_name] = {"rmse_px": None, "p50_px": None}

    # -----------------------------------------------------------------
    # Save method_comparison.json and summary.json
    # -----------------------------------------------------------------
    with open(os.path.join(output_dir, "method_comparison.json"), "w") as f:
        json.dump({
            "overall_metrics": summary_by_method,
            "by_condition": condition_comparison
        }, f, indent=2)

    with open(os.path.join(output_dir, "summary.json"), "w") as f:
        json.dump(summary_by_method, f, indent=2)

    # -----------------------------------------------------------------
    # Generate Analytical Plots
    # -----------------------------------------------------------------
    _generate_sensor_ablation_plots(raw_records, summary_by_method, plots_dir)

    print("[INFO] Tier A Sensor-Level Localization Ablation Complete!")
    return {
        "summary": summary_by_method,
        "by_condition": condition_comparison
    }


def _generate_sensor_ablation_plots(
    records: List[Dict[str, Any]],
    summary: Dict[str, Any],
    plots_dir: str
):
    """Generates all 5 required analytical charts for Tier A."""
    colors = {
        "baseline": "#EF4444",         # Red
        "weighted_centroid": "#10B981", # Emerald Green
        "psf_refined": "#3B82F6"        # Blue
    }

    # 1. localization_error_cdf.png
    plt.figure(figsize=(8, 5))
    for method, color in colors.items():
        errs = [r["error_px"] for r in records if r["method"] == method and r["success"] and r["error_px"] is not None]
        if errs:
            sorted_errs = np.sort(errs)
            cdf = np.linspace(0.0, 1.0, len(sorted_errs))
            plt.plot(sorted_errs, cdf, label=f"{method} (P50={summary[method]['p50_px']}px, RMSE={summary[method]['rmse_px']}px)", color=color, lw=2)
    plt.axvline(0.1, color="black", linestyle="--", alpha=0.6, label="Internal Synthetic Target (0.10 px)")
    plt.title("Sensor-Level Localization Cumulative Error Distribution (CDF)")
    plt.xlabel("Localization Error (pixels)")
    plt.ylabel("Cumulative Probability")
    plt.xlim(0.0, 2.5)
    plt.ylim(0.0, 1.02)
    plt.grid(True, alpha=0.3)
    plt.legend(loc="lower right")
    plt.tight_layout()
    plt.savefig(os.path.join(plots_dir, "localization_error_cdf.png"), dpi=200)
    plt.close()

    # 2. error_vs_snr.png
    plt.figure(figsize=(8, 5))
    for method, color in colors.items():
        sub = [r for r in records if r["method"] == method and r["success"] and r["snr_proxy"] is not None and r["error_px"] is not None]
        if sub:
            snrs = [r["snr_proxy"] for r in sub]
            errs = [r["error_px"] for r in sub]
            plt.scatter(snrs, errs, alpha=0.6, color=color, label=method, s=35)
    plt.title("Localization Error vs Optical Signal-to-Noise Ratio (SNR)")
    plt.xlabel("Estimated SNR Proxy")
    plt.ylabel("Localization Error (pixels)")
    plt.ylim(0.0, 3.0)
    plt.grid(True, alpha=0.3)
    plt.legend()
    plt.tight_layout()
    plt.savefig(os.path.join(plots_dir, "error_vs_snr.png"), dpi=200)
    plt.close()

    # 3. error_vs_blur.png
    blur_conditions = ["clean_psf", "gaussian_blur", "motion_blur", "combined_stress"]
    plt.figure(figsize=(8, 5))
    x_indices = np.arange(len(blur_conditions))
    width = 0.25
    for i, (method, color) in enumerate(colors.items()):
        rmses = []
        for cond in blur_conditions:
            c_errs = [r["error_px"] for r in records if r["method"] == method and r["condition"] == cond and r["success"]]
            rmse = float(np.sqrt(np.mean(np.array(c_errs)**2))) if c_errs else 0.0
            rmses.append(rmse)
        plt.bar(x_indices + (i - 1) * width, rmses, width=width, label=method, color=color, alpha=0.85)
    plt.xticks(x_indices, [c.replace("_", " ").title() for c in blur_conditions])
    plt.title("Localization Robustness Across Optical Blur Conditions")
    plt.ylabel("Localization RMSE (pixels)")
    plt.grid(True, alpha=0.3, axis="y")
    plt.legend()
    plt.tight_layout()
    plt.savefig(os.path.join(plots_dir, "error_vs_blur.png"), dpi=200)
    plt.close()

    # 4. uncertainty_vs_error.png
    plt.figure(figsize=(8, 5))
    wc_recs = [r for r in records if r["method"] == "weighted_centroid" and r["success"] and r["error_px"] is not None]
    if wc_recs:
        uncs = [0.5 * (r["uncertainty_x"] + r["uncertainty_y"]) for r in wc_recs]
        errs = [r["error_px"] for r in wc_recs]
        plt.scatter(uncs, errs, color="#10B981", alpha=0.65, label="Weighted Centroid Measurements")
        # 1:1 diagonal guide
        max_bound = max(np.max(uncs), np.max(errs), 1.0)
        plt.plot([0, max_bound], [0, max_bound], "k--", alpha=0.5, label="1:1 Bound")
    plt.title("Physical Localization Uncertainty Proxy vs Actual Error")
    plt.xlabel("Reported Uncertainty 1-sigma (pixels)")
    plt.ylabel("Actual Measured Error (pixels)")
    plt.xlim(0.0, 3.0)
    plt.ylim(0.0, 3.0)
    plt.grid(True, alpha=0.3)
    plt.legend()
    plt.tight_layout()
    plt.savefig(os.path.join(plots_dir, "uncertainty_vs_error.png"), dpi=200)
    plt.close()

    # 5. latency_comparison.png
    plt.figure(figsize=(7, 5))
    methods = list(colors.keys())
    mean_lats = [summary[m]["mean_latency_ms"] for m in methods]
    p95_lats = [summary[m]["p95_latency_ms"] for m in methods]
    x = np.arange(len(methods))
    plt.bar(x - 0.15, mean_lats, width=0.3, label="Mean Latency (ms)", color="#3B82F6", alpha=0.85)
    plt.bar(x + 0.15, p95_lats, width=0.3, label="P95 Latency (ms)", color="#F59E0B", alpha=0.85)
    plt.xticks(x, [m.replace("_", " ").title() for m in methods])
    plt.ylabel("Processing Time per Frame (ms)")
    plt.title("Localization Processing Latency Comparison")
    plt.grid(True, alpha=0.3, axis="y")
    plt.legend()
    plt.tight_layout()
    plt.savefig(os.path.join(plots_dir, "latency_comparison.png"), dpi=200)
    plt.close()


if __name__ == "__main__":
    run_tier_a_sensor_ablation()
