"""
Automated Plot Generation for FSOC-AI-PAT Benchmark Runs.
Uses headless matplotlib (Agg backend) to generate publication-grade performance charts.
"""
import os
from typing import Dict, Any, List
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np


def generate_benchmark_plots(
    telemetry: List[Dict[str, Any]],
    output_dir: str,
    scenario_name: str,
    seed: int
):
    """
    Generate and save standard performance plots for an experiment run.
    """
    os.makedirs(output_dir, exist_ok=True)
    if not telemetry:
        return

    sim_times = np.array([r["simulation_time"] for r in telemetry])
    
    # -------------------------------------------------------------
    # 1. TRACKING ERROR PLOT
    # -------------------------------------------------------------
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 6), sharex=True)
    fig.suptitle(f"FSOC PAT Tracking Error — {scenario_name} (Seed {seed})", fontsize=12, fontweight='bold')

    ang_errors = [r.get("tracking_error_deg") if r.get("tracking_error_deg") is not None else np.nan for r in telemetry]
    px_errors = [r.get("tracking_error_px") if r.get("tracking_error_px") is not None else np.nan for r in telemetry]

    ax1.plot(sim_times, ang_errors, color='#2563eb', linewidth=1.5, label='Angular Pointing Error')
    ax1.axhline(0.15, color='#10b981', linestyle='--', linewidth=1.2, label='Fine Lock Threshold (0.15°)')
    ax1.set_ylabel("Error (deg)")
    ax1.grid(True, linestyle=':', alpha=0.6)
    ax1.legend(loc='upper right')

    ax2.plot(sim_times, px_errors, color='#dc2626', linewidth=1.5, label='Pixel Error relative to Boresight')
    ax2.set_xlabel("Simulation Time (seconds)")
    ax2.set_ylabel("Pixel Error (px)")
    ax2.grid(True, linestyle=':', alpha=0.6)
    ax2.legend(loc='upper right')

    plt.tight_layout()
    plt.savefig(os.path.join(output_dir, "tracking_error.png"), dpi=150)
    plt.close(fig)

    # -------------------------------------------------------------
    # 2. GIMBAL TRAJECTORY & COMMAND RATES
    # -------------------------------------------------------------
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 6), sharex=True)
    fig.suptitle(f"Gimbal Kinematics & Commands — {scenario_name} (Seed {seed})", fontsize=12, fontweight='bold')

    pan_angles = [r.get("camera_pan", 0.0) for r in telemetry]
    tilt_angles = [r.get("camera_tilt", 0.0) for r in telemetry]
    pan_rates = [r.get("command_pan_rate", 0.0) for r in telemetry]
    tilt_rates = [r.get("command_tilt_rate", 0.0) for r in telemetry]

    ax1.plot(sim_times, pan_angles, color='#0284c7', linewidth=1.5, label='Camera Pan (deg)')
    ax1.plot(sim_times, tilt_angles, color='#7c3aed', linewidth=1.5, label='Camera Tilt (deg)')
    ax1.set_ylabel("Gimbal Angle (deg)")
    ax1.grid(True, linestyle=':', alpha=0.6)
    ax1.legend(loc='upper right')

    ax2.plot(sim_times, pan_rates, color='#0284c7', linestyle='--', linewidth=1.2, label='Pan Rate Cmd (deg/s)')
    ax2.plot(sim_times, tilt_rates, color='#7c3aed', linestyle='--', linewidth=1.2, label='Tilt Rate Cmd (deg/s)')
    ax2.axhline(30.0, color='#ef4444', linestyle=':', linewidth=1.0, label='Saturation Bound (±30°/s)')
    ax2.axhline(-30.0, color='#ef4444', linestyle=':', linewidth=1.0)
    ax2.set_xlabel("Simulation Time (seconds)")
    ax2.set_ylabel("Rate Cmd (deg/s)")
    ax2.grid(True, linestyle=':', alpha=0.6)
    ax2.legend(loc='upper right')

    plt.tight_layout()
    plt.savefig(os.path.join(output_dir, "gimbal_trajectory.png"), dpi=150)
    plt.close(fig)

    # -------------------------------------------------------------
    # 3. SPATIAL 2D FOCAL PLANE TRACK
    # -------------------------------------------------------------
    fig, ax = plt.subplots(figsize=(7, 6))
    fig.suptitle(f"Focal Plane Spatial Trajectory — {scenario_name}", fontsize=12, fontweight='bold')

    gt_x = [r.get("ground_truth_target_x") for r in telemetry if r.get("ground_truth_target_x") is not None]
    gt_y = [r.get("ground_truth_target_y") for r in telemetry if r.get("ground_truth_target_y") is not None]
    est_x = [r.get("estimated_x") for r in telemetry if r.get("estimated_x") is not None]
    est_y = [r.get("estimated_y") for r in telemetry if r.get("estimated_y") is not None]

    if gt_x and gt_y:
        ax.plot(gt_x, gt_y, color='#10b981', linewidth=1.5, label='Ground Truth Target (Evaluation Only)')
    if est_x and est_y:
        ax.plot(est_x, est_y, color='#3b82f6', linestyle='--', linewidth=1.5, label='Kalman Estimate')

    # Boresight center
    ax.scatter([320], [240], color='#ef4444', marker='+', s=120, label='Optical Boresight (320, 240)', zorder=5)

    ax.set_xlim(0, 640)
    ax.set_ylim(480, 0)  # Invert Y for image space
    ax.set_xlabel("Image X (px)")
    ax.set_ylabel("Image Y (px)")
    ax.grid(True, linestyle=':', alpha=0.6)
    ax.legend(loc='upper right')

    plt.tight_layout()
    plt.savefig(os.path.join(output_dir, "spatial_track.png"), dpi=150)
    plt.close(fig)

    # -------------------------------------------------------------
    # 4. LATENCY PROFILE PLOT
    # -------------------------------------------------------------
    fig, ax = plt.subplots(figsize=(10, 4))
    fig.suptitle(f"Execution Latency Profile — {scenario_name}", fontsize=12, fontweight='bold')

    det_lat = [r.get("detector_latency_ms", 0.0) for r in telemetry]
    pipe_lat = [r.get("pipeline_latency_ms", 0.0) for r in telemetry]

    ax.plot(sim_times, pipe_lat, color='#8b5cf6', linewidth=1.2, label='Total Pipeline Latency')
    ax.plot(sim_times, det_lat, color='#06b6d4', linewidth=1.2, label='OpenCV Detector Latency')
    ax.set_xlabel("Simulation Time (seconds)")
    ax.set_ylabel("Latency (ms)")
    ax.grid(True, linestyle=':', alpha=0.6)
    ax.legend(loc='upper right')

    plt.tight_layout()
    plt.savefig(os.path.join(output_dir, "latency_profile.png"), dpi=150)
    plt.close(fig)
