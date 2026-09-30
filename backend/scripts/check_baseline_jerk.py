from app.benchmark.runner import BenchmarkEngine
from app.benchmark.scenarios import SCENARIO_REGISTRY
import numpy as np

eng = BenchmarkEngine(fsm_mode='new_pat_fsm')
scenarios = ['S00_nominal', 'S01_linear', 'S03_high_speed', 'S04_acceleration', 'S05_direction_change', 'S06_vibration', 'S09_temporary_dropout', 'S15_combined_stress']
for sc in scenarios:
    res = eng.run_scenario(SCENARIO_REGISTRY[sc], seed=42, duration=10.0, save_plots=False)
    pans = np.array([r['command_pan_rate'] for r in res.telemetry])
    tilts = np.array([r['command_tilt_rate'] for r in res.telemetry])
    dt = 0.02
    dp = np.diff(pans)/dt
    dtilt = np.diff(tilts)/dt
    acc = np.sqrt(dp**2 + dtilt**2)
    jerk = np.sqrt((np.diff(dp)/dt)**2 + (np.diff(dtilt)/dt)**2)
    err_rms = res.metrics["tracking"]["rms_angular_error_deg"]
    print(f"{sc:25s}: Peak_acc={np.max(acc):8.1f} deg/s^2, Peak_jerk={np.max(jerk):10.1f} deg/s^3, RMS_jerk={np.sqrt(np.mean(jerk**2)):8.1f} deg/s^3, Err_RMS={err_rms:.4f}")
