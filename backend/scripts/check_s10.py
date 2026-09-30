from app.benchmark.runner import BenchmarkEngine
from app.benchmark.scenarios import SCENARIO_REGISTRY

eng = BenchmarkEngine(fsm_mode='new_pat_fsm')
res = eng.run_scenario(SCENARIO_REGISTRY['S10_fov_exit_reentry'], seed=42, duration=10.0, save_plots=False)
for r in res.telemetry:
    t = r['simulation_time']
    s = r['tracking_state']
    gt_in = r.get('is_in_fov')
    meas_acc = r.get('measurement_accepted')
    if 6.8 <= t <= 8.8:
        print(f"t={t:.2f} state={s} gt_in_fov={gt_in} accepted={meas_acc}")
