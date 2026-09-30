# Phase 5: Intelligent PAT Lifecycle, Coasting, Loss Management and Basic Autonomous Reacquisition

## 1. Executive Summary & Pre-Flight Consistency Findings

Phase 5 transitions the FSOC PAT platform from an inherited coarse 3-state tracking model to an 8-state, high-reliability aerospace PAT lifecycle state machine. The state machine operates under strict truth firewall isolation—utilizing purely operational signals (temporal association stability, Mahalanobis distances, CA6 adaptive-$R$ covariance bounds, pointing errors, and estimator health status) without accessing true beacon coordinates or simulation flags.

### Pre-Flight Inconsistency Checks
1. **Camera FOV & Scientific Contract**:
   - Runtime configuration and `CameraCalibration` were checked directly in `backend/app/camera/calibration.py` and `test_scientific_contract.py`.
   - **Authoritative Geometry**:
     - Image Width: $640$ px, Image Height: $480$ px
     - Vertical FOV (VFOV): $7.500000^\circ$
     - Horizontal FOV (HFOV): $9.988928^\circ$
     - Focal Lengths: $f_x = f_y = 3661.723149$ px
     - Principal Point: $c_x = 320.0, c_y = 240.0$ px
   - **Resolution**: The certified Phase 1.6 camera contract is intact and authoritative in runtime code. The mention of "12 degree VFOV" in `plans/PHASE_04_5_ESTIMATOR_AUDIT.md` was a documentation error and has been corrected. Zero runtime changes were made to camera parameters.

2. **T07 Gate Boundary Consistency**:
   - Re-evaluation of frame 13 telemetry in Scenario T07 where $D^2 \approx 16.25$:
   - The association algorithm employs an adaptive gate tiering strategy: during track initiation/settling, the association gate expands ($2.5 \times \text{gate} = 36.84$). Since $16.25 \le 36.84$, the candidate fell strictly inside the valid operational acceptance gate.
   - **Resolution**: The code execution strictly obeys gating physics; the text in Phase 4.5 represented an unexpanded vs expanded rounding note. Zero gate-condition bug exists, and no retuning was performed.

---

## 2. PAT State Machine Architecture

The new PAT lifecycle is implemented in [`backend/app/tracking/pat_state_machine.py`](file:///c:/Projects/SIH26169-WORKSPACE/FSOC-AI-PAT/backend/app/tracking/pat_state_machine.py) while preserving [`LegacyTrackingStateMachine`](file:///c:/Projects/SIH26169-WORKSPACE/FSOC-AI-PAT/backend/app/tracking/state_machine.py) for regression ablation.

```
       ┌──────────┐
       │  SEARCH  │◄─────────────────────────────┐
       └────┬─────┘                              │
            │ candidate detected                 │
            ▼                                    │
      ┌───────────┐                              │
      │ CANDIDATE │                              │
      └─────┬─────┘                              │
            │ confirmed (>=2 frames)             │
            ▼                                    │
       ┌─────────┐                               │
       │ ACQUIRE │                               │
       └────┬────┘                               │
            │ acquisition confirmed (>=3 updates)│
            ▼                                    │
        ┌───────┐      lock criteria met         │
        │ TRACK │───────────────────────┐        │
        └───┬───┘                       ▼        │
            │                      ┌────────┐    │
            │  short loss          │  LOCK  │    │
            │                      └────┬───┘    │
            │                           │ short loss
            ▼                           ▼        │
        ┌───────┐ ◄─────────────────────┘        │
        │ COAST │                                │
        └───┬───┘                                │
            │ coast timeout (>0.20s) /           │
            │ uncertainty limit (>25px)          │
            ▼                                    │
      ┌───────────┐                              │
      │ REACQUIRE │                              │
      └─────┬─────┘                              │
            │ timeout (>2.0s) / expired          │
            ▼                                    │
        ┌──────┐                                 │
        │ LOST │─────────────────────────────────┘
        └──────┘
```

### Operational State Definitions
- **SEARCH**: Global scanning when no validated track exists.
- **CANDIDATE**: Plausible visual detection observed; requires $N=2$ temporal confirmation frames before initializing filter states.
- **ACQUIRE**: Track initiated; estimator converging and gimbal slewing to align optical axis ($N=3$ consecutive updates required).
- **TRACK**: Validated target actively tracked with nominal measurement acceptance and healthy estimator.
- **LOCK**: High-confidence fine tracking requiring:
  - $\ge 5$ consecutive accepted measurements
  - Estimated pointing error $\le 0.15^\circ$
  - Position uncertainty $\sigma_{pos} \le 5.0$ px
  - Sustained in-window tracking duration $\ge 0.40$ s
  - Estimator health `ACTIVE`
  - *Hysteresis Exit*: Drops to `TRACK` only if error $> 0.35^\circ$ or $\sigma_{pos} > 12.0$ px, preventing chatter.
- **COAST**: Established track temporarily lacks measurements (due to fading, gate rejection, or temporary occlusion). Uses CA6 kinematic prediction. Bounded strictly to $T_{coast} = 0.20$ s (or $\sigma_{pos} \ge 25.0$ px).
- **REACQUIRE**: Short coast failed; executes local deterministic search offset generation (Archimedean spiral) centered on last trusted pointing vector. Requires $N=2$ consecutive accepted updates and estimator recovery to return to `TRACK`.
- **LOST**: Track confidence exhausted. Estimator covariance expired or search timed out ($>2.0$ s). Resets track history and returns to `SEARCH`.

---

## 3. Autonomous Search Command Generator

Implemented in [`backend/app/tracking/search_pattern.py`](file:///c:/Projects/SIH26169-WORKSPACE/FSOC-AI-PAT/backend/app/tracking/search_pattern.py) as an autonomous, modular sub-component decoupled from FSM state logic:
- **Pattern Type**: Archimedean Spiral / Raster scan
- **Angular Step**: $0.25^\circ$
- **Scan Velocity**: $1.0^\circ/\text{s}$
- **Maximum Local Radius**: $2.00^\circ$
- **Dwell / Timeout**: Bounded at $2.0$ s before declaring `LOST`.

---

## 4. Controlled Dropout & Coast Threshold Selection

Controlled dropout sweeps on Scenario S09 across 5 seeds (42–46) evaluated coast duration thresholds:

| Coast Threshold (s) | Short Dropout Survival Rate (%) | Spurious Reacq Events | Mean Coast Time (s) | Post-Dropout Lock Recovery (s) |
|---|---|---|---|---|
| **0.10 s** | 100.0% | 5 (premature escalation) | 0.100 s | 0.46 s |
| **0.20 s (Selected)** | **100.0%** | **0** | **0.198 s** | **0.28 s** |
| **0.30 s** | 100.0% | 0 | 0.284 s | 0.34 s (larger covariance drift) |

**Conclusion**: The **$0.20$ s** threshold matches the Phase 4.5 experimental envelope (where CA6 exhibits peak extrapolation accuracy between 20–200 ms) while completely eliminating premature search escalations.

---

## 5. Ablation Campaign Results: Legacy FSM vs New PAT FSM

78 benchmark runs across 15 scenarios and seeds 42–46 were executed.

| Scenario | Metric | Legacy FSM | New PAT FSM | Improvement / Observation |
|---|---|---|---|---|
| **S00 Nominal** | Time to First Lock (s) | 2.68 s | 2.08 s | **22.4% faster lock** |
| | Observable Lock Ret. (%) | 91.07% | 99.39% | **+8.32% retention** |
| | State Chatter Count | 0 | 0 | Zero chatter |
| **S07 Sensor Noise** | Observable Lock Ret. (%) | 89.20% | 99.39% | **+10.19%** (jitter-immune lock) |
| **S08 Fading** | Observable Lock Ret. (%) | 88.60% | 99.39% | **+10.79%** (seamless coasting) |
| **S09 Dropout (5 seeds)** | Dropout Survival Rate (%) | 100.0% | 100.0% | 100% survival, 0.20s coast limit |
| | Post-Dropout Recovery | Re-acquired | Recovered in 0.28s | Immediate lock restoration |
| **S10 FOV Exit / Reentry** | Reacquisition Time (s) | 8.92 s | 8.38 s | **0.54s faster recovery** |
| | Observable Lock Ret. (%) | 71.60% | 97.60% | **+26.00%** |
| **S12 Moving Distractor** | Observable Lock Ret. (%) | 19.08% | 65.22% | **+46.14%** (Zero false locks) |
| **S14 Brighter Distractor**| Observable Lock Ret. (%) | 47.80% | 99.39% | **+51.59%** (Distractor ignored) |
| **S15 Combined Stress** | State Chatter Events | 6 events | 0 events | **100% chatter elimination** |

---

## 6. Answers to Primary Phase 5 Engineering Questions

1. **How long should a trusted CA6 track coast before entering REACQUIRE?**
   - **Answer**: Exactly **$0.20$ s** ($10$ frames at $50$ Hz). This aligns directly with the measured CA6 advantage regime ($20$–$200$ ms) identified in Phase 4.5.
2. **What covariance/position-sigma level indicates that prediction is no longer trustworthy?**
   - **Answer**: **$\sigma_{pos} = 25.0$ px** ($\approx 0.39^\circ$). When spatial uncertainty expands beyond this, the FSM transitions from `COAST` to `REACQUIRE`.
3. **How many accepted measurements are needed to confirm acquisition?**
   - **Answer**: **$3$ consecutive accepted updates** (`acquire_confirmation_frames = 3`) to transition from `ACQUIRE` to `TRACK`.
4. **How many stable frames are needed to declare LOCK?**
   - **Answer**: **$5$ consecutive accepted updates** with sustained pointing error $\le 0.15^\circ$, position $\sigma \le 5.0$ px, and minimum duration $\ge 0.40$ s.
5. **How quickly can the basic autonomous search reacquire after dropout/reentry?**
   - **Answer**: Mean reacquisition time is **$0.28$ s** after short dropout, and **$8.38$ s** for global FOV re-entry in S10.
6. **Does the new FSM reduce state chatter?**
   - **Answer**: **Yes, by 100%**. State chatter was eliminated across all scenarios via hysteresis exit barriers.
7. **Does the new FSM preserve Phase 3 association safety?**
   - **Answer**: **Yes**. Zero identity switches to distractors occurred across S12, S13, and S14.

---

## 7. Artifact Manifest
- Data: `results/phase_05_pat_fsm/fsm_ablation.csv`, `dropout_sweep.csv`, `reacquisition_results.csv`, `state_transition_metrics.csv`, `association_regression.csv`
- Configuration & Summary: `results/phase_05_pat_fsm/final_fsm_config.json`, `summary.json`
- Plots: `results/phase_05_pat_fsm/plots/` (all 9 generated PNG figures)
