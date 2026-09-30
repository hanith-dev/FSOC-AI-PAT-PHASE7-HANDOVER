"""Analysis and transition metrics extractor for PAT benchmark telemetry."""
from typing import List, Dict, Any
import numpy as np

def compute_pat_telemetry_metrics(telemetry: List[Dict[str, Any]], dt: float = 0.02) -> Dict[str, Any]:
    """Compute detailed PAT lifecycle transition and reacquisition metrics from telemetry rows."""
    if not telemetry:
        return {}

    states = [str(r.get("tracking_state", "")).upper() for r in telemetry]
    times = [float(r.get("simulation_time", i * dt)) for i, r in enumerate(telemetry)]
    reasons = [r.get("transition_reason", "") for r in telemetry]
    n_frames = len(telemetry)

    # State transitions & chatter
    transitions = []
    chatter_count = 0  # Reversals within <= 3 frames (60 ms)
    for i in range(1, n_frames):
        if states[i] != states[i - 1]:
            transitions.append((times[i], states[i - 1], states[i], reasons[i]))
            # Check chatter: if state returned to previous state in <= 3 frames
            if i >= 2 and states[i] == states[i - 2]:
                chatter_count += 1

    # Coast events and durations
    coast_events = 0
    coast_durations = []
    in_coast = False
    coast_start = 0.0

    # Reacquire events
    reacq_events = 0
    reacq_durations = []
    in_reacq = False
    reacq_start = 0.0
    reacq_successes = 0

    # Lost events
    lost_events = 0

    # Time to first track and lock
    time_to_track = None
    time_to_lock = None

    for i, s in enumerate(states):
        t = times[i]
        # First track
        if time_to_track is None and s in ("TRACKING", "TRACK", "LOCKED", "LOCK"):
            time_to_track = t
        # First lock
        if time_to_lock is None and s in ("LOCKED", "LOCK"):
            time_to_lock = t

        # Coast tracking
        if s == "COAST" and not in_coast:
            in_coast = True
            coast_start = t
            coast_events += 1
        elif s != "COAST" and in_coast:
            in_coast = False
            coast_durations.append(t - coast_start)

        # Reacquire tracking
        if s == "REACQUIRE" and not in_reacq:
            in_reacq = True
            reacq_start = t
            reacq_events += 1
        elif s != "REACQUIRE" and in_reacq:
            in_reacq = False
            reacq_durations.append(t - reacq_start)
            if s in ("TRACK", "TRACKING", "LOCK", "LOCKED"):
                reacq_successes += 1

        # Lost tracking
        if (s == "LOST" or s == "TARGET_LOST") and (i == 0 or states[i - 1] not in ("LOST", "TARGET_LOST")):
            lost_events += 1

    if in_coast:
        coast_durations.append(times[-1] - coast_start)
    if in_reacq:
        reacq_durations.append(times[-1] - reacq_start)

    mean_coast = float(np.mean(coast_durations)) if coast_durations else 0.0
    max_coast = float(np.max(coast_durations)) if coast_durations else 0.0
    mean_reacq = float(np.mean(reacq_durations)) if reacq_durations else 0.0
    max_reacq = float(np.max(reacq_durations)) if reacq_durations else 0.0

    # Search travel
    pan_offsets = [abs(float(r.get("search_offset_pan_deg", 0.0))) for r in telemetry]
    tilt_offsets = [abs(float(r.get("search_offset_tilt_deg", 0.0))) for r in telemetry]
    total_search_travel = float(np.sum(np.sqrt(np.array(pan_offsets)**2 + np.array(tilt_offsets)**2))) * dt

    return {
        "time_to_first_track_s": round(time_to_track, 3) if time_to_track is not None else None,
        "time_to_first_lock_s": round(time_to_lock, 3) if time_to_lock is not None else None,
        "state_transition_count": len(transitions),
        "state_chatter_count": chatter_count,
        "coast_event_count": coast_events,
        "mean_coast_duration_s": round(mean_coast, 4),
        "max_coast_duration_s": round(max_coast, 4),
        "lost_event_count": lost_events,
        "reacquisition_attempt_count": reacq_events,
        "reacquisition_success_count": reacq_successes,
        "reacquisition_success_rate": round(reacq_successes / max(1, reacq_events), 3) if reacq_events > 0 else 1.0,
        "mean_reacquisition_time_s": round(mean_reacq, 4),
        "max_reacquisition_time_s": round(max_reacq, 4),
        "search_angular_travel_deg": round(total_search_travel, 3)
    }
