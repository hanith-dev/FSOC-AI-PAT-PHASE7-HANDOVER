"""
Multi-Mode Gimbal Controller for FSOC-AI-PAT.
Provides:
  - C0: legacy_pid (unfiltered, raw derivative, standard clamp)
  - C1: filtered_pid (first-order low-pass derivative filter, anti-windup, deadband)
  - C2: predictive_pid (C1 + CA6 kinematic velocity feed-forward + short-horizon error prediction)
  - C3: predictive_smooth (C2 + rate limit + acceleration limit + jerk limit)

Strictly truth-firewalled: operates solely on estimated kinematics and operational telemetry.
"""
from dataclasses import dataclass, field
from enum import Enum
import math
from typing import Dict, Any, Optional, Tuple
import numpy as np


class ControlMode(str, Enum):
    LEGACY_PID = "legacy_pid"
    FILTERED_PID = "filtered_pid"
    PREDICTIVE_PID = "predictive_pid"
    PREDICTIVE_SMOOTH = "predictive_smooth"


@dataclass
class GimbalControllerConfig:
    # PID gains
    kp: float = 1.5
    ki: float = 0.05
    kd: float = 0.1

    # Integral anti-windup
    integral_limit: float = 20.0

    # Deadband
    deadband_deg: float = 0.01

    # Derivative filter time constant (seconds)
    # alpha = dt / (tau_d + dt). At 50Hz (dt=0.02), tau_d=0.04 -> alpha=0.333
    tau_d: float = 0.04

    # Latency horizon prediction bounds (seconds)
    min_prediction_horizon_s: float = 0.02   # 20 ms
    max_prediction_horizon_s: float = 0.20   # 200 ms
    default_prediction_horizon_s: float = 0.04 # 40 ms (2 frames at 50Hz)

    # Engineering Actuator Model (Software-in-the-loop gimbal limits)
    # Note: Not terminal hardware limits; physical plausibility constraints.
    max_rate_deg_s: float = 60.0          # Maximum angular velocity per axis
    max_accel_deg_s2: float = 250.0       # Maximum angular acceleration per axis
    max_jerk_deg_s3: float = 2500.0       # Maximum angular jerk per axis

    # Output clamp for unshaped/legacy mode
    legacy_output_limit: float = 180.0


class SingleAxisController:
    """Controls single axis (pan or tilt) with configurable mode."""

    def __init__(self, config: GimbalControllerConfig):
        self.config = config
        self.reset()

    def reset(self):
        """Reset internal states."""
        self.integral: float = 0.0
        self.prev_error: float = 0.0
        self.has_prev_error: bool = False
        self.filtered_d_error: float = 0.0

        # Motion profile state
        self.cmd_rate: float = 0.0
        self.prev_cmd_rate: float = 0.0
        self.prev_accel: float = 0.0

        # Saturation flags
        self.rate_saturated: bool = False
        self.accel_saturated: bool = False
        self.jerk_saturated: bool = False
        self.deadband_active: bool = False

    def reset_derivative(self, current_error: float):
        """Reset derivative memory across lifecycle boundaries (kick protection)."""
        self.prev_error = current_error
        self.has_prev_error = True
        self.filtered_d_error = 0.0

    def update(
        self,
        error: float,
        dt: float,
        mode: ControlMode,
        feedforward_rate: float = 0.0
    ) -> Tuple[float, Dict[str, Any]]:
        """
        Compute rate command for single axis.
        
        Args:
            error: Target error (deg). Positive error -> positive commanded rate.
            dt: Timestep (s).
            mode: ControlMode.
            feedforward_rate: Kinematic velocity feed-forward (deg/s).
            
        Returns:
            (commanded_rate, debug_telemetry)
        """
        if dt <= 0.0:
            return self.cmd_rate, {}

        self.rate_saturated = False
        self.accel_saturated = False
        self.jerk_saturated = False
        self.deadband_active = False

        # --- Deadband Check ---
        if mode != ControlMode.LEGACY_PID:
            if abs(error) < self.config.deadband_deg:
                self.deadband_active = True
                self.integral = 0.0
                self.prev_error = error
                self.has_prev_error = True
                self.filtered_d_error = 0.0
                raw_pid = 0.0
            else:
                raw_pid = None
        else:
            if abs(error) < self.config.deadband_deg:
                self.deadband_active = True
                self.integral = 0.0
                self.prev_error = error
                self.has_prev_error = True
                self.cmd_rate = 0.0
                return 0.0, {
                    "raw_pid": 0.0, "ff": 0.0, "raw_cmd": 0.0,
                    "limited_rate": 0.0, "accel": 0.0, "jerk": 0.0,
                    "rate_sat": False, "accel_sat": False, "jerk_sat": False,
                    "deadband": True, "integral": 0.0
                }
            raw_pid = None

        if raw_pid is None:
            # 1. Proportional term
            p_term = self.config.kp * error

            # 2. Integral term with anti-windup clamping
            self.integral += error * dt
            self.integral = max(-self.config.integral_limit, min(self.config.integral_limit, self.integral))
            i_term = self.config.ki * self.integral

            # 3. Derivative term
            if not self.has_prev_error:
                d_raw = 0.0
            else:
                d_raw = (error - self.prev_error) / dt

            if mode == ControlMode.LEGACY_PID:
                d_term = self.config.kd * d_raw
            else:
                # First-order low-pass filter on derivative
                alpha_d = dt / (self.config.tau_d + dt)
                self.filtered_d_error = self.filtered_d_error + alpha_d * (d_raw - self.filtered_d_error)
                d_term = self.config.kd * self.filtered_d_error

            self.prev_error = error
            self.has_prev_error = True
            raw_pid = p_term + i_term + d_term

        # Combine Feedforward
        if mode in (ControlMode.PREDICTIVE_PID, ControlMode.PREDICTIVE_SMOOTH):
            raw_cmd = feedforward_rate + raw_pid
            ff_applied = feedforward_rate
        else:
            raw_cmd = raw_pid
            ff_applied = 0.0

        # --- Command Shaping / Limiting ---
        if mode == ControlMode.LEGACY_PID:
            clamped_rate = max(-self.config.legacy_output_limit, min(self.config.legacy_output_limit, raw_cmd))
            self.rate_saturated = (clamped_rate != raw_cmd)
            self.cmd_rate = clamped_rate
            accel = (clamped_rate - self.prev_cmd_rate) / dt
            jerk = (accel - self.prev_accel) / dt
            self.prev_accel = accel
            self.prev_cmd_rate = clamped_rate
            return clamped_rate, {
                "raw_pid": raw_pid, "ff": ff_applied, "raw_cmd": raw_cmd,
                "limited_rate": clamped_rate, "accel": accel, "jerk": jerk,
                "rate_sat": self.rate_saturated, "accel_sat": False, "jerk_sat": False,
                "deadband": self.deadband_active, "integral": self.integral
            }

        elif mode in (ControlMode.FILTERED_PID, ControlMode.PREDICTIVE_PID):
            # Velocity clamp only (at max_rate)
            clamped_rate = max(-self.config.max_rate_deg_s, min(self.config.max_rate_deg_s, raw_cmd))
            self.rate_saturated = (clamped_rate != raw_cmd)
            self.cmd_rate = clamped_rate
            accel = (clamped_rate - self.prev_cmd_rate) / dt
            jerk = (accel - self.prev_accel) / dt
            self.prev_accel = accel
            self.prev_cmd_rate = clamped_rate
            return clamped_rate, {
                "raw_pid": raw_pid, "ff": ff_applied, "raw_cmd": raw_cmd,
                "limited_rate": clamped_rate, "accel": accel, "jerk": jerk,
                "rate_sat": self.rate_saturated, "accel_sat": False, "jerk_sat": False,
                "deadband": self.deadband_active, "integral": self.integral
            }

        else:
            # ControlMode.PREDICTIVE_SMOOTH: 3-stage command shaping (Jerk -> Accel -> Rate)
            # Desired acceleration from desired rate change
            desired_accel = (raw_cmd - self.prev_cmd_rate) / dt

            # 1. Jerk limiting
            delta_accel = desired_accel - self.prev_accel
            max_delta_accel = self.config.max_jerk_deg_s3 * dt
            if abs(delta_accel) > max_delta_accel:
                self.jerk_saturated = True
                delta_accel = math.copysign(max_delta_accel, delta_accel)

            feasible_accel = self.prev_accel + delta_accel

            # 2. Acceleration limiting with stopping/rate-boundary distance lookahead
            if abs(feasible_accel) > self.config.max_accel_deg_s2:
                self.accel_saturated = True
                feasible_accel = math.copysign(self.config.max_accel_deg_s2, feasible_accel)

            # Prevent rate clamp overshoot by constraining acceleration based on distance to max_rate
            # Using v_max braking condition: if accelerating towards +max_rate, ensure accel can ramp down to 0
            # under jerk_max without overshooting max_rate.
            # Delta_v to ramp accel from a to 0 under max_jerk is 0.5 * a^2 / j_max.
            # If feasible_rate + 0.5 * (feasible_accel^2 / max_jerk) > max_rate, decelerate now under jerk limit.
            j_max = self.config.max_jerk_deg_s3
            if feasible_accel > 0:
                stopping_dv = 0.5 * (feasible_accel ** 2) / j_max
                if (self.prev_cmd_rate + stopping_dv) >= self.config.max_rate_deg_s:
                    # Must ramp acceleration down under jerk limit
                    feasible_accel = max(0.0, self.prev_accel - max_delta_accel)
            elif feasible_accel < 0:
                stopping_dv = 0.5 * (feasible_accel ** 2) / j_max
                if (self.prev_cmd_rate - stopping_dv) <= -self.config.max_rate_deg_s:
                    # Must ramp acceleration down under jerk limit
                    feasible_accel = min(0.0, self.prev_accel + max_delta_accel)

            # 3. Rate calculation & Rate limiting
            feasible_rate = self.prev_cmd_rate + feasible_accel * dt
            if abs(feasible_rate) >= (self.config.max_rate_deg_s - 1e-4) or abs(raw_cmd) > self.config.max_rate_deg_s:
                self.rate_saturated = True

            if abs(feasible_rate) > self.config.max_rate_deg_s:
                feasible_rate = math.copysign(self.config.max_rate_deg_s, feasible_rate)
                feasible_accel = (feasible_rate - self.prev_cmd_rate) / dt

            jerk = (feasible_accel - self.prev_accel) / dt
            self.prev_accel = feasible_accel
            self.prev_cmd_rate = feasible_rate
            self.cmd_rate = feasible_rate

            return feasible_rate, {
                "raw_pid": raw_pid, "ff": ff_applied, "raw_cmd": raw_cmd,
                "limited_rate": feasible_rate, "accel": feasible_accel, "jerk": jerk,
                "rate_sat": self.rate_saturated, "accel_sat": self.accel_saturated, "jerk_sat": self.jerk_saturated,
                "deadband": self.deadband_active, "integral": self.integral
            }


class GimbalController:
    """
    Authoritative 2-Axis Gimbal Controller for Pan and Tilt channels.
    Supports C0 (legacy), C1 (filtered), C2 (predictive), C3 (predictive_smooth).
    Default production controller is C1 (filtered_pid) per Phase 6.5 audit.
    """

    def __init__(self, config: Optional[GimbalControllerConfig] = None, mode: ControlMode = ControlMode.FILTERED_PID):
        self.config = config or GimbalControllerConfig()
        self.mode = mode
        self.pan_ctrl = SingleAxisController(self.config)
        self.tilt_ctrl = SingleAxisController(self.config)
        self.prev_pat_state: Optional[str] = None
        self.state_reset_event: bool = False
        self.last_telemetry: Dict[str, Any] = {}

    def set_mode(self, mode: ControlMode):
        """Switch control mode."""
        self.mode = mode

    def reset(self):
        """Full reset of both channels."""
        self.pan_ctrl.reset()
        self.tilt_ctrl.reset()
        self.prev_pat_state = None
        self.state_reset_event = False
        self.last_telemetry = {}

    def notify_lifecycle_state(self, current_state: str):
        """
        Lifecycle state change notification.
        Resets derivative kick and integrator when resuming from COAST/REACQUIRE/LOST.
        """
        if self.prev_pat_state != current_state:
            # Major state transitions requiring derivative/integrator protection
            if current_state in ("SEARCH", "LOST"):
                self.reset()
                self.state_reset_event = True
            elif current_state in ("ACQUIRE", "TRACK") and self.prev_pat_state in ("SEARCH", "LOST", "COAST", "REACQUIRE"):
                # Clear derivative memory and reset integrator to avoid kick
                self.pan_ctrl.integral = 0.0
                self.tilt_ctrl.integral = 0.0
                self.pan_ctrl.has_prev_error = False
                self.tilt_ctrl.has_prev_error = False
                self.state_reset_event = True
            self.prev_pat_state = current_state

    def compute_control(
        self,
        kf_state: Optional[np.ndarray],  # [x, y, vx, vy, ax, ay] in pixel units
        camera_cx: float,
        camera_cy: float,
        camera_fx: float,
        camera_fy: float,
        dt: float,
        pat_state: str = "TRACK",
        search_pan_offset_deg: float = 0.0,
        search_tilt_offset_deg: float = 0.0,
        injected_latency_s: float = 0.0
    ) -> Tuple[float, float, Dict[str, Any]]:
        """
        Compute 2-axis gimbal rate commands [deg/s].
        
        Args:
            kf_state: 6-element Kalman state array or None.
            camera_cx, camera_cy, camera_fx, camera_fy: Authoritative camera calibration.
            dt: Integration time step (s).
            pat_state: Current PAT lifecycle state string.
            search_pan_offset_deg: Autonomous search offset for pan.
            search_tilt_offset_deg: Autonomous search offset for tilt.
            injected_latency_s: Known/measured system delay for prediction horizon tuning.
            
        Returns:
            (pan_rate_cmd_deg_s, tilt_rate_cmd_deg_s, controller_telemetry)
        """
        self.notify_lifecycle_state(pat_state)
        reset_event_occurred = self.state_reset_event
        self.state_reset_event = False

        # --- 1. SEARCH / LOST: Defer authority to Search Generator ---
        if pat_state in ("SEARCH", "LOST") or kf_state is None:
            # Gimbal follows search commands directly or remains stationary
            # Zero tracking error
            pan_out, pan_info = self.pan_ctrl.update(0.0 + search_pan_offset_deg, dt, self.mode)
            tilt_out, tilt_info = self.tilt_ctrl.update(0.0 + search_tilt_offset_deg, dt, self.mode)
            telem = self._build_telemetry(
                pan_out, tilt_out, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                pan_info, tilt_info, 0.0, camera_cx, camera_cy
            )
            self.last_telemetry = telem
            return pan_out, tilt_out, telem

        # --- 2. Target Kinematics Extraction ---
        kx = float(kf_state[0])
        ky = float(kf_state[1])
        kvx = float(kf_state[2]) if len(kf_state) > 2 else 0.0
        kvy = float(kf_state[3]) if len(kf_state) > 3 else 0.0
        kax = float(kf_state[4]) if len(kf_state) > 4 else 0.0
        kay = float(kf_state[5]) if len(kf_state) > 5 else 0.0

        # --- 3. Latency Horizon Prediction ---
        # Bounded between 20 ms and 200 ms (matching Phase 4.5 CA6 envelope)
        pred_horizon = max(
            self.config.min_prediction_horizon_s,
            min(self.config.max_prediction_horizon_s, injected_latency_s or self.config.default_prediction_horizon_s)
        )

        if self.mode in (ControlMode.PREDICTIVE_PID, ControlMode.PREDICTIVE_SMOOTH):
            tau = pred_horizon
            # CA6 forward kinematic extrapolation
            pred_x = kx + kvx * tau + 0.5 * kax * (tau ** 2)
            pred_y = ky + kvy * tau + 0.5 * kay * (tau ** 2)
            # Analytical derivative of camera pinhole projection:
            # theta = atan((x - cx) / fx)  =>  d(theta)/dt = (vx / fx) / (1 + ((x - cx) / fx)^2)
            # phi   = atan((cy - y) / fy)  =>  d(phi)/dt   = (-vy / fy) / (1 + ((cy - y) / fy)^2)
            d_pan_dt = (kvx / camera_fx) / (1.0 + ((kx - camera_cx) / camera_fx) ** 2)
            d_tilt_dt = (-kvy / camera_fy) / (1.0 + ((camera_cy - ky) / camera_fy) ** 2)
            ff_pan_deg_s = float(np.degrees(d_pan_dt))
            ff_tilt_deg_s = float(np.degrees(d_tilt_dt))
        else:
            pred_x = kx
            pred_y = ky
            ff_pan_deg_s = 0.0
            ff_tilt_deg_s = 0.0
            pred_horizon = 0.0

        # --- 4. Angular Error Calculation ---
        # Frame coordinates: dx = pred_x - cx, dy = cy - pred_y (optical convention)
        dx_pred = pred_x - camera_cx
        dy_pred = camera_cy - pred_y
        h_err_pred = float(np.degrees(np.arctan2(dx_pred, camera_fx)))
        v_err_pred = float(np.degrees(np.arctan2(dy_pred, camera_fy)))

        # Invert horizontal error according to runner/gimbal pan convention
        ctrl_h_err = -h_err_pred + search_pan_offset_deg
        ctrl_v_err = v_err_pred + search_tilt_offset_deg

        # --- 5. CANDIDATE State Safety Check ---
        # Do not chase unconfirmed candidate aggressively; scale down error
        if pat_state == "CANDIDATE":
            ctrl_h_err *= 0.2
            ctrl_v_err *= 0.2
            ff_pan_deg_s *= 0.2
            ff_tilt_deg_s *= 0.2

        # --- 6. Multi-Axis Execution ---
        pan_out, pan_info = self.pan_ctrl.update(ctrl_h_err, dt, self.mode, feedforward_rate=ff_pan_deg_s)
        tilt_out, tilt_info = self.tilt_ctrl.update(ctrl_v_err, dt, self.mode, feedforward_rate=ff_tilt_deg_s)

        telem = self._build_telemetry(
            pan_out, tilt_out, ctrl_h_err, ctrl_v_err,
            ff_pan_deg_s, ff_tilt_deg_s, h_err_pred, v_err_pred,
            pan_info, tilt_info, pred_horizon, pred_x, pred_y,
            reset_event_occurred=reset_event_occurred
        )
        self.last_telemetry = telem
        return pan_out, tilt_out, telem

    def _build_telemetry(
        self,
        pan_out: float, tilt_out: float,
        ctrl_h_err: float, ctrl_v_err: float,
        ff_pan: float, ff_tilt: float,
        h_err: float, v_err: float,
        pan_info: Dict[str, Any], tilt_info: Dict[str, Any],
        pred_horizon: float,
        pred_x: float, pred_y: float,
        reset_event_occurred: bool = False
    ) -> Dict[str, Any]:
        """Construct full 20-field controller telemetry."""
        return {
            "control_mode": self.mode.value,
            "predicted_target_x": round(float(pred_x), 2),
            "predicted_target_y": round(float(pred_y), 2),
            "prediction_horizon_ms": round(float(pred_horizon * 1000.0), 1),
            "feedforward_pan_rate": round(float(ff_pan), 4),
            "feedforward_tilt_rate": round(float(ff_tilt), 4),
            "pid_pan_component": round(float(pan_info.get("raw_pid", 0.0)), 4),
            "pid_tilt_component": round(float(tilt_info.get("raw_pid", 0.0)), 4),
            "raw_command_pan_rate": round(float(pan_info.get("raw_cmd", 0.0)), 4),
            "raw_command_tilt_rate": round(float(tilt_info.get("raw_cmd", 0.0)), 4),
            "limited_command_pan_rate": round(float(pan_out), 4),
            "limited_command_tilt_rate": round(float(tilt_out), 4),
            "pan_acceleration": round(float(pan_info.get("accel", 0.0)), 2),
            "tilt_acceleration": round(float(tilt_info.get("accel", 0.0)), 2),
            "pan_jerk": round(float(pan_info.get("jerk", 0.0)), 1),
            "tilt_jerk": round(float(tilt_info.get("jerk", 0.0)), 1),
            "rate_saturated": bool(pan_info.get("rate_sat") or tilt_info.get("rate_sat")),
            "acceleration_saturated": bool(pan_info.get("accel_sat") or tilt_info.get("accel_sat")),
            "jerk_saturated": bool(pan_info.get("jerk_sat") or tilt_info.get("jerk_sat")),
            "deadband_active": bool(pan_info.get("deadband") and tilt_info.get("deadband")),
            "integral_pan": round(float(pan_info.get("integral", 0.0)), 4),
            "integral_tilt": round(float(tilt_info.get("integral", 0.0)), 4),
            "controller_state_reset_event": bool(reset_event_occurred)
        }
