"""
Discrete PID Controller with anti-windup clamping and deadband.
Standardized across FSOC-AI-PAT backend and benchmark simulations.
"""
from typing import Optional


class PIDController:
    """
    Discrete-time PID controller with anti-windup clamping and deadband.
    
    Equations:
        e[k] = target - actual
        P = Kp * e[k]
        I = I + Ki * e[k] * dt (clamped to [-integral_limit, +integral_limit])
        D = Kd * (e[k] - e[k-1]) / dt
        u[k] = clamp(P + I + D, -output_limit, +output_limit)
    """
    def __init__(
        self,
        kp: float = 1.5,
        ki: float = 0.05,
        kd: float = 0.1,
        integral_limit: float = 20.0,
        output_limit: float = 180.0,
        deadband: float = 0.01
    ):
        self.kp = float(kp)
        self.ki = float(ki)
        self.kd = float(kd)
        self.integral_limit = float(integral_limit)
        self.output_limit = float(output_limit)
        self.deadband = float(deadband)
        
        self.integral: float = 0.0
        self.prev_error: float = 0.0
        self.has_prev_error: bool = False
        self.is_saturated: bool = False
        self.last_output: float = 0.0

    def reset(self):
        """Reset internal integrator and derivative state."""
        self.integral = 0.0
        self.prev_error = 0.0
        self.has_prev_error = False
        self.is_saturated = False
        self.last_output = 0.0

    def tune(self, kp: float, ki: float, kd: float):
        """Update PID tuning gains."""
        self.kp = float(kp)
        self.ki = float(ki)
        self.kd = float(kd)

    def update(self, error: float, dt: float) -> float:
        """
        Compute control output for time increment dt.
        
        Args:
            error: Current tracking error (deg)
            dt: Time step (seconds)
            
        Returns:
            Gimbal rate command (deg/s)
        """
        if dt <= 0.0:
            self.is_saturated = False
            return self.last_output
            
        # Deadband check: suppress micro-jitter inside lock core
        if abs(error) < self.deadband:
            self.integral = 0.0
            self.prev_error = error
            self.has_prev_error = True
            self.is_saturated = False
            self.last_output = 0.0
            return 0.0

        # Proportional term
        p_term = self.kp * error

        # Integral term with anti-windup clamping
        self.integral += error * dt
        self.integral = max(-self.integral_limit, min(self.integral_limit, self.integral))
        i_term = self.ki * self.integral

        # Derivative term
        d_term = 0.0
        if self.has_prev_error:
            d_term = self.kd * ((error - self.prev_error) / dt)

        self.prev_error = error
        self.has_prev_error = True

        # Unconstrained output
        raw_output = p_term + i_term + d_term

        # Actuator saturation clamping
        clamped_output = max(-self.output_limit, min(self.output_limit, raw_output))
        self.is_saturated = (clamped_output != raw_output)
        self.last_output = clamped_output
        return clamped_output


# Legacy alias preserved for backwards compatibility and exact C0 ablation
LegacyPIDController = PIDController
