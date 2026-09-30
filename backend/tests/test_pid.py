import pytest
import numpy as np

class PythonPIDController:
    def __init__(self, kp=1.5, ki=0.05, kd=0.1, integral_limit=10.0, output_limit=30.0, deadband=0.01):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral_limit = integral_limit
        self.output_limit = output_limit
        self.deadband = deadband
        self.reset()
        
    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0
        self.has_prev_error = False
        
    def update(self, error, dt):
        if dt <= 0:
            return 0.0
            
        if abs(error) < self.deadband:
            self.integral = 0.0
            self.prev_error = error
            self.has_prev_error = True
            return 0.0
            
        p_term = self.kp * error
        
        self.integral += error * dt
        self.integral = max(-self.integral_limit, min(self.integral_limit, self.integral))
        i_term = self.ki * self.integral
        
        d_term = 0.0
        if self.has_prev_error:
            error_diff = error - self.prev_error
            d_term = self.kd * (error_diff / dt)
            
        self.prev_error = error
        self.has_prev_error = True
        
        output = p_term + i_term + d_term
        output = max(-self.output_limit, min(self.output_limit, output))
        return output

def test_pid_convergence():
    pid = PythonPIDController(kp=2.0, ki=0.1, kd=0.1)
    
    # Simulate a simple system: position x, target 0.0
    # control output drives velocity dx/dt = -output
    x = 10.0
    dt = 0.1
    for _ in range(100):
        error = 0.0 - x
        output = pid.update(error, dt)
        x += output * dt  # Update system state
        
    # Should converge to target (0.0) within deadband
    assert abs(x) < 0.1

def test_pid_output_clamping():
    pid = PythonPIDController(kp=10.0, output_limit=5.0)
    # Huge error should trigger output limit
    output = pid.update(100.0, 0.1)
    assert output == 5.0

def test_pid_anti_windup():
    pid = PythonPIDController(kp=1.0, ki=1.0, integral_limit=2.0)
    # Maintain huge error for a long time
    for _ in range(50):
        pid.update(10.0, 1.0)
    # Integral must be clamped to integral_limit (2.0) instead of accumulating to 500
    assert pid.integral == 2.0

def test_pid_deadband():
    pid = PythonPIDController(deadband=0.5)
    # Error within deadband should produce 0 output and clear integral
    pid.integral = 5.0
    output = pid.update(0.3, 0.1)
    assert output == 0.0
    assert pid.integral == 0.0
