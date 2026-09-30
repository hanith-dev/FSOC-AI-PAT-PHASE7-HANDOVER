/**
 * Multi-Mode Gimbal Controller for Live Mission Control (Mode B).
 * Synchronized with backend/app/tracking/controller.py (Mode A parity).
 * Supports:
 *   - legacy_pid (C0)
 *   - filtered_pid (C1)
 *   - predictive_pid (C2)
 *   - predictive_smooth (C3)
 */

export class GimbalController {
  constructor(config = {}) {
    this.kp = config.kp !== undefined ? config.kp : 1.5;
    this.ki = config.ki !== undefined ? config.ki : 0.05;
    this.kd = config.kd !== undefined ? config.kd : 0.1;
    this.integralLimit = config.integralLimit !== undefined ? config.integralLimit : 20.0;
    this.deadband = config.deadband !== undefined ? config.deadband : 0.01; // deg
    this.tauD = config.tauD !== undefined ? config.tauD : 0.04; // derivative filter constant (s)
    
    // Limits
    this.maxRate = config.maxRate !== undefined ? config.maxRate : 60.0; // deg/s
    this.maxAccel = config.maxAccel !== undefined ? config.maxAccel : 250.0; // deg/s^2
    this.maxJerk = config.maxJerk !== undefined ? config.maxJerk : 2500.0; // deg/s^3
    this.legacyOutputLimit = config.legacyOutputLimit !== undefined ? config.legacyOutputLimit : 180.0;

    this.mode = config.mode || "filtered_pid";
    this.reset();
  }

  reset() {
    this.integral = 0.0;
    this.prevError = 0.0;
    this.hasPrevError = false;
    this.filteredDError = 0.0;

    this.cmdRate = 0.0;
    this.prevCmdRate = 0.0;
    this.prevAccel = 0.0;

    this.rateSaturated = false;
    this.accelSaturated = false;
    this.jerkSaturated = false;
    this.deadbandActive = false;
  }

  resetDerivative(currentError = 0.0) {
    this.prevError = currentError;
    this.hasPrevError = true;
    this.filteredDError = 0.0;
  }

  setMode(mode) {
    this.mode = mode;
  }

  update(error, dt, feedforwardRate = 0.0) {
    if (dt <= 0) return this.cmdRate;

    this.rateSaturated = false;
    this.accelSaturated = false;
    this.jerkSaturated = false;
    this.deadbandActive = false;

    // 1. Deadband
    if (Math.abs(error) < this.deadband) {
      this.deadbandActive = true;
      this.integral = 0.0;
      this.prevError = error;
      this.hasPrevError = true;
      this.filteredDError = 0.0;
      if (this.mode === "legacy_pid") {
        this.cmdRate = 0.0;
        this.prevCmdRate = 0.0;
        this.prevAccel = 0.0;
        return 0.0;
      }
    }

    let rawPid = 0.0;
    if (!this.deadbandActive) {
      // P term
      const pTerm = this.kp * error;

      // I term
      this.integral += error * dt;
      this.integral = Math.max(-this.integralLimit, Math.min(this.integralLimit, this.integral));
      const iTerm = this.ki * this.integral;

      // D term
      const dRaw = this.hasPrevError ? (error - this.prevError) / dt : 0.0;
      let dTerm = 0.0;
      if (this.mode === "legacy_pid") {
        dTerm = this.kd * dRaw;
      } else {
        const alphaD = dt / (this.tauD + dt);
        this.filteredDError = this.filteredDError + alphaD * (dRaw - this.filteredDError);
        dTerm = this.kd * this.filteredDError;
      }

      this.prevError = error;
      this.hasPrevError = true;
      rawPid = pTerm + iTerm + dTerm;
    }

    // Combine feed-forward
    let rawCmd = rawPid;
    if (this.mode === "predictive_pid" || this.mode === "predictive_smooth") {
      rawCmd += feedforwardRate;
    }

    // Command Shaping
    if (this.mode === "legacy_pid") {
      const clamped = Math.max(-this.legacyOutputLimit, Math.min(this.legacyOutputLimit, rawCmd));
      this.rateSaturated = (clamped !== rawCmd);
      this.cmdRate = clamped;
      const accel = (clamped - this.prevCmdRate) / dt;
      this.prevAccel = accel;
      this.prevCmdRate = clamped;
      return clamped;
    } else if (this.mode === "filtered_pid" || this.mode === "predictive_pid") {
      const clamped = Math.max(-this.maxRate, Math.min(this.maxRate, rawCmd));
      this.rateSaturated = (clamped !== rawCmd);
      this.cmdRate = clamped;
      const accel = (clamped - this.prevCmdRate) / dt;
      this.prevAccel = accel;
      this.prevCmdRate = clamped;
      return clamped;
    } else {
      // predictive_smooth (Jerk -> Accel -> Rate shaping)
      const desiredAccel = (rawCmd - this.prevCmdRate) / dt;
      let deltaAccel = desiredAccel - this.prevAccel;
      const maxDeltaAccel = this.maxJerk * dt;

      if (Math.abs(deltaAccel) > maxDeltaAccel) {
        this.jerkSaturated = true;
        deltaAccel = Math.sign(deltaAccel) * maxDeltaAccel;
      }

      let feasibleAccel = this.prevAccel + deltaAccel;
      if (Math.abs(feasibleAccel) > this.maxAccel) {
        this.accelSaturated = true;
        feasibleAccel = Math.sign(feasibleAccel) * this.maxAccel;
      }

      let feasibleRate = this.prevCmdRate + feasibleAccel * dt;
      if (Math.abs(feasibleRate) > this.maxRate) {
        this.rateSaturated = true;
        feasibleRate = Math.sign(feasibleRate) * this.maxRate;
        feasibleAccel = (feasibleRate - this.prevCmdRate) / dt;
      }

      this.prevAccel = feasibleAccel;
      this.prevCmdRate = feasibleRate;
      this.cmdRate = feasibleRate;
      return feasibleRate;
    }
  }

  tune(kp, ki, kd) {
    this.kp = kp;
    this.ki = ki;
    this.kd = kd;
  }
}

// Backwards compatibility alias
export class PIDController extends GimbalController {
  constructor(config = {}) {
    super({ ...config, mode: "legacy_pid" });
  }
}
