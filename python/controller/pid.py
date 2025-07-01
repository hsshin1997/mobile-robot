from __future__ import annotations
import numpy as np
from dataclasses import dataclass

@dataclass
class PIDController:
    kp: float
    ki: float = 0.0
    kd: float = 0.0
    output_limit: float | None = None

    # Internal StateDifferential drive robot model with kinematics
    _integrator: float = 0.0
    _prev_error: float = 0.0

    def reset(self) -> None:
        self._integrator = 0.0
        self._prev_error = None

    def __call__(self, error: float, dt: float) -> float:
        """Compute the control effort."""
        # Integral
        self._integrator += error * dt
        # Derivative
        d_err = 0.0 if self._prev_error is None else (error - self._prev_error) / dt
        self._prev_error = error
        # PID formula
        u = self.kp * error + self.ki * self._integrator + self.kd * d_err
        # Clamp
        if self.output_limit is not None:
            u = np.clip(u, -self.output_limit, self.output_limit)
        return u