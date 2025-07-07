from __future__ import annotations
import numpy as np
from math import cos, sin
from robot.robot_state import RobotState

class DifferentialDriveRobot:
    """Pure kinematic differential‑drive model."""
    def __init__(self, wheel_radius: float = 0.05, wheelbase: float = 0.30):
        self.R = wheel_radius
        self.L = wheelbase
    
    # -------- Kinematics --------
    def forward_kinematics(self, w_r: float, w_l: float) -> tuple[float, float]:
        """Return (v, omega) given wheel angular speeds."""
        v = self.R * 0.5 * (w_r + w_l)
        omega = self.R / self.L * (w_r - w_l)
        return v, omega

    def inverse_kinematics(self, v: float, omega: float) -> tuple[float, float]:
        """Return (w_r, w_l) to realise body twist (v, omega)."""
        w_r = (2 * v + omega * self.L) / (2 * self.R)
        w_l = (2 * v - omega * self.L) / (2 * self.R)
        return w_r, w_l

    # -------- Dynamics (kinematic integration) --------
    def step(self, state: RobotState, w_r: float, w_l: float, dt: float) -> RobotState:
        v, omega = self.forward_kinematics(w_r, w_l)
        x_dot = v * cos(state.theta)
        y_dot = v * sin(state.theta)
        theta_dot = omega
        return RobotState(
            x=state.x + x_dot * dt,
            y=state.y + y_dot * dt,
            theta=_wrap_to_pi(state.theta + theta_dot * dt),
        )
    
    # -------- Motion models --------
    def motion_model(self, state: RobotState, u: np.ndarray, dt: float) -> RobotState:
        x_hat = state.x + dt * u[0] * cos(state.theta)
        y_hat = state.y + dt * u[0] * sin(state.theta)
        theta_hat = _wrap_to_pi(state.theta + dt * u[1])
        return RobotState(x=x_hat, y=y_hat, theta=theta_hat)

    # -------- Jacobians of the process and measurement models --------
    def motion_model_jacobian(self, state: RobotState, u: np.ndarray, dt: float) -> tuple[np.ndarray, np.ndarray]:
        """Jacobian of the process model."""
        F = np.array([
            [1, 0, -dt * u[0] * sin(state.theta)],
            [0, 1, dt * u[0] * cos(state.theta)],
            [0, 0, 1]
        ])

        L = np.array([
            [self.R / 2 * cos(state.theta), self.R / 2 * cos(state.theta)],
            [self.R / 2 * sin(state.theta), self.R / 2 * sin(state.theta)],
            [self.R / (2 * self.L), -self.R / (2 * self.L)]
        ])

        return F, L

# ---------------- Utilities ----------------
def _wrap_to_pi(angle: float) -> float:
    return (angle + np.pi) % (2 * np.pi) - np.pi