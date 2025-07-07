"""Differential‑drive kinematics + simple simulator."""
from __future__ import annotations
import numpy as np
from dataclasses import dataclass
from ..utils import wrap_to_pi

# --------------------------------------------------------------------- #
# Dataclasses for parameters & state
# --------------------------------------------------------------------- #
@dataclass
class RobotParams:
    R: float = 0.05   # wheel radius (m)
    L: float = 0.16   # half axle width (m)
    m: float = 9.0    # mass (kg)
    Iz: float = 0.18  # moment of inertia about z (kg·m²)

@dataclass
class RobotState:
    x: float = 0.0
    y: float = 0.0
    theta: float = 0.0
    v: float = 0.0
    w: float = 0.0

    def copy(self):
        return RobotState(self.x, self.y, self.theta, self.v, self.w)
    
# --------------------------------------------------------------------- #
# Pure kinematics in interface form (for EKF & controllers)
# --------------------------------------------------------------------- #
class DiffDriveKinematics:
    """Stateless helper that provides f() and jacobians() for a diff-drive."""

    def __init__(self, params: RobotParams):
        self.p = params

    # motion model for EKF ---------------------------------------------------- #
    def f(self, x: np.ndarray, u: np.ndarray, dt: float):
        v, w = u
        theta = x[2]
        dx = v * np.cos(theta) * dt
        dy = v * np.sin(theta) * dt
        dtheta = w * dt
        return np.array([x[0] + dx, x[1] + dy, wrap_to_pi(theta + dtheta)])

    def jacobians(self, x: np.ndarray, u: np.ndarray, dt: float):
        v, _ = u
        theta = x[2]
        F = np.array([
            [1, 0, -v * dt * np.sin(theta)],
            [0, 1,  v * dt * np.cos(theta)],
            [0, 0, 1],
        ])
        R = self.p.R
        L = self.p.L
        Lmat = (
            np.array([
                [R/2*np.cos(theta),  R/2*np.cos(theta)],
                [R/2*np.sin(theta),  R/2*np.sin(theta)],
                [R/(2*L),          -R/(2*L)],
            ])
            * dt
        )
        return F, Lmat

    # wheel↔twist convenience -------------------------------------------------- #
    def forward_wheel_speeds(self, omega_r: float, omega_l: float):
        v = self.p.R * (omega_r + omega_l) / 2.0
        w = self.p.R * (omega_r - omega_l) / (2.0 * self.p.L)
        return v, w

    def inverse_wheel_speeds(self, v: float, w: float):
        omega_r = (v + self.p.L * w) / self.p.R
        omega_l = (v - self.p.L * w) / self.p.R
        return omega_r, omega_l

# --------------------------------------------------------------------- #
# Lightweight simulator that reuses the same kinematic math
# --------------------------------------------------------------------- #
class DiffDriveSimulator:
    def __init__(self, params: RobotParams, dt: float = 0.02):
        self.model = DiffDriveKinematics(params)
        self.state = RobotState()
        self.dt = dt

    def step(self, v_cmd: float, w_cmd: float):
        x_vec = np.array([self.state.x, self.state.y, self.state.theta])
        x_next = self.model.f(x_vec, np.array([v_cmd, w_cmd]), self.dt)
        self.state.x, self.state.y, self.state.theta = x_next
        self.state.v, self.state.w = v_cmd, w_cmd