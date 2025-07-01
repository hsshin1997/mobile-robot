import numpy as np
from typing import Tuple

from robot.robot_state import RobotState

class SimpleRobot:
    """Simple robot model with kinematics for differential drive robots."""

    # ---- parameters (geometry & limits) ----
    r: float # wheel radius [m]
    L: float # wheel base [m]

    v_max: float # max linear velocity [m/s]
    w_max: float # max angular velocity [rad/s]

    # ---- state (pose) ----
    state: RobotState
    # x: float # x position [m]
    # y: float # y position [m]
    # theta: float # orientation [rad]

    # ---- cahced control ----
    v: float = 0.0 # linear velocity [m/s]
    w: float = 0.0 # angular velocity [rad/s]

    # ---- constructor ----
    def __init__(self, r: float, L: float, v_max: float, w_max: float):
        self.r = r
        self.L = L
        self.v_max = v_max
        self.w_max = w_max
        self.state = RobotState()  # Initialize robot state

    # ----------------------------------------------------------
    # public API
    # ----------------------------------------------------------
    def set_twist(self, v:float, w:float):
        """Set linear and angular velocities."""
        self.v = np.clip(v, -self.v_max,  self.v_max)
        self.w = np.clip(w, -self.w_max,  self.w_max)
    
    def update(self, dt: float) -> None:
        """Update robot state based on current velocities."""
        if abs(self.state.theta) < 1e-6:                # straight
            self.state.x += self.v * np.cos(self.state.theta) * dt
            self.state.y += self.v * np.sin(self.state.theta) * dt

        else:                                           # arc
            radius = self.v / self.w if abs(self.w) > 1e-6 else np.inf
            dtheta = self.w * dt
            dx = radius * (np.sin(self.state.theta + dtheta) - np.sin(self.state.theta))
            dy = radius * (-np.cos(self.state.theta + dtheta) + np.cos(self.state.theta))
            self.state.x += dx
            self.state.y += dy
            self.state.theta += dtheta

        # wrap heading
        self.state.theta = np.arctan2(np.sin(self.state.theta), np.cos(self.state.theta))
    