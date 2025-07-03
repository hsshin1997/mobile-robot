from __future__ import annotations
import numpy as np
from math import atan2, hypot
from robot.robot_state import RobotState
from robot.differential_drive import _wrap_to_pi, DifferentialDriveRobot

class PoseController:
    """Go‑to‑goal pose controller (Samson style)."""
    def __init__(self, k_rho=1.0, k_alpha=4.0, k_beta=-1.5, robot: DifferentialDriveRobot | None=None):
        self.k_rho, self.k_alpha, self.k_beta = k_rho, k_alpha, k_beta
        self.robot = robot or DifferentialDriveRobot()

    def control(self, state: RobotState, goal: RobotState) -> tuple[float, float]:
        # -------- Error coordinates --------
        dx, dy = goal.x - state.x, goal.y - state.y
        rho = hypot(dx, dy)
        alpha = _wrap_to_pi(atan2(dy, dx) - state.theta)
        beta = _wrap_to_pi(goal.theta - state.theta - alpha)

        # -------- Body‑twist -------
        v = self.k_rho * rho
        omega = self.k_alpha * alpha + self.k_beta * beta
        return v, omega

    def wheel_speeds(self, state: RobotState, goal: RobotState):
        v, omega = self.control(state, goal)
        return self.robot.inverse_kinematics(v, omega)