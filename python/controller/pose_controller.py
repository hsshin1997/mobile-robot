# pose_controller.py
import numpy as np
from typing import Tuple

from controller.pid import PIDController
from robot.robot_state import RobotState
from robot.differential_drive import DifferentialDriveRobot  # your class

class PosePIDController:
    """
    Very small pose tracker:
      • linear PID on Euclidean distance to goal
      • angular PID on heading error
    """
    def __init__(self,
                 robot: DifferentialDriveRobot,
                 kp_lin=1.5, ki_lin=0.0, kd_lin=0.2,
                 kp_ang=4.0, ki_ang=0.0, kd_ang=0.3,
                 max_lin=None, max_ang=None):
        self.robot = robot
        self.pid_lin = PIDController(kp_lin, ki_lin, kd_lin, output_limit=max_lin or robot.max_velocity)
        self.pid_ang = PIDController(kp_ang, ki_ang, kd_ang, output_limit=max_ang or robot.max_angular_velocity)

    def reset(self):
        self.pid_lin.reset()
        self.pid_ang.reset()

    def compute_cmd(self, goal: tuple[float, float, float], dt: float) -> tuple[float, float]:
        """Return (linear_cmd, angular_cmd) towards `goal`."""
        x, y, th = self.robot.state.get_pose()
        gx, gy, gth = goal

        # --- 1. Distance and heading errors
        dx, dy = gx - x, gy - y
        dist = np.hypot(dx, dy)                     # Euclidean distance
        heading_des = np.arctan2(dy, dx)            # desired heading to goal
        heading_err = np.arctan2(np.sin(heading_des - th),
                                 np.cos(heading_des - th))  # wrap to [-π,π]
        # For final orientation, add a second heading term when close enough
        if dist < 0.05:                             # 5 cm threshold
            heading_err = np.arctan2(np.sin(gth - th),
                                     np.cos(gth - th))

        # --- 2. Two independent PIDs
        v_cmd  = self.pid_lin(dist,      dt)
        w_cmd  = self.pid_ang(heading_err, dt)
        return v_cmd, w_cmd
