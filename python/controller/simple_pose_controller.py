import numpy as np

from python.controller.pid import PIDController
from python.robot.robot_state import RobotState
from python.robot.simple_robot import SimpleRobot


class SimplePoseController:
    """Robot Pose Controller"""
    def __init__(self,
                 robot: SimpleRobot):
        self.robot = robot
        kp_lin=1.5, ki_lin=0.0, kd_lin=0.2,
        self.linear_controller = PIDController(kp_lin, ki_lin, kd_lin, robot.v_max)
        kp_ang=4.0, ki_ang=0.0, kd_ang=0.3
        self.angular_controller = PIDController(kp_ang, ki_ang, kd_ang, robot.w_max)

    def reset(self):
        """Reset the controller state."""
        self.linear_controller.reset()
        self.angular_controller.reset()

    def compute_cmd(self, goal: RobotState, dt: float) -> tuple[float, float]:
        """Return (linear_cmd, angular_cmd) towards `goal`."""
        x, y, theta = self.robot.state.get_pose()
        gx, gy, gtheta = goal.get_pose()

        # --- 1. Distance and heading errors
        dx, dy = gx - x, gy - y
        dist = np.hypot(dx, dy)                     # Euclidean distance
        heading_des = np.arctan2(dy, dx)            # desired heading to
        heading_err = np.arctan2(np.sin(heading_des - theta),
                                 np.cos(heading_des - theta))
        # For final orientation, add a second heading term when close enough
        if dist < 0.05:                             # 5 cm threshold
            heading_err = np.arctan2(np.sin(gtheta - theta),
                                       np.cos(gtheta - theta))
            
        # --- 2. Two independent PIDs
        v_cmd = self.linear_controller(dist, dt)
        w_cmd = self.angular_controller(heading_err, dt)
        return v_cmd, w_cmd