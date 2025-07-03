from __future__ import annotations
import numpy as np
from robot.robot_state import RobotState
from robot.differential_drive import DifferentialDriveRobot

class DynamicDifferentialDriveRobot(DifferentialDriveRobot):
    """Torque‑driven DD model with viscous wheel friction."""
    def __init__(self, *, wheel_radius=0.05, wheelbase=0.30,
                 mass=4.0, inertia_z=0.06, wheel_inertia=0.002,
                 tau_max=0.8, viscous=0.2, dt=0.01):
        super().__init__(wheel_radius, wheelbase)
        self.M, self.Jz, self.Jw = mass, inertia_z, wheel_inertia
        self.tau_max, self.b, self.dt = tau_max, viscous, dt
        self.state = RobotState()
        self.w_r = self.w_l = 0.0

    def step(self, tau_r: float, tau_l: float) -> RobotState:
        # Saturate torques
        tau_r = np.clip(tau_r, -self.tau_max, self.tau_max)
        tau_l = np.clip(tau_l, -self.tau_max, self.tau_max)
        # Wheel dynamics (Euler)
        self.w_r += (tau_r - self.b * self.w_r) / self.Jw * self.dt
        self.w_l += (tau_l - self.b * self.w_l) / self.Jw * self.dt
        # Pose integration via superclass kinematics
        self.state = super().step(self.state, self.w_r, self.w_l, self.dt)
        return self.state