from __future__ import annotations
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from robot.robot_state import RobotState
from robot.differential_drive import DifferentialDriveRobot
from robot.controller import PoseController

from filters.ekf import ExtendedKalmanFilter as ekf
import numpy as np

class BasicSimulator:
    """
    A basic simulator for a differential drive robot. 
    Given robot and controller with initial and goal states,
    it simulates the robot's motion over time. 
    """
    def __init__(self, robot: DifferentialDriveRobot, controller: PoseController, dt= 0.01):
        self.robot = robot
        self.ctrl = controller
        self.dt = dt
        self.ekf = ekf

    def run(self, state: RobotState, goal: RobotState, t_final=10.0):
        """
        Simulate the robot's motion from initial state to goal state.
        
        Parameters
        ----------
        state : RobotState
            Initial state of the robot.
        goal : RobotState
            Goal state of the robot.
        t_final : float
            Total simulation time in seconds.
        
        Returns
        -------
        list[RobotState]
            List of robot states at each time step.
        """
        states = [state]
        t = 0.0
        while t < t_final:
            w_r, w_l = self.ctrl.wheel_speeds(state, goal)
            state = self.robot.step(state, w_r, w_l, self.dt)
            states.append(state)
            t += self.dt
        return states