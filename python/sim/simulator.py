from __future__ import annotations
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from robot.robot_state import RobotState
from robot.differential_drive import DifferentialDriveRobot
from robot.controller import PoseController

class Simulator:
    def __init__(self, robot: DifferentialDriveRobot, controller: PoseController, dt: float = 0.02):
        self.robot = robot
        self.ctrl = controller
        self.dt = dt

    def run(self, state: RobotState, goal: RobotState, t_final=10.0):
        states = [state]
        t = 0.0
        while t < t_final:
            w_r, w_l = self.ctrl.wheel_speeds(state, goal)
            state = self.robot.step(state, w_r, w_l, self.dt)
            states.append(state)
            t += self.dt
        return states

    # -------- Simple live plot --------
    def animate(self, states):
        xs = [s.x for s in states]
        ys = [s.y for s in states]
        fig, ax = plt.subplots()
        ax.set_aspect('equal')
        ax.plot(xs, ys, label="trajectory")
        ax.scatter([xs[0]], [ys[0]], marker='o', label='start')
        ax.scatter([xs[-1]], [ys[-1]], marker='*', label='goal')
        ax.legend()
        plt.show()