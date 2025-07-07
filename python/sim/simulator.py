from __future__ import annotations
import time
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from robot.robot_state import RobotState
from robot.differential_drive import DifferentialDriveRobot
from robot.controller import PoseController
import numpy as np

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
    def animate(self, states, trail_len=100):
        """
        Visualise the robot moving along the pre-computed `states` list.

        Parameters
        ----------
        states : list[RobotState]
            Pose history returned by Simulator.run().
        trail_len : int, optional
            How many past samples to keep visible as a “motion trail”.
        """
        xs = np.array([s.x for s in states])
        ys = np.array([s.y for s in states])
        thetas = np.array([s.theta for s in states])

        fig, ax = plt.subplots()
        ax.set_aspect("equal")
        ax.set_xlim(xs.min() - 0.2, xs.max() + 0.2)
        ax.set_ylim(ys.min() - 0.2, ys.max() + 0.2)
        ax.set_title("Differential-drive simulation")
        ax.scatter(xs[0], ys[0], c="green", marker="o", label="start")
        ax.scatter(xs[-1], ys[-1], c="red", marker="*", label="goal")

        # graphics objects we’ll update each frame
        trail, = ax.plot([], [], "b-", lw=2)            # recent path
        body,  = ax.plot([], [], "ko", ms=6)            # robot centre
        heading, = ax.plot([], [], "k-", lw=2)          # orientation line
        ax.legend()

        def init():
            trail.set_data([], [])
            body.set_data([], [])
            heading.set_data([], [])
            return trail, body, heading

        def update(frame):
            # limit visible trail
            start = max(0, frame - trail_len)
            trail.set_data(xs[start:frame+1], ys[start:frame+1])

            # --- FIX here: wrap scalars in [...]
            body.set_data([xs[frame]], [ys[frame]])

            # heading line
            hx = xs[frame] + 0.15 * np.cos(thetas[frame])
            hy = ys[frame] + 0.15 * np.sin(thetas[frame])
            heading.set_data([xs[frame], hx], [ys[frame], hy])
            return trail, body, heading

        ani = FuncAnimation(fig, update, frames=len(states),
                            init_func=init, interval=self.dt*1000, blit=True,
                            repeat=False)
        plt.show()
    
    # -------- Static plot of the full trajectory --------
    def plot_trajectory(self, states, title="Full path"):
        """
        Draw the whole path once the animation is finished.
        Call this *after* Simulator.animate(...).
        """
        import matplotlib.pyplot as plt
        xs = [s.x for s in states]
        ys = [s.y for s in states]

        fig, ax = plt.subplots()
        ax.set_aspect("equal")
        ax.plot(xs, ys, "b-", lw=2, label="trajectory")
        ax.scatter(xs[0], ys[0], c="green", marker="o", label="start")
        ax.scatter(xs[-1], ys[-1], c="red", marker="*", label="goal")
        ax.set_title(title)
        ax.legend()
        plt.show()