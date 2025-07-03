import numpy as np
from robot.robot_state import RobotState
from robot.differential_drive import DifferentialDriveRobot
from robot.controller import PoseController
from sim.simulator import Simulator

if __name__ == "__main__":
    robot = DifferentialDriveRobot(wheel_radius=0.05, wheelbase=0.3)
    controller = PoseController(robot=robot)
    sim = Simulator(robot, controller, dt=0.02)

    start = RobotState(0.0, 0.0, 0.0)
    goal = RobotState(1.5, 1.5, -np.pi )

    states = sim.run(start, goal, t_final=15.0)
    sim.animate(states)