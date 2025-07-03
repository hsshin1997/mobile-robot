from robot.robot_state import RobotState
from robot.differential_drive import DifferentialDriveRobot
from robot.controller import PoseController
from planning.obstacle_avoidance import adjust_cmd
from sim.simulator import Simulator

robot = DifferentialDriveRobot()
ctrl = PoseController(robot=robot)
start = RobotState(0, 0, 0);  goal = RobotState(2, 0, 0)
obstacles = [(1, 0, 0.25)]  # one circular obstacle

sim = Simulator(robot, dt=0.02)

def wheel_cmd(state):
    v, w = ctrl.body_twist(state, goal)
    v, w = adjust_cmd(state, v, w, obstacles)
    return robot.inverse(v, w)

states = sim.run(start, wheel_cmd, duration=20.0)
sim.animate(states)