import numpy as np
from robot.controller import PoseController
from robot.differential_drive import DifferentialDriveRobot
from robot.robot_state import RobotState
from robot.sensors.gyro import Gyro
from filters.ekf import ExtendedKalmanFilter as EKF

import matplotlib.pyplot as plt

# ------------------------------------------------------------------ #
# ground‑truth simulator
# ------------------------------------------------------------------ #
# Robot model
robot = DifferentialDriveRobot()
controller = PoseController(robot=robot)
gyro = Gyro(sigma_theta=np.deg2rad(0.1))

# controller & EKF
ctrl = PoseController(robot=robot)


# initial conditions
init_state = RobotState(x=0.0, y=0.0, theta=0.0)
init_sigma = np.diag([.05**2,.05**2,np.deg2rad(5)**2])  # initial uncertainty

# goal state
goal_state = RobotState(x=30.0, y=3.0, theta=np.pi/4)



# Extended Kalman Filter
ekf = EKF(init_state, init_sigma)

state = init_state
states = []
states.append(state)

X = []
x = init_state
X.append(x)

for t in range(100):
    # Get control inputs
    wheel_speeds = controller.wheel_speeds(x, goal_state)
    state_pred = robot.step(x, *wheel_speeds, dt=0.1)
    u = robot.forward_kinematics(*wheel_speeds)
    
    # Predict next state using the robot model
    u = controller.control(state, goal_state)
    ekf.predict(state, u, dt=0.1, model=robot, Q=np.diag([(2*np.pi/1000)**2, (2*np.pi/1000)**2]))
    
    # Simulate gyro measurement
    gyro_measurement = gyro.measurement_model(state) + np.random.normal(0, gyro.sigma, 1)[0]
    
    # Update EKF with the predicted state and gyro measurement
    ekf.update(gyro_measurement, gyro)
    
    # Update the initial state for the next iteration
    print(f"Time step {t}: State: {state_pred}, EKF State: {ekf.x}")
    state = RobotState(x=ekf.x.x, y=ekf.x.y, theta=ekf.x.theta)
    states.append(state)
    X.append(state_pred)
    x= state_pred

#plot
plt.figure(figsize=(10, 5))
plt.plot([s.x for s in states], [s.y for s in states], label='EKF State', marker='o')
# plt.plot([s.x for s in X], [s.y for s in X], label='State', marker='o')
plt.show()