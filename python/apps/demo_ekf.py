"""End-to-end demo: simulator + EKF + controller + gyro."""
from __future__ import annotations
import numpy as np
import matplotlib.pyplot as plt

from core import (
    RobotParams,
    DiffDriveSimulator,
    DiffDriveKinematics,
    EKF,
    GyroZ,
    wrap_to_pi,
    cov_to_ellipse,
)
from control.controllers.pose import PoseController

# --- initialisation -------------------------------------------------- #
np.random.seed(1)
params = RobotParams()
model  = DiffDriveKinematics(params)
sim    = DiffDriveSimulator(params, dt=0.05)

# controller & estimator wiring
ctrl = PoseController(model)
Q_enc = (2 * np.pi / 1000) ** 2  # one encoder tick variance
Q = np.diag([Q_enc, Q_enc])

ekf = EKF(x0=np.zeros(3), P0=np.diag([0.05**2, 0.05**2, np.deg2rad(5)**2]))
gyro = GyroZ()

# goal pose
goal = np.array([4.0, 2.0, np.pi / 2])

true_xy, est_xy, cov = [], [], []

for k in range(300):
    # control based on **estimated** pose
    x_est, _ = ekf.get()
    v_cmd, w_cmd = ctrl.control(x_est, goal)

    # propagate ground truth
    sim.step(v_cmd, w_cmd)

    # encoder‑based body twist with noise
    v_noise = v_cmd + np.random.normal(0, np.sqrt(Q_enc))
    w_noise = w_cmd + np.random.normal(0, np.sqrt(Q_enc))
    ekf.predict(np.array([v_noise, w_noise]), sim.dt, model, Q)

    # gyro heading update
    z = np.array([wrap_to_pi(sim.state.theta + np.random.normal(0, np.sqrt(gyro.R[0,0])))])
    ekf.update(z, gyro)

    true_xy.append((sim.state.x, sim.state.y))
    est_xy.append((ekf.x[0], ekf.x[1]))
    cov.append(ekf.P[:2, :2])

# --- plot ------------------------------------------------------------ #
xs, ys = zip(*true_xy)
xe, ye = zip(*est_xy)
plt.figure(figsize=(7, 4))
plt.plot(xs, ys, 'k-', lw=2, label='truth')
plt.plot(xe, ye, 'C1--', lw=1.5, label='EKF')
for i in range(0, len(cov), 20):
    w, h, ang = cov_to_ellipse(cov[i], 2)
    e = plt.matplotlib.patches.Ellipse((xe[i], ye[i]), w, h, ang,
                                       edgecolor='C0', facecolor='none', alpha=.7)
    plt.gca().add_patch(e)
plt.axis('equal'); plt.legend(); plt.tight_layout(); plt.show()
