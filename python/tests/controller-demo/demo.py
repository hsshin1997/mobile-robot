# demo.py
import numpy as np
import matplotlib.pyplot as plt
from robot.differential_drive import DifferentialDriveRobot
from controller.pose_controller import PosePIDController

# ---------------------------------------------------------------------
# 1.  Initialise robot + controller
# ---------------------------------------------------------------------
bot  = DifferentialDriveRobot()
ctl  = PosePIDController(bot)

goal_pose = (1.0, 1.0, 0.0)      # 1 m straight ahead
dt        = 0.02                 # 50 Hz
t_end     = 10.0                 # seconds
steps     = int(t_end / dt)

# Containers for logs
log_xy        = np.zeros((steps, 2))
log_cmd_vel   = np.zeros((steps, 2))   # v_cmd, w_cmd
log_true_vel  = np.zeros((steps, 2))   # v_true, w_true
time_axis     = np.linspace(0.0, t_end, steps)

# ---------------------------------------------------------------------
# 2.  Simulation loop
# ---------------------------------------------------------------------
for k in range(steps):
    v_cmd, w_cmd = ctl.compute_cmd(goal_pose, dt)
    bot.set_velocities(v_cmd, w_cmd)
    bot.update(dt)

    # Store logs
    log_xy[k]       = bot.state.get_pose()[:2]
    log_cmd_vel[k]  = [v_cmd, w_cmd]
    log_true_vel[k] = bot.get_velocities()

# ---------------------------------------------------------------------
# 3.  Plot trajectory + velocity profiles
# ---------------------------------------------------------------------
fig, (ax_traj, ax_vel) = plt.subplots(2, 1, figsize=(6, 8), sharex=False)

# --- trajectory
ax_traj.plot(log_xy[:, 0], log_xy[:, 1], label="path")
ax_traj.plot(*goal_pose[:2], "rx", label="goal")
ax_traj.set_aspect("equal", adjustable="box")
ax_traj.set_title("Robot trajectory")
ax_traj.set_xlabel("x  [m]")
ax_traj.set_ylabel("y  [m]")
ax_traj.grid(True)
ax_traj.legend()

# --- positions
# ax_vel.plot(time_axis, log_xy[:, 0], "b-", label="x")

# --- velocities
ax_vel.plot(time_axis, log_cmd_vel[:, 0], "b--",  label="v_cmd  (m/s)")
ax_vel.plot(time_axis, log_true_vel[:, 0], "b-",   label="v_true")
ax_vel.plot(time_axis, log_cmd_vel[:, 1], "r--",  label="ω_cmd (rad/s)")
ax_vel.plot(time_axis, log_true_vel[:, 1], "r-",   label="ω_true")
ax_vel.set_title("Velocity profiles")
ax_vel.set_xlabel("time  [s]")
ax_vel.set_ylabel("velocity")
ax_vel.grid(True)
ax_vel.legend(loc="upper right")

plt.tight_layout()
plt.show()