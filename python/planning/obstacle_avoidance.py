import math
from robot.robot_state import RobotState

def adjust_cmd(state: RobotState, v: float, w: float, obstacles, safe=0.3, alpha=2.0):
    """Return safe (v, w) given a list of (x, y, r)."""
    for ox, oy, r in obstacles:
        dx, dy = ox - state.x, oy - state.y
        d = math.hypot(dx, dy)
        h = d - (r + safe)
        if h <= 0:
            return 0.0, 0.0  # already in collision
        ang = math.atan2(dy, dx)
        h_dot = v*math.cos(ang - state.theta) - d*w*math.sin(ang - state.theta)
        if h_dot + alpha*h < 0:
            scale = (h_dot + alpha*h) / max(abs(h_dot), 1e-6)
            v *= scale;  w *= scale
    return v, w