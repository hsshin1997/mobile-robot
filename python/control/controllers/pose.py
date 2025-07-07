"""Go‑to‑goal controller that depends only on MotionModel interface."""
from __future__ import annotations
from math import atan2, hypot
import numpy as np
from typing import Protocol
from ...core.utils import wrap_to_pi

class MotionModel(Protocol):
    def inverse_wheel_speeds(self, v: float, w: float): ...

class PoseController:
    def __init__(self, model: MotionModel,
                 k_rho: float = 1.25, k_alpha: float = 4.0, k_beta: float = -1.5):
        self.model = model
        self.k_rho, self.k_alpha, self.k_beta = k_rho, k_alpha, k_beta

    def control(self, state: np.ndarray, goal: np.ndarray):
        dx, dy = goal[0] - state[0], goal[1] - state[1]
        rho = hypot(dx, dy)
        alpha = wrap_to_pi(atan2(dy, dx) - state[2])
        beta = wrap_to_pi(goal[2] - state[2] - alpha)
        v = self.k_rho * rho
        w = self.k_alpha * alpha + self.k_beta * beta
        return v, w

    def wheel_speeds(self, state: np.ndarray, goal: np.ndarray):
        v, w = self.control(state, goal)
        return self.model.inverse_wheel_speeds(v, w)