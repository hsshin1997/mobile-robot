"""Yaw‑only gyro (absolute orientation measurement)."""
from __future__ import annotations
import numpy as np
from ..utils import wrap_to_pi

class GyroZ:
    def __init__(self, sigma_theta: float = np.deg2rad(0.8)):
        self.R = np.array([[sigma_theta ** 2]])

    def h(self, x: np.ndarray):
        return x[2:3]  # shape (1,)

    def H(self, x: np.ndarray):
        return np.array([[0.0, 0.0, 1.0]])