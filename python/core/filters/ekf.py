"""Minimal 3‑state EKF (x, y, θ)."""
from __future__ import annotations
import numpy as np
from typing import Protocol
from ..utils import wrap_to_pi

class MotionModel(Protocol):
    def f(self, x: np.ndarray, u: np.ndarray, dt: float) -> np.ndarray: ...
    def jacobians(self, x: np.ndarray, u: np.ndarray, dt: float): ...

class SensorModel(Protocol):
    R: np.ndarray
    def h(self, x: np.ndarray) -> np.ndarray: ...
    def H(self, x: np.ndarray) -> np.ndarray: ...

class EKF:
    def __init__(self, x0: np.ndarray, P0: np.ndarray):
        self.x = x0.copy()
        self.P = P0.copy()

    def predict(self, u: np.ndarray, dt: float, model: MotionModel, Q: np.ndarray):
        self.x = model.f(self.x, u, dt)
        F, L = model.jacobians(self.x, u, dt)
        self.P = F @ self.P @ F.T + L @ Q @ L.T

    def update(self, z: np.ndarray, sensor: SensorModel):
        z_hat = sensor.h(self.x)
        H = sensor.H(self.x)
        S = H @ self.P @ H.T + sensor.R
        K = self.P @ H.T @ np.linalg.inv(S)
        self.x += K @ (z - z_hat)
        self.P = (np.eye(3) - K @ H) @ self.P
        self.x[2] = wrap_to_pi(self.x[2])

    def get(self):
        return self.x.copy(), self.P.copy()