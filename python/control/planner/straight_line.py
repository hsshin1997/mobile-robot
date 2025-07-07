"""Trivial planner: straight segment from start to goal."""
from __future__ import annotations
import numpy as np

class StraightLinePlanner:
    def plan(self, start: np.ndarray, goal: np.ndarray, n: int = 50):
        """Return np.ndarray of shape (n, 3) linearly interpolating pose."""
        t = np.linspace(0, 1, n)
        path = np.outer(1 - t, start) + np.outer(t, goal)
        return path