from __future__ import annotations
from dataclasses import dataclass
import numpy as np

@dataclass
class RobotState:
    """Pose of a differential-drive robot in ℝ²×S¹."""
    x: float = 0.0
    y: float = 0.0
    theta: float = 0.0  # rad

    # ---------------- Convenience -----------------
    def as_vector(self) -> np.ndarray:
        return np.array([self.x, self.y, self.theta], dtype=float)

    @classmethod
    def from_vector(cls, vec: np.ndarray) -> "RobotState":
        assert vec.shape == (3,), "Vector must be length-3"
        return cls(*vec)

    def copy(self) -> "RobotState":
        return RobotState(self.x, self.y, self.theta)