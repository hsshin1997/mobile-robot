import numpy as np
from dataclasses import dataclass
from typing import Tuple

@dataclass
class RobotState:
    """Represents the state of a differential drive robot."""
    x: float = 0.0
    y: float = 0.0
    theta: float = 0.0

    def to_array(self) -> np.ndarray:
        """Convert the robot state to a numpy array."""
        return np.array([self.x, self.y, self.theta])

    @classmethod
    def from_array(cls, arr: np.ndarray) -> 'RobotState':
        """Create state from numpy array"""
        return cls(x=arr[0], y=arr[1], theta=arr[2])
    
    def get_pose(self) -> Tuple[float, float, float]:
        """Get robot pose (x, y, theta)"""
        return self.x, self.y, self.theta