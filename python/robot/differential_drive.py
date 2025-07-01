import numpy as np
from typing import Tuple
from robot.robot_state import RobotState

class DifferentialDriveRobot:
    """Differential drive robot model with kinematics"""
    
    def __init__(self, wheel_radius: float = 0.1, wheel_base: float = 0.5,
                 max_velocity: float = 2.0, max_angular_velocity: float = np.pi):
        """
        Initialize differential drive robot
        
        Args:
            wheel_radius: Radius of wheels in meters
            wheel_base: Distance between wheels in meters
            max_velocity: Maximum linear velocity in m/s
            max_angular_velocity: Maximum angular velocity in rad/s
        """
        self.wheel_radius = wheel_radius
        self.wheel_base = wheel_base
        self.max_velocity = max_velocity
        self.max_angular_velocity = max_angular_velocity
        self.state = RobotState()
        
    def set_wheel_velocities(self, v_left: float, v_right: float):
        """Set wheel velocities (rad/s)"""
        # Convert to linear velocities and clamp
        v_left_linear = np.clip(v_left * self.wheel_radius, 
                               -self.max_velocity, self.max_velocity)
        v_right_linear = np.clip(v_right * self.wheel_radius, 
                                -self.max_velocity, self.max_velocity)
        
        self.state.v_left = v_left_linear
        self.state.v_right = v_right_linear
    
    def set_velocities(self, linear: float, angular: float):
        """Set robot velocities using linear and angular velocities"""
        # Clamp velocities
        linear = np.clip(linear, -self.max_velocity, self.max_velocity)
        angular = np.clip(angular, -self.max_angular_velocity, self.max_angular_velocity)
        
        # Convert to wheel velocities
        v_left = (linear - angular * self.wheel_base / 2) / self.wheel_radius
        v_right = (linear + angular * self.wheel_base / 2) / self.wheel_radius
        
        self.set_wheel_velocities(v_left, v_right)
    
    def get_velocities(self) -> Tuple[float, float]:
        """Get linear and angular velocities"""
        linear = (self.state.v_left + self.state.v_right) / 2
        angular = (self.state.v_right - self.state.v_left) / self.wheel_base
        return linear, angular
    
    def update(self, dt: float):
        """Update robot state using differential drive kinematics"""
        linear, angular = self.get_velocities()
        
        if abs(angular) < 1e-6:  # Straight line motion
            dx = linear * np.cos(self.state.theta) * dt
            dy = linear * np.sin(self.state.theta) * dt
            dtheta = 0
        else:  # Arc motion
            radius = linear / angular
            dtheta = angular * dt
            dx = radius * (np.sin(self.state.theta + dtheta) - np.sin(self.state.theta))
            dy = radius * (-np.cos(self.state.theta + dtheta) + np.cos(self.state.theta))
        
        # Update state
        self.state.x += dx
        self.state.y += dy
        self.state.theta += dtheta
        
        # Normalize theta to [-pi, pi]
        self.state.theta = np.arctan2(np.sin(self.state.theta), np.cos(self.state.theta))
        
    def reset(self, x: float = 0, y: float = 0, theta: float = 0):
        """Reset robot to initial state"""
        self.state = RobotState(x=x, y=y, theta=theta)