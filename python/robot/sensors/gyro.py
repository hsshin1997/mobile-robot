import numpy as np
from robot.robot_state import RobotState

class Gyro:
    def __init__(self, sigma_theta: float = 0.01):
        """
        Initialize the Gyro sensor with a noise standard deviation.
        
        Parameters
        ----------
        sigma_theta : float
            Standard deviation of the gyro noise (in radians).
        """
        self.sigma = sigma_theta
        self.R = np.array([[sigma_theta**2]])

    def measurement_model(self, state: RobotState) -> np.ndarray:
        """
        Return the gyro measurement (angular velocity).
        
        Parameters
        ----------
        state : RobotState
            The current state of the robot.
        
        Returns
        -------
        float
            The angular velocity (omega) of the robot.
        """
        return np.array([state.theta]) 
    
    def measurement_model_jacobian(self):
        """
        Return the Jacobian of the gyro measurement model.
        
        Returns
        -------
        np.ndarray
            The Jacobian matrix of the gyro measurement model.
        """
        return np.array([[0, 0, 1]])