import numpy as np

def _wrap_to_pi(theta: float) -> float:
    """Wrap angle to [-pi, pi]."""
    return (theta + np.pi) % (2 * np.pi) - np.pi

class ExtendedKalmanFilter:
    def __init__(self, x0, p0):
        self.x = x0  # state vector
        self.P = p0  # state covariance

    def predict(self, state, u, dt, model, Q):
        # 1. motion model
        # self.x = model.step(state, u, dt)
        self.x = model.motion_model(state, u, dt)

        # 2. Jacobians
        F, L = model.motion_model_jacobian(state, u, dt)   # recomputed every call
        self.P = F @ self.P @ F.T + L @ Q @ L.T

    def update(self, z, sensor):
        z_hat = sensor.measurement_model(self.x)
        H     = sensor.measurement_model_jacobian()
        R     = sensor.R
        v     = z - z_hat                     # innovation
        S     = H @ self.P @ H.T + R
        K     = self.P @ H.T @ np.linalg.inv(S)
        
        temp = K @ v
        self.x.x = self.x.x + temp[0] # update state
        self.x.y = self.x.y + temp[1] # update state
        self.x.theta = self.x.theta + temp[2] # update state

        I = np.eye(self.P.shape[0])
        temp = I - K @ H
        self.P = temp @ self.P @ temp.T + K @ R @ K.T  # update covariance
        self.x.theta = _wrap_to_pi(self.x.theta)
        return self.x, self.P

        