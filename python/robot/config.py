class RobotConfig:
    """
    Configuration for the robot.
    """
    def __init__(self, wheel_radius: float = 0.1, wheel_base: float = 0.5):
        self.wheel_radius = wheel_radius  # Radius of the wheels in meters
        self.wheel_base = wheel_base        # Distance between the wheels in meters

    def __repr__(self):
        return f"RobotConfig(wheel_radius={self.wheel_radius}, wheel_base={self.wheel_base})"