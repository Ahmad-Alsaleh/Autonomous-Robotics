from controller import Robot
import numpy as np


class Controller(Robot):
    MAX_SPEED = 6

    def __init__(self, goal: np.ndarray, distance_threshold: float = 0.05) -> None:
        super().__init__()

        self.goal = goal
        self.distance_threshold = distance_threshold

        # initialize the devices of the robot

        self.timeStep = int(self.getBasicTimeStep())

        self.pen = self.getDevice("pen")

        self.gps = self.getDevice("gps")
        self.gps.enable(self.timeStep)

        self.imu = self.getDevice("IMU")
        self.imu.enable(self.timeStep)

        self.left_motor = self.getDevice("left wheel motor")
        self.right_motor = self.getDevice("right wheel motor")
        for motor in [self.left_motor, self.right_motor]:
            motor.setPosition(float("inf"))
            motor.setVelocity(0.0)

    def get_gps_position(self) -> np.ndarray:
        """Returns the (x, y) position of the robot from the GPS device."""
        return np.array(self.gps.getValues())[:2]

    def get_orientation(self) -> np.float64:
        """Returns the yaw angle of the robot in radians"""
        _, _, yaw = self.imu.getRollPitchYaw()
        return np.float64(yaw)

    def get_heading(self, position: np.ndarray) -> np.float64:
        """Returns the angle between the robot's heading and the goal in radians"""
        heading_vector = self.goal - position
        return np.arctan2(heading_vector[1], heading_vector[0])

    def get_proportional_control(self, target_angle, current_angle, Kp=0.9):
        """Returns the proportional control output for a given target and current value."""
        error = target_angle - current_angle

        # take into account the circular nature of angles by converting the error to the range [-pi/2, pi/2]
        error = np.arctan2(np.sin(error), np.cos(error))

        # Calculate the proportional control output
        return Kp * error

    def map(self, value, from_min, from_max, to_min, to_max):
        """Maps a value from the range [`from_min`, `from_max`] to the range [`to_min`, `to_max`]."""
        return (value - from_min) * (to_max - to_min) / (from_max - from_min) + to_min

    def get_motors_speeds(
        self, heading_angle: float, orientation: float
    ) -> tuple[float, float]:
        """Computes the left and right motor speeds based on the heading angle and the robot's orientation."""

        # stop if too close to goal
        position = self.get_gps_position()
        distance_to_goal = np.linalg.norm(self.goal - position)
        if distance_to_goal < self.distance_threshold:
            return 0, 0

        control_output = self.get_proportional_control(heading_angle, orientation)

        left_speed = Controller.MAX_SPEED - control_output
        left_speed = np.clip(left_speed, -Controller.MAX_SPEED, Controller.MAX_SPEED)

        right_speed = Controller.MAX_SPEED + control_output
        right_speed = np.clip(right_speed, -Controller.MAX_SPEED, Controller.MAX_SPEED)

        return left_speed, right_speed

    def run(self) -> None:
        while self.step(self.timeStep) != -1:
            position = self.get_gps_position()
            heading_angle = self.get_heading(position)
            orientation = self.get_orientation()

            left_speed, right_speed = self.get_motors_speeds(heading_angle, orientation)
            self.left_motor.setVelocity(left_speed)
            self.right_motor.setVelocity(right_speed)

            print(
                f"Position: {position}\t\tOrientation: {orientation}\t\tHeading angle: {heading_angle}"
            )


controller = Controller(goal=np.array([0, 0]))
controller.run()
