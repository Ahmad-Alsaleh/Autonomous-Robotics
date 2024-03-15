from controller import Robot
import numpy as np


class Controller(Robot):
    MAX_SPEED = 6.2

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

        # self.distance_sensors_angles = np.array([1.27, 0.77, 0, 5.21, 4.21, np.pi, 2.37, 1.87])
        self.distance_sensors_angles = np.array(
            [
                1.27 - np.pi / 2,
                0.77 - np.pi / 2,
                -np.pi / 2,
                -(2 * np.pi - 5.21) - np.pi / 2,
                4.21 - np.pi / 2,
                np.pi / 2,
                2.37 - np.pi / 2,
                1.87 - np.pi / 2,
            ]
        )

        self.distance_sensors = [self.getDevice(f"ds{i}") for i in range(8)]

        for ds in self.distance_sensors:
            ds.enable(self.timeStep)

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

    def get_heading(self, position: np.ndarray) -> np.ndarray:
        """Returns the heading vector and angle between the robot's heading and the goal in radians"""
        heading_vector = self.goal - position
        return heading_vector, np.arctan2(heading_vector[1], heading_vector[0])

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

    def vector_from_magnitude_and_angle(
        self, magnitude: float, angle: float
    ) -> np.ndarray:
        # Calculate the x and y components of the vector
        x = magnitude * np.cos(angle)
        y = magnitude * np.sin(angle)
        return np.array([x, y])

    def get_distances(self) -> np.ndarray:
        return np.array([ds.getValue() for ds in self.distance_sensors])

    def get_distance_vectors(self, max_magnitude=2) -> np.ndarray:
        """return the vectors from the distances recorder from distance sensors"""

        angles = self.get_orientation() + self.distance_sensors_angles
        distances = self.map(self.get_distances(), 0, 1000, max_magnitude, 0)

        return np.array(
            [
                self.vector_from_magnitude_and_angle(dist, angle)
                for dist, angle in zip(distances, angles)
            ]
        )

    def get_motors_speeds(
        self, heading_vector: np.ndarray, orientation: float
    ) -> tuple[float, float]:
        """Computes the left and right motor speeds based on the heading angle and the robot's orientation."""

        # stop if too close to goal
        distance_to_goal = np.linalg.norm(heading_vector)
        if distance_to_goal < self.distance_threshold:
            return 0, 0

        # The -ve sign is to invert the vectors (repulsive)
        repulsive_vectors = -self.get_distance_vectors(4)

        direction = heading_vector + repulsive_vectors.sum(axis=0)
        direction_angle = np.arctan2(direction[1], direction[0])

        raw_speed = (
            Controller.MAX_SPEED
        )  # np.linalg.norm(direction) #PLAY WITH THIS LINE

        control_output = self.get_proportional_control(direction_angle, orientation)

        left_speed = raw_speed - control_output
        print(left_speed)
        left_speed = np.clip(left_speed, -Controller.MAX_SPEED, Controller.MAX_SPEED)
        right_speed = raw_speed + control_output
        print(right_speed)
        right_speed = np.clip(right_speed, -Controller.MAX_SPEED, Controller.MAX_SPEED)
        print(direction)
        print(raw_speed)
        return left_speed, right_speed

    def run(self) -> None:
        while self.step(self.timeStep) != -1:
            position = self.get_gps_position()
            heading_vector, heading_angle = self.get_heading(position)
            orientation = self.get_orientation()

            left_speed, right_speed = self.get_motors_speeds(
                heading_vector, orientation
            )
            self.left_motor.setVelocity(left_speed)
            self.right_motor.setVelocity(right_speed)

            print(
                f"Position: {position}\tOrientation: {orientation}\tHeading angle: {heading_angle}"
            )

            # print(f'distances :{self.map(self.get_distances(), 0, 1000, 10, 0)}')
            # print(f'distance vectors :\n {-self.get_distance_vectors().sum(axis=0)}')


controller = Controller(goal=np.array([1.136, 1.136]))
controller.run()
