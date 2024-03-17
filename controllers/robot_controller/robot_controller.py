from typing import Tuple
from controller import Robot
import numpy as np


class Controller(Robot):
    MAX_SPEED = 6.2
    RELATIVE_ANGLES_OF_DISTANCE_SENSORS = np.array(
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

    def __init__(
        self, goal_position: np.ndarray, distance_threshold: float = 0.07
    ) -> None:
        super().__init__()

        self.goal_position = goal_position
        self.distance_threshold = distance_threshold

        # initialize the devices of the robot

        self.timeStep = int(self.getBasicTimeStep())

        self.pen = self.getDevice("pen")

        self.gps = self.getDevice("gps")
        self.gps.enable(self.timeStep)

        self.imu = self.getDevice("IMU")
        self.imu.enable(self.timeStep)

        self.distance_sensors = [self.getDevice(f"ds{i}") for i in range(8)]
        for ds in self.distance_sensors:
            ds.enable(self.timeStep)

        self.left_motor = self.getDevice("left wheel motor")
        self.right_motor = self.getDevice("right wheel motor")
        for motor in [self.left_motor, self.right_motor]:
            motor.setPosition(float("inf"))
            motor.setVelocity(0.0)

    def get_robot_position(self) -> np.ndarray:
        """Returns the (x, y) position of the robot from the GPS device."""
        return np.array(self.gps.getValues())[:2]

    def get_robot_angle(self) -> np.float64:
        """Returns the yaw angle of the robot in radians"""
        _, _, yaw = self.imu.getRollPitchYaw()
        return np.float64(yaw)

    def get_robot_heading(self) -> Tuple[np.ndarray, np.float64]:
        """Calculates the heading vector from the current position to the goal position."""
        return self.goal_position - self.get_robot_position()

    def filter_angle(self, angle):
        """Returns the proportional control output for a given target and current value."""
        # changing the range of the angle to the range [-pi, pi]
        angle = np.arctan2(np.sin(angle), np.cos(angle))

        front_distance = min(self.get_distances()[[0, 7]])
        if front_distance < 200:
            filter_amount = self.map(front_distance, 0, 200, 12, 0.9)
        else:
            filter_amount = 0.9

        return filter_amount * angle

    def map(self, value, from_lower, from_higher, to_lower, to_higher):
        """Maps a value from the range [`from_lower`, `from_higher`] to the range [`to_lower`, `to_higher`]."""
        mapped_value = (value - from_lower) * (to_higher - to_lower) / (
            from_higher - from_lower
        ) + to_lower
        return np.clip(mapped_value, min(to_lower, to_higher), max(to_lower, to_higher))

    def get_vector_components(self, magnitude: float, angle: float) -> np.ndarray:
        """Calculate the x and y components of the vector"""
        x = magnitude * np.cos(angle)
        y = magnitude * np.sin(angle)
        return np.array([x, y])

    def get_distances(self) -> np.ndarray:
        return np.array([ds.getValue() for ds in self.distance_sensors])

    def get_total_repulsive_force(self, max_magnitude=3) -> np.ndarray:
        """Return the repulsive forces relative to the obstacles distances as 2D vectors."""
        sensors_angles = (
            self.get_robot_angle() + Controller.RELATIVE_ANGLES_OF_DISTANCE_SENSORS
        )
        distances = self.map(self.get_distances(), 0, 1000, max_magnitude, 0)

        return -np.sum(
            [
                self.get_vector_components(distance, angle)
                for distance, angle in zip(distances, sensors_angles)
            ],
            axis=0,
        )

    def get_total_force(self):
        return self.get_total_repulsive_force() + self.get_robot_heading()

    def get_motors_speeds(
        self, distance_to_goal: float, total_force: np.ndarray
    ) -> tuple[float, float]:
        """Computes the left and right motor speeds based on the heading angle and the robot's orientation."""
        # stop if too close to goal
        if distance_to_goal <= self.distance_threshold:
            return 0, 0

        target_angle = np.arctan2(total_force[1], total_force[0])
        angle_difference = target_angle - self.get_robot_angle()
        angle_difference = self.filter_angle(angle_difference)

        raw_speed = self.map(angle_difference, np.pi / 2, 0, 0, Controller.MAX_SPEED)
        
        left_speed = raw_speed - angle_difference
        left_speed = np.clip(left_speed, -Controller.MAX_SPEED, Controller.MAX_SPEED)

        right_speed = raw_speed + angle_difference
        right_speed = np.clip(right_speed, -Controller.MAX_SPEED, Controller.MAX_SPEED)

        return left_speed, right_speed

    def get_initial_distance_to_goal(self) -> float:
        self.step(self.timeStep)  # needed to enable gps
        return np.linalg.norm(self.goal_position - self.get_robot_position())

    def get_distance_to_goal(self):
        return np.linalg.norm(self.goal_position - self.get_robot_position())

    def set_motors_speeds(self, left_speed: float, right_speed: float) -> None:
        self.left_motor.setVelocity(left_speed)
        self.right_motor.setVelocity(right_speed)

    def run(self) -> None:
        self.initial_distance_to_goal = self.get_initial_distance_to_goal()

        while self.step(self.timeStep) != -1:
            distance_to_goal = self.get_distance_to_goal()

            total_force = self.get_total_force()

            left_speed, right_speed = self.get_motors_speeds(
                distance_to_goal, total_force
            )
            self.set_motors_speeds(left_speed, right_speed)

            if distance_to_goal <= self.distance_threshold:
                print("Goal reached!")
                break


controller = Controller(goal_position=np.array([1.14, 1.14]))
controller.run()
