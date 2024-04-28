from typing import Iterator, Tuple
from robot import Robot
from path_finder import Waypoint, Path
import numpy as np


class EmptyPath(Exception):
    def __init__(self, message: str = None):
        self.message = message if message is not None else "The provided path is empty."
        super().__init__(self.message)


class APFController:
    def __init__(
        self,
        robot: Robot,
        *,
        path: Path,
        distance_to_goal_threshold: float = 0.1,
    ) -> None:
        self.__robot = robot
        self.__path: Iterator[Waypoint] = iter(path)
        self.__distance_threshold = distance_to_goal_threshold
        self.__initial_distance_to_goal: float | None = None
        self.__final_goal_reached: bool = False
        try:
            self.__destination = next(self.__path)
        except StopIteration as e:
            raise EmptyPath from e

    def __get_heading_vector(self) -> np.ndarray:
        """Calculates the heading vector from the current position to the goal position."""
        return self.__destination.to_numpy() - self.__robot.get_current_position()

    def __get_distance_to_goal(self) -> float:
        return np.linalg.norm(self.__get_heading_vector())

    def __filter_angle(self, angle: float) -> float:
        """Returns the proportional control output for a given target and current value."""
        # changing the range of the angle to the range [-pi, pi]
        angle = np.arctan2(np.sin(angle), np.cos(angle))

        front_distance = self.__robot.get_front_distance()

        filter_amount = (
            self.__map(front_distance, 0, 200, 12, 0.9) if front_distance < 200 else 0.9
        )
        return angle * filter_amount

    def __map(self, value, from_lower, from_higher, to_lower, to_higher):
        """
        Maps a value from the range [`from_lower`, `from_higher`] to
            the range [`to_lower`, `to_higher`].
        """
        mapped_value = (value - from_lower) * (to_higher - to_lower) / (
            from_higher - from_lower
        ) + to_lower
        return np.clip(mapped_value, min(to_lower, to_higher), max(to_lower, to_higher))

    def __vector_to_components(self, magnitude: float, angle: float) -> np.ndarray:
        """Calculate the x and y components of a vector."""
        x = magnitude * np.cos(angle)
        y = magnitude * np.sin(angle)
        return np.array([x, y])

    def __get_total_repulsive_force(self, max_magnitude: float = 20) -> np.ndarray:
        """Return the repulsive forces relative to the obstacles distances as 2D vectors."""
        distances = self.__map(
            self.__robot.get_distances(),
            0,
            self.__robot.MAX_DISTANCE_SENSOR_VALUE,
            max_magnitude,
            0,
        )
        sensors_angles = self.__robot.get_sensors_angles()
        return -np.sum(
            [
                self.__vector_to_components(distance, angle)
                for distance, angle in zip(distances, sensors_angles)
            ],
            axis=0,
        )

    def __get_total_force(self) -> np.ndarray:
        """Sums the repulsive force with the attractive force"""
        return self.__get_total_repulsive_force() + self.__get_heading_vector()

    def final_goal_reached(self) -> bool:
        return self.__final_goal_reached

    def compute_motors_speed(self) -> Tuple[float, float]:
        distance_to_goal = self.__get_distance_to_goal()
        if self.__initial_distance_to_goal is None:
            self.__initial_distance_to_goal = distance_to_goal

        total_force = self.__get_total_force()

        # stop if too close to the goal
        if distance_to_goal <= self.__distance_threshold:
            print(f"{self.__destination} reached.", end=" ")
            try:
                self.__destination = next(self.__path)
                print(f"Going to {self.__destination}.")
            except StopIteration:
                print("Final goal reached!!")
                self.__final_goal_reached = True
                return 0, 0

        raw_speed = self.__map(
            distance_to_goal,
            0,
            self.__initial_distance_to_goal,
            0,
            self.__robot.MAX_SPEED,
        )

        target_goal = np.arctan2(total_force[1], total_force[0])
        angle_difference = target_goal - self.__robot.get_current_angle()
        angle_difference = self.__filter_angle(angle_difference)

        left_speed = raw_speed - angle_difference
        right_speed = raw_speed + angle_difference

        left_speed, right_speed = map(
            lambda speed: np.clip(
                speed, -self.__robot.MAX_SPEED, self.__robot.MAX_SPEED
            ),
            [left_speed, right_speed],
        )

        return left_speed, right_speed
