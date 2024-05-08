from typing import Tuple
from main.robot import Robot
from main.path_finder import Waypoint, DeliberativeLayer, PathTraversalCompleted
import numpy as np

class APFController:
    def __init__(
        self,
        robot: Robot,
        deliberative_layer: DeliberativeLayer,
        *,
        distance_to_goal_threshold: float = 0.1,
    ) -> None:
        self.__robot = robot
        self.__distance_threshold = distance_to_goal_threshold
        self.__final_goal_reached: bool = False
        self.__deliberative_layer = deliberative_layer
        
        self.__robot.simulator_step() # essential to enable gps in order to obtain intial distance to goal
        
        self.__destination = self.__get_destination()

    def __get_destination(self) -> Waypoint:
        
        return self.__deliberative_layer.get_next_waypoint()    

    def __get_heading_vector(self) -> np.ndarray:
        """Calculates the heading vector from the current position to the goal position."""
        return self.__destination.to_numpy() - self.__robot.get_current_position()

    def __get_attractive_force(self, max_magnitude= 5) -> np.ndarray:
        heading = self.__get_heading_vector()
        return (max_magnitude / np.linalg.norm(heading)) * (heading)

    def __get_distance_to_waypoint(self) -> float:
        return np.linalg.norm(self.__get_heading_vector())

    def __filter_angle(self, angle: float) -> float:
        """Returns the proportional control output for a given target and current value."""
    
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

    def __get_total_repulsive_force(self, max_magnitude: float = 10) -> np.ndarray:
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
        return self.__get_total_repulsive_force() + self.__get_attractive_force()

    def final_goal_reached(self) -> bool:
        return self.__final_goal_reached

    def compute_motors_speed(self) -> Tuple[float, float]:
        distance_to_waypoint = self.__get_distance_to_waypoint()
        total_force = self.__get_total_force()

        # stop if too close to the goal
        if distance_to_waypoint <= self.__distance_threshold:
            print(f"{self.__destination} reached.", end=" ")
            try:
                self.__destination = self.__get_destination()
                print(f"Going to {self.__destination}.")
            except PathTraversalCompleted:
                print("Final goal reached!!")
                self.__final_goal_reached = True
                return 0, 0

        target_goal = np.arctan2(total_force[1], total_force[0])
        angle_difference = target_goal - self.__robot.get_current_angle()
        # changing the range of the angle to the range [-pi, pi]
        angle_difference = np.arctan2(np.sin(angle_difference), np.cos(angle_difference))

        raw_speed = self.__map(
            abs(angle_difference),
            np.pi/2,
            0,
            0,
            self.__robot.MAX_SPEED,
        )
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
