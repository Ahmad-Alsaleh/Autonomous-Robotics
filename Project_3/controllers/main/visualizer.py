from typing import Tuple
import os, sys, numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from main.deliberative_layer import ObstaclesMap, Rectangle
from main.constants import obstacle_map
from main.robot import Robot

WIDTH, HEIGHT = 256, 256
MAP_MIN_X = 0
MAP_MAX_X = 1.12
MAP_MIN_Y = 0
MAP_MAX_Y = 1.12


class Visualizer:
    def __init__(self, robot: Robot, obstacle_map: ObstaclesMap) -> None:
        super().__init__()
        self.__robot = robot
        self.__display = self.__robot.getDevice("display")
        self.__obstacle_map = obstacle_map

    def __map(self, value, from_lower, from_higher, to_lower, to_higher):
        """Maps a value from the range [`from_lower`, `from_higher`] to the range [`to_lower`, `to_higher`]."""
        mapped_value = (value - from_lower) * (to_higher - to_lower) / (
            from_higher - from_lower
        ) + to_lower
        return np.clip(mapped_value, min(to_lower, to_higher), max(to_lower, to_higher))

    def __map_to_display(self, coord: int, is_x: bool = True):
        if is_x:
            return self.__map(coord, MAP_MIN_X, MAP_MAX_X, 0, WIDTH)
        else:
            return self.__map(coord, MAP_MIN_Y, MAP_MAX_Y, 0, HEIGHT)

    def __draw_obstacle(self, obstacle: Rectangle) -> None:
        # get bottom left and top right
        x_min, y_min, x_max, y_max = obstacle.to_list()
        x = self.__map_to_display(x_min, True)
        y = self.__map_to_display(y_max, False)
        width = self.__map_to_display(x_max - x_min, True)
        height = self.__map_to_display(y_max - y_min, False)
        self.__display.fillRectangle(x, HEIGHT - y, width, height)

    def draw_detected_objects(self, location: Tuple):
        self.__set_display_color(0xFF00FF)
        self.__display.fillOval(*location, 7, 7)

    def draw_obstacles(self) -> None:
        self.__set_display_color(0x0000FF)
        for obstacle in self.__obstacle_map:
            self.__draw_obstacle(obstacle)

    def __set_display_color(self, color: int) -> None:
        """Color is given as a hexadecimal number. For example, 0xFF00FF is purple."""
        self.__display.setColor(color)


if __name__ == "__main__":
    visualizer = Visualizer(obstacle_map)
    visualizer.draw_obstacles()
    # visualizer.draw_detected_objects()
