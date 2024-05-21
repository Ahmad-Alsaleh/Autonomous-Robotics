from controller import Supervisor
import os, sys, re, numpy as np
from math import atan2, sqrt

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from main.deliberative_layer import ObstaclesMap, Rectangle

WIDTH, HEIGHT = 256, 256
MAP_MIN_X = 0
MAP_MAX_X = 1.12
MAP_MIN_Y = 0
MAP_MAX_Y = 1.12


class Visualizer(Supervisor):
    def __init__(self, obstacle_map: ObstaclesMap) -> None:
        super().__init__()
        self.__root_children = self.getRoot().getField("children")
        self.__epuck = self.getFromDef("EPUCK")
        self.__display = self.getDevice("display")
        self.__obstacle_map = obstacle_map

    def __draw_path_segment(
        self, waypoint_1, waypoint_2, *, is_path=True, thickness=0.01
    ):
        # finding the vector from waypoint_a to waypoint_b
        delta_x = waypoint_2.x - waypoint_1.x
        delta_y = waypoint_2.y - waypoint_1.y

        length = sqrt(delta_x**2 + delta_y**2)
        angle = atan2(delta_y, delta_x)

        mid_x = (waypoint_1.x + waypoint_2.x) / 2
        mid_y = (waypoint_1.y + waypoint_2.y) / 2

        color = "1 1 1" if is_path else "0 0 0"

        # VRML string for the cube
        cube_string = f"""
        Transform {{
            translation {mid_x} {mid_y} 0
            rotation 0 0 1 {angle}  # Rotate to align along the vector between waypoints
            children [Shape {{
                appearance MattePaint {{
                    baseColor {color}
                }}
                geometry Box {{
                    size {length} {thickness} {thickness}  # Length, height, and width of the box
                }}
            }}]
        }}
        """
        self.__root_children.importMFNodeFromString(-1, cube_string)

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
        x_min, y_min, x_max, y_max = map(lambda x: x, obstacle.to_list())
        print(x_min, y_min, x_max, y_max)
        x = self.__map_to_display(x_min, True)
        y = self.__map_to_display(y_max, False)
        width = self.__map_to_display(x_max - x_min, True)
        height = self.__map_to_display(y_max - y_min, False)
        self.__display.fillRectangle(x, HEIGHT - y, width, height)

    # def draw_objects(self, location: ):
    #     self.__display.fillOval(0, 0, WIDTH, HEIGHT)

    def draw_obstacles(self) -> None:
        for obstacle in self.__obstacle_map:
            self.__draw_obstacle(obstacle)

    def setDisplayColor(self, color: int) -> None:
        """Color is given as a hexadecimal number. For example, 0xFF00FF is purple."""
        self.__display.setColor(color)

    def set_robot_initial_position(self) -> None:
        start_position = [*self.__graph.get_start().to_numpy(), 0]
        self.__epuck.getField("translation").setSFVec3f(start_position)


# if __name__ == "__main__":
#     supervisor = Supervisor(obstacle_map)
#     supervisor.setDisplayColor(0xFF00FF)
#     supervisor.draw_obstacles()
