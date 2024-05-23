from typing import Tuple
import numpy as np
from deliberative_layer import ObstaclesMap, Rectangle, Waypoint, Path
from robot import Robot
from math import atan2, sqrt
import re
from sklearn.cluster import DBSCAN
from controller import Supervisor
from constants import play_area

WIDTH, HEIGHT = 256, 256
BACKGROUND_COLOR = 0x000000
MAP_MIN_X, MAP_MAX_X, MAP_MIN_Y, MAP_MAX_Y = play_area

PROTO_COUNTER = 0


class Visualizer:
    def __init__(self, robot: Robot, obstacle_map: ObstaclesMap) -> None:
        super().__init__()
        self.__robot = robot
        self.__display = self.__robot.getDevice("display")
        self.__obstacle_map = obstacle_map
        self.__supervisor = Supervisor()
        self.__root_children = self.__supervisor.getRoot().getField("children")
        self.__waypoint_template = self.__supervisor.getFromDef("WAYPOINT_TEMPLATE")
        self.__generated_nodes = []
        self.__detected_objects = []

    def __map(self, value, from_lower, from_higher, to_lower, to_higher):
        """Maps a value from the range [`from_lower`, `from_higher`] to the range [`to_lower`, `to_higher`]."""
        mapped_value = (value - from_lower) * (to_higher - to_lower) / (
            from_higher - from_lower
        ) + to_lower
        return np.clip(mapped_value, min(to_lower, to_higher), max(to_lower, to_higher))

    def __map_to_display(self, coord_x: float, coord_y: float):
        x_t = self.__map(coord_x, MAP_MIN_X, MAP_MAX_X, 0, WIDTH)
        y_t = self.__map(coord_y, MAP_MIN_Y, MAP_MAX_Y, 0, WIDTH)
        return x_t, y_t

    def __display_to_map(self, x: int, y: int):
        x_t = self.__map(x, 0, WIDTH, MAP_MIN_X, MAP_MAX_X)
        y_t = self.__map(y, 0, HEIGHT, MAP_MIN_Y, MAP_MAX_Y)
        return x_t, y_t

    def __draw_obstacle_on_display(self, obstacle: Rectangle) -> None:
        # get bottom left and top right
        x_min, y_min, x_max, y_max = obstacle.to_list()
        x, y = self.__map_to_display(x_min, y_max)
        width, height = self.__map_to_display(x_max - x_min, y_max - y_min)
        self.__display.fillRectangle(x, HEIGHT - y, width, height)

    def __draw_waypoint_on_map(self, waypoint: Waypoint) -> None:
        global PROTO_COUNTER
        waypoint_string = (
            self.__waypoint_template.getDef()
            + " "
            + self.__waypoint_template.exportString()
        )

        waypoint_string = re.sub(
            r"(.*) DEF (.*) Solid", f"DEF \\2_{PROTO_COUNTER} Solid", waypoint_string
        )
        # changing the translation of the waypoint
        waypoint_string = re.sub(
            r"translation .*",
            f"translation {waypoint.x} {waypoint.y} 0",
            waypoint_string,
        )

        # changing the name of the waypoint
        waypoint_string = re.sub(
            r'name ".*"', f'name "{waypoint.name}"', waypoint_string
        )

        # changing the color of the start and goal waypoints
        # waypoint_string = re.sub(r".*(Solid.*)", r"\1", waypoint_string)
        if waypoint.name == "goal":  # color it green
            waypoint_string = re.sub(
                r"baseColor .*", "baseColor 0 1 0", waypoint_string
            )
        elif waypoint.name == "start":  # color it red
            waypoint_string = re.sub(
                r"baseColor .*", "baseColor 1 0 0", waypoint_string
            )
        # import the modified waypoint string
        self.__root_children.importMFNodeFromString(-1, waypoint_string)
        self.__generated_nodes.append(
            self.__supervisor.getFromDef(f"WAYPOINT_TEMPLATE_{PROTO_COUNTER}")
        )
        PROTO_COUNTER += 1

    def __draw_waypoint_on_display(self, waypoint: Waypoint) -> None:
        x, y = self.__map_to_display(waypoint.x, waypoint.y)
        self.__display.fillOval(x, HEIGHT - y, 3, 3)

    def __draw_path_segment_on_map(
        self, waypoint_1, waypoint_2, *, is_path=True, thickness=0.01
    ):
        global PROTO_COUNTER
        # finding the vector from waypoint_a to waypoint_b
        delta_x = waypoint_2.x - waypoint_1.x
        delta_y = waypoint_2.y - waypoint_1.y

        length = sqrt(delta_x**2 + delta_y**2)
        angle = atan2(delta_y, delta_x)

        mid_x = (waypoint_1.x + waypoint_2.x) / 2
        mid_y = (waypoint_1.y + waypoint_2.y) / 2

        color = "1 1 1" if is_path else "0 0 0"

        if length <= 1e-4:
            length = 1e-4

        # VRML string for the cube
        cube_string = f"""
        DEF EDGE_TEMPLATE_{PROTO_COUNTER} Transform {{
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
        self.__generated_nodes.append(
            self.__supervisor.getFromDef(f"EDGE_TEMPLATE_{PROTO_COUNTER}")
        )
        PROTO_COUNTER += 1

    def __draw_path_segment_on_display(self, waypoint_1, waypoint_2):
        x1, y1 = self.__map_to_display(waypoint_1.x, waypoint_1.y)
        x2, y2 = self.__map_to_display(waypoint_2.x, waypoint_2.y)
        self.__display.drawLine(x1, HEIGHT - y1, x2, HEIGHT - y2)

    def __clear_generated_nodes(self):
        for node in self.__generated_nodes:
            if node is not None:
                node.remove()
        self.__generated_nodes = []

    def __draw_detected_objects(self, location: Tuple, final=False):
        x, y = location
        self.__display.setColor(0xFF00FF)
        self.__display.setOpacity(0.2)
        if final:
            self.__display.setOpacity(1)
        self.__display.fillOval(x, HEIGHT - y, 7, 7)

    def __refresh_display(self, final=False):
        """Redraws the background, obstacles, and detected objects on the display."""
        self.__display.setOpacity(1)
        self.__display.setColor(BACKGROUND_COLOR)
        self.__display.fillRectangle(0, 0, WIDTH, HEIGHT)
        self.draw_rectangular_obstacles()
        for location in self.__detected_objects:
            self.__draw_detected_objects(location, final)

    def draw_robot(self):
        self.__display.setColor(0xFF0000)
        self.__display.setOpacity(0.15)
        x, y = self.__robot.get_current_position()
        x, y = self.__map_to_display(x, y)
        self.__display.fillOval(x, HEIGHT - y, 1, 1)

    def draw_detected_objects_on_display(self, location: Tuple):
        distance_to_goal, goal_angle = location
        robot_x, robot_y = self.__robot.get_current_position()
        robot_angle = self.__robot.get_current_angle() - goal_angle / 2

        x = robot_x + distance_to_goal * np.cos(robot_angle)
        y = robot_y + distance_to_goal * np.sin(robot_angle)

        x, y = self.__map_to_display(x, y)

        self.__detected_objects.append((x, y))
        self.__draw_detected_objects((x, y))

    def draw_rectangular_obstacles(self) -> None:
        self.__display.setColor(0x0000FF)
        for obstacle in self.__obstacle_map:
            self.__draw_obstacle_on_display(obstacle)

    def draw_path_on_map(self, path: Path) -> None:
        self.__clear_generated_nodes()
        start = path[0]
        self.__draw_waypoint_on_map(start)
        for node in path[1:]:
            self.__draw_waypoint_on_map(node)
            self.__draw_path_segment_on_map(start, node)
            start = node

    def draw_path_on_display(self, path: Path) -> None:
        self.__refresh_display()
        self.__display.setOpacity(1)
        self.__display.setColor(0x00FF00)
        start = path[0]
        self.__draw_waypoint_on_display(start)
        for node in path[1:]:
            self.__draw_waypoint_on_display(node)
            self.__draw_path_segment_on_display(start, node)
            start = node

    def finetune_detected_objects(self):
        self.__detected_objects = np.array(self.__detected_objects)
        db = DBSCAN(eps=30, min_samples=2).fit(self.__detected_objects)
        centroids = []
        for label in np.unique(db.labels_):
            if label == -1:  # ignore noise points
                continue
            cluster = self.__detected_objects[db.labels_ == label]
            centroid = np.mean(cluster, axis=0)
            centroids.append(centroid)
        self.__detected_objects = centroids
        self.__refresh_display(final=True)
        return list(
            map(lambda centroid: Waypoint(*self.__display_to_map(*centroid)), centroids)
        )
