from controller import Supervisor
import os, sys, re
from math import atan2, sqrt

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from main.deliberative_layer import Graph, Waypoint, DeliberativeLayer, Path, a_star
from main.main import graph


class Supervisor(Supervisor):
    def __init__(self, graph: Graph, path: Path) -> None:
        super().__init__()
        self.__graph = graph
        self.__path = path
        self.__root_children = self.getRoot().getField("children")
        self.__epuck = self.getFromDef("EPUCK")
        self.__waypoint_template = self.getFromDef("WAYPOINT_TEMPLATE")

    def __draw_waypoint(self, waypoint: Waypoint):
        waypoint_string = (
            self.__waypoint_template.getDef()
            + " "
            + self.__waypoint_template.exportString()
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
        waypoint_string = re.sub(r".*(Solid.*)", r"\1", waypoint_string)
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

        color = "0 1 0" if is_path else "0 0 0"

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

    def render_graph(self):
        for waypoint, neighbors in self.__graph.get_adjacency_graph().items():
            self.__draw_waypoint(waypoint)
            for neighbor, _ in neighbors:
                try:
                    is_path = (
                        self.__path.index(neighbor) == self.__path.index(waypoint) + 1
                    )
                except ValueError:
                    is_path = False
                self.__draw_path_segment(waypoint, neighbor, is_path=is_path)

    def set_robot_initial_position(self):
        start_position = [*self.__graph.get_start().to_numpy(), 0]
        self.__epuck.getField("translation").setSFVec3f(start_position)


if __name__ == "__main__":
    path = a_star(graph)
    supervisor = Supervisor(graph, path)
    supervisor.render_graph()
    supervisor.set_robot_initial_position()
