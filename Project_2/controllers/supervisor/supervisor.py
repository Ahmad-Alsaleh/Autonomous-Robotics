from controller import Supervisor
import sys
import re

sys.path.append("..")
from main import path_finder


class Supervisor(Supervisor):
    def __init__(
        self, graph: path_finder.Graph, path: list[path_finder.Waypoint]
    ) -> None:
        super().__init__()
        self.__time_step = int(self.getBasicTimeStep())
        self.__root_children = self.getRoot().getField("children")
        self.__node_template = self.getFromDef("NODE_TEMPLATE")
        self.__graph = graph
        self.__path = path

    def simulator_step(self) -> int:
        """Runs a single step in the simulator."""
        return self.step(self.__time_step)

    def __insert_node(self, node: path_finder.Waypoint):
        template_string = (
            self.__node_template.getDef() + " " + self.__node_template.exportString()
        )
        new_node_string = re.sub(
            r"translation .*", f"translation {node.x} {node.y} 0", template_string
        )
        new_node_string = re.sub(r'name ".*"', f'name "{node.name}"', new_node_string)
        new_node_string = re.sub(r".*(Solid.*)", r"\1", new_node_string)
        if node.name == "goal":
            # color it green
            new_node_string = re.sub(
                r"baseColor .*", "baseColor 0 1 0", new_node_string
            )
        elif node.name == "start":
            # color it yellow
            new_node_string = re.sub(
                r"baseColor .*", "baseColor 1 1 0", new_node_string
            )

        # Import the modified node string
        self.__root_children.importMFNodeFromString(-1, new_node_string)

    def __insert_edge(self, node_a, node_b, *, is_path=True, thickness=0.01):
        from math import atan2, sqrt

        # Vector from node_a to node_b
        dx = node_b.x - node_a.x
        dy = node_b.y - node_a.y

        length = sqrt(dx**2 + dy**2)

        mid_x = (node_a.x + node_b.x) / 2
        mid_y = (node_a.y + node_b.y) / 2

        angle = atan2(dy, dx)

        # VRML string for the cube
        cube_string = f"""
        Transform {{
            translation {mid_x} {mid_y} 0
            rotation 0 0 1 {angle}  # Rotate to align along the vector between nodes
            children [Shape {{
                appearance MattePaint {{
                    baseColor {0 if is_path else 1} 1 0
                }}
                geometry Box {{
                    size {length} {thickness} {thickness}  # Length, height, and width of the box
                }}
            }}]
        }}
        """
        print(cube_string)
        self.__root_children.importMFNodeFromString(-1, cube_string)

    def render_graph(self):
        for node, neighbors in self.__graph._adjacency_graph.items():
            neighbors = [neighbor[0] for neighbor in neighbors]

            self.__insert_node(node)
            for neighbor in neighbors:
                is_path = False
                try:
                    n1 = self.__path.index(node)
                    n2 = self.__path.index(neighbor)
                    is_path = n2 == n1 + 1
                except ValueError:
                    pass
                self.__insert_edge(node, neighbor, is_path=is_path)



width = 1.12 / 33
height = 1.12 / 30
test_num = 4
tests = {
    "test1": {
        "start": (2 * width, 1.12 - 2 * height),
        "goal": (13 * width, 1.12 - 16 * height),
    },
    "test2": {
        "start": (22 * width, 1.12 - 3 * height),
        "goal": (16 * width, 3 * height),
    },
    "test3": {
        "start": (2 * width, 13 * height),
        "goal": (1.12 - 5 * width, 10 * height),
    },
    "test4": {
        "start": (13 * width, 1.12 - 3 * height),
        "goal": (1.12 - 10 * width, 3 * height),
    },
}
start = path_finder.Waypoint(*tests[f"test{test_num}"]["start"], "start")
goal = path_finder.Waypoint(*tests[f"test{test_num}"]["goal"], "goal")
p1 = path_finder.Waypoint(5 * width, 1.12 - 4 * height, "p1")
p2 = path_finder.Waypoint(11 * width, 1.12 - 2 * height, "p2")
p3 = path_finder.Waypoint(5 * width, 1.12 - 9 * height, "p3")
p4 = path_finder.Waypoint(15 * width, 1.12 - 9 * height, "p4")
p5 = path_finder.Waypoint(25 * width, 1.12 - 4 * height, "p5")
p6 = path_finder.Waypoint(25 * width, 1.12 - 9 * height, "p6")
p7 = path_finder.Waypoint(31 * width, 1.12 - 9 * height, "p7")
p8 = path_finder.Waypoint(31 * width, 1.12 - 17 * height, "p8")
p9 = path_finder.Waypoint(30 * width, 1.12 - 28 * height, "p9")
p10 = path_finder.Waypoint(20 * width, 1.12 - 26 * height, "p10")
p12 = path_finder.Waypoint(21 * width, 1.12 - 19 * height, "p12")
p13 = path_finder.Waypoint(4 * width, 1.12 - 26 * height, "p13")
p14 = path_finder.Waypoint(3 * width, 1.12 - 21 * height, "p14")
p15 = path_finder.Waypoint(3 * width, 1.12 - 13 * height, "p15")
p16 = path_finder.Waypoint(10 * width, 1.12 - 17 * height, "p16")
p17 = path_finder.Waypoint(15 * width, 1.12 - 14 * height, "p17")


graph = {
    p1: [p2, p3],
    p2: [p1],
    p3: [p1, p4, p15],
    p4: [p3, p6, p17],
    p5: [p6],
    p6: [p4, p5, p7],
    p7: [p6, p8],
    p8: [p7, p9],
    p9: [p8, p10],
    p10: [p9, p12, p13],
    p12: [p10, p17],
    p13: [p10, p14],
    p14: [p13, p15],
    p15: [p3, p14],
    p16: [p17],
    p17: [p4, p12, p16],
}
graph = path_finder.Graph(graph, start=start, goal=goal)

dl = path_finder.DeliberativeLayer(graph)
dl.generate_path()
path = list(dl._path)
supervisor = Supervisor(graph, path)
supervisor.render_graph()
