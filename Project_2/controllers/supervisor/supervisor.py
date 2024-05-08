from controller import Supervisor
import os, sys, re

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from main.path_finder import Graph, Waypoint, DeliberativeLayer
from main.main import graph


class Supervisor(Supervisor):
    def __init__(
        self, graph: Graph, path: list[Waypoint]
    ) -> None:
        super().__init__()
        self.__time_step = int(self.getBasicTimeStep())
        self.__root_children = self.getRoot().getField("children")
        self._epuck = self.getFromDef("EPUCK")
        self.__node_template = self.getFromDef("NODE_TEMPLATE")
        self.__graph = graph
        self.__path = path

    def simulator_step(self) -> int:
        """Runs a single step in the simulator."""
        return self.step(self.__time_step)

    def __insert_node(self, node: Waypoint):
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

        color = "0 1 0" if is_path else "0 0 0"

        # VRML string for the cube
        cube_string = f"""
        Transform {{
            translation {mid_x} {mid_y} 0
            rotation 0 0 1 {angle}  # Rotate to align along the vector between nodes
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


dl = DeliberativeLayer(graph)
dl.generate_path()
path = list(dl._path)
supervisor = Supervisor(graph, path)
supervisor.render_graph()

pos_field = supervisor._epuck.getField("translation")
new_pos = [*graph.get_start().to_numpy(), 0]
pos_field.setSFVec3f(new_pos)
