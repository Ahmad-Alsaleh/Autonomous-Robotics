from dataclasses import dataclass
from typing import Dict, List, Tuple
import numpy as np


@dataclass(frozen=True)
class Waypoint:
    x: int
    y: int
    name: int | None = None

    def __repr__(self) -> str:
        return f"{self.name if self.name is not None else 'WAY_POINT'}({self.x:.3f}, {self.y:.3f})"

    def to_numpy(self) -> np.ndarray:
        return np.array([self.x, self.y])


class Path(list):
    def __repr__(self) -> str:
        return " --> ".join(map(str, self))


def _euclidean_distance(a: Waypoint, b: Waypoint):
    return np.linalg.norm(a.to_numpy() - b.to_numpy())


class Graph:
    """
    - Defines a graph of waypoints.
    - Uses euclidean distance as the heuristic.
    - Uses the distance traveled as the cost.
    """

    def __init__(self, adjacency_graph: Dict[Waypoint, List[Waypoint]]) -> None:
        # appending the cost to each each neighbor to the neighbor
        # i.e.: {waypoint_1: [neighbor_1, neighbor_2]} becomes
        # {waypoint_1: [(neighbor_1, cost_1), (neighbor_2, cost_2)]}
        self.__adjacency_graph: Dict[Waypoint, List[Tuple[Waypoint, float]]] = dict()
        for waypoint, neighbors in adjacency_graph.items():
            self.__adjacency_graph[waypoint] = [
                (neighbor, _euclidean_distance(waypoint, neighbor))
                for neighbor in neighbors
            ]

    def get_neighbors(self, waypoint: Waypoint) -> List[Waypoint]:
        """Returns a list of neighbors for a given waypoint."""
        return self.__adjacency_graph[waypoint]

    def get_heuristic(self, current: Waypoint, goal) -> float:
        """Returns the euclidean distance from the current waypoint to the goal."""
        return _euclidean_distance(current, goal)


def find_path(graph: Graph, *, start: Waypoint, goal: Waypoint) -> Path | None:
    """Returns the path found by A-Star."""
    open_list = set([start])
    closed_list = set([])
    g = {start: 0}
    parents = {start: start}
    while len(open_list) > 0:
        n = None
        for v in open_list:
            if n == None or g[v] + graph.get_heuristic(v, goal) < g[
                n
            ] + graph.get_heuristic(n, goal):
                n = v
        if n == None:
            print("==========Path does not exist!!==========")
            return None
        if n == goal:
            reconstruction_path = []
            while parents[n] != n:
                reconstruction_path.append(n)
                n = parents[n]
            reconstruction_path.append(start)
            reconstruction_path.reverse()
            return Path(reconstruction_path)
        for m, weight in graph.get_neighbors(n):
            if m not in open_list and m not in closed_list:
                open_list.add(m)
                parents[m] = n
                g[m] = g[n] + weight
            else:
                if g[m] > g[n] + weight:
                    g[m] = g[n] + weight
                    parents[m] = n
                    if m in closed_list:
                        closed_list.remove(m)
                        open_list.add(m)
        open_list.remove(n)
        closed_list.add(n)
    print("==========Path does not exist!!==========")
    return None


# a simple example
if __name__ == "__main__":
    p0 = Waypoint(2, 2, "p0")
    p1 = Waypoint(4, 3, "p1")
    p2 = Waypoint(10, 2, "p2")
    p3 = Waypoint(5, 9, "p3")
    p4 = Waypoint(15, 9, "p4")
    p5 = Waypoint(25, 4, "p5")
    p6 = Waypoint(25, 9, "p6")
    p7 = Waypoint(31, 9, "p7")
    p8 = Waypoint(31, 17, "p8")
    p9 = Waypoint(30, 28, "p9")
    p10 = Waypoint(20, 26, "p10")
    p11 = Waypoint(13, 16, "p11")
    p12 = Waypoint(21, 19, "p12")
    p13 = Waypoint(4, 26, "p13")
    p14 = Waypoint(3, 21, "p14")
    p15 = Waypoint(3, 13, "p15")
    p16 = Waypoint(10, 17, "p16")
    p17 = Waypoint(14, 14, "p17")

    graph = {
        p0: [p1],
        p1: [p0, p2, p3],
        p2: [p1],
        p3: [p1, p4, p15],
        p4: [p3, p6, p17],
        p5: [p6],
        p6: [p4, p5, p7],
        p7: [p6, p8],
        p8: [p7, p9],
        p9: [p8, p10],
        p10: [p9, p12, p13],
        p11: [p16, p17],
        p12: [p10, p17],
        p13: [p10, p14],
        p14: [p13, p15],
        p15: [p3, p14],
        p16: [p11, p17],
        p17: [p4, p11, p12, p16],
    }

    graph = Graph(graph)
    path = find_path(graph, start=p0, goal=p11)
    print(path)
