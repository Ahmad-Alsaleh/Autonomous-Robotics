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

    def __init__(self, adjacency_graph: Dict[Waypoint, List[Waypoint]], start: Waypoint, goal: Waypoint) -> None:
        # appending the cost to each each neighbor to the neighbor
        # i.e.: {waypoint_1: [neighbor_1, neighbor_2]} becomes
        # {waypoint_1: [(neighbor_1, cost_1), (neighbor_2, cost_2)]}
        self.__start = start
        self.__goal = goal
        closest_to_start = (None, float("inf"))
        closest_to_goal = (None, float("inf"))
        self.__adjacency_graph: Dict[Waypoint, List[Tuple[Waypoint, float]]] = dict()
        for waypoint, neighbors in adjacency_graph.items():
            if (dist := _euclidean_distance(waypoint, self.__start)) < closest_to_start[1]:
                closest_to_start = (waypoint, dist)
            if (dist := _euclidean_distance(waypoint, self.__goal)) < closest_to_goal[1]:
                closest_to_goal = (waypoint, dist)
            self.__adjacency_graph[waypoint] = [
                (neighbor, _euclidean_distance(waypoint, neighbor))
                for neighbor in neighbors
            ]
        self.__adjacency_graph[start] = [closest_to_start]
        self.__adjacency_graph[goal] = [closest_to_goal]
        self.__adjacency_graph[closest_to_start[0]].append((self.__start, closest_to_start[1]))
        self.__adjacency_graph[closest_to_goal[0]].append((self.__goal, closest_to_goal[1]))

    def get_neighbors(self, waypoint: Waypoint) -> List[Waypoint]:
        """Returns a list of neighbors for a given waypoint."""
        return self.__adjacency_graph[waypoint]

    def get_heuristic(self, current: Waypoint, goal) -> float:
        """Returns the euclidean distance from the current waypoint to the goal."""
        return _euclidean_distance(current, goal)

    def get_start(self) -> Waypoint:
        return self.__start
    
    def get_goal(self) -> Waypoint:
        return self.__goal

def find_path(graph: Graph) -> Path | None:
    """Returns the path found by A-Star."""
    start = graph.get_start()
    goal = graph.get_goal()
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
    width = 1.12 / 33
    height = 1.12 / 30

    start = Waypoint(13 * width, 1.12 - 3 * height, "start")
    goal = Waypoint(1.12 - 10 * width, 3 * height, "goal")
    p1 = Waypoint(4 * width, 1.12 - 3 * height, "p1")
    p2 = Waypoint(10 * width, 1.12 - 2 * height, "p2")
    p3 = Waypoint(5 * width, 1.12 - 9 * height, "p3")
    p4 = Waypoint(15 * width, 1.12 - 9 * height, "p4")
    p5 = Waypoint(25 * width, 1.12 - 4 * height, "p5")
    p6 = Waypoint(25 * width, 1.12 - 9 * height, "p6")
    p7 = Waypoint(31 * width, 1.12 - 9 * height, "p7")
    p8 = Waypoint(31 * width, 1.12 - 17 * height, "p8")
    p9 = Waypoint(30 * width, 1.12 - 28 * height, "p9")
    p10 = Waypoint(20 * width, 1.12 - 26 * height, "p10")
    p12 = Waypoint(21 * width, 1.12 - 19 * height, "p12")
    p13 = Waypoint(4 * width, 1.12 - 26 * height, "p13")
    p14 = Waypoint(3 * width, 1.12 - 21 * height, "p14")
    p15 = Waypoint(3 * width, 1.12 - 13 * height, "p15")
    p16 = Waypoint(10 * width, 1.12 - 17 * height, "p16")
    p17 = Waypoint(14 * width, 1.12 - 14 * height, "p17")

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
    graph = Graph(graph, start=start, goal=goal)
    path = find_path(graph)
    print(path)
