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

    def __init__(
        self,
        adjacency_graph: Dict[Waypoint, List[Waypoint]],
        start: Waypoint,
        goal: Waypoint,
    ) -> None:
        # appending the cost to each each neighbor to the neighbor
        # i.e.: {waypoint_1: [neighbor_1, neighbor_2]} becomes
        # {waypoint_1: [(neighbor_1, cost_1), (neighbor_2, cost_2)]}
        self.__start = start
        self.__goal = goal
        closest_to_start = (None, float("inf"))
        closest_to_goal = (None, float("inf"))
        self._adjacency_graph: Dict[Waypoint, List[Tuple[Waypoint, float]]] = dict()
        for waypoint, neighbors in adjacency_graph.items():
            if (dist := _euclidean_distance(waypoint, self.__start)) < closest_to_start[
                1
            ]:
                closest_to_start = (waypoint, dist)
            if (dist := _euclidean_distance(waypoint, self.__goal)) < closest_to_goal[
                1
            ]:
                closest_to_goal = (waypoint, dist)
            self._adjacency_graph[waypoint] = [
                (neighbor, _euclidean_distance(waypoint, neighbor))
                for neighbor in neighbors
            ]
        self._adjacency_graph[start] = [closest_to_start]
        self._adjacency_graph[goal] = [closest_to_goal]
        self._adjacency_graph[closest_to_start[0]].append(
            (self.__start, closest_to_start[1])
        )
        self._adjacency_graph[closest_to_goal[0]].append(
            (self.__goal, closest_to_goal[1])
        )

    def get_neighbors(self, waypoint: Waypoint) -> List[Waypoint]:
        """Returns a list of neighbors for a given waypoint."""
        return self._adjacency_graph[waypoint]

    def get_heuristic(self, current: Waypoint, goal) -> float:
        """Returns the euclidean distance from the current waypoint to the goal."""
        return _euclidean_distance(current, goal)

    def get_start(self) -> Waypoint:
        return self.__start

    def get_goal(self) -> Waypoint:
        return self.__goal


class PathTraversalCompleted(Exception):
    pass


class DeliberativeLayer:

    def __init__(self, graph: Graph) -> None:
        self.__graph = graph
        self._path = None

    def get_goal(self) -> Waypoint:
        return self.__graph.get_goal()

    def generate_path(self) -> Path | None:
        self._path = iter(DeliberativeLayer.find_path(self.__graph))

    def get_next_waypoint(self) -> Waypoint:
        if self._path is None:
            self._path = iter([])
            return self.__graph.get_goal()

        try:
            return next(self._path)
        except StopIteration:
            raise PathTraversalCompleted("The path has been traversed.")

    @staticmethod
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
