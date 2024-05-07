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


class ObstacleMap:
    def __init__(self, points: List[List[Waypoint]]):
        """representation of the obstacles on the map

        Args:
            points (List[List[Tuple[int, int]]]): a list containing lists of 4 waypoints of
            the rectangle obstacle (top left, top right, bottom right, bottom left)
        """
        self.__obstacles = []
        for top_left, bottom_right in points:
            top_right = Waypoint(bottom_right.x, top_left.y)
            bottom_left = Waypoint(top_left.x, bottom_right.y)
            # Store the full rectangle
            self.__obstacles.append([top_left, top_right, bottom_right, bottom_left])

    def __perpendicular_distance(self, x, y, x1, y1, x2, y2):
        A = x - x1
        B = y - y1
        C = x2 - x1
        D = y2 - y1

        dot = A * C + B * D
        len_sq = C * C + D * D
        param = -1
        if len_sq != 0:  # in case of 0 length line
            param = dot / len_sq

        if param < 0:
            xx, yy = x1, y1
        elif param > 1:
            xx, yy = x2, y2
        else:
            xx = x1 + param * C
            yy = y1 + param * D

        dx = x - xx
        dy = y - yy
        return np.sqrt(dx**2 + dy**2)

    def get_closest_obstacle_distance(self, point: Waypoint) -> float:
        """Calculates the shortest distance from the point to any edge of the obstacle rectangles."""
        point_np = point.to_numpy()
        x, y = point_np
        min_distance = float("inf")
        for obstacle in self.__obstacles:
            # Calculate distance to each edge of the rectangle
            for i in range(4):
                start_vertex = obstacle[i]
                end_vertex = obstacle[(i + 1) % 4]
                dist = self.__perpendicular_distance(
                    x, y, start_vertex.x, start_vertex.y, end_vertex.x, end_vertex.y
                )
                if dist < min_distance:
                    min_distance = dist
        return min_distance if min_distance != float("inf") else 1e-6


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
        obstacle_map: ObstacleMap,
    ) -> None:
        # appending the cost to each each neighbor to the neighbor
        # i.e.: {waypoint_1: [neighbor_1, neighbor_2]} becomes
        # {waypoint_1: [(neighbor_1, cost_1), (neighbor_2, cost_2)]}
        self.__start = start
        self.__goal = goal
        self.__obstacle_map = obstacle_map
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
        self.__add_obstacle_cost()

    def __add_obstacle_cost(self):
        """adds (closest) obstacle distance to the cost of each edge
           and disregards the direct cost.
        """
        for waypoint, edges in self._adjacency_graph.items():
            updated_edges = []
            for neighbor, direct_cost in edges:
                obstacle_distance = self.__obstacle_map.get_closest_obstacle_distance(
                    waypoint
                )
                total_cost = 1. / obstacle_distance
                updated_edges.append((neighbor, total_cost))
            self._adjacency_graph[waypoint] = updated_edges

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
