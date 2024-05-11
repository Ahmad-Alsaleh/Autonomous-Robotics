from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple, Callable
import numpy as np


class PathTraversalCompleted(Exception):
    pass


class PathDoesNotExist(Exception):
    pass


@dataclass(frozen=True)
class Point:
    x: float
    y: float
    name: str | None = None

    def __repr__(self) -> str:
        name = self.name if self.name is not None else "POINT"
        return f"{name}({self.x:.3f}, {self.y:.3f})"

    def to_numpy(self) -> np.ndarray:
        return np.array([self.x, self.y])


@dataclass
class Rectangle:
    top_left: Point
    bottom_right: Point


@dataclass
class Line:
    start: Point
    end: Point


# type aliases
Waypoint = Point
Neighbors = List[Waypoint]
NeighborsWithCosts = List[Tuple[Waypoint, float]]
Path = List[Waypoint]


class ObstaclesMap:
    def __init__(self, rectangular_obstacles: List[Rectangle]) -> None:
        """Representation of the rectangular obstacles on the map

        Args:
            obstacles (List[Rectangle]): a list of rectangular obstacles.
        """
        self.__obstacles: List[Tuple[Point, Point, Point, Point]] = []
        # storing the four points of the rectangle
        for rectangle in rectangular_obstacles:
            top_left = rectangle.top_left
            bottom_right = rectangle.bottom_right
            top_right = Point(bottom_right.x, top_left.y)
            bottom_left = Point(top_left.x, bottom_right.y)
            self.__obstacles.append((top_left, top_right, bottom_right, bottom_left))

    def __get_perpendicular_distance(self, point: Point, line: Line) -> float:
        A = point.x - line.start.x
        B = point.y - line.start.y
        C = line.end.x - line.start.x
        D = line.end.y - line.start.y

        dot = A * C + B * D
        len_sq = C * C + D * D
        param = -1
        if len_sq != 0:  # in case of 0 length line
            param = dot / len_sq

        if param < 0:
            xx, yy = line.start.x, line.start.y
        elif param > 1:
            xx, yy = line.end.x, line.end.y
        else:
            xx = line.start.x + param * C
            yy = line.start.y + param * D

        dx = point.x - xx
        dy = point.y - yy
        return np.sqrt(dx**2 + dy**2)

    def get_closest_obstacle_distance(self, point: Waypoint, _) -> float:
        """Calculates the shortest distance from the point to any edge of the obstacle rectangles."""
        min_distance = float("inf")
        for obstacle in self.__obstacles:
            # Calculate distance to each edge of the rectangle
            for i in range(4):
                start_vertex = obstacle[i]
                end_vertex = obstacle[(i + 1) % 4]
                line = Line(start_vertex, end_vertex)
                dist = self.__get_perpendicular_distance(point, line)
                if dist < min_distance:
                    min_distance = dist
        return 1.0 / min_distance if min_distance != float("inf") else 1e-6


class Graph:
    def __init__(
        self,
        adjacency_graph: Dict[Waypoint, Neighbors],
        start: Waypoint,
        goal: Waypoint,
        cost_function: Callable[[Waypoint, Waypoint], float],
        heuristic_function: Callable[[Waypoint, Waypoint], float],
    ) -> None:
        self.__start = start
        self.__goal = goal
        self.__heuristic_function = heuristic_function

        closest_to_start = (None, float("inf"))
        closest_to_goal = (None, float("inf"))
        self.__adjacency_graph: Dict[Waypoint, NeighborsWithCosts] = dict()
        for waypoint, neighbors in adjacency_graph.items():
            distance = euclidean_distance(waypoint, self.__start)
            if distance < closest_to_start[1]:
                closest_to_start = (waypoint, distance)

            distance = euclidean_distance(waypoint, self.__goal)
            if distance < closest_to_goal[1]:
                closest_to_goal = (waypoint, distance)

            # appending the cost to each each neighbor
            # i.e.: {waypoint_1: [neighbor_1, neighbor_2, ...], ...} becomes
            # {waypoint_1: [(neighbor_1, cost_1), (neighbor_2, cost_2), ...], ...}
            self.__adjacency_graph[waypoint] = [
                (neighbor, cost_function(neighbor, waypoint)) for neighbor in neighbors
            ]

        # connecting the start and goal to the rest of the graph
        closest_to_start = closest_to_start[0]
        closest_to_goal = closest_to_goal[0]
        self.__adjacency_graph[self.__start] = [
            (closest_to_start, cost_function(closest_to_start, self.__start))
        ]
        self.__adjacency_graph[self.__goal] = [
            (closest_to_goal, cost_function(closest_to_goal, self.__goal))
        ]
        self.__adjacency_graph[closest_to_start].append(
            (self.__start, cost_function(self.__start, closest_to_start))
        )
        self.__adjacency_graph[closest_to_goal].append(
            (self.__goal, cost_function(self.__goal, closest_to_goal))
        )

    def get_neighbors(self, waypoint: Waypoint) -> Neighbors:
        """Returns a list of neighbors for a given waypoint."""
        return self.__adjacency_graph[waypoint]

    def get_adjacency_graph(self) -> Dict[Waypoint, NeighborsWithCosts]:
        return self.__adjacency_graph

    def get_heuristic(self, current: Waypoint, goal: Waypoint) -> float:
        """Calculates the heuristic value between the current waypoint and the goal."""
        return self.__heuristic_function(current, goal)

    def get_start(self) -> Waypoint:
        return self.__start

    def get_goal(self) -> Waypoint:
        return self.__goal


class DeliberativeLayer:
    def __init__(self, graph: Graph) -> None:
        self.__graph = graph
        self.__path = iter(a_star(self.__graph))

    def get_next_waypoint(self) -> Waypoint:
        try:
            return next(self.__path)
        except StopIteration:
            raise PathTraversalCompleted("The path has been traversed.")


def euclidean_distance(waypoint_1: Waypoint, waypoint_2: Waypoint) -> float:
    return np.linalg.norm(waypoint_1.to_numpy() - waypoint_2.to_numpy())


def a_star(graph: Graph) -> Path:
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
            raise PathDoesNotExist
        if n == goal:
            reconstruction_path = []
            while parents[n] != n:
                reconstruction_path.append(n)
                n = parents[n]
            reconstruction_path.append(start)
            reconstruction_path.reverse()
            return reconstruction_path
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
    raise PathDoesNotExist
