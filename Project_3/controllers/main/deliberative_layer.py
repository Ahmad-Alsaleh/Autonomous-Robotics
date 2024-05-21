from dataclasses import dataclass
from typing import List, Tuple
import numpy as np
import math
import matplotlib.pyplot as plt
import random


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

    def to_tuple(self) -> Tuple[float, float]:
        return (self.x, self.y)


@dataclass
class Rectangle:
    top_left: Point
    bottom_right: Point

    def to_list(self) -> List[float]:
        # return a list in the format [x_min, y_min, x_max, y_max]
        return [
            self.top_left.x,
            self.bottom_right.y,
            self.bottom_right.x,
            self.top_left.y,
        ]


@dataclass
class Line:
    start: Point
    end: Point


# type aliases
Waypoint = Point
Neighbors = List[Waypoint]
NeighborsWithCosts = List[Tuple[Waypoint, float]]


class Path(List[Waypoint]):
    def __repr__(self) -> str:
        return " --> ".join(map(str, self))


class ObstaclesMap:
    def __init__(self, rectangular_obstacles: List[Rectangle]) -> None:
        """Representation of the rectangular obstacles on the map"""
        self.__rectangle_obstacles = rectangular_obstacles
        self.__obstacles: List[Tuple[Point, Point, Point, Point]] = []
        # storing the four points of the rectangle
        for rectangle in rectangular_obstacles:
            top_left = rectangle.top_left
            bottom_right = rectangle.bottom_right
            top_right = Point(bottom_right.x, top_left.y)
            bottom_left = Point(top_left.x, bottom_right.y)
            self.__obstacles.append((top_left, top_right, bottom_right, bottom_left))

    def __iter__(self):
        return iter(self.__rectangle_obstacles)

    @staticmethod
    def __enlarge_obstacle(
        obstacle: Tuple[Point, Point, Point, Point], robot_radius: float
    ) -> Tuple[Point, Point, Point, Point]:
        """Enlarges the given obstacle by the robot radius."""
        top_left, _, bottom_right, _ = obstacle

        enlarged_top_left = Point(top_left.x - robot_radius, top_left.y + robot_radius)
        enlarged_bottom_right = Point(
            bottom_right.x + robot_radius, bottom_right.y - robot_radius
        )

        enlarged_top_right = Point(enlarged_bottom_right.x, enlarged_top_left.y)
        enlarged_bottom_left = Point(enlarged_top_left.x, enlarged_bottom_right.y)

        return (
            enlarged_top_left,
            enlarged_top_right,
            enlarged_bottom_right,
            enlarged_bottom_left,
        )

    def is_inside_obstacle(self, point: Point, robot_radius: float = 0) -> bool:
        """Checks if the point is inside any of the obstacles."""
        for obstacle in self.__obstacles:
            if self.__is_inside_obstacle(
                point, self.__enlarge_obstacle(obstacle, robot_radius)
            ):
                return True
        return False

    def __is_inside_obstacle(
        self, point: Point, obstacle: Tuple[Point, Point, Point, Point]
    ) -> bool:
        """Checks if the point is inside the given obstacle."""
        count = 0
        for i in range(len(obstacle)):
            a, b = obstacle[i], obstacle[(i + 1) % len(obstacle)]

            if ((point.y < a.y) != (point.y < b.y)) and point.x < (b.x - a.x) * (
                point.y - a.y
            ) / (b.y - a.y) + a.x:
                count += 1

        return count % 2 == 1

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


class RRT:
    """
    Class for RRT planning
    """

    class Node:
        """
        RRT Node
        """

        def __init__(self, x, y):
            self.x = x
            self.y = y
            self.path_x = []
            self.path_y = []
            self.parent = None

    class AreaBounds:

        def __init__(self, area):
            self.xmin = float(area[0])
            self.xmax = float(area[1])
            self.ymin = float(area[2])
            self.ymax = float(area[3])

    def __init__(
        self,
        start: Tuple[float, float],
        goal: Tuple[float, float],
        obstacle_list: List[Rectangle],
        rand_area,
        expand_dis=3.0,
        path_resolution=0.5,
        goal_sample_rate=5,
        max_iter=500,
        play_area=None,
        robot_radius=0.0,
    ):
        """
        Setting Parameter

        start:Start Position [x,y]
        goal:Goal Position [x,y]
        obstacleList:obstacle Positions [[x,y,size],...]
        randArea:Random Sampling Area [min,max]
        play_area:stay inside this area [xmin,xmax,ymin,ymax]
        robot_radius: robot body modeled as circle with given radius

        """
        self.start = self.Node(start[0], start[1])
        self.end = self.Node(goal[0], goal[1])
        self.min_rand = rand_area[0]
        self.max_rand = rand_area[1]
        if play_area is not None:
            self.play_area = self.AreaBounds(play_area)
        else:
            self.play_area = None
        self.expand_dis = expand_dis
        self.path_resolution = path_resolution
        self.goal_sample_rate = goal_sample_rate
        self.max_iter = max_iter
        self.obstacle_list = obstacle_list
        self.node_list = []
        self.robot_radius = robot_radius

    def planning(self, animation=True) -> Path:
        """
        rrt path planning

        animation: flag for animation on or off
        """

        self.node_list = [self.start]
        for i in range(self.max_iter):
            rnd_node = self.get_random_node()
            nearest_ind = self.get_nearest_node_index(self.node_list, rnd_node)
            nearest_node = self.node_list[nearest_ind]

            new_node = self.steer(nearest_node, rnd_node, self.expand_dis)

            if self.check_if_outside_play_area(
                new_node, self.play_area
            ) and self.check_collision(new_node, self.obstacle_list, self.robot_radius):
                self.node_list.append(new_node)

            if animation and i % 5 == 0:
                self.draw_graph(rnd_node)

            if (
                self.calc_dist_to_goal(self.node_list[-1].x, self.node_list[-1].y)
                <= self.expand_dis
            ):
                final_node = self.steer(self.node_list[-1], self.end, self.expand_dis)
                if self.check_collision(
                    final_node, self.obstacle_list, self.robot_radius
                ):
                    return self.generate_final_course(len(self.node_list) - 1)

            if animation and i % 5:
                self.draw_graph(rnd_node)

        raise PathDoesNotExist  # cannot find path (all iterations consumes)

    def steer(self, from_node, to_node, extend_length=float("inf")):

        new_node = self.Node(from_node.x, from_node.y)
        d, theta = self.calc_distance_and_angle(new_node, to_node)

        new_node.path_x = [new_node.x]
        new_node.path_y = [new_node.y]

        if extend_length > d:
            extend_length = d

        n_expand = math.floor(extend_length / self.path_resolution)

        for _ in range(n_expand):
            new_node.x += self.path_resolution * math.cos(theta)
            new_node.y += self.path_resolution * math.sin(theta)
            new_node.path_x.append(new_node.x)
            new_node.path_y.append(new_node.y)

        d, _ = self.calc_distance_and_angle(new_node, to_node)
        if d <= self.path_resolution:
            new_node.path_x.append(to_node.x)
            new_node.path_y.append(to_node.y)
            new_node.x = to_node.x
            new_node.y = to_node.y

        new_node.parent = from_node
        return new_node

    def generate_final_course(self, goal_ind) -> Path:
        path = [Waypoint(self.end.x, self.end.y)]
        node = self.node_list[goal_ind]
        while node.parent is not None:
            path.append(Waypoint(node.x, node.y))
            node = node.parent
        path.append(Waypoint(node.x, node.y))
        path.reverse()
        return Path(path)

    def calc_dist_to_goal(self, x, y):
        dx = x - self.end.x
        dy = y - self.end.y
        return math.hypot(dx, dy)

    def get_random_node(self):
        if random.randint(0, 100) > self.goal_sample_rate:
            rnd = self.Node(
                random.uniform(self.min_rand, self.max_rand),
                random.uniform(self.min_rand, self.max_rand),
            )
        else:  # goal point sampling
            rnd = self.Node(self.end.x, self.end.y)
        return rnd

    def draw_graph(self, rnd=None):
        plt.clf()
        # for stopping simulation with the esc key.
        plt.gcf().canvas.mpl_connect(
            "key_release_event",
            lambda event: [exit(0) if event.key == "escape" else None],
        )
        if rnd is not None:
            plt.plot(rnd.x, rnd.y, "^r") # the random node
            if self.robot_radius > 0.0:
                self.plot_circle(rnd.x, rnd.y, self.robot_radius, "-r")
        for node in self.node_list:
            if node.parent:
                plt.plot(node.path_x, node.path_y, "-g")

        # for (ox, oy, size) in self.obstacle_list:
        #     self.plot_circle(ox, oy, size)
        for rectangle in self.obstacle_list:
            self.plot_rectangle(rectangle)

        if self.play_area is not None:
            plt.plot(
                [
                    self.play_area.xmin,
                    self.play_area.xmax,
                    self.play_area.xmax,
                    self.play_area.xmin,
                    self.play_area.xmin,
                ],
                [
                    self.play_area.ymin,
                    self.play_area.ymin,
                    self.play_area.ymax,
                    self.play_area.ymax,
                    self.play_area.ymin,
                ],
                "-k",
            )

        plt.plot(self.start.x, self.start.y, "or") # the start
        plt.plot(self.end.x, self.end.y, "xr") # the goal
        plt.axis("equal")
        plt.axis([self.min_rand, self.max_rand, self.min_rand, self.max_rand])
        plt.pause(0.01)

    @staticmethod
    def plot_rectangle(rect: Rectangle, color="b"):
        x_min, y_min, x_max, y_max = rect.to_list()
        rectangle = plt.Rectangle(
            (x_min, y_min), x_max - x_min, y_max - y_min, fill=True, edgecolor=color
        )
        plt.gca().add_patch(rectangle)

    @staticmethod
    def plot_circle(x, y, size, color="-b"):
        deg = list(range(0, 360, 5))
        deg.append(0)
        xl = [x + size * math.cos(np.deg2rad(d)) for d in deg]
        yl = [y + size * math.sin(np.deg2rad(d)) for d in deg]
        plt.plot(xl, yl, color)

    @staticmethod
    def get_nearest_node_index(node_list, rnd_node):
        dlist = [
            (node.x - rnd_node.x) ** 2 + (node.y - rnd_node.y) ** 2
            for node in node_list
        ]
        minind = dlist.index(min(dlist))

        return minind

    @staticmethod
    def check_if_outside_play_area(node, play_area):

        if play_area is None:
            return True  # no play_area was defined, every pos should be ok

        if (
            node.x < play_area.xmin
            or node.x > play_area.xmax
            or node.y < play_area.ymin
            or node.y > play_area.ymax
        ):
            return False  # outside - bad
        else:
            return True  # inside - ok

    @staticmethod
    def check_collision(
        node: Node, obstacle_list: List[Rectangle], robot_radius: float
    ):
        if node is None:
            return False

        for rectangle in obstacle_list:
            # Adjust the bounds of the rectangle by the robot's radius
            x_min, y_min, x_max, y_max = rectangle.to_list()
            x_min -= robot_radius
            y_min -= robot_radius
            x_max += robot_radius
            y_max += robot_radius

            for x, y in zip(node.path_x, node.path_y):
                if x_min <= x <= x_max and y_min <= y <= y_max:
                    return False

        return True

    @staticmethod
    def calc_distance_and_angle(from_node, to_node):
        dx = to_node.x - from_node.x
        dy = to_node.y - from_node.y
        d = math.hypot(dx, dy)
        theta = math.atan2(dy, dx)
        return d, theta


class RRTStar(RRT):
    """
    Class for RRT Star planning
    """

    class Node(RRT.Node):
        def __init__(self, x, y):
            super().__init__(x, y)
            self.cost = 0.0

    def __init__(
        self,
        start: Tuple[float, float],
        goal: Tuple[float, float],
        obstacle_list: List[Rectangle],
        rand_area,
        expand_dis=30.0,
        path_resolution=1.0,
        goal_sample_rate=20,
        max_iter=500,
        connect_circle_dist=50.0,
        search_until_max_iter=False,
        play_area=None,
        robot_radius=0.0,
    ):
        """
        Setting Parameter

        start:Start Position [x,y]
        goal:Goal Position [x,y]
        obstacleList:obstacle Positions [[x,y,size],...]
        randArea:Random Sampling Area [min,max]

        """
        super().__init__(
            start,
            goal,
            obstacle_list,
            rand_area,
            expand_dis,
            path_resolution,
            goal_sample_rate,
            max_iter,
            play_area=play_area,
            robot_radius=robot_radius,
        )
        self.connect_circle_dist = connect_circle_dist
        self.goal_node = self.Node(goal[0], goal[1])
        self.search_until_max_iter = search_until_max_iter
        self.node_list = []

    def planning(self, animation=True):
        """
        rrt star path planning

        animation: flag for animation on or off .
        """

        self.node_list = [self.start]
        for i in range(self.max_iter):
            # print("Iter:", i, ", number of nodes:", len(self.node_list))
            rnd = self.get_random_node()
            nearest_ind = self.get_nearest_node_index(self.node_list, rnd)
            new_node = self.steer(self.node_list[nearest_ind], rnd, self.expand_dis)
            near_node = self.node_list[nearest_ind]
            new_node.cost = near_node.cost + math.hypot(
                new_node.x - near_node.x, new_node.y - near_node.y
            )

            if self.check_if_outside_play_area(
                new_node, self.play_area
            ) and self.check_collision(new_node, self.obstacle_list, self.robot_radius):
                near_inds = self.find_near_nodes(new_node)
                node_with_updated_parent = self.choose_parent(new_node, near_inds)
                if node_with_updated_parent:
                    self.rewire(node_with_updated_parent, near_inds)
                    self.node_list.append(node_with_updated_parent)
                else:
                    self.node_list.append(new_node)

            if animation and i % 10 == 0:
                self.draw_graph(rnd)

            if (not self.search_until_max_iter) and new_node:  # if reaches goal
                last_index = self.search_best_goal_node()
                if last_index is not None:
                    return self.generate_final_course(last_index)

        print("reached max iteration")

        last_index = self.search_best_goal_node()
        if last_index is not None:
            return self.generate_final_course(last_index)

        raise PathDoesNotExist

    def choose_parent(self, new_node, near_inds):
        """
        Computes the cheapest point to new_node contained in the list
        near_inds and set such a node as the parent of new_node.
            Arguments:
            --------
                new_node, Node
                    randomly generated node with a path from its neared point
                    There are not coalitions between this node and th tree.
                near_inds: list
                    Indices of indices of the nodes what are near to new_node

            Returns.
            ------
                Node, a copy of new_node
        """
        if not near_inds:
            return None

        # search nearest cost in near_inds
        costs = []
        for i in near_inds:
            near_node = self.node_list[i]
            t_node = self.steer(near_node, new_node)
            if t_node and self.check_collision(
                t_node, self.obstacle_list, self.robot_radius
            ):
                costs.append(self.calc_new_cost(near_node, new_node))
            else:
                costs.append(float("inf"))  # the cost of collision node
        min_cost = min(costs)

        if min_cost == float("inf"):
            print("There is no good path.(min_cost is inf)")
            return None

        min_ind = near_inds[costs.index(min_cost)]
        new_node = self.steer(self.node_list[min_ind], new_node)
        new_node.cost = min_cost

        return new_node

    def search_best_goal_node(self):
        dist_to_goal_list = [self.calc_dist_to_goal(n.x, n.y) for n in self.node_list]
        goal_inds = [
            dist_to_goal_list.index(i)
            for i in dist_to_goal_list
            if i <= self.expand_dis
        ]

        safe_goal_inds = []
        for goal_ind in goal_inds:
            t_node = self.steer(self.node_list[goal_ind], self.goal_node)
            if self.check_collision(t_node, self.obstacle_list, self.robot_radius):
                safe_goal_inds.append(goal_ind)

        if not safe_goal_inds:
            return None

        safe_goal_costs = [
            self.node_list[i].cost
            + self.calc_dist_to_goal(self.node_list[i].x, self.node_list[i].y)
            for i in safe_goal_inds
        ]

        min_cost = min(safe_goal_costs)
        for i, cost in zip(safe_goal_inds, safe_goal_costs):
            if cost == min_cost:
                return i

        return None

    def find_near_nodes(self, new_node):
        """
        1) defines a ball centered on new_node
        2) Returns all nodes of the three that are inside this ball
            Arguments:
            ---------
                new_node: Node
                    new randomly generated node, without collisions between
                    its nearest node
            Returns:
            -------
                list
                    List with the indices of the nodes inside the ball of
                    radius r
        """
        nnode = len(self.node_list) + 1
        r = self.connect_circle_dist * math.sqrt(math.log(nnode) / nnode)
        # if expand_dist exists, search vertices in a range no more than
        # expand_dist
        if hasattr(self, "expand_dis"):
            r = min(r, self.expand_dis)
        dist_list = [
            (node.x - new_node.x) ** 2 + (node.y - new_node.y) ** 2
            for node in self.node_list
        ]
        near_inds = [dist_list.index(i) for i in dist_list if i <= r**2]
        return near_inds

    def rewire(self, new_node, near_inds):
        """
        For each node in near_inds, this will check if it is cheaper to
        arrive to them from new_node.
        In such a case, this will re-assign the parent of the nodes in
        near_inds to new_node.
        Parameters:
        ----------
            new_node, Node
                Node randomly added which can be joined to the tree

            near_inds, list of uints
                A list of indices of the self.new_node which contains
                nodes within a circle of a given radius.
        Remark: parent is designated in choose_parent.

        """
        for i in near_inds:
            near_node = self.node_list[i]
            edge_node = self.steer(new_node, near_node)
            if not edge_node:
                continue
            edge_node.cost = self.calc_new_cost(new_node, near_node)

            no_collision = self.check_collision(
                edge_node, self.obstacle_list, self.robot_radius
            )
            improved_cost = near_node.cost > edge_node.cost

            if no_collision and improved_cost:
                for node in self.node_list:
                    if node.parent == self.node_list[i]:
                        node.parent = edge_node
                self.node_list[i] = edge_node
                self.propagate_cost_to_leaves(self.node_list[i])

    def calc_new_cost(self, from_node, to_node):
        d, _ = self.calc_distance_and_angle(from_node, to_node)
        return from_node.cost + d

    def propagate_cost_to_leaves(self, parent_node):

        for node in self.node_list:
            if node.parent == parent_node:
                node.cost = self.calc_new_cost(parent_node, node)
                self.propagate_cost_to_leaves(node)


class DeliberativeLayer:
    ROBOT_RADIUS = 0.0355

    def __init__(
        self,
        obstacle_map: ObstaclesMap,
        rand_area=(0, 1.14),
        path_resolution=0.001,
        expand_dis=0.05,
        play_area=(0, 1.12, 0, 1.12),
        max_iter=1000,
    ) -> None:
        self.__obstacle_map = obstacle_map
        self.__rand_area = rand_area
        self.__path_resolution = path_resolution
        self.__expand_dis = expand_dis
        self.__play_area = play_area
        self.__max_iter = max_iter
        self.__path = None
        self.__path_iterator = None

    def is_inside_obstacle(self, point: Tuple[float, float]) -> bool:
        return self.__obstacle_map.is_inside_obstacle(
            Waypoint(*point), robot_radius=self.ROBOT_RADIUS
        )

    def generate_path(
        self,
        start: Tuple[float, float],
        goal: Tuple[float, float],
        show_animation=False,
    ):
        """Raises PathDoesNotExist if the path does not exist."""
        rrt_star = RRTStar(
            start=start,
            goal=goal,
            rand_area=self.__rand_area,
            obstacle_list=self.__obstacle_map,
            path_resolution=self.__path_resolution,
            expand_dis=self.__expand_dis,
            play_area=self.__play_area,
            robot_radius=self.ROBOT_RADIUS,
            max_iter=self.__max_iter,
        )
        self.__path = rrt_star.planning(animation=show_animation)
        self.__path_iterator = iter(self.__path)
        if show_animation: # plot the final path
            plt.plot(
                [point.x for point in self.__path],
                [point.y for point in self.__path],
                "-r",
                linewidth=2.5,
            )
            plt.pause(0.01)

    def get_path(self) -> Path:
        return self.__path

    def get_next_waypoint(self) -> Waypoint:
        try:
            return next(self.__path_iterator)
        except StopIteration:
            self.__path = None
            self.__path_iterator = None
            raise PathTraversalCompleted("The path has been traversed.")
