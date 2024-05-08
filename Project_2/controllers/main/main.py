import numpy as np
from controller import Robot
from dataclasses import dataclass
from typing import Dict, List, Tuple, Callable

class Robot(Robot):
    MAX_SPEED = 6.2
    RELATIVE_ANGLES_OF_DISTANCE_SENSORS = np.array(
        [
            -0.30079633,
            -0.80079633,
            -1.57079633,
            -2.64398163,
            2.63920367,
            1.57079633,
            0.79920367,
            0.29920367,
        ]
    )

    def __init__(self, max_distance_sensor_value=1000) -> None:
        super().__init__()

        self.MAX_DISTANCE_SENSOR_VALUE = max_distance_sensor_value

        self.__time_step = int(self.getBasicTimeStep())

        self.__pen = self.getDevice("pen")

        self.__gps = self.getDevice("gps")
        self.__gps.enable(self.__time_step)

        self.__imu = self.getDevice("IMU")
        self.__imu.enable(self.__time_step)

        self.__distance_sensors = [self.getDevice(f"ds{i}") for i in range(8)]
        for ds in self.__distance_sensors:
            ds.enable(self.__time_step)

        self.__left_motor = self.getDevice("left wheel motor")
        self.__right_motor = self.getDevice("right wheel motor")
        for motor in [self.__left_motor, self.__right_motor]:
            motor.setPosition(float("inf"))
            motor.setVelocity(0.0)

    def get_current_position(self) -> np.ndarray:
        """Returns the (x, y) position of the robot from the GPS device."""
        return np.array(self.__gps.getValues())[:2]

    def get_current_angle(self) -> np.float64:
        """Returns the yaw angle of the robot in radians"""
        _, _, yaw = self.__imu.getRollPitchYaw()
        return np.float64(yaw)

    def get_sensors_angles(self):
        return self.get_current_angle() + Robot.RELATIVE_ANGLES_OF_DISTANCE_SENSORS

    def get_distances(self) -> np.ndarray:
        """Returns the a list of distances returned by the distance sensors."""
        return np.array([ds.getValue() for ds in self.__distance_sensors])

    def get_front_distance(self) -> float:
        """Returns the distance of the obstacles in front of the robot."""
        return min(self.get_distances()[[0, 7]])

    def set_motors_speeds(self, left_speed: float, right_speed: float) -> None:
        self.__left_motor.setVelocity(left_speed)
        self.__right_motor.setVelocity(right_speed)

    def simulator_step(self):
        """Runs a single step in the simulator."""
        return self.step(self.__time_step)

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

    def get_closest_obstacle_distance(self, point: Waypoint, *args) -> float:
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
        return 1. / min_distance if min_distance != float("inf") else 1e-6


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
        cost_function: Callable,
        heuristic_function: Callable
    ) -> None:
        # appending the cost to each each neighbor to the neighbor
        # i.e.: {waypoint_1: [neighbor_1, neighbor_2]} becomes
        # {waypoint_1: [(neighbor_1, cost_1), (neighbor_2, cost_2)]}
        self.__start = start
        self.__goal = goal
        self.__obstacle_map = obstacle_map
        self.__heuristic_function = heuristic_function
        closest_to_start = (None, float("inf"))
        closest_to_goal = (None, float("inf"))
        self._adjacency_graph: Dict[Waypoint, List[Tuple[Waypoint, float]]] = dict()
        for waypoint, neighbors in adjacency_graph.items():
            if (dist := Graph.euclidean_distance(waypoint, self.__start)) < closest_to_start[
                1
            ]:
                closest_to_start = (waypoint, dist)
            if (dist := Graph.euclidean_distance(waypoint, self.__goal)) < closest_to_goal[
                1
            ]:
                closest_to_goal = (waypoint, dist)

            self._adjacency_graph[waypoint] = [
                (neighbor, cost_function(neighbor, waypoint))
                for neighbor in neighbors
            ]

        closest_to_start = closest_to_start[0]
        closest_to_goal = closest_to_goal[0]

        self._adjacency_graph[self.__start] = [(closest_to_start, cost_function(closest_to_start, self.__start))]
        self._adjacency_graph[self.__goal] = [(closest_to_goal, cost_function(closest_to_goal, self.__goal))]
        self._adjacency_graph[closest_to_start].append(
            (self.__start, cost_function(self.__start))
        )
        self._adjacency_graph[closest_to_goal].append(
            (self.__goal, cost_function(self.__goal, closest_to_goal))
        )

    def __add_cost(self, cost_function: Callable):
        """adds (closest) obstacle distance to the cost of each edge
           and disregards the direct cost.
        """
        for waypoint, edges in self._adjacency_graph.items():
            updated_edges = []
            for neighbor, direct_cost in edges:
                cost = cost_function(neighbor, waypoint)
                updated_edges.append((neighbor, cost))
            self._adjacency_graph[waypoint] = updated_edges

    def get_neighbors(self, waypoint: Waypoint) -> List[Waypoint]:
        """Returns a list of neighbors for a given waypoint."""
        return self._adjacency_graph[waypoint]

    def get_heuristic(self, current: Waypoint, goal) -> float:
        """Returns the euclidean distance from the current waypoint to the goal."""
        return self.__heuristic_function(current, goal)

    def get_start(self) -> Waypoint:
        return self.__start

    def get_goal(self) -> Waypoint:
        return self.__goal

    @staticmethod
    def euclidean_distance(a: Waypoint, b: Waypoint):
        return np.linalg.norm(a.to_numpy() - b.to_numpy())
    
    def no_heauristic(*args) -> float:
        return 0

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



class APFController:
    def __init__(
        self,
        robot: Robot,
        deliberative_layer: DeliberativeLayer,
        *,
        distance_to_goal_threshold: float = 0.1,
    ) -> None:
        self.__robot = robot
        self.__distance_threshold = distance_to_goal_threshold
        self.__final_goal_reached: bool = False
        self.__deliberative_layer = deliberative_layer
        
        self.__robot.simulator_step() # essential to enable gps in order to obtain intial distance to goal
        
        self.__destination = self.__get_destination()
        
        

    def __get_destination(self) -> Waypoint:
        
        return self.__deliberative_layer.get_next_waypoint()    

    def __get_heading_vector(self) -> np.ndarray:
        """Calculates the heading vector from the current position to the goal position."""
        return self.__destination.to_numpy() - self.__robot.get_current_position()

    def __get_attractive_force(self, max_magnitude= 5) -> np.ndarray:
        heading = self.__get_heading_vector()
        return (max_magnitude / np.linalg.norm(heading)) * (heading)

    def __get_distance_to_waypoint(self) -> float:
        return np.linalg.norm(self.__get_heading_vector())

    def __filter_angle(self, angle: float) -> float:
        """Returns the proportional control output for a given target and current value."""
    
        front_distance = self.__robot.get_front_distance()

        filter_amount = (
            self.__map(front_distance, 0, 200, 12, 0.9) if front_distance < 200 else 0.9
        )
        return angle * filter_amount

    def __map(self, value, from_lower, from_higher, to_lower, to_higher):
        """
        Maps a value from the range [`from_lower`, `from_higher`] to
            the range [`to_lower`, `to_higher`].
        """
        mapped_value = (value - from_lower) * (to_higher - to_lower) / (
            from_higher - from_lower
        ) + to_lower
        return np.clip(mapped_value, min(to_lower, to_higher), max(to_lower, to_higher))

    def __vector_to_components(self, magnitude: float, angle: float) -> np.ndarray:
        """Calculate the x and y components of a vector."""
        x = magnitude * np.cos(angle)
        y = magnitude * np.sin(angle)
        return np.array([x, y])

    def __get_total_repulsive_force(self, max_magnitude: float = 10) -> np.ndarray:
        """Return the repulsive forces relative to the obstacles distances as 2D vectors."""
        distances = self.__map(
            self.__robot.get_distances(),
            0,
            self.__robot.MAX_DISTANCE_SENSOR_VALUE,
            max_magnitude,
            0,
        )
        sensors_angles = self.__robot.get_sensors_angles()
        return -np.sum(
            [
                self.__vector_to_components(distance, angle)
                for distance, angle in zip(distances, sensors_angles)
            ],
            axis=0,
        )

    def __get_total_force(self) -> np.ndarray:
        """Sums the repulsive force with the attractive force"""
        return self.__get_total_repulsive_force() + self.__get_attractive_force()

    def final_goal_reached(self) -> bool:
        return self.__final_goal_reached

    def compute_motors_speed(self) -> Tuple[float, float]:
        distance_to_waypoint = self.__get_distance_to_waypoint()
        total_force = self.__get_total_force()

        # stop if too close to the goal
        if distance_to_waypoint <= self.__distance_threshold:
            print(f"{self.__destination} reached.", end=" ")
            try:
                self.__destination = self.__get_destination()
                print(f"Going to {self.__destination}.")
            except PathTraversalCompleted:
                self.__final_goal_reached = True
                return 0, 0

        

        target_goal = np.arctan2(total_force[1], total_force[0])
        angle_difference = target_goal - self.__robot.get_current_angle()
        # changing the range of the angle to the range [-pi, pi]
        angle_difference = np.arctan2(np.sin(angle_difference), np.cos(angle_difference))

        raw_speed = self.__map(
            abs(angle_difference),
            np.pi/2,
            0,
            0,
            self.__robot.MAX_SPEED,
        )
        angle_difference = self.__filter_angle(angle_difference)

        left_speed = raw_speed - angle_difference
        right_speed = raw_speed + angle_difference

        left_speed, right_speed = map(
            lambda speed: np.clip(
                speed, -self.__robot.MAX_SPEED, self.__robot.MAX_SPEED
            ),
            [left_speed, right_speed],
        )

        return left_speed, right_speed



TEST_ID = 4
width = 1.12 / 33
height = 1.12 / 30
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

start = Waypoint(*tests[f"test{TEST_ID}"]["start"], "start")
goal = Waypoint(*tests[f"test{TEST_ID}"]["goal"], "goal")
p1 = Waypoint(5 * width, 1.12 - 4 * height, "p1")
p2 = Waypoint(11 * width, 1.12 - 2 * height, "p2")
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
p17 = Waypoint(15 * width, 1.12 - 14 * height, "p17")

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

obstacle_map = ObstacleMap(
    [
        # stores the top left, top right, bottom right, bottom left corners of the obstacle
        [
            Waypoint(0.49, 1.12),
            Waypoint(0.56, 0.92),
        ],
        [
            Waypoint(0.27, 0.92),
            Waypoint(0.56, 0.85),
        ],
        [
            Waypoint(0.208, 0.706),
            Waypoint(0.438, 0.636),
        ],
        [
            Waypoint(0.208, 0.636),
            Waypoint(0.268, 0.356),
        ],
        [
            Waypoint(0.198, 0.356),
            Waypoint(0.538, 0.296),
        ],
        [
            Waypoint(0.619, 0.698),
            Waypoint(1.01, 0.636),
        ],
        [
            Waypoint(0.87, 0.636),
            Waypoint(0.936, 0.172),
        ],
        # add map walls
        [
            Waypoint(-0.005, 1.12),
            Waypoint(0, 0),
        ],
        [
            Waypoint(-0.005, 1.13),
            Waypoint(1.12, 1.12),
        ],
        [
            Waypoint(1.12, 1.12),
            Waypoint(1.13, -0.0142),
        ],
        [
            Waypoint(-0.0149, -0.005),
            Waypoint(1.13, -0.014),
        ],
    ]
)

graph = Graph(graph, start=start, goal=goal, obstacle_map=obstacle_map, cost_function=obstacle_map.get_closest_obstacle_distance, heuristic_function=Graph.no_heauristic)

if __name__ == "__main__":
    robot = Robot()
    deliberative_layer = DeliberativeLayer(graph)
    deliberative_layer.generate_path()
    speed_controller = APFController(
        robot, deliberative_layer, distance_to_goal_threshold=0.05
    )
    while robot.simulator_step() != -1:
        left_speed, right_speed = speed_controller.compute_motors_speed()
        robot.set_motors_speeds(left_speed, right_speed)
        if speed_controller.final_goal_reached():
            break