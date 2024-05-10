import os, sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from main.path_finder import Waypoint, Graph, ObstacleMap
from enum import Enum, auto


class PathType(Enum):
    SHORTEST = auto()
    SAFEST = auto()


PATH_TYPE = PathType.SHORTEST
TEST_ID = 2

WIDTH = 1.12 / 33
HEIGHT = 1.12 / 30

tests = {
    "test1": {
        "start": (2 * WIDTH, 1.12 - 2 * HEIGHT),
        "goal": (13 * WIDTH, 1.12 - 16 * HEIGHT),
    },
    "test2": {
        "start": (22 * WIDTH, 1.12 - 3 * HEIGHT),
        "goal": (16 * WIDTH, 3 * HEIGHT),
    },
    "test3": {
        "start": (2 * WIDTH, 13 * HEIGHT),
        "goal": (1.12 - 5 * WIDTH, 10 * HEIGHT),
    },
    "test4": {
        "start": (13 * WIDTH, 1.12 - 3 * HEIGHT),
        "goal": (1.12 - 10 * WIDTH, 3 * HEIGHT),
    },
}

start = Waypoint(*tests[f"test{TEST_ID}"]["start"], "start")
goal = Waypoint(*tests[f"test{TEST_ID}"]["goal"], "goal")
p1 = Waypoint(5 * WIDTH, 1.12 - 4 * HEIGHT, "p1")
p2 = Waypoint(11 * WIDTH, 1.12 - 2 * HEIGHT, "p2")
p3 = Waypoint(5 * WIDTH, 1.12 - 9 * HEIGHT, "p3")
p4 = Waypoint(15 * WIDTH, 1.12 - 9 * HEIGHT, "p4")
p5 = Waypoint(25 * WIDTH, 1.12 - 4 * HEIGHT, "p5")
p6 = Waypoint(25 * WIDTH, 1.12 - 9 * HEIGHT, "p6")
p7 = Waypoint(31 * WIDTH, 1.12 - 9 * HEIGHT, "p7")
p8 = Waypoint(31 * WIDTH, 1.12 - 17 * HEIGHT, "p8")
p9 = Waypoint(30 * WIDTH, 1.12 - 28 * HEIGHT, "p9")
p10 = Waypoint(20 * WIDTH, 1.12 - 26 * HEIGHT, "p10")
p12 = Waypoint(21 * WIDTH, 1.12 - 19 * HEIGHT, "p12")
p13 = Waypoint(4 * WIDTH, 1.12 - 26 * HEIGHT, "p13")
p14 = Waypoint(3 * WIDTH, 1.12 - 21 * HEIGHT, "p14")
p15 = Waypoint(3 * WIDTH, 1.12 - 13 * HEIGHT, "p15")
p16 = Waypoint(10 * WIDTH, 1.12 - 17 * HEIGHT, "p16")
p17 = Waypoint(15 * WIDTH, 1.12 - 14 * HEIGHT, "p17")

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

if PATH_TYPE == PathType.SHORTEST:
    print("Using shortest path")
    cost_function = Graph.euclidean_distance
    heuristic_function = Graph.euclidean_distance
else:
    print("Using safest path")
    cost_function = obstacle_map.get_closest_obstacle_distance
    heuristic_function = Graph.no_heuristic

graph = Graph(
    graph,
    start=start,
    goal=goal,
    cost_function=cost_function,
    heuristic_function=heuristic_function,
)

if __name__ == "__main__":
    from main.robot import Robot
    from main.apf_controller import APFController
    from main.path_finder import DeliberativeLayer

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
