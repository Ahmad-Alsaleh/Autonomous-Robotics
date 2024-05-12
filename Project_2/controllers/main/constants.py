import os, sys
from typing import Callable, Dict, Literal, Tuple

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from main.deliberative_layer import (
    Neighbors,
    Waypoint,
    ObstaclesMap,
    Rectangle,
    euclidean_distance,
)


def get_cost_and_heuristic_functions(
    path_type: Literal["shortest", "safest"]
) -> Tuple[Callable, Callable]:
    if path_type == "shortest":
        cost_function = euclidean_distance
        heuristic_function = euclidean_distance
    elif path_type == "safest":
        obstacle_map = ObstaclesMap(
            [
                # the 7 rectangular obstacles
                Rectangle(Waypoint(0.49, 1.12), Waypoint(0.56, 0.92)),
                Rectangle(Waypoint(0.27, 0.92), Waypoint(0.56, 0.85)),
                Rectangle(Waypoint(0.208, 0.706), Waypoint(0.438, 0.636)),
                Rectangle(Waypoint(0.208, 0.636), Waypoint(0.268, 0.356)),
                Rectangle(Waypoint(0.198, 0.356), Waypoint(0.538, 0.296)),
                Rectangle(Waypoint(0.619, 0.698), Waypoint(1.01, 0.636)),
                Rectangle(Waypoint(0.87, 0.636), Waypoint(0.936, 0.172)),
                # the 4 map walls
                Rectangle(Waypoint(-0.005, 1.12), Waypoint(0, 0)),
                Rectangle(Waypoint(-0.005, 1.13), Waypoint(1.12, 1.12)),
                Rectangle(Waypoint(1.12, 1.12), Waypoint(1.13, -0.0142)),
                Rectangle(Waypoint(-0.0149, -0.005), Waypoint(1.13, -0.014)),
            ]
        )
        cost_function = obstacle_map.get_closest_obstacle_distance
        heuristic_function = obstacle_map.get_closest_obstacle_distance
    else:
        raise ValueError("Invalid path type. Choose between 'shortest' and 'safest'.")

    return cost_function, heuristic_function


def get_start_and_goal(
    test_id: Literal["test1", "test2", "test3", "test4"]
) -> Tuple[Waypoint, Waypoint]:
    tests = {
        "test1": {
            "start": (0.07, 1.05),
            "goal": (0.4412, 0.5227),
        },
        "test2": {
            "start": (0.75, 1.01),
            "goal": (0.54, 0.11),
        },
        "test3": {
            "start": (0.07, 0.49),
            "goal": (0.99, 0.37),
        },
        "test4": {
            "start": (0.44, 1.01),
            "goal": (0.78, 0.11),
        },
        "extra1": {
            "start": (0.07, 1.05),
            "goal": (0.07, 0.12),
        },
    }

    try:
        test_case = tests[test_id]
    except KeyError:
        raise ValueError(
            f"Invalid test id: {test_id}. Choose from {', '.join(tests.keys())}."
        )
    start = Waypoint(*test_case["start"], "start")
    goal = Waypoint(*test_case["goal"], "goal")

    return start, goal


def get_waypoints() -> Dict[Waypoint, Neighbors]:
    p1 = Waypoint(0.17, 0.97, "p1")
    p2 = Waypoint(0.37, 1.05, "p2")
    p3 = Waypoint(0.17, 0.78, "p3")
    p4 = Waypoint(0.51, 0.78, "p4")
    p5 = Waypoint(0.85, 0.97, "p5")
    p6 = Waypoint(0.85, 0.78, "p6")
    p7 = Waypoint(1.05, 0.78, "p7")
    p8 = Waypoint(1.05, 0.49, "p8")
    p9 = Waypoint(1.02, 0.07, "p9")
    p10 = Waypoint(0.68, 0.15, "p10")
    p12 = Waypoint(0.71, 0.41, "p12")
    p13 = Waypoint(0.14, 0.15, "p13")
    p14 = Waypoint(0.10, 0.34, "p14")
    p15 = Waypoint(0.10, 0.63, "p15")
    p16 = Waypoint(0.34, 0.49, "p16")
    p17 = Waypoint(0.51, 0.60, "p17")

    waypoints = {
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

    return waypoints
