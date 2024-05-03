import os, sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from main.path_finder import Waypoint, Graph

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
graph = Graph(graph, start=start, goal=goal)
