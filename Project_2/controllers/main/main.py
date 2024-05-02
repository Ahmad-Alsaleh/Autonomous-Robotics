from robot import Robot
from path_finder import Graph, Waypoint, Path
from apf_controller import APFController
from path_finder import DeliberativeLayer

if __name__ == "__main__":
    width = 1.12 / 33
    height = 1.12 / 30

    start = Waypoint(2 * width, 1.12 - 2 * height, "start")
    goal = Waypoint(1.12 - 10 * width, 3 * height, "goal")
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
    # print(f"Path to follow: {path}")

    robot = Robot()
    deliberative_layer = DeliberativeLayer(graph)
    deliberative_layer.generate_path()
    speed_controller = APFController(robot, deliberative_layer, distance_to_goal_threshold=0.05)

    while robot.simulator_step() != -1:
        left_speed, right_speed = speed_controller.compute_motors_speed()
        robot.set_motors_speeds(left_speed, right_speed)
        if speed_controller.final_goal_reached():
            break
