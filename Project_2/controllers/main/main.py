from robot import Robot
from path_finder import Graph, find_path, Waypoint, Path
from apf_controller import APFController


if __name__ == "__main__":
    width = 1.19 / 33
    height = 1.19 / 30

    p0 = Waypoint(2 * width, 1.19 - 2 * height, "p0")
    p1 = Waypoint(4 * width, 1.19 - 3 * height, "p1")
    p2 = Waypoint(10 * width, 1.19 - 2 * height, "p2")
    p3 = Waypoint(5 * width, 1.19 - 9 * height, "p3")
    p4 = Waypoint(15 * width, 1.19 - 9 * height, "p4")
    p5 = Waypoint(25 * width, 1.19 - 4 * height, "p5")
    p6 = Waypoint(25 * width, 1.19 - 9 * height, "p6")
    p7 = Waypoint(31 * width, 1.19 - 9 * height, "p7")
    p8 = Waypoint(31 * width, 1.19 - 17 * height, "p8")
    p9 = Waypoint(30 * width, 1.19 - 28 * height, "p9")
    p10 = Waypoint(20 * width, 1.19 - 26 * height, "p10")
    p11 = Waypoint(13 * width, 1.19 - 16 * height, "p11")
    p12 = Waypoint(21 * width, 1.19 - 19 * height, "p12")
    p13 = Waypoint(4 * width, 1.19 - 26 * height, "p13")
    p14 = Waypoint(3 * width, 1.19 - 21 * height, "p14")
    p15 = Waypoint(3 * width, 1.19 - 13 * height, "p15")
    p16 = Waypoint(10 * width, 1.19 - 17 * height, "p16")
    p17 = Waypoint(14 * width, 1.19 - 14 * height, "p17")

    graph = {
        p0: [p1],
        p1: [p0, p2, p3],
        p2: [p1],
        p3: [p1, p4, p15],
        p4: [p3, p6, p17],
        p5: [p6],
        p6: [p4, p5, p7],
        p7: [p6, p8],
        p8: [p7, p9],
        p9: [p8, p10],
        p10: [p9, p12, p13],
        p11: [p16, p17],
        p12: [p10, p17],
        p13: [p10, p14],
        p14: [p13, p15],
        p15: [p3, p14],
        p16: [p11, p17],
        p17: [p4, p11, p12, p16],
    }
    graph = Graph(graph)

    path = find_path(graph, start=p0, goal=p11)
    print(f"Path to follow: {path}")

    robot = Robot()

    speed_controller = APFController(robot, path=path)

    while robot.simulator_step() != -1:
        left_speed, right_speed = speed_controller.compute_motors_speed()
        robot.set_motors_speeds(left_speed, right_speed)
        if speed_controller.final_goal_reached():
            break
