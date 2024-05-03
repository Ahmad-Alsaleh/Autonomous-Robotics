if __name__ == "__main__":
    from constants import graph
    from robot import Robot
    from apf_controller import APFController
    from path_finder import DeliberativeLayer

    # print(f"Path to follow: {path}")

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
