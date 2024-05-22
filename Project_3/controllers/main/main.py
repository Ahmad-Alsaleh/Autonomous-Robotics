import os, sys
from typing import Tuple
import random
import logging

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from main.deliberative_layer import DeliberativeLayer
from main.apf_controller import APFController
from main.robot import Robot
from main.constants import obstacle_map, rand_area, play_area
from main.object_recognizer import ObjectRecognizer
from visualizer import Visualizer

SHOW_RRT_ANIMATION = True
ENABLE_OBJECT_DETECTION = True
ENABLE_LOGGING = True


if __name__ == "__main__":
    random.seed(0)
    logging.basicConfig(
        level=logging.INFO if ENABLE_LOGGING else logging.CRITICAL, format="%(message)s"
    )

    logging.info("Starting simulation...")
    logging.info(
        f"""Options:
        - {SHOW_RRT_ANIMATION = }
        - {ENABLE_OBJECT_DETECTION = }
        - {ENABLE_LOGGING = }
    """
    )

    deliberative_layer = DeliberativeLayer(
        obstacle_map, rand_area=rand_area, play_area=play_area
    )
    robot = Robot()
    speed_controller = APFController(robot, deliberative_layer)
    object_recognizer = ObjectRecognizer()
    visualizer = Visualizer(robot, obstacle_map)
    visualizer.draw_rectangular_obstacles()

    while robot.simulator_step() != -1:
        if deliberative_layer.get_path() is None:
            start = robot.get_current_position()
            goal = deliberative_layer.get_random_goal()
            path = deliberative_layer.generate_path(
                start, goal, show_animation=SHOW_RRT_ANIMATION
            )
            logging.info(f"Path: {path}")

        left_speed, right_speed = speed_controller.compute_motors_speed()
        robot.set_motors_speeds(left_speed, right_speed)
        visualizer.draw_robot()

        if ENABLE_OBJECT_DETECTION:
            image = robot.get_camera_image()
            detected_objects = object_recognizer.detect_objects(
                image,
            )
            if detected_objects is not None:
                for object_location in detected_objects:
                    logging.info(f"Object detected at: {object_location}")
                    visualizer.draw_detected_objects(object_location)
