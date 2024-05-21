import os, sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from main.deliberative_layer import DeliberativeLayer
from main.apf_controller import APFController
from main.robot import Robot
from main.constants import obstacle_map
from main.object_recognizer import ObjectRecognizer
from main.deliberative_layer import PathDoesNotExist, Waypoint
import random


if __name__ == "__main__":
    print("Starting simulation...")

    AREA = [0, 1.12]

    deliberative_layer = DeliberativeLayer(obstacle_map, rand_area=AREA, expand_dis=0.1)
    robot = Robot()
    speed_controller = APFController(robot, deliberative_layer)
    object_recognizer = ObjectRecognizer()

    while robot.simulator_step() != -1:
        if deliberative_layer.get_path() is None:
            # generate a new random goal

            start = tuple(robot.get_current_position())

            while deliberative_layer.is_inside_obstacle(
                Waypoint(*(goal := (random.uniform(*AREA), random.uniform(*AREA))))
            ):
                print(f"Goal: {goal} is inside an obstacle. Generating new goal...")

            print(
                f"Start: {Waypoint(start[0], start[1])}, Goal: {Waypoint(goal[0], goal[1])}"
            )

            while True:
                try:

                    deliberative_layer.generate_path(start, goal, show_animation=True)

                    print("Path generated!")
                    break
                except PathDoesNotExist:
                    pass

        left_speed, right_speed = speed_controller.compute_motors_speed()
        robot.set_motors_speeds(left_speed, right_speed)

        # object recognition part
        image = robot.get_image()
        if image is not None:
            detected_objects = object_recognizer.detect_objects(image)
            if detected_objects is not None:
                for obj in detected_objects:
                    print(f"Object detected at: {obj}")
