import random
import logging
from deliberative_layer import DeliberativeLayer
from apf_controller import APFController
from robot import Robot
from constants import obstacle_map, rand_area, play_area
from object_recognizer import ObjectRecognizer
from visualizer import Visualizer

SHOW_RRT_ANIMATION = True
ENABLE_OBJECT_DETECTION = True
ENABLE_LOGGING = True
SKIP_FRAMES = 8 # My lucky number
YOLO = True # Set to True to use YOLO for object detection. Otherwise SIFT will be used.

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
        - {YOLO = }
        - {ENABLE_LOGGING = }
    """
    )

    object_recognizer = ObjectRecognizer()
    deliberative_layer = DeliberativeLayer(
        obstacle_map, rand_area=rand_area, play_area=play_area
    )
    robot = Robot()
    speed_controller = APFController(robot, deliberative_layer)
    visualizer = Visualizer(robot, obstacle_map)
    visualizer.draw_rectangular_obstacles()
    counter = 0
    while robot.simulator_step() != -1:
        if deliberative_layer.get_path() is None:
            start = robot.get_current_position()
            goal = deliberative_layer.get_random_goal()
            path = deliberative_layer.generate_path(
                start, goal, show_animation=SHOW_RRT_ANIMATION
            )
            visualizer.draw_path_on_map(path)
            visualizer.draw_path_on_display(path)
            logging.info(f"Path: {path}")

        left_speed, right_speed = speed_controller.compute_motors_speed()
        robot.set_motors_speeds(left_speed, right_speed)
        visualizer.draw_robot()

        if ENABLE_OBJECT_DETECTION and counter == 0:
            image = robot.get_camera_image()
            if YOLO:
                detected_objects = object_recognizer.detect_objects_yolo(image)
            else:
                detected_objects = object_recognizer.detect_objects(image)
            if detected_objects is not None:
                for object_location in detected_objects:
                    logging.info(f"Object detected at: {object_location}")
                    visualizer.draw_detected_objects(object_location)
        
        counter = (counter + 1) % SKIP_FRAMES
