import random
import logging
from deliberative_layer import DeliberativeLayer
from apf_controller import APFController
from robot import Robot
from constants import obstacle_map, rand_area, play_area
from object_recognizer import ObjectRecognizer
from visualizer import Visualizer
from matplotlib import use

# general options
SHOW_RRT_ANIMATION = True  # use matplotlib to show the RRT algorithm
ENABLE_LOGGING = True  # print useful messages

# object detection options
ENABLE_OBJECT_DETECTION = True  # enable object detection
OBJECT_DETECTION_ALGORITHM = "YOLO"  # "SIFT" or "YOLO"
SKIP_FRAMES = 8  # increase this number to increase simulation speed at the cost of object detection accuracy

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
        - {OBJECT_DETECTION_ALGORITHM = }
        - {ENABLE_LOGGING = }
    """
    )

    # initialize objects
    object_recognizer = ObjectRecognizer()
    use("TkAgg")  # use TkAgg backend to avoid conflicts with other libraries
    deliberative_layer = DeliberativeLayer(
        obstacle_map, rand_area=rand_area, play_area=play_area
    )
    robot = Robot()
    speed_controller = APFController(robot, deliberative_layer)
    visualizer = Visualizer(robot, obstacle_map)
    visualizer.draw_rectangular_obstacles()

    # wander around the map, avoid obstacles and detect objects
    counter = 0
    while robot.simulator_step() != -1:
        # 1. generate path
        if deliberative_layer.get_path() is None:
            start = robot.get_current_position()
            goal = deliberative_layer.get_random_goal()
            path = deliberative_layer.generate_path(
                start, goal, show_animation=SHOW_RRT_ANIMATION
            )
            visualizer.draw_path_on_map(path)
            visualizer.draw_path_on_display(path)
            logging.info(f"Path: {path}")

        # 2. follow path
        left_speed, right_speed = speed_controller.compute_motors_speed()
        robot.set_motors_speeds(left_speed, right_speed)
        visualizer.draw_robot()

        # 3. object detection
        if ENABLE_OBJECT_DETECTION and counter == 0:
            image = robot.get_camera_image()
            detected_objects = object_recognizer.detect_objects(
                image, model=OBJECT_DETECTION_ALGORITHM
            )
            if detected_objects is not None:
                for object_location in detected_objects:
                    logging.info(f"Object detected at: {object_location}")
                    visualizer.draw_detected_objects(object_location)

        counter = (counter + 1) % SKIP_FRAMES
