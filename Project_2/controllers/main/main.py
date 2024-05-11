import os, sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from main.deliberative_layer import Graph, DeliberativeLayer
from main.apf_controller import APFController
from main.robot import Robot
from main import constants

# choose from ["safest", "shortest"]
PATH_TYPE = "safest"

# choose from ["test1", "test2", "test3", "test4"]
TEST_ID = "test4"

waypoints = constants.get_waypoints()
start, goal = constants.get_start_and_goal(TEST_ID)
cost_func, heuristic_func = constants.get_cost_and_heuristic_functions(PATH_TYPE)

graph = Graph(
    adjacency_graph=waypoints,
    start=start,
    goal=goal,
    cost_function=cost_func,
    heuristic_function=heuristic_func,
)

if __name__ == "__main__":
    print("Starting simulation...")
    print(f"Path type: {PATH_TYPE}")
    print(f"Test ID: {TEST_ID}")

    robot = Robot()
    deliberative_layer = DeliberativeLayer(graph)
    speed_controller = APFController(robot, deliberative_layer)

    while robot.simulator_step() != -1:
        left_speed, right_speed = speed_controller.compute_motors_speed()
        robot.set_motors_speeds(left_speed, right_speed)
        if speed_controller.final_goal_reached():
            break
