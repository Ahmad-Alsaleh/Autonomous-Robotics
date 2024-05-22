import os, sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from main.deliberative_layer import Waypoint, ObstaclesMap, Rectangle


rand_area = (0, 1.12)
play_area = (0, 1.12, 0, 1.12)
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
