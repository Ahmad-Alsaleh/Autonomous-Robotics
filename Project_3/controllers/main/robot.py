from controller import Robot
from controller import Keyboard
import numpy as np


class Robot(Robot):
    MAX_SPEED = 6.2
    RELATIVE_ANGLES_OF_DISTANCE_SENSORS = np.array(
        [
            -0.30079633,
            -0.80079633,
            -1.57079633,
            -2.64398163,
            2.63920367,
            1.57079633,
            0.79920367,
            0.29920367,
        ]
    )

    def __init__(self, max_distance_sensor_value=1000) -> None:
        super().__init__()

        self.MAX_DISTANCE_SENSOR_VALUE = max_distance_sensor_value

        self.__time_step = int(self.getBasicTimeStep())

        self.__pen = self.getDevice("pen")

        self.__keyboard = self.getKeyboard()
        self.__keyboard.enable(self.__time_step)

        self.__gps = self.getDevice("gps")
        self.__gps.enable(self.__time_step)

        self.__imu = self.getDevice("IMU")
        self.__imu.enable(self.__time_step)

        self.__camera = self.getDevice("camera")
        self.__camera.enable(self.__time_step)

        self.__display = self.getDevice("display")

        self.__distance_sensors = [self.getDevice(f"ds{i}") for i in range(8)]
        for ds in self.__distance_sensors:
            ds.enable(self.__time_step)

        self.__left_motor = self.getDevice("left wheel motor")
        self.__right_motor = self.getDevice("right wheel motor")
        for motor in [self.__left_motor, self.__right_motor]:
            motor.setPosition(float("inf"))
            motor.setVelocity(0.0)

    def get_current_position(self) -> np.ndarray:
        """Returns the (x, y) position of the robot from the GPS device."""
        return np.array(self.__gps.getValues())[:2]

    def get_camera_image(self):
        """Returns RGB channels of the image from the camera."""
        img = self.__camera.getImageArray()  # returns RGBA image
        return np.array(img, dtype=np.uint8)[
            :, :, :3
        ]  # remove alpha channel and return image

    def get_current_angle(self) -> np.float64:
        """Returns the yaw angle of the robot in radians"""
        _, _, yaw = self.__imu.getRollPitchYaw()
        return np.float64(yaw)

    def get_sensors_angles(self) -> np.ndarray:
        return self.get_current_angle() + Robot.RELATIVE_ANGLES_OF_DISTANCE_SENSORS

    def get_distances(self) -> np.ndarray:
        """Returns the a list of distances returned by the distance sensors."""
        return np.array([ds.getValue() for ds in self.__distance_sensors])

    def get_front_distance(self) -> float:
        """Returns the distance of the obstacles in front of the robot."""
        return min(self.get_distances()[[0, 7]])

    def set_motors_speeds(self, left_speed: float, right_speed: float) -> None:
        self.__left_motor.setVelocity(left_speed)
        self.__right_motor.setVelocity(right_speed)

    def simulator_step(self) -> int:
        """Runs a single step in the simulator."""
        return self.step(self.__time_step)

    def getKey(self):
        return self.__keyboard.getKey()

    def saveDisplay(self, filename: str):
        self.__display.imageSave(None, filename)
