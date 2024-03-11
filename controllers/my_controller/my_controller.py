from controller import Robot
import numpy as np


class Controller(Robot):
    MAX_SPEED = 6
    GOAL = np.array([0, 0])

    def __init__(self) -> None:
        super().__init__()
        self.timeStep = int(self.getBasicTimeStep())

        self.pen = self.getDevice("pen")

        self.gps = self.getDevice("gps")
        self.gps.enable(self.timeStep)

        self.imu = self.getDevice("IMU")
        self.imu.enable(self.timeStep)

        self.left_motor = self.getDevice("left wheel motor")
        self.right_motor = self.getDevice("right wheel motor")
        for motor in [self.left_motor, self.right_motor]:
            motor.setPosition(float("inf"))
            motor.setVelocity(0.0)

        self.keyboard = self.getKeyboard()
        self.keyboard.enable(self.timeStep)

    def get_gps_position(self) -> np.ndarray:
        """Returns the (x, y) position of the robot from the GPS device."""
        return np.array(self.gps.getValues())[:2]

    def get_orientation(self) -> np.float64:
        """Returns the yaw angle of the robot in radians"""
        _, _, yaw = self.imu.getRollPitchYaw()
        return np.float64(yaw)

    def listen_to_key_presses(self) -> None:
        """Prints the GPS position and speed vectors on key presses."""
        key = chr(self.keyboard.getKey() & 0xFF)
        if key == "G":
            position = self.get_gps_position()
            print(f"GPS position: ({position[0]:.3f}, {position[1]:.3f})")
        elif key == "V":
            speed_x, speed_y, _ = self.gps.getSpeedVector()
            total_speed = self.gps.getSpeed()
            print(
                f"GPS speed vectors: ({speed_x:.3f}, {speed_y:.3f}). Total speed: {total_speed:.3f}"
            )

    def get_heading(self, position: np.ndarray) -> np.float64:
        """Returns the angle between the robot's heading and the goal in radians"""
        heading_vector = Controller.GOAL - position
        return np.arctan2(heading_vector[1], heading_vector[0])

    def get_proportional_control(self, target_angle, current_angle, Kp=0.9):
        """Returns the proportional control output for a given target and current value."""
        error = target_angle - current_angle

        # take into account the circular nature of angles by converting the error to the range [-pi/2, pi/2]
        error = np.arctan2(np.sin(error), np.cos(error))

        # Calculate the proportional control output
        return Kp * error

    def map(self, value, from_min, from_max, to_min, to_max):
        """Maps a value from the range [`from_min`, `from_max`] to the range [`to_min`, `to_max`]."""
        return (value - from_min) * (to_max - to_min) / (from_max - from_min) + to_min

    def vector_to_speed(self, heading_angle, orientation):
        """Converts heading angle to speed"""

        # stop if close to goal
        position = self.get_gps_position()
        distance_to_goal = np.linalg.norm(Controller.GOAL - position)
        if distance_to_goal < 0.05:  # Adjust threshold as needed
            return 0, 0

        control_output = self.get_proportional_control(heading_angle, orientation)

        left_speed = Controller.MAX_SPEED - control_output
        left_speed = np.clip(left_speed, -Controller.MAX_SPEED, Controller.MAX_SPEED)

        right_speed = Controller.MAX_SPEED + control_output
        right_speed = np.clip(right_speed, -Controller.MAX_SPEED, Controller.MAX_SPEED)

        speed = max(abs(left_speed), abs(right_speed))
        left_speed *= Controller.MAX_SPEED / speed
        right_speed *= Controller.MAX_SPEED / speed

        # normalize speeds
        minimum = min(left_speed, right_speed)
        maximum = max(left_speed, right_speed)

        # ! (@Alsaleh) TODO: i don't like this part. It's not clear what it does. I might remove it soon
        if maximum > Controller.MAX_SPEED:
            left_speed = self.map(
                left_speed,
                minimum,
                maximum,
                -Controller.MAX_SPEED,
                Controller.MAX_SPEED,
            )
            right_speed = self.map(
                right_speed,
                minimum,
                maximum,
                -Controller.MAX_SPEED,
                Controller.MAX_SPEED,
            )
        return left_speed, right_speed

    def run(self):
        print("Press 'G' to read the GPS device's position")
        print("Press 'V' to read the GPS device's speed vector")
        while self.step(self.timeStep) != -1:
            self.listen_to_key_presses()

            position = self.get_gps_position()
            heading_angle = self.get_heading(position)
            orientation = self.get_orientation()

            left_speed, right_speed = self.vector_to_speed(heading_angle, orientation)
            self.left_motor.setVelocity(left_speed)
            self.right_motor.setVelocity(right_speed)

            print(
                f"Position: {position}\t\tOrientation: {orientation}\t\tHeading angle: {heading_angle}"
            )


controller = Controller()
controller.run()
