from controller import Robot, Camera, AnsiCodes
import time
from random import random
import numpy as np
np.random.seed(42)

class RobotState:
    WANDER = 1
    FORWARD = 2
    RETURN = 3
    RECOVER = 4


class Colors:
    RED = 0
    GREEN = 1
    YELLOW = 2
    BLUE = 3


class Controller(Robot):
    # determines how much more one color should be relative to the other two
    DETECTION_RATIO = 1.55
    MAX_CENTERING_DURATION = 30
    MAX_WANDERING_COUNTER = 10
    MAX_SPEED = 7
    NUM_CYLINDERS = 3
    COLOR_NAMES = ["red", "green", "yellow", "blue"]
    ANSI_COLORS = [
        AnsiCodes.RED_FOREGROUND,
        AnsiCodes.GREEN_FOREGROUND,
        AnsiCodes.YELLOW_FOREGROUND,
        AnsiCodes.BLUE_FOREGROUND,
    ]
    RECOVERY_DURATION = 50

    def __init__(self):
        super(Controller, self).__init__()
        self.timeStep = int(self.getBasicTimeStep())

        self.state = None

        self.camera = self.getDevice("camera")
        self.camera.enable(self.timeStep)

        self.bumper = self.getDevice("bumper")
        self.bumper.enable(self.timeStep)

        self.left_motor = self.getDevice("left wheel motor")
        self.right_motor = self.getDevice("right wheel motor")
        self.left_motor.setPosition(float("inf"))
        self.right_motor.setPosition(float("inf"))
        self.left_motor.setVelocity(0.0)
        self.right_motor.setVelocity(0.0)

        self.left_speed = Controller.MAX_SPEED / 2
        self.right_speed = Controller.MAX_SPEED / 2

        self.completed_cylinders = []
        # the COLOR of the currently-detected cylinder
        self.current_target = None
        
        self.wandering_counter = 0

    def get_image_colors(self, camera):
        """returns the summation of intensities in the 3 channels RGB"""
        width = camera.getWidth()
        height = camera.getHeight()
        image = camera.getImage()
        red, green, blue = 0, 0, 0
        for i in range(int(width / 3), int(2 * width / 3)):
            for j in range(int(height / 2), int(3 * height / 4)):
                red += camera.imageGetRed(image, width, i, j)
                green += camera.imageGetGreen(image, width, i, j)
                blue += camera.imageGetBlue(image, width, i, j)
        return red, green, blue

    def detect_target(self, colors):
        """
        sets self.current_cylinder to the appropriate value if a cylinder is detected
        """

        red, green, blue = colors
        # If a color is much more represented than the other ones,
        # a cylinder (or cube) is detected
        if (
            red > Controller.DETECTION_RATIO * green
            and red > Controller.DETECTION_RATIO * blue
        ):
            self.current_target = Colors.RED
        elif (
            green > Controller.DETECTION_RATIO * red
            and green > Controller.DETECTION_RATIO * blue
        ):
            self.current_target = Colors.GREEN
        elif (
            red > Controller.DETECTION_RATIO * blue
            and green > Controller.DETECTION_RATIO * blue
        ):
            self.current_target = Colors.YELLOW
        
        else:
            self.current_target = None

        if (
            self.state == RobotState.RETURN
            and blue > Controller.DETECTION_RATIO * red
            and blue > Controller.DETECTION_RATIO * green
        ):
            self.current_target = Colors.BLUE
            print("base")
            
        if self.current_target in self.completed_cylinders:
            self.current_target = None

        if self.current_target != None:
            self.state = RobotState.FORWARD
            self.stop_moving()
            print(
                "Looks like I found a "
                + Controller.ANSI_COLORS[self.current_target]
                + Controller.COLOR_NAMES[self.current_target]
                + AnsiCodes.RESET
                + " cylinder"
            )

    def bumped(self):
        return bool(self.bumper.getValue())

    def stop_moving(self):
        self.left_motor.setVelocity(0)
        self.right_motor.setVelocity(0)

    def wander(self):
        if self.wandering_counter >= Controller.MAX_WANDERING_COUNTER:
            rand = np.random.uniform(-Controller.MAX_SPEED / 4, Controller.MAX_SPEED / 4)
            self.left_speed = np.clip(self.left_speed + rand, 0, Controller.MAX_SPEED / 2)
            self.right_speed = np.clip(self.right_speed - rand, 0, Controller.MAX_SPEED / 2)
            self.wandering_counter = 0
        else:
            self.wandering_counter += 1 
        if self.bumped():
            self.state = RobotState.RECOVER

    def forward(self):
        """moves towards the detected cylinder until it bumps into it"""
        if self.bumped():
            # TODO: verify we are bumping into target
            self.completed_cylinders.append(self.current_target)
            self.state = RobotState.RECOVER
            if self.state == RobotState.RETURN:
                print("robot completed mission, exiting...")
                exit()
            if len(self.completed_cylinders) == Controller.NUM_CYLINDERS:
                self.state = RobotState.RETURN
            print(self.completed_cylinders)

    def recover(self, recovery_counter):
        if recovery_counter < Controller.RECOVERY_DURATION // 2:
            self.left_speed = -Controller.MAX_SPEED / 2
            self.right_speed = -Controller.MAX_SPEED / 2
        elif (
            Controller.RECOVERY_DURATION // 2
            <= recovery_counter
            < Controller.RECOVERY_DURATION
        ):
            self.left_speed = -Controller.MAX_SPEED / 4
            self.right_speed = Controller.MAX_SPEED / 4
        else:
            recovery_counter = 0
            if self.current_target != None:
                self.state = RobotState.FORWARD
            else:
                self.state = RobotState.WANDER

        return recovery_counter

    def center_color(self, target_color):
        image = self.camera.getImage()
        width, height = self.camera.getWidth(), self.camera.getHeight()

        color_positions = []
        for x in range(width):
            for y in range(height):
                r = self.camera.imageGetRed(image, width, x, y)
                g = self.camera.imageGetGreen(image, width, x, y)
                b = self.camera.imageGetBlue(image, width, x, y)

                # Define simple thresholds for red, green, and yellow
                # TODO: make a better COLOR class with thresholds
                is_target_color = False
                if target_color == Colors.RED and r > 200 and g < 50 and b < 50:
                    is_target_color = True
                elif target_color == Colors.GREEN and g > 200 and r < 50 and b < 50:
                    is_target_color = True
                elif target_color == Colors.YELLOW and r > 200 and g > 200 and b < 50:
                    is_target_color = True

                if is_target_color:
                    color_positions.append(x)

        if not color_positions:
            print('PROBLEM')
            self.state = RobotState.WANDER
            return False

        # Find the average position of the detected color
        average_position = sum(color_positions) / len(color_positions)
        center_position = width / 2

        threshold = width * 0.05

        if average_position < center_position - threshold:
            # Color is to the left, rotate left
            self.left_speed = -1.0
            self.right_speed = 1.0
        elif average_position > center_position + threshold:
            # Color is to the right, rotate right
            self.left_speed = 1.0
            self.right_speed = -1.0
        else:
            # Color is centered, stop rotating
            self.left_speed = 0
            self.right_speed = 0
            return True  # Target color is centered

        return False  # Target color is not yet centered

    def robot_sleep(self, duration):
        """
        freezes the last state for duration seconds
        actions(list of functions): the set of behaviors to continue executing
        """
        end_time = self.getTime() + duration
        while self.getTime() < end_time:
            if self.step(self.timeStep) == -1:
                break

    def run(self):
        self.state = RobotState.WANDER
        recovery_counter = 0
        while self.step(self.timeStep) != -1:
            colors = self.get_image_colors(self.camera)
            if self.state == RobotState.WANDER:
                self.wander()
                self.detect_target(colors)
            elif self.state == RobotState.RECOVER:
                recovery_counter += 1
                recovery_counter = self.recover(recovery_counter)
                self.detect_target(colors)
            elif self.state == RobotState.FORWARD:
                if self.center_color(self.current_target):
                    self.right_speed = Controller.MAX_SPEED
                    self.left_speed = Controller.MAX_SPEED
                self.forward()
            elif self.state == RobotState.RETURN:
                # TODO: implement the returning behavior
                self.wander()
                self.detect_target(colors)

            self.left_speed = np.clip(
                self.left_speed, -Controller.MAX_SPEED, Controller.MAX_SPEED
            )
            self.right_speed = np.clip(
                self.right_speed, -Controller.MAX_SPEED, Controller.MAX_SPEED
            )

            self.left_motor.setVelocity(self.left_speed)
            self.right_motor.setVelocity(self.right_speed)
            # self.left_motor.setVelocity(0)
            # self.right_motor.setVelocity(0)


controller = Controller()
controller.run()
