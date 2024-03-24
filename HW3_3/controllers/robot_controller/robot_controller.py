from controller import Robot, Camera, AnsiCodes
import time
from random import random
import numpy as np

class RobotState:
    WANDERING = 1
    APPROACHING = 2
    RETURNING = 3

class Colors:
    RED = 0
    GREEN = 1
    YELLOW = 2

class Direction:
    LEFT = 0
    RIGHT = 1

class Controller(Robot):
    # determines how much more one color should be relative to the other two
    DETECTION_RATIO = 1.4
    MAX_SPEED = 10
    COLOR_NAMES = ['red', 'green', 'yellow']
    ANSI_COLORS = [AnsiCodes.RED_FOREGROUND, AnsiCodes.GREEN_FOREGROUND, AnsiCodes.YELLOW_FOREGROUND]
    
    def __init__(self):
        super(Controller, self).__init__()
        self.timeStep = int(self.getBasicTimeStep())
        
        self.camera = self.getDevice('camera')
        self.camera.enable(self.timeStep)

        self.bumper = self.getDevice('bumper')
        self.bumper.enable(self.timeStep)

        self.left_motor = self.getDevice('left wheel motor')
        self.right_motor = self.getDevice('right wheel motor')
        self.left_motor.setPosition(float('inf'))
        self.right_motor.setPosition(float('inf'))
        self.left_motor.setVelocity(0.0)
        self.right_motor.setVelocity(0.0)

        self.left_speed = Controller.MAX_SPEED / 2
        self.right_speed = Controller.MAX_SPEED / 2

        self.found_cylinders = []

        self.current_cylinder = None


    def get_image_colors(self, camera):
        """returns (red, green, blue)"""
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
    
    def detect_cylinder(self, colors):
        """
        sets self.current_cylinder to the appropriate value if a cylinder is detected
        """
        
        red, green, blue = colors
        # If a color is much more represented than the other ones,
        # a cylinder is detected
        if red > Controller.DETECTION_RATIO * green and red > Controller.DETECTION_RATIO * blue:
            self.current_cylinder = Colors.RED
        elif green > Controller.DETECTION_RATIO * red and green > Controller.DETECTION_RATIO * blue:
            self.current_cylinder = Colors.GREEN
        elif red > Controller.DETECTION_RATIO * blue and green > Controller.DETECTION_RATIO * blue:
            self.current_cylinder = Colors.YELLOW
        else:
            self.current_cylinder = None
        
        if self.current_cylinder != None:
            print('Looks like I found a ' + Controller.ANSI_COLORS[self.current_cylinder] +
                    Controller.COLOR_NAMES[self.current_cylinder] + AnsiCodes.RESET + ' cylinder')
    
    def wander(self):
        delta_l = (random() - 0.5) * (self.MAX_SPEED) / self.timeStep
        delta_r = (random() - 0.5) * (self.MAX_SPEED) / self.timeStep
        self.left_speed += delta_l
        self.right_speed += delta_r

    def robot_sleep(self, duration):
        """freezes the last state for duration seconds"""
        end_time = self.getTime() + duration
        while self.getTime() < end_time:
            if self.step(self.timeStep) == -1:
                break

    def run(self):
        self.left_speed = Controller.MAX_SPEED
        self.left_speed = Controller.MAX_SPEED
        while self.step(self.timeStep) != -1:
            image = self.camera.getImage()

            colors = self.get_image_colors(self.camera)
            red, green, blue = colors

            self.wander()
            self.detect_cylinder(colors)
            if self.bumper.getValue() and 0:
                self.left_motor.setVelocity(-Controller.MAX_SPEED)
                self.right_motor.setVelocity(-Controller.MAX_SPEED)
                self.robot_sleep(1)
                self.left_motor.setVelocity(-Controller.MAX_SPEED / 2)
                self.right_motor.setVelocity(Controller.MAX_SPEED / 2)
                self.robot_sleep(1)

            self.left_speed = np.clip(self.left_speed, -Controller.MAX_SPEED, Controller.MAX_SPEED)
            self.right_speed = np.clip(self.right_speed, -Controller.MAX_SPEED, Controller.MAX_SPEED)
            
            self.left_motor.setVelocity(self.left_speed)
            self.right_motor.setVelocity(self.right_speed)
            self.left_motor.setVelocity(0)
            self.right_motor.setVelocity(0)
 

controller = Controller()
controller.run()