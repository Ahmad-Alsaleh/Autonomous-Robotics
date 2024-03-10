from controller import Robot
import numpy as np

class Controller(Robot):
    SPEED = 6
    GOAL = np.array([1.136, 1.136])

    def __init__(self):
        super().__init__()
        self.timeStep = int(self.getBasicTimeStep())
        # self.ds0 = self.getDevice('ds0')
        # self.ds1 = self.getDevice('ds1')
        # self.ds0.enable(self.timeStep)
        # self.ds1.enable(self.timeStep)
        self.pen = self.getDevice('pen')
        self.gps = self.getDevice('gps')
        self.gps.enable(self.timeStep)
        self.imu = self.getDevice('IMU')
        self.imu.enable(self.timeStep)
        # self.receiver = self.getDevice('receiver')
        # self.receiver.enable(self.timeStep)

        # Get a handler to the motors and set target position to infinity (speed control).
        self.left_motor = self.getDevice('left wheel motor')
        self.right_motor = self.getDevice('right wheel motor')
        self.left_motor.setPosition(float('inf'))
        self.right_motor.setPosition(float('inf'))
        self.left_motor.setVelocity(0.0)
        self.right_motor.setVelocity(0.0)

        # get key presses from keyboard
        self.keyboard = self.getKeyboard()
        self.keyboard.enable(self.timeStep)

    def get_position(self):
        return np.array(self.gps.getValues())[:2]
    def get_orientaiton(self):
        roll, pitch, yaw = self.imu.getRollPitchYaw()
        return yaw
        
    def print_gps(self, key):
        if key == 'G':
            position = self.get_position()
            print(f'GPS position: {position[0]:.3f} {position[1]:.3f}')
        elif key == 'V':
            speed, speed_vector_values = self.gps.getSpeed(), self.gps.getSpeedVector()
            print(f'GPS speed vector:', speed, *speed_vector_values)
            
    def get_heading_angle(self, position):
        # Calculate the heading angle to the goal
        heading_vector = Controller.GOAL - position
        return np.arctan2(heading_vector[1], heading_vector[0])

    def get_proportional_control(self, target, current, Kp):
        # Calculate the error in a way that takes into account the circular nature of angles
        error = np.arctan2(np.sin(target - current), np.cos(target - current))
        # Calculate the proportional control output
        return Kp * error
    def minmax(self, x, min, max, new_min, new_max):
        return (x - min) * (new_max - new_min) / (max - min) + new_min

    def vector_to_speed(self, heading_angle, orientation, max_speed):
        # convert heading angle to speed
        # stop if close to goal
        # position = self.get_position()
        distance_to_goal = np.linalg.norm(Controller.GOAL - position)
        if distance_to_goal < 0.05:  # Adjust threshold as needed
            return 0, 0
        # print(heading_angle, orientation)
        # if abs(heading_angle - orientation) < 0.01:
            # return 0, 0
        control_output = self.get_proportional_control(heading_angle, orientation, 0.9)
        control_output = np.clip(control_output, -max_speed, max_speed)
        left_speed = max_speed - control_output
        right_speed = max_speed + control_output
        speed = max(abs(left_speed), abs(right_speed))
        left_speed *= max_speed / speed
        right_speed *= max_speed / speed
        # normalize speeds
        minimum = min(left_speed, right_speed)
        maximum = max(left_speed, right_speed)
        if minimum < -max_speed:
            left_speed = self.minmax(left_speed, -max_speed, max_speed, -max_speed, max_speed)
            right_speed = self.minmax(right_speed, -max_speed, max_speed, -max_speed, max_speed)
        if maximum > max_speed:
            left_speed = self.minmax(left_speed, minimum, maximum, -max_speed, max_speed)
            right_speed = self.minmax(right_speed, minimum, maximum, -max_speed, max_speed)
        return left_speed, right_speed
    def run(self):
        print("Press 'G' to read the GPS device's position")
        print("Press 'V' to read the GPS device's speed vector")
        previous_position = None
        while self.step(self.timeStep) != -1:
            key = chr(self.keyboard.getKey() & 0xff)
            self.print_gps(key)
            position = self.get_position()
            orientation = self.get_orientaiton()
            heading_angle = self.get_heading_angle(position)
            left_speed, right_speed = self.vector_to_speed(heading_angle, orientation, self.SPEED)
            self.left_motor.setVelocity(left_speed)
            self.right_motor.setVelocity(right_speed)


controller = Controller()
controller.run()
