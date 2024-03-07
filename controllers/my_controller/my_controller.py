from controller import Robot


class Controller(Robot):
    SPEED = 6

    def __init__(self):
        super().__init__()
        self.timeStep = int(self.getBasicTimeStep())
        # self.ds0 = self.getDevice('ds0')
        # self.ds1 = self.getDevice('ds1')
        # self.ds0.enable(self.timeStep)
        # self.ds1.enable(self.timeStep)

        self.gps = self.getDevice('gps')
        self.gps.enable(self.timeStep)

        # self.receiver = self.getDevice('receiver')
        # self.receiver.enable(self.timeStep)

        # Get a handler to the motors and set target position to infinity (speed control).
        # self.left_motor = self.getDevice('left wheel motor')
        # self.right_motor = self.getDevice('right wheel motor')
        # self.left_motor.setPosition(float('inf'))
        # self.right_motor.setPosition(float('inf'))
        # self.left_motor.setVelocity(0.0)
        # self.right_motor.setVelocity(0.0)

        # get key presses from keyboard
        self.keyboard = self.getKeyboard()
        self.keyboard.enable(self.timeStep)

    def run(self):
        print("Press 'G' to read the GPS device's position")
        print("Press 'V' to read the GPS device's speed vector")
        
        while self.step(self.timeStep) != -1:
            key = chr(self.keyboard.getKey() & 0xff)
            if key == 'G':
                gps_values = self.gps.getValues()
                print('GPS position:', *gps_values)
            elif key == 'V':
                speed_vector_values = self.gps.getSpeedVector()
                print(f'GPS speed vector: {speed_vector_values[0]} {speed_vector_values[1]} {speed_vector_values[2]}')

            # ds0_value = self.ds0.getValue()
            # ds1_value = self.ds1.getValue()
            # if ds1_value > 500:
            #     # If both distance sensors are detecting something, this means that
            #     # we are facing a wall. In this case we need to move backwards.
            #     if ds0_value > 200:
            #         left_speed = -self.SPEED / 2
            #         right_speed = -self.SPEED
            #     else:
            #         # we turn proportionnaly to the sensors value because the
            #         # closer we are from the wall, the more we need to turn.
            #         left_speed = -ds1_value / 100
            #         right_speed = (ds0_value / 100) + 0.5
            # elif ds0_value > 500:
            #     left_speed = (ds1_value / 100) + 0.5
            #     right_speed = -ds0_value / 100
            # else:  # if nothing was detected we can move forward at maximal speed.
            #     left_speed = self.SPEED
            #     right_speed = self.SPEED

            # self.left_motor.setVelocity(left_speed)
            # self.right_motor.setVelocity(right_speed)


controller = Controller()
controller.run()
