from controller import Robot, AnsiCodes
import numpy as np

class RobotState:
    WANDER_FOR_CAN = 1
    FORWARD_TO_CAN = 2
    WANDER_FOR_BASE = 3
    FORWARD_TO_BASE = 4
    RECOVER_FROM_BUMP = 5

class Colors:
    RED = 0
    GREEN = 1
    YELLOW = 2
    BLUE = 3
    
class Controller(Robot):
    DETECTION_RATIO = 1.4
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
        super().__init__()
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
        # the COLOR of the currently-detected target
        self.current_target = None
        self.recovered_from = None
        
        self.wandering_counter = 0
        self.recovery_counter = 0

    def get_image_colors(self):
        """returns the summation of intensities in the 3 channels RGB"""
        width = self.camera.getWidth()
        height = self.camera.getHeight()
        image = self.camera.getImage()
        red, green, blue = 0, 0, 0
        for i in range(int(width / 3), int(2 * width / 3)):
            for j in range(int(height / 2), int(3 * height / 4)):
                red += self.camera.imageGetRed(image, width, i, j)
                green += self.camera.imageGetGreen(image, width, i, j)
                blue += self.camera.imageGetBlue(image, width, i, j)
        return red, green, blue
    
    def bumped(self):
        return bool(self.bumper.getValue())
    
    def wander(self):
        if self.wandering_counter >= Controller.MAX_WANDERING_COUNTER:
            rand = np.random.uniform(-Controller.MAX_SPEED / 4, Controller.MAX_SPEED / 4)
            self.left_speed = np.clip(self.left_speed + rand, 0, Controller.MAX_SPEED / 2)
            self.right_speed = np.clip(self.right_speed - rand, 0, Controller.MAX_SPEED / 2)
            self.wandering_counter = 0
        else:
            self.wandering_counter += 1 
    
    def detect_can(self):
        """
        sets self.current_cylinder to the appropriate value if a cylinder is detected
        """

        red, green, blue = self.get_image_colors()
        # If a color is much more represented than the other ones,
        # a cylinder (or cube) is detected
        if (
            red > Controller.DETECTION_RATIO * green
            and red > Controller.DETECTION_RATIO * blue
        ):
            color = Colors.RED
        elif (
            green > Controller.DETECTION_RATIO * red
            and green > Controller.DETECTION_RATIO * blue
        ):
            color = Colors.GREEN
        elif (
            red > Controller.DETECTION_RATIO * blue
            and green > Controller.DETECTION_RATIO * blue
        ):
            color = Colors.YELLOW
        else:
            return None

        print(
            "Looks like I found a "
            + Controller.ANSI_COLORS[color]
            + Controller.COLOR_NAMES[color]
            + AnsiCodes.RESET
            + " cylinder"
        )
        return color

    def detect_base(self):
        red, green, blue = self.get_image_colors()
        return (
            blue > (Controller.DETECTION_RATIO - 0.2) * red
            and blue > (Controller.DETECTION_RATIO - 0.2) * green
        )
    
    def center_color(self, target_color):
        """
            returns a tuple (can_see_target, left_speed, right_speed)
        """
        image = self.camera.getImage()
        width, height = self.camera.getWidth(), self.camera.getHeight()

        color_positions = []
        for x in range(width):
            for y in range(height):
                r = self.camera.imageGetRed(image, width, x, y)
                g = self.camera.imageGetGreen(image, width, x, y)
                b = self.camera.imageGetBlue(image, width, x, y)

                # Define simple thresholds for red, green, and yellow
                is_target_color = False
                if target_color == Colors.RED and r > 175 and g < 50 and b < 50:
                    is_target_color = True
                elif target_color == Colors.GREEN and g > 175 and r < 50 and b < 50:
                    is_target_color = True
                elif target_color == Colors.BLUE and b > 100 and r < 50 and g < 50:
                    is_target_color = True
                elif target_color == Colors.YELLOW and r > 175 and g > 175 and b < 50:
                    is_target_color = True

                if is_target_color:
                    color_positions.append(x)

        if not color_positions:
            return False, None, None

        # Find the average position of the detected color
        average_position = sum(color_positions) / len(color_positions)
        center_position = width / 2

        threshold = width * 0.05

        if average_position < center_position - threshold:
            # Color is to the left, rotate left
            left_speed = -1.0
            right_speed = 1.0
        elif average_position > center_position + threshold:
            # Color is to the right, rotate right
            left_speed = 1.0
            right_speed = -1.0
        else:
            # Color is centered, stop rotating
            left_speed = Controller.MAX_SPEED
            right_speed = Controller.MAX_SPEED
            
        return True, left_speed, right_speed
    
    def recover(self):
        self.recovery_counter += 1
        if self.recovery_counter < Controller.RECOVERY_DURATION // 2:
            self.left_speed = -Controller.MAX_SPEED / 2
            self.right_speed = -Controller.MAX_SPEED / 2
        elif (
            Controller.RECOVERY_DURATION // 2
            <= self.recovery_counter
            < Controller.RECOVERY_DURATION
        ):
            self.left_speed = -Controller.MAX_SPEED / 4
            self.right_speed = Controller.MAX_SPEED / 4
        else:
            self.recovery_counter = 0
            return True

        return False
    
    def run(self):
        self.state = RobotState.WANDER_FOR_CAN
        while self.step(self.timeStep) != -1:
            print(self.state)
            if self.state == RobotState.WANDER_FOR_CAN:
                self.wander()
                target = self.detect_can()
                if target is not None and target not in self.completed_cylinders:
                    self.current_target = target
                    self.state = RobotState.FORWARD_TO_CAN
                elif self.bumped():
                    self.recovered_from = RobotState.WANDER_FOR_CAN
                    self.state = RobotState.RECOVER_FROM_BUMP
            elif self.state == RobotState.FORWARD_TO_CAN:
                see_target, left_speed, right_speed = self.center_color(self.current_target)
                if not see_target:
                    self.state = RobotState.WANDER_FOR_CAN
                    continue
                
                self.left_speed = left_speed
                self.right_speed = right_speed
                if self.bumped():
                    self.state = RobotState.RECOVER_FROM_BUMP
                    # if self.current_target and self.current_target not in self.completed_cylinders:
                    self.completed_cylinders.append(self.current_target)
                    print('APPENDED', self.completed_cylinders)
                    if len(self.completed_cylinders) == Controller.NUM_CYLINDERS:
                        self.recovered_from = RobotState.WANDER_FOR_BASE
                    else:
                        self.recovered_from = RobotState.WANDER_FOR_CAN
                        
            elif self.state == RobotState.RECOVER_FROM_BUMP:
                recovered = self.recover()
                if recovered:
                    self.state = self.recovered_from    
            elif self.state == RobotState.WANDER_FOR_BASE:
                self.wander()
                if self.detect_base():
                    self.current_target = Colors.BLUE
                    self.state = RobotState.FORWARD_TO_BASE
                elif self.bumped():
                    self.recovered_from = RobotState.WANDER_FOR_BASE
                    self.state = RobotState.RECOVER_FROM_BUMP
            elif self.state == RobotState.FORWARD_TO_BASE:
                see_target, left_speed, right_speed = self.center_color(self.current_target)
                if not see_target:
                    self.state = RobotState.WANDER_FOR_BASE
                    continue
                self.left_speed = left_speed
                self.right_speed = right_speed
                if self.bumped():
                    print('Mission Completed !!!!')
                    exit()                    


            self.left_speed = np.clip(
                self.left_speed, -Controller.MAX_SPEED, Controller.MAX_SPEED
            )
            self.right_speed = np.clip(
                self.right_speed, -Controller.MAX_SPEED, Controller.MAX_SPEED
            )

            self.left_motor.setVelocity(self.left_speed)
            self.right_motor.setVelocity(self.right_speed)
    
controller = Controller()
controller.run()