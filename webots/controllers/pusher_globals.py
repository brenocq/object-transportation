from controller import Robot, Camera, DistanceSensor, Motor
from enum import Enum
import random

# State
class State(Enum):
    # Paper states
    RANDOM_WALK = 1
    APPROACH_OBJECT = 2
    MOVE_AROUND_OBJECT = 3
    PUSH_OBJECT = 4
    # Additional states
    BE_A_GOAL = 5

# Color
class Color:
    def __init__(self, rmin, rmax, gmin, gmax, bmin, bmax):
        self.rmin = rmin
        self.rmax = rmax
        self.gmin = gmin
        self.gmax = gmax
        self.bmin = bmin
        self.bmax = bmax

    def check(self, r, g, b):
        if r >= self.rmin and r <= self.rmax and g >= self.gmin and g <= self.gmax and b >= self.bmin and b <= self.bmax:
            return True
        else:
            return False

def init():
    global TIME_STEP
    global MAX_SPEED
    global IMAGE_SIZE
    global IR_SENSOR_LIMIT
    global ROBOT_RADIUS
    global IMAGE_MAX_ROW
    global state
    global worldTime
    global objectColor
    global goalColor
    global robotColor
    global robot
    global irs
    global cams
    global leftMotor
    global rightMotor

    TIME_STEP = 512
    MAX_SPEED = 10
    IMAGE_SIZE = 50
    IR_SENSOR_LIMIT = 950.0
    ROBOT_RADIUS = 0.02

    # Robot image analysis
    # +---------+
    # |         |
    # |         |
    # |         | <--- IMAGE_MAX_ROW row just before robot seeing itself
    # | rrrrrrr |
    # +rrrrrrrrr+
    IMAGE_MAX_ROW = 50-10

    state = State.RANDOM_WALK
    worldTime = 0

    # Color interval for object and goal
    objectColor = Color(110, 255, 0, 60, 0, 60)
    goalColor = Color(0, 60, 110, 255, 0, 60)
    robotColor = Color(0, 60, 0, 60, 110, 255)

    # Devices
    robot = Robot()
    irs = []# Infrared devices
    cams = []# Camera devices
    leftMotor = robot.getDevice('leftMotor')
    rightMotor = robot.getDevice('rightMotor')
