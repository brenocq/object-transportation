from controller import Robot, Camera, DistanceSensor, Motor
from enum import Enum

# Global definitions
TIME_STEP = 32
MAX_SPEED = 6.28

# Robot states
class State(Enum):
    SEARCH_BOX = 0
    MOVE_TORWARD = 1
    MOVE_AROUND = 1
    PUSH = 2
    BE_A_GOAL = 3
state = State.SEARCH_BOX

# Devices
robot = Robot()
irs = []
cams = []
leftMotor = robot.getDevice('leftMotor')
rightMotor = robot.getDevice('rightMotor')

########## INITIALIZE ##########
def init():
    # Initialize infra red sensors
    for i in range(8):
        irs.append(robot.getDevice('ir'+str(i)))
        irs[i].enable(TIME_STEP)

    # Initialize cameras
    for i in range(4):
        cams.append(robot.getDevice('cam'+str(i)))
        cams[i].enable(TIME_STEP)

    # Initialize motors
    leftMotor.setPosition(float('inf'))
    rightMotor.setPosition(float('inf'))
    leftMotor.setVelocity(0.0)
    rightMotor.setVelocity(0.0)

########## SENSORS ##########
def canSeeBox():
    return canSeeColor(127, 255, 0, 32, 0, 32)

def canSeeGoal():
    return canSeeColor(0, 32, 127, 255, 0, 32)

def canSeeColor(rmin, rmax, gmin, gmax, bmin, bmax):
    # NOTE: Only checking cam0
    cam = cams[0]
    #for cam in cams:
    w = cam.getWidth()
    h = cam.getHeight()
    image = cam.getImage()
    for x in range(w):
        for y in range(h):
            r = cam.imageGetRed(image, w, x, y)
            g = cam.imageGetGreen(image, w, x, y)
            b = cam.imageGetBlue(image, w, x, y)
            if r >= rmin and r <= rmax and g >= gmin and g <= gmax and b >= bmin and b <= bmax:
                file = 'recognized.png'
                cam.saveImage(file, 100)
                print(f'r:{r} g:{g} b{b} -> image saved {file}')
                return True
    return False

########## STATES ##########
def searchBox():
    print('Search box')
    global state
    leftMotor.setVelocity(MAX_SPEED*0.5)
    rightMotor.setVelocity(MAX_SPEED*0.5)
    if canSeeBox():
        state = State.MOVE_TORWARD

def moveTorward():
    print('Move torward')
    global state
    leftMotor.setVelocity(MAX_SPEED*0.5)
    rightMotor.setVelocity(MAX_SPEED*0.5)
    if irs[0].getValue() < 0.1:
        state = State.MOVE_AROUND

def moveAround():
    print('Move around')
    global state
    leftMotor.setVelocity(MAX_SPEED*0.5)
    rightMotor.setVelocity(MAX_SPEED*0.5)
    if irs[0].getValue() < 0.05:
        state = State.PUSH

def push():
    print('Push')
    leftMotor.setVelocity(MAX_SPEED)
    rightMotor.setVelocity(MAX_SPEED)

def beAGoal():
    print('Be a goal')

def main():
    init()
    print('Robot initialized')
    while robot.step(TIME_STEP) != -1:
        if state is State.SEARCH_BOX:
            searchBox()
        elif state is State.MOVE_TORWARD:
            moveTorward()
        elif state is State.MOVE_AROUND:
            moveAround()
        elif state is State.PUSH:
            push()
        elif state is State.BE_A_GOAL:
            beAGoal()

if __name__ == '__main__':
    main()
