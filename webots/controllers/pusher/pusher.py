from controller import Robot, Camera, DistanceSensor, Motor
from enum import Enum
import random

# Global definitions
TIME_STEP = 32
MAX_SPEED = 6.28
IMAGE_SIZE = 32
# Robot image analysis
# +---------+
# |         |
# |         |
# |         | <--- IMAGE_MAX_ROW row just before robot seeing itself
# | rrrrrrr |
# +rrrrrrrrr+
IMAGE_MAX_ROW = 32-4

# State
states = ["SEARCH_BOX", "MOVE_TOWARD", "ARRIVED_AT_BOX", "MOVE_AROUND", "PUSH", "BE_A_GOAL", "RANDOM_WALK"]
state = "SEARCH_BOX"
def changeState(newState):
    global state
    print(f'Changed to state "{newState}"')
    state = newState

# Colors
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

# Color interval for box and goal
boxColor = Color(110, 255, 0, 60, 0, 60)
goalColor = Color(0, 60, 127, 255, 0, 60)

# Devices
robot = Robot()
irs = []# Infrared devices
cams = []# Camera devices
leftMotor = robot.getDevice('leftMotor')
rightMotor = robot.getDevice('rightMotor')

# Auxiliary
counter = 0
RW_turning = False  # helps manage the random walk, for when it reaches a wall
turnTimer = random.randint(30, 100) # the amount of timesteps to turn in randomWalk

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
    return cameraCheckColor(boxColor)
def canSeeGoal():
    return cameraCheckColor(goalColor)
def cameraCheckColor(color):
    images = [cams[0].getImage(), cams[1].getImage(), cams[2].getImage(), cams[3].getImage()]
    for y in range(IMAGE_MAX_ROW, -1, -1):# From IMAGE_MAX_ROW to 0
        for i in range(4):
            for x in range(IMAGE_SIZE):
                r = cams[i].imageGetRed(images[i], IMAGE_SIZE, x, y)
                g = cams[i].imageGetGreen(images[i], IMAGE_SIZE, x, y)
                b = cams[i].imageGetBlue(images[i], IMAGE_SIZE, x, y)
                if color.check(r, g, b):
                    #file = 'recognized.png'
                    #cam.saveImage(file, 100)
                    #print(f'r:{r} g:{g} b{b} -> image saved {file}')
                    return True
    return False


# to determine which direction the box is in (relative to the robot)
def directionToColor(color):
    # NOTE: Only checking one row in the image
    direction = ['front', 'right', 'back', 'left']
    for i in range(4):
        cam = cams[i]
        w = cam.getWidth()
        h = cam.getHeight()
        image = cam.getImage()
        # an array to store which pixels we can see the object in
        # (helps to determine which direction to move in)
        index_target = []
        for y in range(h):
            if index_target:
                # we've parsed a row in the image where the box was seen, 
                # we know enough about the box's location to turn towards it
                # (saves having to loop through the rest of the image)
                break

            for x in range(w):
                r = cam.imageGetRed(image, w, x, y)
                g = cam.imageGetGreen(image, w, x, y)
                b = cam.imageGetBlue(image, w, x, y)
                if color.check(r, g, b):
                    index_target.append(x)

        # object found, 
        # return the camera that found it and where in the image it was found
        if index_target:
            direction = direction[i]
            center = sum(index_target)/len(index_target)
            return [direction, center]

    return [False, 0] # object not found

######### MOVEMENT CONTROLLERS ###########
def moveForward():
    #print("  going straight ahead")
    leftMotor.setVelocity(MAX_SPEED*1)
    rightMotor.setVelocity(MAX_SPEED*1)

def moveSlightLeft():
    #print("  turning slight left")
    leftMotor.setVelocity(MAX_SPEED*0.5)
    rightMotor.setVelocity(MAX_SPEED*1)

def moveSlightRight():
    #print("  turning slight right")
    leftMotor.setVelocity(MAX_SPEED*1)
    rightMotor.setVelocity(MAX_SPEED*0.5)

def TurnRight():
    #print("  turning right")
    leftMotor.setVelocity(MAX_SPEED*1)
    rightMotor.setVelocity(MAX_SPEED*-1)

def TurnLeft():
    #print("  turning left")
    leftMotor.setVelocity(MAX_SPEED*-1)
    rightMotor.setVelocity(MAX_SPEED*1)


########## STATES ##########
def searchBox():
    #print('Search box')
    leftMotor.setVelocity(MAX_SPEED*-0.5)
    rightMotor.setVelocity(MAX_SPEED*0.5)
    if canSeeBox():
        changeState("MOVE_TOWARD")
    else:
        changeState("RANDOM_WALK")

def moveToward():
    #print('Move toward')
    [direction, center] = directionToColor(boxColor) # find where the box is
    if direction:
        if direction == "front":
            if center<14:
                moveSlightRight()
            elif 14<=center<=18:
                moveForward()
            else:
                moveSlightLeft()
        elif direction == "left":
            TurnRight()
        elif direction == "right":
            TurnLeft()
        else: # the target is behind the robot
            if center<=15:
                TurnLeft()
            else:
                TurnRight()
    else:
        changeState("SEARCH_BOX")

    if irs[0].getValue() < 10:
    # irs sensor typically doesn't get close to 0 
    # before jumping up to 1000 when hitting the box
        changeState("ARRIVED_AT_BOX")


def arrivedAtBox():
    #print('Arrived At Box')
    #leftMotor.setVelocity(MAX_SPEED*0)
    #rightMotor.setVelocity(MAX_SPEED*0)
    if canSeeGoal():
        changeState("MOVE_AROUND")
    else:
        changeState("PUSH")


def randomWalk():
    #print('Random Walking')
    global counter, RW_turning, turnTimer
    wallDistParam = 400
    nearWall = (irs[0].getValue() < wallDistParam) \
        or (irs[1].getValue() < wallDistParam)     \
        or (irs[7].getValue() < wallDistParam)

    if canSeeBox():
        changeState("MOVE_TOWARD")

    elif not nearWall and not RW_turning:
        #print("   go straight")
        counter = 0
        leftMotor.setVelocity(MAX_SPEED*1)
        rightMotor.setVelocity(MAX_SPEED*1)
    else:
        RW_turning = True
        if counter < turnTimer:
            #print("   turn")
            leftMotor.setVelocity(MAX_SPEED*-1)
            rightMotor.setVelocity(MAX_SPEED*1)
            counter += 1
        else:
            RW_turning = False
            turnTimer = random.randint(30, 100)
        #counter = 0



def moveAround():
    #print('Move around')
    #idk just spin around for now or smth
    leftMotor.setVelocity(MAX_SPEED*0.5)
    rightMotor.setVelocity(MAX_SPEED*-0.5)
    #if irs[0].getValue() < 0.05:
    #    changeState("PUSH")

def push():
    if not canSeeGoal():
        leftMotor.setVelocity(MAX_SPEED)
        rightMotor.setVelocity(MAX_SPEED)
    else:
        changeState("MOVE_AROUND")

def beAGoal():
    #print('Be a goal')
    pass

def main():
    init()
    print('Robot initialized')
    while robot.step(TIME_STEP) != -1:
        if state == "SEARCH_BOX":
            searchBox()
        elif state == "MOVE_TOWARD":
            moveToward()
        elif state == "ARRIVED_AT_BOX":
            arrivedAtBox()
        elif state == "MOVE_AROUND":
            moveAround()
        elif state == "PUSH":
            push()
        elif state == "RANDOM_WALK":
            randomWalk()
        elif state == "BE_A_GOAL":
            beAGoal()


if __name__ == '__main__':
    main()
