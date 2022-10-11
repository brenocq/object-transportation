from controller import Robot, Camera, DistanceSensor, Motor
from enum import Enum
import random

# Global definitions
TIME_STEP = 32
MAX_SPEED = 6.28
IMAGE_SIZE = 32
IR_SENSOR_LIMIT = 950.0
# Robot image analysis
# +---------+
# |         |
# |         |
# |         | <--- IMAGE_MAX_ROW row just before robot seeing itself
# | rrrrrrr |
# +rrrrrrrrr+
IMAGE_MAX_ROW = 32-6

# State
class State(Enum):
    SEARCH_OBJECT = 1
    APPROACH_OBJECT = 2
    MOVE_AROUND_OBJECT = 3
    PUSH_OBJECT = 4
    BE_A_GOAL = 5
state = State.SEARCH_OBJECT

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


######### MOVEMENT CONTROLLERS ###########
def moveForward(power=1.0):
    leftMotor.setVelocity(MAX_SPEED*power)
    rightMotor.setVelocity(MAX_SPEED*power)

def moveSlightLeft():
    leftMotor.setVelocity(MAX_SPEED*0.5)
    rightMotor.setVelocity(MAX_SPEED*1)

def moveSlightRight():
    leftMotor.setVelocity(MAX_SPEED*1)
    rightMotor.setVelocity(MAX_SPEED*0.5)

def turnRight(power=1.0):
    leftMotor.setVelocity(MAX_SPEED*power)
    rightMotor.setVelocity(-MAX_SPEED*power)

def turnLeft(power=1.0):
    leftMotor.setVelocity(-MAX_SPEED*power)
    rightMotor.setVelocity(MAX_SPEED*power)

########## SENSING ##########
def canSeeObject():
    return cameraCheckColor(objectColor)
def canSeeGoal():
    return cameraCheckColor(goalColor)
def cameraCheckColor(color):
    for y in range(IMAGE_MAX_ROW, -1, -1):# From IMAGE_MAX_ROW to 0
        for i in range(4):
            for x in range(IMAGE_SIZE):
                r = cams[i].imageGetRed(cams[i].getImage(), IMAGE_SIZE, x, y)
                g = cams[i].imageGetGreen(cams[i].getImage(), IMAGE_SIZE, x, y)
                b = cams[i].imageGetBlue(cams[i].getImage(), IMAGE_SIZE, x, y)
                if color.check(r, g, b):
                    #file = 'recognized.png'
                    #cam.saveImage(file, 100)
                    #print(f'r:{r} g:{g} b{b} -> image saved {file}')
                    return True
    return False

def colorDirectionScan(color, rowRange, checkFree):
    '''
    Color direction scan algorithm

    Input: color, rowRange, checkFree
    color -> Color that is search for
    rowRange -> Sequence of rows to check in the images
    checkFree -> If should ignore color if there is robot color below

    Return: found, direction, freeSpace
    found -> If found a row of interest
    direction -> Direction from -180.0 to 180.0 (0.0 is in front of the robot)
    freeSpace -> Size of the free space in pixels

    The algorithm will scan the image following the rowRange searching for the first row
    of color pixels (if checkFree, the pixel should not have robot pixel below). After
    finding the row of interest, the algorithm calculates the angle from the robot to
    the color [-180, 180] (0 being front, -90 being left, 90 being right). Only largest
    interval of color pixels in the row is used to calculate the angle.
    '''
    images = [cams[0].getImage(), cams[1].getImage(), cams[2].getImage(), cams[3].getImage()]
    for y in rowRange:# From IMAGE_MAX_ROW to 0
        for i in range(4):
            for x in range(IMAGE_SIZE):
                r = cams[i].imageGetRed(images[i], IMAGE_SIZE, x, y)
                g = cams[i].imageGetGreen(images[i], IMAGE_SIZE, x, y)
                b = cams[i].imageGetBlue(images[i], IMAGE_SIZE, x, y)

                # Check if pixel is object pixel
                if color.check(r, g, b):
                    # Find intervals of object pixels in this row without robot pixels below
                    intervals = []
                    start = 0
                    end = 0
                    for I in range(4):
                        for X in range(IMAGE_SIZE):
                            r = cams[I].imageGetRed(images[I], IMAGE_SIZE, X, y)
                            g = cams[I].imageGetGreen(images[I], IMAGE_SIZE, X, y)
                            b = cams[I].imageGetBlue(images[I], IMAGE_SIZE, X, y)
                            rBelow = cams[I].imageGetRed(images[I], IMAGE_SIZE, X, y+1)
                            gBelow = cams[I].imageGetGreen(images[I], IMAGE_SIZE, X, y+1)
                            bBelow = cams[I].imageGetBlue(images[I], IMAGE_SIZE, X, y+1)

                            if not color.check(r, g, b) or (checkFree and robotColor.check(rBelow, gBelow, bBelow)):
                                if start != end:
                                    intervals.append([start, end])
                                start = X + I*IMAGE_SIZE
                                end = X + I*IMAGE_SIZE
                            else:
                                end = X + I*IMAGE_SIZE
                    if start != end:
                        intervals.append([start, end])

                    # Merge intervals (wrapping around)
                    idxBeginInt = -1
                    idxEndInt = -1
                    for i, interval in enumerate(intervals):
                        if interval[0] == 0:
                            idxBeginInt = i
                        if interval[1] == 4*IMAGE_SIZE-1:
                            idxEndInt = i
                    if idxBeginInt != -1 and idxEndInt != -1:
                        intervals[idxBeginInt][0] = intervals[idxEndInt][0]
                        intervals = intervals[:idxEndInt] + intervals[idxEndInt+1 :]

                    # Find largest interval
                    idxLargest = 0
                    sizeLargest = 0
                    for i, interval in enumerate(intervals):
                        size = interval[1] - interval[0]
                        if interval[0] > interval[1]:
                            size = interval[1] + (IMAGE_SIZE*4-interval[0])
                        if size >= sizeLargest:
                            idxLargest = i
                            sizeLargest = size

                    # Calculate direction
                    if sizeLargest == 0:
                        continue
                    else:
                        interval = intervals[idxLargest]
                        # Calculate mean pixel position
                        pixelPos = (interval[0] + sizeLargest/2)%(IMAGE_SIZE*4)
                        # Convert to [0,1]
                        meanPos = pixelPos/(IMAGE_SIZE*4.0)
                        # Convert to [-2, 2]
                        meanPos = (meanPos*4 - 0.5)
                        if meanPos > 2:
                            meanPos = -2+(meanPos-2)
                        # Calculate direction
                        direction = meanPos*90
                        return True, direction, sizeLargest
    return False, 0, 0

def freeDirectionToObject():
    '''
    Free direction to object algorithm

    Return: isFree, direction, freeSpace
    isFree -> If there is free space to push the object
    direction -> Direction from -180.0 to 180.0 (0.0 is in front of the robot)
    freeSpace -> Size of the free space in pixels

    The algorithm will scan the image from bottom to top searching for the first row
    of object pixels that does not contain robot pixels below. After finding the row of
    interest, the algorithm calculates the angle from the robot to the object [-180, 180]
    (0 being front, -90 being left, 90 being right). Only largest interval of object pixels
    in the row is used to calculate the angle.
    '''
    return colorDirectionScan(color=objectColor, rowRange=range(IMAGE_MAX_ROW, -1, -1), checkFree=True)


def directionToObject():
    '''
    Direction to object algorithm

    Return: found, direction
    found -> If found the object
    direction -> Estimated object direction

    The algorithm will scan the image from top to bottom searching the first row
    with object pixels. The average object pixel position in that row is converted
    to direction angle and is returned
    '''
    found, direction, _ = colorDirectionScan(color=objectColor, rowRange=range(IMAGE_SIZE), checkFree=False)
    return found, direction

def distanceToObject():
    '''
    Distance to algorithm

    Return: distance
    distance -> Number of row between top of the image and first object pixel

    The algorithm will scan the image from top to bottom searching for the first row
    that contains object pixel. If the object is visible in the whole image, 0 is
    returned. If the object is not in the image, IMAGE_SIZE is returned.
    '''
    for y in range(IMAGE_SIZE):# From 0 to IMAGE_SIZE-1
        for i in range(4):
            for x in range(IMAGE_SIZE):
                r = cams[i].imageGetRed(cams[i].getImage(), IMAGE_SIZE, x, y)
                g = cams[i].imageGetGreen(cams[i].getImage(), IMAGE_SIZE, x, y)
                b = cams[i].imageGetBlue(cams[i].getImage(), IMAGE_SIZE, x, y)

                # Check if pixel is object pixel
                if objectColor.check(r, g, b):
                    return y
    return IMAGE_SIZE

########## STATES ##########
def searchObject():
    global counter, RW_turning, turnTimer
    wallDistParam = 400
    nearWall = (irs[0].getValue() < wallDistParam) \
        or (irs[1].getValue() < wallDistParam)     \
        or (irs[7].getValue() < wallDistParam)

    if canSeeObject():
        changeState(State.APPROACH_OBJECT)
    elif not nearWall and not RW_turning:
        counter = 0
        leftMotor.setVelocity(MAX_SPEED)
        rightMotor.setVelocity(MAX_SPEED)
    else:
        RW_turning = True
        if counter < turnTimer:
            leftMotor.setVelocity(-MAX_SPEED)
            rightMotor.setVelocity(MAX_SPEED)
            counter += 1
        else:
            RW_turning = False
            turnTimer = random.randint(30, 100)

def approachObject():
    minDist = 5# Minimum distance to the object to change state
    minAngle = 10# The front angle interval is [-minAngle, minAngle]

    isVisible, direction = directionToObject()
    isInFront = direction < minAngle and direction > -minAngle

    # Move towards the object
    if isVisible:
        if isInFront:
            moveForward()
        elif direction < -minAngle:
            turnRight()
        else:
            turnLeft()
    else:
        # Could not see object, search object
        changeState(State.SEARCH_OBJECT)

    # Check if arrived
    dist = distanceToObject()
    print(f'dir: {direction} dist: {dist}')
    if dist < minDist and isInFront:
        # Close enough to object and is looking straight at it
        changeState(State.MOVE_AROUND_OBJECT)

def moveAroundObject():
    #----- Parameters -----#
    distMin = 200
    distMax = IR_SENSOR_LIMIT
    distToObject = distanceToObject()
    visible, dirToObject = directionToObject()

    #----- Check visible -----#
    if not visible:
        changeState(State.SEARCH_OBJECT)

    #----- Read sensor -----#
    irFront = irs[0].getValue()
    irLeft = irs[7].getValue()
    # Deal with wrong measurements when too close
    #if irFront >= IR_SENSOR_LIMIT and distToObject == 0 and :
    #    irFront = 0
    #if irLeft >= IR_SENSOR_LIMIT and distToObject == 0:
    #    irLeft = 0

    #----- Left hand wall following behavior -----#
    print(f'L:{irLeft} F:{irFront}')
    if (irFront < distMax):
        print("Front can see, turn right")
        turnRight(-0.5)
    elif irLeft < distMin:
        print("Too close, turn right")
        turnRight(-0.5)
    elif irLeft >= distMax:
        print("Left can't see, turn left")
        turnLeft(-0.5)
    else:
        moveForward(0.5)
    #elif ir <= distMin:
    #    # Too close, Move backward while rotating to the right
    #    print("Close")
    #    #leftMotor.setVelocity(-0.5*MAX_SPEED)
    #    #rightMotor.setVelocity(-MAX_SPEED)

def pushObject():
    isFree, direction, _ = freeDirectionToObject()
    if isFree:
        if direction < 10 and direction > -10:
            moveForward()
        elif direction < -10:
            moveSlightRight()
        else:
            moveSlightLeft()
    else:
        changeState(State.SEARCH_OBJECT)

def beAGoal():
    #print('Be a goal')
    pass

def main():
    init()
    print('Robot initialized')
    while robot.step(TIME_STEP) != -1:
        if state == State.SEARCH_OBJECT:
            searchObject()
        elif state == State.APPROACH_OBJECT:
            approachObject()
        elif state == State.MOVE_AROUND_OBJECT:
            moveAroundObject()
        elif state == State.PUSH_OBJECT:
            pushObject()
        elif state == State.BE_A_GOAL:
            beAGoal()

if __name__ == '__main__':
    main()
