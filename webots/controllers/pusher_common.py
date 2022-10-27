import math
import numpy as np
import random
import pusher_globals as g

def changeState(newState):
    print(f'Changed to state "{newState}"')
    g.state = newState

########## INITIALIZE ##########
def initRobot():
    # Initialize infra red sensors
    for i in range(8):
        g.irs.append(g.robot.getDevice('ir'+str(i)))
        g.irs[i].enable(g.TIME_STEP)

    # Initialize cameras
    for i in range(4):
        g.cams.append(g.robot.getDevice('cam'+str(i)))
        g.cams[i].enable(g.TIME_STEP)

    # Initialize motors
    g.leftMotor.setPosition(float('inf'))
    g.rightMotor.setPosition(float('inf'))
    g.leftMotor.setVelocity(0.0)
    g.rightMotor.setVelocity(0.0)

######## MOVEMENT CONTROLLERS ###########
def dirToVec(direction):
    '''
    Given the direction, convert to vector.
    Y axis is forward, X axis is to the right.
    '''
    return np.array([math.sin(math.radians(direction)), math.cos(math.radians(direction))])

def vecToMotor(moveVector):
    '''
    Given the movement vector, set the motor speeds.
    Y axis is forward, X axis is to the right.
    '''
    norm = np.linalg.norm(moveVector)
    ang = 0

    # Allow swap front every X milliseconds to avoid getting stuck
    if g.worldTime % vecToMotor.swapTime == 0:
        vecToMotor.moveForward =  moveVector[1] >= 0

    if vecToMotor.moveForward:
        ang = math.atan2(moveVector[0], moveVector[1])
    else:
        ang = math.atan2(-moveVector[0], moveVector[1])

    # Calculate power
    power = np.array([math.cos(ang) - math.sin(ang), math.cos(ang) + math.sin(ang)])
    power *= norm

    # Normalize power
    if abs(power[0]) > 1.0:
        power /= abs(power[0])
    if abs(power[1]) > 1.0:
        power /= abs(power[1])

    # Apply power
    g.leftMotor.setVelocity(g.MAX_SPEED*power[0])
    g.rightMotor.setVelocity(g.MAX_SPEED*power[1])

vecToMotor.moveForward = 1
vecToMotor.swapTime = 512

########## SENSING ##########
def canSeeObject():
    return cameraCheckColor(g.objectColor)
def canSeeGoal():
    return cameraCheckColor(g.goalColor)
def cameraCheckColor(color):
    for y in range(g.IMAGE_MAX_ROW, -1, -1):# From IMAGE_MAX_ROW to 0
        for i in range(4):
            for x in range(g.IMAGE_SIZE):
                r = g.cams[i].imageGetRed(g.cams[i].getImage(), g.IMAGE_SIZE, x, y)
                gr = g.cams[i].imageGetGreen(g.cams[i].getImage(), g.IMAGE_SIZE, x, y)
                b = g.cams[i].imageGetBlue(g.cams[i].getImage(), g.IMAGE_SIZE, x, y)
                if color.check(r, gr, b):
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
    images = [g.cams[0].getImage(), g.cams[1].getImage(), g.cams[2].getImage(), g.cams[3].getImage()]
    for y in rowRange:# From IMAGE_MAX_ROW to 0
        for i in range(4):
            for x in range(g.IMAGE_SIZE):
                r = g.cams[i].imageGetRed(images[i], g.IMAGE_SIZE, x, y)
                gr = g.cams[i].imageGetGreen(images[i], g.IMAGE_SIZE, x, y)
                b = g.cams[i].imageGetBlue(images[i], g.IMAGE_SIZE, x, y)

                # Check if pixel is object pixel
                if color.check(r, gr, b):
                    # Find intervals of object pixels in this row without robot pixels below
                    intervals = []
                    start = 0
                    end = 0
                    for I in range(4):
                        for X in range(g.IMAGE_SIZE):
                            r = g.cams[I].imageGetRed(images[I], g.IMAGE_SIZE, X, y)
                            gr = g.cams[I].imageGetGreen(images[I], g.IMAGE_SIZE, X, y)
                            b = g.cams[I].imageGetBlue(images[I], g.IMAGE_SIZE, X, y)
                            rBelow = g.cams[I].imageGetRed(images[I], g.IMAGE_SIZE, X, y+1)
                            gBelow = g.cams[I].imageGetGreen(images[I], g.IMAGE_SIZE, X, y+1)
                            bBelow = g.cams[I].imageGetBlue(images[I], g.IMAGE_SIZE, X, y+1)

                            if not color.check(r, gr, b) or (checkFree and g.robotColor.check(rBelow, gBelow, bBelow)):
                                if start != end:
                                    intervals.append([start, end])
                                start = X + I*g.IMAGE_SIZE
                                end = X + I*g.IMAGE_SIZE
                            else:
                                end = X + I*g.IMAGE_SIZE
                    if start != end:
                        intervals.append([start, end])

                    # Merge intervals (wrapping around)
                    idxBeginInt = -1
                    idxEndInt = -1
                    for i, interval in enumerate(intervals):
                        if interval[0] == 0:
                            idxBeginInt = i
                        if interval[1] == 4*g.IMAGE_SIZE-1:
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
                            size = interval[1] + (g.IMAGE_SIZE*4-interval[0])
                        if size >= sizeLargest:
                            idxLargest = i
                            sizeLargest = size

                    # Calculate direction
                    if sizeLargest == 0:
                        continue
                    else:
                        interval = intervals[idxLargest]
                        # Calculate mean pixel position
                        pixelPos = (interval[0] + sizeLargest/2)%(g.IMAGE_SIZE*4)
                        # Convert to [0,1]
                        meanPos = pixelPos/(g.IMAGE_SIZE*4.0)
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
    return colorDirectionScan(color=g.objectColor, rowRange=range(g.IMAGE_MAX_ROW, -1, -1), checkFree=True)


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
    found, direction, _ = colorDirectionScan(color=g.objectColor, rowRange=range(g.IMAGE_SIZE), checkFree=False)
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
    for y in range(g.IMAGE_SIZE):# From 0 to IMAGE_SIZE-1
        for i in range(4):
            for x in range(g.IMAGE_SIZE):
                r = g.cams[i].imageGetRed(g.cams[i].getImage(), g.IMAGE_SIZE, x, y)
                gr = g.cams[i].imageGetGreen(g.cams[i].getImage(), g.IMAGE_SIZE, x, y)
                b = g.cams[i].imageGetBlue(g.cams[i].getImage(), g.IMAGE_SIZE, x, y)

                # Check if pixel is object pixel
                if g.objectColor.check(r, gr, b):
                    return y
    return g.IMAGE_SIZE


########## STATES ##########
def searchObject():
    wallDistParam = 400
    nearWall = (g.irs[0].getValue() < wallDistParam) \
        or (g.irs[1].getValue() < wallDistParam)     \
        or (g.irs[7].getValue() < wallDistParam)

    if canSeeObject():
        changeState(g.State.APPROACH_OBJECT)
    elif not nearWall and not g.RW_turning:
        g.counter = 0
        g.leftMotor.setVelocity(g.MAX_SPEED)
        g.rightMotor.setVelocity(g.MAX_SPEED)
    else:
        RW_turning = True
        if counter < turnTimer:
            g.leftMotor.setVelocity(-g.MAX_SPEED)
            g.rightMotor.setVelocity(g.MAX_SPEED)
            g.counter += 1
        else:
            g.RW_turning = False
            g.turnTimer = random.randint(30, 100)

def approachObject():
    minDist = 5# Minimum distance to the object to change state
    minAngle = 10# The front angle interval is [-minAngle, minAngle]

    isVisible, direction = directionToObject()
    if isVisible:
        vecToMotor(dirToVec(direction))
    else:
        changeState(g.State.SEARCH_OBJECT)

    # Check if arrived
    dist = distanceToObject()
    isInFront = (direction < minAngle and direction > -minAngle) \
        or (direction > -math.pi+minAngle or direction > math.pi-minAngle)
    if dist < minDist and isInFront:
        # Close enough to object and is looking straight at it
        changeState(g.State.MOVE_AROUND_OBJECT)

def moveAroundObject():
    #----- Check goal visible -----#
    if g.worldTime % 2048 == 0:
        # Check every 2 seconds if should push the object
        if not canSeeGoal():
            changeState(g.State.PUSH_OBJECT)

    #----- Parameters -----#
    moveVec = np.array([0, 1]) # Robot move vector (X is to the right, Y is forward)
    minDist = 300

    #----- Input -----#
    distToObject = distanceToObject()
    visible, dirToObject = directionToObject()

    idxF = 0
    if dirToObject < 0:
        idxF = 4# If robot moving "backward", front sensor is behind

    irF = g.irs[idxF].getValue()# IR front
    irFR = g.irs[idxF+1].getValue()# IR front-right
    irR = g.irs[idxF+2].getValue()# IR right

    #----- Check visible -----#
    if not visible:
        changeState(g.State.SEARCH_OBJECT)

    #----- Force field -----#
    objVec = dirToVec(dirToObject)

    # Force to move around the object
    moveVec = np.array([-objVec[1], objVec[0]])

    # Force to keep distance from object
    if irR > g.IR_SENSOR_LIMIT:
        moveVec += objVec
    elif irR < minDist:
        moveVec += -objVec

    # Force to deviate from obstacles
    if irF < minDist:
        moveVec = np.array([-1,0])

    #----- Output - move -----#
    vecToMotor(moveVec)

def pushObject():
    isFree, direction, _ = freeDirectionToObject()
    dist = distanceToObject()
    if isFree:
        if dist == 0:
            # If pushing the object, push slowly
            vecToMotor(dirToVec(direction)*0.3)
        else:
            # If approaching the object, move fast
            vecToMotor(dirToVec(direction))
    else:
        changeState(g.State.MOVE_AROUND_OBJECT)

    if g.worldTime % 2048 == 0:
        # Check every 2 seconds if shuold move around the object
        if canSeeGoal():
            changeState(g.State.MOVE_AROUND_OBJECT)

