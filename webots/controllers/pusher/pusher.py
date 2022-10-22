from controller import Robot, Camera, DistanceSensor, Motor, Emitter, Receiver
from enum import Enum
import math
import numpy as np
import random
import struct


# Global definitions
TIME_STEP = 128
MAX_SPEED = 10
IMAGE_SIZE = 32
IR_SENSOR_LIMIT = 950.0
ROBOT_RADIUS = 0.02
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
    TURN_TO_GOAL_ROBOT = 6
state = State.SEARCH_OBJECT
worldTime = 0
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
radioEmitter = robot.getDevice('radio emitter') # emitter
IREmitter = robot.getDevice('IR emitter')       # emitter
IRReceiver = robot.getDevice('IR receiver')
IRReceiver.enable(TIME_STEP)
leftMotor = robot.getDevice('leftMotor')
rightMotor = robot.getDevice('rightMotor')


# Wheel sensors
encoder = []
encoderNames = ['leftMotor sensor', 'rightMotor sensor']
for i in range(2):
    encoder.append(robot.getDevice(encoderNames[i]))
    encoder[i].enable(TIME_STEP)

# Auxiliary
r = 0.005 # wheel radius
counter = 0
RW_turning = False  # helps manage the random walk, for when it reaches a wall
turnTimer = random.randint(5, 15) # the amount of timesteps to turn in randomWalk
goalIsVisible = False
goalWasVisible = False
dirToRobot = 0
sawRobotTurnGreen = False
timeStartTurn = 0
oldEncoderValues = []
visionCounter = 0
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


def getRotationOverTime(oldEncoderValues, delta_t):
    """
    uses the number of times each wheel has rotated to calulate how much the robot's angle has changed
    wheel rotations (ie. how many rotations they have done) stored in the 'encoders'
    """
    
    r = 0.005 # radius of wheels
    d = 0.05  # approx distance between wheels
    # get amount of rotations done
    encoderValues = []
    for i in range(2):
        encoderValues.append(encoder[i].getValue())    # [rad]

    # compute speed of the wheels between these timesteps
    wl = (encoderValues[0] - oldEncoderValues[0])/delta_t
    wr = (encoderValues[1] - oldEncoderValues[1])/delta_t
    
    # compute angular speed
    w = r/d * (wr - wl)
    
    # compute change in angle over delta_t
    delta_phi = w * delta_t * (180/math.pi)
    return delta_phi

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
    if worldTime % vecToMotor.swapTime == 0:
        vecToMotor.moveForward =  moveVector[1] >= 0

    if vecToMotor.moveForward:
        ang = math.atan2(moveVector[0], moveVector[1])
    else:
        ang = math.atan2(-moveVector[0], moveVector[1])

    # Calculate power
    power = np.array([math.cos(ang) - math.sin(ang), math.cos(ang) + math.sin(ang)])
    power *= norm

    # Normalize power
    if power[0] > 1.0:
        power /= power[0]
    if power[1] > 1.0:
        power /= power[1]

    # Apply power
    leftMotor_speed = MAX_SPEED*power[0]
    rightMotor_speed = MAX_SPEED*power[1]
    # scale speeds if exceeding MAX_SPEED or -1*MAXSPEED
    if leftMotor_speed > MAX_SPEED or leftMotor_speed < -1*MAX_SPEED:
        rightMotor_speed = rightMotor_speed/abs(leftMotor_speed) * MAX_SPEED
        leftMotor_speed = leftMotor_speed/abs(leftMotor_speed) * MAX_SPEED
    if rightMotor_speed > MAX_SPEED or rightMotor_speed < -1*MAX_SPEED:
        leftMotor_speed = leftMotor_speed/abs(rightMotor_speed) * MAX_SPEED
        rightMotor_speed = rightMotor_speed/abs(rightMotor_speed) * MAX_SPEED

    leftMotor.setVelocity(leftMotor_speed)
    rightMotor.setVelocity(rightMotor_speed)
vecToMotor.moveForward = 1
vecToMotor.swapTime = 512

########## SENSING ##########
def canSeeObject():
    return cameraCheckColor(objectColor)
    
    
def canSeeGoal():
    isVisible = cameraCheckColor(goalColor)
    return isVisible
    
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



def sendMessagetoController(color):
    Name = robot.getName()
    # get the # of the robot from its name
    if Name == 'pusher': # for some reason pusher #0 just has this name, the 0 doesn't show
        id = 0
    else:
        id = int(Name[Name.find("(")+1:Name.find(")")])
    
    if color == 'green':
        color_string = b'G'
    elif color == 'blue':
        color_string = b'B'
    else:
        print('invalid color input')
        return
    
    # send the number, and the colour we want to be
    # pack and send the message (we have to use pack to convert it to C...)
    message = struct.pack('I 1s',id, color_string)
    radioEmitter.send(message)


def sendIRMessage(message):
    if message == 'changeToGoal':
        color_string = b'CTG'
    elif message == 'iAmGoal':
        color_string = b'IAG'
    else:
        print('invalid message input')
        return
    
    # send the number, and the colour we want to be
    # pack and send the message (we have to use pack to convert it to C...)
    message = struct.pack('3s',color_string)
    IREmitter.send(message)


def checkIRMessages():
    
    isVisible = False
    direction = 0
    message = 0
    if IRReceiver.getQueueLength() > 0:
        while IRReceiver.getQueueLength() > 0:
            # the message is the number of the robot, and a string (B or G) which is what colour to be
            msg = IRReceiver.getData() # read the message at the head of the receiver queue
            msg =struct.unpack("3s",msg) # unpack the first element of the mssage
            msg = msg[0]
            if msg == b'IAG':
                isVisible = True
                direction = IRReceiver.getEmitterDirection()[:2]               
                direction  = math.atan2(direction[0], direction[1])*180.0/math.pi
                message = msg
            elif msg == b'CTG':
                isVisible = True
                direction = IRReceiver.getEmitterDirection()[:2]
                direction  = math.atan2(direction[0], direction[1])*180.0/math.pi
                message = msg
                while IRReceiver.getQueueLength() > 0: # get rid of all other messages
                    IRReceiver.nextPacket()
                return isVisible, message, direction

                
            IRReceiver.nextPacket()
    return isVisible, message, direction

#############################
########## STATES ###########
#############################

def searchObject():
    global counter, RW_turning, turnTimer, goalWasVisible, goalIsVisible, visionCounter
    global sawRobotTurnGreen, oldEncoderValues, timeStartTurn, worldTime
    wallDistParam = 400
    nearWall = (irs[0].getValue() < wallDistParam) \
        or (irs[1].getValue() < wallDistParam)     \
        or (irs[7].getValue() < wallDistParam)
    if sawRobotTurnGreen:
        changeState(State.TURN_TO_GOAL_ROBOT)
        timeStartTurn = worldTime
        oldEncoderValues = []
        for i in range(2):
            oldEncoderValues.append(encoder[i].getValue())
        return
    if worldTime % 1024 == 0: # check every 1 second to see if the goal or object is visible
        if goalIsVisible:
            goalWasVisible = True
        if goalWasVisible and not goalIsVisible:
            changeState(State.BE_A_GOAL)
            sendMessagetoController('green')
            sendIRMessage("changeToGoal")
            return
        # maybe use a counter (another global variable...) and if the goal is
        # visible for 10 seconds, then change to APPROACH_OBJECT
        elif visionCounter >= 10:
            visionCounter = 0
            changeState(State.APPROACH_OBJECT)
            return
        SeeObject = canSeeObject()
        if SeeObject and not goalIsVisible:  # can see just the box
            visionCounter = 0
            changeState(State.APPROACH_OBJECT)
        elif SeeObject and goalIsVisible:  # can see red and gree, add to counter
            visionCounter += 1
        
    #elif direction != 0:
    #    pass
    # movement controls
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
            turnTimer = random.randint(5, 15)


def turnToGoalRobot():
    global dirToRobot, worldTime, sawRobotTurnGreen, oldEncoderValues
        
    elapsedTime = worldTime - timeStartTurn
    turned = getRotationOverTime(oldEncoderValues, timeStartTurn)
    delta_phi = (dirToRobot - turned)
    
    if abs(delta_phi) > 1:
        vecToMotor(dirToVec(delta_phi))
    else:    
        changeState(State.SEARCH_OBJECT)
        sawRobotTurnGreen = False        
   

def approachObject():
    global worldTime, goalIsVisible
    minDist = 10 # Minimum distance to the object to change state
    minAngle = 10# The front angle interval is [-minAngle, minAngle]

    isVisible, direction = directionToObject()
    if isVisible:
        vecToMotor(dirToVec(direction))
    else:
        changeState(State.SEARCH_OBJECT)

    # Check if arrived
    dist = distanceToObject()
    isInFront = (direction < minAngle and direction > -minAngle) \
        or (direction > -math.pi+minAngle or direction > math.pi-minAngle)
    if dist < minDist and isInFront:
        # Close enough to object and is looking straight at it
        changeState(State.MOVE_AROUND_OBJECT)


def moveAroundObject():
    global worldTime, goalIsVisible
    
    #----- Input -----#
    #distToObject = distanceToObject()  # isn't being used?
    objectVisible, dirToObject = directionToObject()
    
    #----- Check visible -----#
    if not objectVisible:
        changeState(State.SEARCH_OBJECT)
        
    # check if goal is  visible every 2 seconds
    elif worldTime % 2048 == 0 and not goalIsVisible:
        changeState(State.PUSH_OBJECT)
           
    else:     
        #----- Parameters -----#
        moveVec = np.array([0, 1]) # Robot move vector (X is to the right, Y is forward)
        minDist = 300
    
        idxF = 0
        if dirToObject < 0:
            idxF = 4# If robot moving "backward", front sensor is behind
    
        irF = irs[idxF].getValue()# IR front
        irFR = irs[idxF+1].getValue()# IR front-right
        irR = irs[idxF+2].getValue()# IR right
 
        #----- Force field -----#
        objVec = dirToVec(dirToObject)
    
        # Force to move around the object
        moveVec = np.array([-objVec[1], objVec[0]])
    
        # Force to keep distance from object
        if irR > IR_SENSOR_LIMIT:
            moveVec += objVec
        elif irR < minDist:
            moveVec += -objVec
    
        # Force to deviate from obstacles
        if irF < minDist:
            moveVec = np.array([-1,0])
    
        #----- Output - move -----#
        vecToMotor(moveVec)
    

def pushObject():
    global worldTime, goalIsVisible
    
    if not goalIsVisible:
        isFree, direction, _ = freeDirectionToObject()
        dist = distanceToObject()
        if isFree:
            if dist == 0:
                # If pushing the object, push slowly
                vecToMotor(dirToVec(direction)*0.6)
            else:
                # If approaching the object, move fast
                vecToMotor(dirToVec(direction))
        else:
            changeState(State.MOVE_AROUND_OBJECT)  
            leftMotor.setVelocity(-MAX_SPEED)
            rightMotor.setVelocity(-MAX_SPEED)
                 
    else: # goal is visible, take a step back and move around the object
        changeState(State.MOVE_AROUND_OBJECT)
        leftMotor.setVelocity(-MAX_SPEED)
        rightMotor.setVelocity(-MAX_SPEED)

    

def beAGoal():
    # placeholder behaviour, needs an exit condition!
    leftMotor.setVelocity(0)
    rightMotor.setVelocity(0)        # just for use to see that its hit this state
    sendIRMessage("iAmGoal")
    
    # IDEA FOR EXIT CONDITION:
    if canSeeObject():
        # something related to the distance
        distToObject = distanceToObject()
        if distToObject == 0:
            changeState(State.MOVE_AROUND_OBJECT)
            sendMessagetoController('blue')
    


def main():
    global worldTime, goalIsVisible, dirToRobot, sawRobotTurnGreen
    init()
    print('Robot initialized')
    while robot.step(TIME_STEP) != -1:    
        worldTime += TIME_STEP
 
        # process messages
        if worldTime % 2048 == 0: # check messages (from goals) every 2 seconds
            goalIsVisible, msg, dir = checkIRMessages()
            if msg == b'CTG':
                # direction to green robot, with some noise (to prevent some collisions)
                dirToRobot = dir + random.randint(-5, 5) 
                sawRobotTurnGreen = True
            elif goalIsVisible == False:  # no messages, check cameras
                goalIsVisible = canSeeGoal()
        
        # states
        if state == State.SEARCH_OBJECT:
            searchObject()
        elif state == State.TURN_TO_GOAL_ROBOT:
            turnToGoalRobot()
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