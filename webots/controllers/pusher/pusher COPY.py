from controller import Robot, Camera, DistanceSensor, Motor
from enum import Enum
# Global definitions
TIME_STEP = 32
MAX_SPEED = 6.28

states = ["SEARCH_BOX", "MOVE_TOWARD", "ARRIVED_AT_BOX", "MOVE_AROUND", "PUSH", "BE_A_GOAL"]

# Robot states
class State(Enum):
    SEARCH_BOX = 0
    MOVE_TOWARD = 1
    ARRIVED_AT_BOX = 2
    MOVE_AROUND = 3
    PUSH = 4
    BE_A_GOAL = 5
    
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
    return canSeeColor(110, 255, 0, 60, 0, 60)  # 32 chanegd to 60 for brighter sides of the box

def canSeeGoal():
    return canSeeColor(0, 60, 127, 255, 0, 60) 

def canSeeColor(rmin, rmax, gmin, gmax, bmin, bmax):
    # NOTE: Checking all cameras (more of a global line-of-sight check)
    for i in range(4):
        cam = cams[i]
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



def directionToColor(rmin, rmax, gmin, gmax, bmin, bmax):
    # NOTE: Only checking one row in the image
    cameras = [0,1,2,3]
    direction = ['front', 'right', 'back', 'left']
    for i in cameras:
        cam = cams[i]
        w = cam.getWidth()
        h = cam.getHeight()
        image = cam.getImage()
        # an array to store which pixels we can see the object in
        # (helps to determine which direction to move in)
        index_target = []
        for x in range(w):
            for y in range(h):
                r = cam.imageGetRed(image, w, x, y)
                g = cam.imageGetGreen(image, w, x, y)
                b = cam.imageGetBlue(image, w, x, y)
                if r >= rmin and r <= rmax and g >= gmin and g <= gmax and b >= bmin and b <= bmax:
                    index_target.append(x)
                    
        # object found, 
        # return the camera that found it and where in the image it was found
        if index_target:  
            direction = direction[i]
            center = sum(index_target)/len(index_target)
            return [direction, center] 
    
    return [False, 0] # object not found
    
        
        
            
######### MOVEMENT CONTROLLERS   ###########

def moveForward():
    print("  going straight ahead")
    leftMotor.setVelocity(MAX_SPEED*1)
    rightMotor.setVelocity(MAX_SPEED*1)

def moveSlightLeft():
    print("  turning slight left")
    leftMotor.setVelocity(MAX_SPEED*0.5)
    rightMotor.setVelocity(MAX_SPEED*1)

def moveSlightRight():
    print("  turning slight right")
    leftMotor.setVelocity(MAX_SPEED*1)
    rightMotor.setVelocity(MAX_SPEED*0.5)

def TurnRight():
    print("  turning right")
    leftMotor.setVelocity(MAX_SPEED*1)
    rightMotor.setVelocity(MAX_SPEED*-1)

def TurnLeft():
    print("  turning left")
    leftMotor.setVelocity(MAX_SPEED*-1)
    rightMotor.setVelocity(MAX_SPEED*1)


########## STATES ##########

def searchBox():
    print('Search box')
    global state
    leftMotor.setVelocity(MAX_SPEED*-0.5)
    rightMotor.setVelocity(MAX_SPEED*0.5)
    if canSeeBox():
        print('can see box')
        state = State.MOVE_TOWARD

def moveToward():
    print('Move forward')
    global state
    [direction, center] = directionToColor(110, 255, 0, 60, 0, 60) # find where the box is
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
        state = State.SEARCH_BOX
       
    if irs[0].getValue() < 10:  
    # irs sensor typically doesn't get close to 0 before jumping up to 1000 when hitting the box
        state = State.ARRIVED_AT_BOX


def arrivedAtBox():
    print('Arrived At Box')
    leftMotor.setVelocity(MAX_SPEED*0)
    rightMotor.setVelocity(MAX_SPEED*0)
    if canSeeGoal():
        state = State.MOVE_AROUND
        print("i should move around this")
    else:
        state = State.MOVE_AROUND
        print("i should push")
    
        
   

def moveAround():
    print('Move around')
    global state
    #idk just spin for now or smth
    leftMotor.setVelocity(MAX_SPEED*0.5)
    rightMotor.setVelocity(MAX_SPEED*-0.5)
    #if irs[0].getValue() < 0.05:
    #    state = State.PUSH

def push():
    print('Push')
    if not canSeeGoal():
        leftMotor.setVelocity(MAX_SPEED)
        rightMotor.setVelocity(MAX_SPEED)
    else:
        state = State.MOVE_AROUND
        

def beAGoal():
    print('Be a goal')

def main():
    init()
    print('Robot initialized')
    while robot.step(TIME_STEP) != -1:  
         
        if state is State.SEARCH_BOX:
            searchBox()
        elif state is State.MOVE_TOWARD:
            moveToward()
        elif state is State.ARRIVED_AT_BOX:
            arrivedAtBox()
        elif state is State.MOVE_AROUND:
            moveAround()
        elif state is State.PUSH:
            push()
        elif state is State.BE_A_GOAL:
            beAGoal()
        print(state)
        print(" ")
        

if __name__ == '__main__':
    main()
