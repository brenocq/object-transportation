from controller import Robot, Camera, DistanceSensor, Motor
from enum import Enum
import random
# Global definitions
TIME_STEP = 32
MAX_SPEED = 6.28

states = ["SEARCH_BOX", "MOVE_TOWARD", "ARRIVED_AT_BOX", "MOVE_AROUND", "PUSH", "BE_A_GOAL", "RANDOM_WALK"]

state = "SEARCH_BOX"
counter = 0
RW_turning = False  # helps manage the random walk, for when it reaches a wall
turnTimer = random.randint(30, 100) # the amount of timesteps to turn in randomWalk

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
    # NOTE: Checking ALL cameras
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


# to determine which direction the box is in (relative to the robot)
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
        state = "MOVE_TOWARD"
    else:
        state = "RANDOM_WALK"

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
        state = "SEARCH_BOX"
       
    if irs[0].getValue() < 10:  
    # irs sensor typically doesn't get close to 0 
    # before jumping up to 1000 when hitting the box
        state = "ARRIVED_AT_BOX"


def arrivedAtBox():
    print('Arrived At Box')
    global state
    #leftMotor.setVelocity(MAX_SPEED*0)
    #rightMotor.setVelocity(MAX_SPEED*0)
    if canSeeGoal():
        state = "MOVE_AROUND"
    else:
        state = "PUSH"
    
    
def randomWalk():
    print('Random Walking')
    global state, counter, RW_turning, turnTimer
    wallDistParam = 400
    nearWall = (irs[0].getValue() < wallDistParam) \
        or (irs[1].getValue() < wallDistParam)     \
        or (irs[7].getValue() < wallDistParam)
    
    if canSeeBox():
        state = "MOVE_TOWARD"
    
    elif not nearWall and not RW_turning:
        print("   go straight")
        counter = 0
        leftMotor.setVelocity(MAX_SPEED*1)
        rightMotor.setVelocity(MAX_SPEED*1)
    else:
        RW_turning = True
        if counter < turnTimer:
            print("   turn")
            leftMotor.setVelocity(MAX_SPEED*-1)
            rightMotor.setVelocity(MAX_SPEED*1)
            counter += 1
        else:
            RW_turning = False
            turnTimer = random.randint(30, 100)
        #counter = 0
            
   

def moveAround():
    print('Move around')
    global state
    #idk just spin around for now or smth
    leftMotor.setVelocity(MAX_SPEED*0.5)
    rightMotor.setVelocity(MAX_SPEED*-0.5)
    #if irs[0].getValue() < 0.05:
    #    state = "PUSH"

def push():
    print('Push')
    global state
    if not canSeeGoal():
        leftMotor.setVelocity(MAX_SPEED)
        rightMotor.setVelocity(MAX_SPEED)
    else:
        state = "MOVE_AROUND"
    
        

def beAGoal():
    print('Be a goal')

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
        print(" ")
        

if __name__ == '__main__':
    main()
