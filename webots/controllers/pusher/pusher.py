import os, sys
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import pusher_globals as g
import pusher_common as common
import struct
import random

########## COLOR_CHANGING ##########
def sendMessageToController(color):
    global radioEmitter
    Name = g.robot.getName()
    # Get the # of the robot from its name
    id = int(Name[Name.find("(")+1:Name.find(")")])

    if color == 'green':
        color_string = b'G'
    elif color == 'blue':
        color_string = b'B'
    else:
        print('invalid color input')
        return

    # Send the number, and the colour we want to be pack and send 
    # the message (we have to use pack to convert it to C...)
    message = struct.pack('I 1s',id, color_string)
    radioEmitter.send(message)

########## GOAL SENSING ##########
#def numVisibleGoals():
#    '''
#    Calculate number of visible goals
#    '''
#    numVisibleGoals = 0
#    images = [g.cams[0].getImage(), g.cams[1].getImage(), g.cams[2].getImage(), g.cams[3].getImage()]
#    visited = [0 for p in range(g.IMAGE_SIZE*g.IMAGE_SIZE*4)]
#    for y in range(g.IMAGE_MAX_ROW):# From 0 to IMAGE_MAX_ROW
#        for i in range(4):
#            for x in range(g.IMAGE_SIZE):
#                r = g.cams[i].imageGetRed(images[i], g.IMAGE_SIZE, x, y)
#                gr = g.cams[i].imageGetGreen(images[i], g.IMAGE_SIZE, x, y)
#                b = g.cams[i].imageGetBlue(images[i], g.IMAGE_SIZE, x, y)
#
#                w = g.IMAGE_SIZE*4
#                gx = x + g.IMAGE_SIZE*i
#                gy = y
#                # Check if pixel is goal pixel (and was not visited before)
#                if g.goalColor.check(r, gr, b) and visited[gy*w + gx] == 0:
#                    numVisibleGoals += 1
#                    # Mark neighbors as visited
#                    stack = [[gx, gy]]
#                    while len(stack) > 0:
#                        gx, gy = stack[len(stack)-1]
#                        stack.pop()
#                        gx = (gx+w)%w
#                        if gy < 0 or gy >= g.IMAGE_SIZE:
#                            continue
#                        i = gx//g.IMAGE_SIZE
#                        r = g.cams[i].imageGetRed(images[i], g.IMAGE_SIZE, gx%g.IMAGE_SIZE, gy)
#                        gr = g.cams[i].imageGetGreen(images[i], g.IMAGE_SIZE, gx%g.IMAGE_SIZE, gy)
#                        b = g.cams[i].imageGetBlue(images[i], g.IMAGE_SIZE, gx%g.IMAGE_SIZE, gy)
#
#                        if g.goalColor.check(r, gr, b) and visited[gy*w + gx] == 0:
#                            visited[gy*w + gx] = 1
#                            stack.append([gx - 1, gy - 1])
#                            stack.append([gx, gy - 1])
#                            stack.append([gx + 1, gy - 1])
#                            stack.append([gx - 1, gy])
#                            stack.append([gx, gy])
#                            stack.append([gx + 1, gy])
#                            stack.append([gx - 1, gy + 1])
#                            stack.append([gx, gy + 1])
#                            stack.append([gx + 1, gy + 1])
#    return numVisibleGoals

########## STATES ##########
def randomWalk():
    # Check if could not see goal before, and now can see 1 goal
    allowStateChange = False
    if randomWalk.shouldCheck == 0:
        allowStateChange = True
        randomWalk.shouldCheck = random.randint(2, 5)*g.TIME_STEP
        canSeeGoalNow, goalDir = common.directionToGoal()
        canSeeObjectNow, objectDir = common.directionToObject()
        if randomWalk.couldSeeGoal and not canSeeGoalNow:
            if canSeeObjectNow:
                if common.angleDistance(randomwalk.objectDir, randomwalk.goalDir) > 90:
                    common.changeState(g.State.BE_A_GOAL)
            else:
                common.changeState(g.State.BE_A_GOAL)
        if canSeeGoalNow and canSeeObjectNow:
            randomWalk.goalDir = goalDir
            randomWalk.objectDir = objectDir
        randomWalk.couldSeeGoal = canSeeGoalNow
    else:
        randomWalk.shouldCheck -= g.TIME_STEP

    # If still doing random walk
    if g.state == g.State.RANDOM_WALK:
        common.randomWalk()
        if not allowStateChange:
            g.state = g.State.RANDOM_WALK

    # Handle changed state
    if g.state != g.State.RANDOM_WALK:
        randomWalk.couldSeeGoal = False
        randomWalk.shouldCheck = 0
randomWalk.couldSeeGoal = False
randomWalk.goalDir = 0
randomWalk.objectDir = 0
randomWalk.shouldCheck = 0

def approachObject():
    common.approachObject()

    canSeeGoal, goalDir = common.directionToGoal()
    canSeeObject, objectDir = common.directionToObject()
    if g.state == g.State.RANDOM_WALK:
        if canSeeObject:
            if common.angleDistance(approachObject.objectDir, approachObject.goalDir) > 90:
                common.changeState(g.State.BE_A_GOAL)
        else:
            common.changeState(g.State.BE_A_GOAL)
    if canSeeGoal and canSeeObject:
        approachObject.goalDir = goalDir
        approachObject.objectDir = objectDir

approachObject.objectDir = 0
approachObject.goalDir = 0

def moveAroundObject():
    # Choose direction when enter the state for the first time
    if not moveAroundObject.directionWasChosen:
        moveAroundObject.directionWasChosen = True
        goalVisible, dirToGoal = common.directionToGoal()
        if goalVisible:
            # Handle dynamic front
            _, dirToObject = common.directionToObject()
            shouldInvert = dirToObject < -90 or dirToObject > 90

            # Choose best direction
            if shouldInvert:
                dirToGoal += 180
                if dirToGoal > 180:
                    dirToGoal -= 360

            if dirToGoal > 0:# If goal to the right
                moveAroundObject.clockwise = True# Move clockwise
            else:# If goal to the left
                moveAroundObject.clockwise = False# Move anti-clockwise

    # Move around object
    common.moveAroundObject(moveAroundObject.clockwise)

    # If timer reached zero
    if moveAroundObject.timer == 0:
        common.changeState(g.State.RANDOM_WALK)
    moveAroundObject.timer -= g.TIME_STEP

    # Reset direction/timer when leave the state
    if g.state != g.State.MOVE_AROUND_OBJECT:
        moveAroundObject.directionWasChosen = False
        moveAroundObject.timer = moveAroundObject.timeout
moveAroundObject.directionWasChosen = False
moveAroundObject.clockwise = True
moveAroundObject.timeout = 1024*2*60# Timeout 2min
moveAroundObject.timer = moveAroundObject.timeout

def pushObject():
    common.pushObject()

    # If timer reached zero, can't find goal anymore
    if pushObject.timer == 0:
        common.changeState(g.State.RANDOM_WALK)
    pushObject.timer -= g.TIME_STEP

    # Reset timer when leave the state
    if g.state != g.State.PUSH_OBJECT:
        moveAroundObject.directionWasChosen = False
        moveAroundObject.timer = moveAroundObject.timeout
pushObject.timeout = 1024*2*60# Timeout 2min pushing
pushObject.timer = moveAroundObject.timeout

def beAGoal():
    if beAGoal.transitionWait >= 0:
        # Avoid robots changed changing to be a goal together (blinking robots)
        beAGoal.transitionWait -= 1
        if beAGoal.transitionWait == -1:
            common.changeState(g.State.RANDOM_WALK)
    else:
        goRandomWalk = False
        if beAGoal.firstTime:
            beAGoal.firstTime = False
            # Change color
            sendMessageToController('green')
            # Stop
            g.leftMotor.setVelocity(0.0)
            g.rightMotor.setVelocity(0.0)

        if common.canSeeObject() and common.distanceToObject() == 0:
            print("Too close to object, not a goal anymore")
            goRandomWalk = True

        if common.canSeeGoal():
            print("Can see another goal, not a goal anymore")
            goRandomWalk = True

        # If timer reached zero, can't find goal anymore
        if beAGoal.timer == 0:
            print("Timer timeout")
            goRandomWalk = True
        beAGoal.timer -= g.TIME_STEP

        if goRandomWalk:
            beAGoal.firstTime = True
            sendMessageToController('blue')
            beAGoal.transitionWait = random.randint(1, 5)
beAGoal.firstTime = True
beAGoal.timeout = 1024*2*60# Timeout 2min being a goal
beAGoal.timer = beAGoal.timeout
beAGoal.transitionWait = -1

########## MAIN ##########
def main():
    global radioEmitter
    g.init()
    common.initRobot()

    radioEmitter = g.robot.getDevice('radio emitter')

    print('Robot initialized')
    while g.robot.step(g.TIME_STEP) != -1:
        g.worldTime += g.TIME_STEP
        if g.state == g.State.RANDOM_WALK:
            randomWalk()
        elif g.state == g.State.APPROACH_OBJECT:
            approachObject()
        elif g.state == g.State.MOVE_AROUND_OBJECT:
            moveAroundObject()
        elif g.state == g.State.PUSH_OBJECT:
            pushObject()
        elif g.state == g.State.BE_A_GOAL:
            beAGoal()

if __name__ == '__main__':
    main()
