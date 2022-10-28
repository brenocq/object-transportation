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

########## STATES ##########
def randomWalk():
    # Check if could not see goal before, and now can see 1 goal
    allowStateChange = False
    if randomWalk.shouldCheck == 0:
        allowStateChange = True
        randomWalk.shouldCheck = g.TIME_STEP
        canSeeGoalNow, goalDir = common.directionToGoal()
        canSeeObjectNow, objectDir = common.directionToObject()
        if randomWalk.couldSeeGoal and not canSeeGoalNow:
            if canSeeObjectNow:
                if randomWalk.goodDirMeasure:
                    if common.angleDistance(randomWalk.objectDir, randomWalk.goalDir) > 90:
                        print(f'[randomWalk] Turn into goal, can see object ({randomWalk.objectDir},{randomWalk.goalDir} -> {common.angleDistance(randomWalk.objectDir, randomWalk.goalDir)})')
                        common.changeState(g.State.BE_A_GOAL)
            else:
                common.changeState(g.State.BE_A_GOAL)
                print(f'[randomWalk] Turn into goal, CANT see object ({randomWalk.objectDir},{randomWalk.goalDir} -> {common.angleDistance(randomWalk.objectDir, randomWalk.goalDir)})')
        if canSeeGoalNow and canSeeObjectNow:
            randomWalk.goalDir = goalDir
            randomWalk.objectDir = objectDir
            randomWalk.goodDirMeasure = True
        else:
            randomWalk.goodDirMeasure = False
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
randomWalk.goodDirMeasure = False
randomWalk.couldSeeGoal = False
randomWalk.goalDir = 0
randomWalk.objectDir = 0
randomWalk.shouldCheck = 0

def approachObject():
    common.approachObject(checkGoal = True)

    canSeeGoal, goalDir = common.directionToGoal()
    canSeeObject, objectDir = common.directionToObject()
    if g.state == g.State.RANDOM_WALK and not canSeeGoal and randomWalk.couldSeeGoal:
        if canSeeObject:
            if approachObject.goodDirMeasure:
                if common.angleDistance(approachObject.objectDir, approachObject.goalDir) > 90:
                    common.changeState(g.State.BE_A_GOAL)
                    print(f'[approachObject] Turn into goal, big angle ({approachObject.objectDir},{approachObject.goalDir} -> {common.angleDistance(approachObject.objectDir, approachObject.goalDir)})')
        else:
            print(f'[approachObject] Turn into goal, CANT see object ({approachObject.objectDir},{approachObject.goalDir} -> {common.angleDistance(approachObject.objectDir, approachObject.goalDir)})')
            common.changeState(g.State.BE_A_GOAL)
    randomWalk.couldSeeGoal = canSeeGoal

    if canSeeGoal and canSeeObject:
        approachObject.goalDir = goalDir
        approachObject.objectDir = objectDir
        approachObject.goodDirMeasure = True
    else:
        approachObject.goodDirMeasure = False
approachObject.goodDirMeasure = False
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

    # Reset direction when leave the state
    if g.state != g.State.MOVE_AROUND_OBJECT:
        moveAroundObject.directionWasChosen = False
moveAroundObject.directionWasChosen = False
moveAroundObject.clockwise = True

def pushObject():
    common.pushObject()

    # If timer reached zero, can't find goal anymore
    if pushObject.timer == 0:
        common.changeState(g.State.RANDOM_WALK)

    # Reset timer when leave the state
    pushObject.timer -= g.TIME_STEP
    if g.state != g.State.PUSH_OBJECT:
        pushObject.directionWasChosen = False
        pushObject.timer = pushObject.timeout
pushObject.timeout = 1024*2*60# Timeout 2min pushing
pushObject.timer = pushObject.timeout

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
            print("[beAGoal] Too close to object, not a goal anymore")
            goRandomWalk = True

        if common.canSeeGoal():
            print("[beAGoal] Can see another goal, not a goal anymore")
            goRandomWalk = True

        # If timer reached zero, can't find goal anymore
        if beAGoal.timer == 0:
            print("[beAGoal] Timer timeout")
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
