import os, sys
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import pusher_globals as g
import pusher_common as common
import struct

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
def searchObject():
    common.searchObject()

def approachObject():
    common.approachObject()

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

def beAGoal():
    sendMessageToController('green')
    if False:
        sendMessageToController('blue')

########## MAIN ##########
def main():
    global radioEmitter
    g.init()
    common.initRobot()

    radioEmitter = g.robot.getDevice('radio emitter')

    print('Robot initialized')
    while g.robot.step(g.TIME_STEP) != -1:
        g.worldTime += g.TIME_STEP
        if g.state == g.State.SEARCH_OBJECT:
            searchObject()
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
