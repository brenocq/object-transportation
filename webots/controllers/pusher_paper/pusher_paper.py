import os, sys
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import pusher_globals as g
import pusher_common as common

########## STATES ##########
def randomWalk():
    common.randomWalk()

def approachObject():
    common.approachObject()

def moveAroundObject():
    common.moveAroundObject(clockwise = True)

def pushObject():
    common.pushObject()

########## MAIN ##########
def main():
    g.init()
    common.initRobot()
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

if __name__ == '__main__':
    main()
