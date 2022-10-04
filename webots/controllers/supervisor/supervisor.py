from controller import Supervisor
import random
from math import sqrt
import datetime
import json

########## GLOBAL VARIABLES ##########
TIME_STEP = 128 # Record positions every 128ms
NUM_ROBOTS = 3 # Number of robots to spawn
ARENA_SIZE = 0.4
ROBOT_RADIUS = 0.02
MIN_BOX_GOAL_DIST = 0.01

sup = Supervisor()

def spawnRobots():
    '''
        Spawn robots in random valid positions
    '''
    gap = ROBOT_RADIUS
    goalPos = sup.getFromDef('GOAL').getField('translation').getSFVec3f()
    goalRadius = sup.getFromDef('GOAL_GEOMETRY').getField('radius').getSFFloat()
    boxPos = sup.getFromDef('BOX').getField('translation').getSFVec3f()
    boxSize = sup.getFromDef('BOX_GEOMETRY').getField('size').getSFVec3f()
    robotsPos = []

    for i in range(NUM_ROBOTS):
        # Random free position
        freePosition = False
        while not freePosition:
            freePosition = True
            robotPos = [ random.uniform(-ARENA_SIZE, ARENA_SIZE), random.uniform(-ARENA_SIZE, ARENA_SIZE) ]

            # Check goal collision
            dx = robotPos[0] - goalPos[0]
            dy = robotPos[1] - goalPos[1]
            if sqrt(dx*dx + dy*dy) < goalRadius + ROBOT_RADIUS + gap:
                freePosition = False
                continue

            # Check box collision
            dx = robotPos[0] - boxPos[0]
            dy = robotPos[1] - boxPos[1]
            if sqrt(dx*dx + dy*dy) < boxSize[0] * sqrt(2.0) * 0.5 + ROBOT_RADIUS + gap:
                freePosition = False
                continue

            # Check robot collision
            for otherPos in robotsPos:
                dx = robotPos[0] - otherPos[0]
                dy = robotPos[1] - otherPos[1]
                if sqrt(dx*dx + dy*dy) < 2*ROBOT_RADIUS + gap:
                    freePosition = False
                    continue

        # Robot creation string
        robotsPos.append(robotPos)
        robotDef = "PUSHER"+str(i)
        robotString = f"""DEF {robotDef} Pusher {{
                            translation {robotPos[0]} {robotPos[1]} 0.016
                        }}"""

        # Add robot to root node
        sup.getRoot().getField('children').importMFNodeFromString(-1, robotString)

def destroyRobots():
    '''
        Destroy all spawned robots
    '''
    for i in range(NUM_ROBOTS):
        robotDef = "PUSHER"+str(i)
        sup.getFromDef(robotDef).remove()


def main():
    currRepetition = 0
    numRepetitions = 5 # Number of repetitions to be performed (box reaching goal or timeout)
    currRepetitionTime = 0.0 # Current repetition time
    maxRepetitionTime = 10.0 # Timeout in seconds
    recording = {"config": {}, "repetitions": []} # Recorded data
    recording['config']['numRobots'] = NUM_ROBOTS
    recording['config']['maxRepetitionTime'] = maxRepetitionTime
    recording['config']['numRepetitions'] = numRepetitions
    recording['config']['initalGoalPosition'] = sup.getFromDef('GOAL').getField('translation').getSFVec3f()
    recording['config']['initalBoxPosition'] = sup.getFromDef('BOX').getField('translation').getSFVec3f()
    initialGoalRotation = sup.getFromDef('GOAL').getField('rotation').getSFRotation()
    initialBoxRotation = sup.getFromDef('GOAL').getField('rotation').getSFRotation()

    spawnRobots()

    print(f'Configuration:\n  - Repetitions: {numRepetitions}\n  - Max time: {maxRepetitionTime}s')
    print('Recording...')
    boxPosRecording = []
    while sup.step(TIME_STEP) != -1 and currRepetition < numRepetitions:
        # Advance recording time
        currRepetitionTime += TIME_STEP/1000.0

        goalPos = sup.getFromDef('GOAL').getField('translation').getSFVec3f()
        boxPos = sup.getFromDef('BOX').getField('translation').getSFVec3f()
        boxPosRecording.append(boxPos)

        # Check if should stop repetition
        dx = goalPos[0] - boxPos[0]
        dy = goalPos[1] - boxPos[1]
        if sqrt(dx*dx + dy*dy) <= MIN_BOX_GOAL_DIST or currRepetitionTime >= maxRepetitionTime:
            # Add repetition data to recording
            repetitionData = {}
            repetitionData["boxPositions"] = boxPosRecording
            repetitionData["currRepetitionTime"] = currRepetitionTime
            recording["repetitions"].append(repetitionData)
            print(f'Finished repetition {currRepetition+1}/{numRepetitions}')

            # Reset world
            destroyRobots()
            sup.getFromDef('GOAL').getField('translation').setSFVec3f(recording['config']['initalGoalPosition'])
            sup.getFromDef('BOX').getField('translation').setSFVec3f(recording['config']['initalBoxPosition'])
            sup.getFromDef('GOAL').getField('rotation').setSFRotation(initialGoalRotation)
            sup.getFromDef('BOX').getField('rotation').setSFRotation(initialBoxRotation)
            spawnRobots()

            # Advance to next repetition
            currRepetitionTime = 0
            boxPosRecording = []
            currRepetition += 1

    # Save recording to file
    filename = 'recording_'+str(datetime.datetime.now().timestamp())+'.json'
    with open(filename, 'w') as outfile:
        json.dump(recording, outfile)
    print(f'Saved recording to {filename}')

if __name__ == '__main__':
    main()