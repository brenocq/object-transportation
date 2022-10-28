from controller import Supervisor, Receiver
import random
from math import sqrt, pi
import datetime
import json
import csv
import matplotlib.pyplot as plt
import pandas as pd
import math
import struct

########## GLOBAL VARIABLES ##########
TIME_STEP = 256 # Record positions every 128ms
ARENA_SIZE = 3.0
WALL_THICKNESS = 0.01
ROBOT_RADIUS = 0.02

maps = {
    'reference': {
        'goal': {'pos': [1.0, 1.0]},
        'object': {'pos': [-1.0, -1.0]},
        'walls': []
    },
    'corner': {
        'goal': {'pos': [-1.0, 1.0]},
        'object': {'pos': [-1.0, -1.0]},
        'walls': [
            {'pos': [-0.5, 0.0], 'size': [2.0, 0.1]}
        ]
    },
    'middle': {
        'goal': {'pos': [0.0, 1.0]},
        'object': {'pos': [0.0, -1.0]},
        'walls': [
            {'pos': [0.0, 0.0], 'size': [2.0, 0.1]}
        ]
    },
    '2-corners': {
        'goal': {'pos': [1.0, 1.0]},
        'object': {'pos': [-1.0, -1.0]},
        'walls': [
            {'pos': [0.0, 0.0], 'size': [2.0, 0.1]}
        ]
    }
}

sup = Supervisor()

# get the message reciever
receiver = sup.getDevice('radio receiver')
receiver.enable(TIME_STEP)

def createWorld(experiment):
    '''
        Spawn robots in random valid positions and create walls
    '''

    # Move object
    boxPos = maps[experiment['map']]['object']['pos']
    sup.getFromDef('BOX').getField('translation').setSFVec3f([boxPos[0], boxPos[1], 0.05])
    boxSize = sup.getFromDef('BOX_GEOMETRY').getField('size').getSFVec3f()

    # Move goal
    goalPos = maps[experiment['map']]['goal']['pos']
    sup.getFromDef('GOAL').getField('translation').setSFVec3f([goalPos[0], goalPos[1], 0.05])
    goalRadius = sup.getFromDef('GOAL_GEOMETRY').getField('radius').getSFFloat()

    # Create walls
    for i in range(len(maps[experiment["map"]]["walls"])):
        pass

    # Create robots
    gap = ROBOT_RADIUS
    robotsPos = []
    for i in range(experiment["numRobots"]):
        # Random free position
        freePosition = False
        while not freePosition:
            freePosition = True
            minArenaSize = -1*ARENA_SIZE/2 + ROBOT_RADIUS + WALL_THICKNESS + gap
            maxArenaSize = ARENA_SIZE/2 - ROBOT_RADIUS - WALL_THICKNESS - gap
            robotPos = [ random.uniform(minArenaSize, maxArenaSize), random.uniform(minArenaSize, maxArenaSize) ]

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

        # Random orientation
        randomAngle = random.uniform(0, 3.1415926535*2)
        robotOri = [ 0, 0, math.sin(randomAngle/2.0), math.cos(randomAngle/2.0)]

        # Robot creation string
        robotsPos.append(robotPos)
        robotDef = "PUSHER"+str(i)
        robotString = f"""DEF {robotDef} Pusher {{
                            name "pusher({i})"
                            translation {robotPos[0]} {robotPos[1]} 0.016
                            rotation {robotOri[0]} {robotOri[1]} {robotOri[2]} {robotOri[3]}
                            controller "{experiment["controller"]}"
                        }}"""

        # Add robot to root node
        sup.getRoot().getField('children').importMFNodeFromString(-1, robotString)

def destroyWorld(experiment):
    '''
        Destroy all spawned robots and walls
    '''
    for i in range(experiment["numRobots"]):
        robotDef = "PUSHER"+str(i)
        sup.getFromDef(robotDef).remove()
    for i in range(len(maps[experiment["map"]]["walls"])):
        wallDef = "WALL"+str(i)
        sup.getFromDef(wallDef).remove()

def handleRobotMessage():
    # the message is the number of the robot, and a string (B or G) which is what colour to be
    msg = receiver.getData() # read the message at the head of the receiver queue
    msg = struct.unpack("I 1s",msg) # unpack the first element of the mssage
    num = msg[0]
    color = msg[1]
    print('PUSHER'+str(num), 'COLOR=', color)

    # get the pusher and the color field
    pusher = sup.getFromDef('PUSHER'+str(num))
    color_field = pusher.getField("color")

    # change the color of the robot
    if color == b"G":
        #print('green!')
        color_field.setSFColor([0, 1, 0])
    elif color == b"B":
        #print('blue!')
        color_field.setSFColor([0, 0, 1])

    receiver.nextPacket() # remove the message at the head of the queue

def main():
    # Calculate min distance between goal and object
    goalRadius = sup.getFromDef('GOAL_GEOMETRY').getField('radius').getSFFloat()
    boxSize = sup.getFromDef('BOX_GEOMETRY').getField('size').getSFVec3f()
    minObjectGoalDist = boxSize[0]*math.sqrt(2.0)*0.5+goalRadius

    # Experiments to be performed
    experiments = [
        {'numRepetitions': 2, 'timeout': 10*60,'numRobots': 5, 'map': 'reference', 'controller': 'pusher_paper'},
        {'numRepetitions': 0, 'timeout': 10*60,'numRobots': 10, 'map': 'reference', 'controller': 'pusher_paper'},
        {'numRepetitions': 0, 'timeout': 10*60,'numRobots': 15, 'map': 'reference', 'controller': 'pusher_paper'},
        {'numRepetitions': 0, 'timeout': 10*60,'numRobots': 20, 'map': 'reference', 'controller': 'pusher_paper'},
    ]

    # Recording of each experiment
    recordings = []
    for experiment in experiments:
        recording = {'config': {}, 'repetitions': []} # Recorded data
        recording['config']['numRepetitions'] = experiment['numRepetitions']
        recording['config']['numRobots'] = experiment['numRobots']
        recording['config']['timeout'] = experiment['timeout']
        recording['config']['timeStep'] = TIME_STEP
        recording['config']['minObjectGoalDist'] = minObjectGoalDist
        recording['config']['map'] = maps[experiment['map']]

        for repetition in range(experiment["numRepetitions"]):
            # Create world
            createWorld(experiment)

            # Run repetition
            finishRepetition = False
            currRepetitionTime = 0.0
            data = {"success": False, "path": []}
            while not finishRepetition:
                if sup.step(TIME_STEP) != -1:
                    # Advance recording time
                    currRepetitionTime += TIME_STEP/1000.0

                    # Check for messages from pushers wanting to change colour
                    while receiver.getQueueLength() > 0:
                        handleRobotMessage()

                    # Add path data
                    boxPos = sup.getFromDef('BOX').getField('translation').getSFVec3f()
                    if len(data['path']):
                        lastBoxPos = data['path'][-1]
                        dx = lastBoxPos[0] - boxPos[0]
                        dy = lastBoxPos[1] - boxPos[1]
                        if sqrt(dx*dx + dy*dy) >= 0.01:
                            data["path"].append(boxPos[:2])
                    else:
                        data["path"].append(boxPos[:2])

                    # Check if box reached the goal
                    goalPos = sup.getFromDef('GOAL').getField('translation').getSFVec3f()
                    dx = goalPos[0] - boxPos[0]
                    dy = goalPos[1] - boxPos[1]
                    if sqrt(dx*dx + dy*dy) <= minObjectGoalDist:
                        print("Finished repetition (success)")
                        data["success"] = True
                        finishRepetition = True

                    # Check timeout
                    if currRepetitionTime >= experiment["timeout"]:
                        print("Finished repetition (timeout failed)")
                        data["success"] = False
                        finishRepetition = True
            # Destroy world
            destroyWorld(experiment)
            # Save repetition recording
            recording["repetitions"].append(data)

        # Save experiment recording
        recordings.append(recording)

    # Save experiment recordings
    filename = 'experiment_result-'+str(int(datetime.datetime.now().timestamp()))+'.json'
    with open(filename, 'w') as outfile:
        json.dump(recordings, outfile)
    print(f'Saved recording to {filename}')

if __name__ == '__main__':
    main()
