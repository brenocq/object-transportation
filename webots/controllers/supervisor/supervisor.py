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
            {'pos': [-0.5, 0.0], 'size': [2.0, 0.5]}
        ]
    },
    'middle': {
        'goal': {'pos': [0.0, 1.0]},
        'object': {'pos': [0.0, -1.0]},
        'walls': [
            {'pos': [0.0, 0.0], 'size': [1.5, 0.5]}
        ]
    },
    '2-corners': {
        'goal': {'pos': [1.0, 1.0]},
        'object': {'pos': [-1.0, -1.0]},
        'walls': [
            {'pos': [-0.5, -0.5], 'size': [2.0, 0.1]},
            {'pos': [0.5, 0.5], 'size': [2.0, 0.1]}
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
    sup.getFromDef('BOX').getField('rotation').setSFRotation([0, 0, -1, 0])
    boxSize = sup.getFromDef('BOX_GEOMETRY').getField('size').getSFVec3f()

    # Move goal
    goalPos = maps[experiment['map']]['goal']['pos']
    sup.getFromDef('GOAL').getField('translation').setSFVec3f([goalPos[0], goalPos[1], 0.05])
    sup.getFromDef('GOAL').getField('rotation').setSFRotation([0, 0, -1, 0])
    goalRadius = sup.getFromDef('GOAL_GEOMETRY').getField('radius').getSFFloat()

    # Create walls
    for i in range(len(maps[experiment["map"]]["walls"])):
        wall = maps[experiment["map"]]["walls"][i]
        wallDef = "WALL"+str(i)
        wallString = f"""DEF {wallDef} Wall {{
                            name "wall({i})"
                            translation {wall['pos'][0]} {wall['pos'][1]} 0.11
                            size {wall['size'][0]} {wall['size'][1]} 0.2
                        }}"""
        sup.getRoot().getField('children').importMFNodeFromString(-1, wallString)

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
            if sqrt(dx*dx + dy*dy) < boxSize[0]*sqrt(2.0) * 0.5  + ROBOT_RADIUS + gap:
                freePosition = False
                continue

            # Check wall collision
            for wall in maps[experiment["map"]]["walls"]:
                wallMinX = wall['pos'][0] - wall['size'][0] * 0.5
                wallMaxX = wall['pos'][0] + wall['size'][0] * 0.5
                wallMinY = wall['pos'][1] - wall['size'][1] * 0.5
                wallMaxY = wall['pos'][1] + wall['size'][1] * 0.5
                if robotPos[0] + ROBOT_RADIUS + gap >= wallMinX and \
                        robotPos[0] - ROBOT_RADIUS - gap <= wallMaxX and \
                        robotPos[1] + ROBOT_RADIUS + gap >= wallMinY and \
                        robotPos[1] - ROBOT_RADIUS - gap <= wallMaxY:
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

def saveImageFromRecording(recording, filename):
    initialBoxPos = recording['config']['map']['object']['pos']
    initialGoalPos = recording['config']['map']['goal']['pos']
    boxPaths = []
    for repetition in recording['repetitions']:
        path = {"x": [], "y": []}
        for pos in repetition["path"]:
            path["x"].append(pos[0])
            path["y"].append(pos[1])
        boxPaths.append(path)

    # Plot config
    figSize = 5
    localToPlt = figSize * 0.4 * 72  # 1 point = dpi / 72 pixels
    plt.figure(figsize=[figSize, figSize])
    ax = plt.axes([0.1, 0.1, 0.8, 0.8], xlim=(-ARENA_SIZE*0.5, ARENA_SIZE*0.5), ylim=(-ARENA_SIZE*0.5, ARENA_SIZE*0.5))

    # Plot MIN_BOX_GOAL_DIST
    pointSize = (1.35*recording['config']['minObjectGoalDist'] * localToPlt)**2
    plt.scatter(initialGoalPos[0], initialGoalPos[1], s=pointSize, facecolors='none', edgecolors='k', linestyle='--')

    # Plot initial and goal positions
    plt.scatter(initialBoxPos[0], initialBoxPos[1], s=30, color='r')
    plt.scatter(initialGoalPos[0], initialGoalPos[1], s=50, color='g')

    # Plot paths
    for i, path in enumerate(boxPaths):
        plt.plot(path["x"], path["y"])

    plt.title(f'Map {recording["config"]["map"]["name"]} - {recording["config"]["numRobots"]} {recording["config"]["controller"]}')
    plt.savefig(filename)


def main():
    # Calculate min distance between goal and object
    goalRadius = sup.getFromDef('GOAL_GEOMETRY').getField('radius').getSFFloat()
    boxSize = sup.getFromDef('BOX_GEOMETRY').getField('size').getSFVec3f()
    minObjectGoalDist = boxSize[0]*math.sqrt(2.0)*0.5+goalRadius

    # Experiments to be performed
    experiments = [
        {'numRepetitions': 1, 'timeout': 20*60,'numRobots': 5, 'map': 'reference', 'controller': 'pusher'},
        {'numRepetitions': 1, 'timeout': 40*60,'numRobots': 5, 'map': 'corner', 'controller': 'pusher'},
        {'numRepetitions': 1, 'timeout': 40*60,'numRobots': 5, 'map': 'middle', 'controller': 'pusher'},
        {'numRepetitions': 1, 'timeout': 60*60,'numRobots': 5, 'map': '2-corners', 'controller': 'pusher'},

        {'numRepetitions': 1, 'timeout': 20*60,'numRobots': 10, 'map': 'reference', 'controller': 'pusher'},
        {'numRepetitions': 1, 'timeout': 40*60,'numRobots': 10, 'map': 'corner', 'controller': 'pusher'},
        {'numRepetitions': 1, 'timeout': 40*60,'numRobots': 10, 'map': 'middle', 'controller': 'pusher'},
        {'numRepetitions': 1, 'timeout': 60*60,'numRobots': 10, 'map': '2-corners', 'controller': 'pusher'},

        {'numRepetitions': 1, 'timeout': 20*60,'numRobots': 20, 'map': 'reference', 'controller': 'pusher'},
        {'numRepetitions': 1, 'timeout': 40*60,'numRobots': 20, 'map': 'corner', 'controller': 'pusher'},
        {'numRepetitions': 1, 'timeout': 40*60,'numRobots': 20, 'map': 'middle', 'controller': 'pusher'},
        {'numRepetitions': 1, 'timeout': 60*60,'numRobots': 20, 'map': '2-corners', 'controller': 'pusher'},
    ]

    # Recording of each experiment
    for experiment in experiments:
        recording = {'config': {}, 'repetitions': []} # Recorded data
        recording['config']['numRepetitions'] = experiment['numRepetitions']
        recording['config']['numRobots'] = experiment['numRobots']
        recording['config']['controller'] = experiment['controller']
        recording['config']['map'] = maps[experiment['map']]
        recording['config']['map']['name'] = experiment['map']
        recording['config']['timeout'] = experiment['timeout']
        recording['config']['timeStep'] = TIME_STEP
        recording['config']['minObjectGoalDist'] = minObjectGoalDist

        for repetition in range(experiment["numRepetitions"]):
            # Create world
            createWorld(experiment)

            # Run repetition
            finishRepetition = False
            currRepetitionTime = 0.0
            data = {"success": False, "time": 0, "distance": 0, "path": []}
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

                    shouldStop = False
                    distance = sqrt(dx*dx + dy*dy)
                    if distance <= minObjectGoalDist:
                        shouldStop = True
                        data["success"] = True
                        print(f'({repetition+1}/{experiment["numRepetitions"]}) Success')

                    # Check timeout
                    if currRepetitionTime >= experiment["timeout"]:
                        shouldStop = True
                        data["success"] = False
                        print(f'({repetition+1}/{experiment["numRepetitions"]}) Timeout')

                    if shouldStop:
                        data["path"].append(boxPos[:2])
                        data["time"] = currRepetitionTime
                        data["distance"] = distance
                        finishRepetition = True

            # Destroy world
            destroyWorld(experiment)
            # Save repetition recording
            recording["repetitions"].append(data)

        # Save experiment recordings
        filename = f'{experiment["map"]}-{experiment["controller"]}-{experiment["numRobots"]}_robots-{experiment["numRepetitions"]}_rep'
        filenameJson = filename + '.json'
        filenamePng = filename + '.png'
        with open(filenameJson, 'w') as outfile:
            json.dump(recording, outfile)
        print(f'Saved recording to {filenameJson}')
        saveImageFromRecording(recording, filenamePng)
        print(f'Saved image to {filenamePng}')


if __name__ == '__main__':
    main()
