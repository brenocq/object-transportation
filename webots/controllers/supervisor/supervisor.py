from controller import Supervisor
import random
from math import sqrt
import datetime
import json
import csv
import matplotlib.pyplot as plt
import pandas as pd

########## GLOBAL VARIABLES ##########
TIME_STEP = 128 # Record positions every 128ms
#NUM_ROBOTS = 6 # Number of robots to spawn

ARENA_SIZE = 0.4
ROBOT_RADIUS = 0.02
MIN_BOX_GOAL_DIST = 0.13  # CHANGED from 0.01!

sup = Supervisor()

def spawnRobots(NUM_ROBOTS):
    '''
        Spawn robots in random valid positions
    '''
    gap = ROBOT_RADIUS
    goalPos = sup.getFromDef('GOAL').getField('translation').getSFVec3f()
    goalRadius = sup.getFromDef('GOAL_GEOMETRY').getField('radius').getSFFloat()
    boxPos = sup.getFromDef('BOX').getField('translation').getSFVec3f()
    boxSize = sup.getFromDef('BOX_GEOMETRY').getField('size').getSFVec3f()
    wallPos = sup.getFromDef('WALL').getField('translation').getSFVec3f()
    wallSize = sup.getFromDef('WALL').getField('size').getSFVec3f()
    
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
                
            # check wall collison
            wallPos_x_max = wallPos[0] + wallSize[0] + 0.3
            wallPos_x_min = wallPos[0] - wallSize[0] - 0.3 # the wall is so narrow that they still spawn into it
            wallPos_y_max = wallPos[1] + wallSize[1]/1.9
            wallPos_y_min = wallPos[1] - wallSize[1]/1.9
            
            if (wallPos_x_min < robotPos[0] - ROBOT_RADIUS - gap and wallPos_x_max > robotPos[0] + ROBOT_RADIUS + gap) or (wallPos_y_min < robotPos[1] - ROBOT_RADIUS - gap and wallPos_y_max > robotPos[1] + ROBOT_RADIUS + gap):
                freePosiition = False
            
                
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

def destroyRobots(NUM_ROBOTS):
    '''
        Destroy all spawned robots
    '''
    for i in range(NUM_ROBOTS):
        robotDef = "PUSHER"+str(i)
        sup.getFromDef(robotDef).remove()

def outputs(timings):
    df = pd.DataFrame(timings)
    df.columns = df.iloc[0]
    df = df[1:]
    # show results in terminal
    sortedData = df.groupby(by=['NumRobots','NewImplementation'])
    output = pd.DataFrame()
    output['Success Rate'] = sortedData['Success'].mean()
    output['Mean Successful Duration'] = sortedData['Duration'].mean()
    output['Duration Variance'] = sortedData['Duration'].var()
    print(output)
    
    fig = df.groupby(['NumRobots','NewImplementation']).mean()['Duration'].unstack() \
        .plot(title='Duration of Successful Box Moves Over Different Numbers of Robots', \
        xlabel='Number of Robots', ylabel='Duration (s)').get_figure()
    fig.savefig('Duration over numRobots Plot.png')
    print('plot saved')
    

def main():
    numberRobotsPerTrial = [1, 3, 5, 10]
    numRepetitions = 3 # Number of repetitions to be performed (box reaching goal or timeout)
    currRepetitionTime = 0.0 # Current repetition time
    maxRepetitionTime = 10.0 # Timeout in seconds
    
    
    recording = {"config": {}, "repetitions": []} # Recorded data
    recording['config']['numRobots'] = numberRobotsPerTrial
    recording['config']['timeStep'] = TIME_STEP
    recording['config']['maxRepetitionTime'] = maxRepetitionTime
    recording['config']['numRepetitions'] = numRepetitions
    recording['config']['initalGoalPosition'] = sup.getFromDef('GOAL').getField('translation').getSFVec3f()
    recording['config']['initalBoxPosition'] = sup.getFromDef('BOX').getField('translation').getSFVec3f()
    initialGoalRotation = sup.getFromDef('GOAL').getField('rotation').getSFRotation()
    initialBoxRotation = sup.getFromDef('GOAL').getField('rotation').getSFRotation()

    timings = []
    header = ['NumRobots', 'NewImplementation', 'Success', 'Duration']
    timings.append(header)
    print(f'Configuration:\n  - Num robots: {numberRobotsPerTrial}\n  - Repetitions: {numRepetitions}\n  - Max time: {maxRepetitionTime}s')
    print('Recording...')
    boxPosRecording = []
    
    for NUM_ROBOTS in numberRobotsPerTrial:
        currRepetition = 0
        print("NUMBER OF ROBOTS", NUM_ROBOTS)
        spawnRobots(NUM_ROBOTS)
        
        while sup.step(TIME_STEP) != -1 and currRepetition < numRepetitions:
            # Advance recording time
            currRepetitionTime += TIME_STEP/1000.0
    
            goalPos = sup.getFromDef('GOAL').getField('translation').getSFVec3f()
            boxPos = sup.getFromDef('BOX').getField('translation').getSFVec3f()
            boxPosRecording.append(boxPos)
    
            # Check if should stop repetition
            dx = goalPos[0] - boxPos[0]
            dy = goalPos[1] - boxPos[1]
            
            #print('distance = ', sqrt(dx*dx + dy*dy))
            
            if sqrt(dx*dx + dy*dy) <= MIN_BOX_GOAL_DIST or currRepetitionTime >= maxRepetitionTime:
                # Add repetition data to recording
                repetitionData = {}
                repetitionData["boxPositions"] = boxPosRecording
                repetitionData["currRepetitionTime"] = currRepetitionTime
                recording["repetitions"].append(repetitionData)
                print(f'Finished repetition {currRepetition+1}/{numRepetitions}')
    
                # Reset world
                destroyRobots(NUM_ROBOTS)
                sup.getFromDef('GOAL').getField('translation').setSFVec3f(recording['config']['initalGoalPosition'])
                sup.getFromDef('BOX').getField('translation').setSFVec3f(recording['config']['initalBoxPosition'])
                sup.getFromDef('GOAL').getField('rotation').setSFRotation(initialGoalRotation)
                sup.getFromDef('BOX').getField('rotation').setSFRotation(initialBoxRotation)
                if currRepetition != (numRepetitions - 1):
                    spawnRobots(NUM_ROBOTS)
                
                # print and save time taken for box to reach goal
                if sqrt(dx*dx + dy*dy) <= MIN_BOX_GOAL_DIST:
                	timings.append([NUM_ROBOTS, False, True, currRepetitionTime])
                	print(f'Successful box movement duration: {currRepetitionTime}.')
                else:
                	timings.append([NUM_ROBOTS, False, False, None])
                	print('Task not completed within time limit.')
                	
                # Advance to next repetition
                currRepetitionTime = 0.0
                boxPosRecording = []
                currRepetition += 1          
            

    
    # Save recording to file
    filename = 'recording_'+str(datetime.datetime.now().timestamp())+'.json'
    with open(filename, 'w') as outfile:
        json.dump(recording, outfile)
    print(f'Saved recording to {filename}')
    
    filename = 'expTimings.csv'
    
    # Save experiment timings to csv (for easier visualisation)
    with open(filename, 'w') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerows(timings)
        print('hi')
    print(f'Saved timings to {filename}')
    
    
    outputs(timings)
    
    
    return
    

if __name__ == '__main__':
    main()