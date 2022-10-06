# cooperative-transport
Cooperative multi-robot object transport with obstacle avoidance

Problems/tasks:
1. freeDirectionToBox isn't actually being used I think? it could replace directionToColor() in the 'moveToward' state, but I'm not 100% sure how the how it would alter the conditions in the state (ie. what values correspond to what direction).
2. If we do that ^, then what does the robot do if there is no free space near the box while it is still some distance from the box? (can't do left wall following if there's no wall)
3. Robots think they are at the box if they collide with another robot. We rely only on the front IR distance sensor, perhaps some condition like 'if the box fills over 60% of the field of view' could be added as an output to the freeDirectionToBox function and as a conditon in the 'moveToward' state.
4. moveAround() does nothing, probably needs left-wall following
5. That ^ function needs an exit condition to move to another state (perhaps a combination of some box-seeing condition and a time-elapsed counter)
6. idk what the plan is with the json file, but I've added a csv file that stores the timings for each run (because this is quick for me to manipulate and plot in python later on) 
7. Is it possible to make webots stop running after it does one full set of experiments (ie. once it has done the 'supervisor' code fully), would make it a bit easier to automate the exporting of data/plots without it dumping tiny files of incomplete runs.

For the 'new' implementation stuff we wanted to do:
1. I don't know if we can change the colour of the robot from the controller, but it is possible to add LED lights to protos, perhaps adding those is a work-around.
2. or, what might be possible, use the supervisor to check when the robot decides to become an interim goal (however it determines that...), then delete that robot, and replace it with a new type of robot proto (which is the same form as the pusher, but a different colour, and perhaps a different controller file) at the exact same location.
