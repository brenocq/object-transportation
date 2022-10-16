# cooperative-transport
Cooperative multi-robot object transport with obstacle avoidance

"New" Implementation

1. The pushers and supervisor can now comminucate via emitter and receivers. This is one-way communication from the pusher to the supervisor. The pusher emits a message to the supervisor using the 'sendMessagetoController' function, which only requires as input a string ('blue' or 'green') to tell the supervisor which colour to change it to. The supervisor parses through all messages using its "changeRobotColor" function and performs the colour change for all robots that have sent a message.
2. The 'BE_A_GOAL' state has a placeholder behaviour, and is (for now) activated when the robot sees both the goal and the box while its in the 'searchObject' state (again, this was just to test the colour changing).
3. I think to coordinate which robots change to green, it might be an idea to add an infra-red emitter and reciever to the pushers as well (on top of the radio emitter it uses for communicating to the supervisor). This works on line-of-sight, and can't pass through objects, so I think it would work on the same principles taht we agreed on with using cameras for line-of-sight control.
4. That ^ could be something like 'I tell all the robots that I see not to become a goal, because I've already become a goal', or something similar, which might stop too many of them becoming a goal.
5. We should consider making a second map, one that has a wall or two, to test the new implementation.
