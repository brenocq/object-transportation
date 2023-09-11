# box-pushing
Occlusion-Based Object Transportation Around Obstacles With a Swarm of Multiple Miniature Robots

This repository contains the code used in MacRae & Queiroz (2023) to perform our experiments utilising a swarm of miniature occlusion-based robots to transport objects in complex environments. An extension of Chen (2015), we augment the purely vision-based finite state machine to enable the robots to form "sub-goals" that allows observing robots to trace a path around obstacles; allowing the robots to successfully complete the object transportation task, while maintaining the decentralised, communication-free and vision-based implementation of the original Chen (2015) strategy.

The implementation requires the use of Atta (CITE), which can be found here (LINK). 



![purple-divider](https://user-images.githubusercontent.com/7065401/52071927-c1cd7100-2562-11e9-908a-dde91ba14e59.png)

### Abstract

Swarm robotics utilises decentralised self-organising systems to form complex collective behaviours built from the bottom-up using individuals that have simple, yet limited, capabilities. Previous works have explored the use of swarm robotics for the task of transporting objects to a goal position. In this paper, we propose an improvement to an occlusion-based strategy for such a task that enables the strategy to work in environments where obstacles block the line-of-sight between the object and the goal. Our approach allows the robots to form sub-goals; allowing any member of the swarm to establish a wider range of visibility of the goal, ultimately forming a chain of sub-goals between the object and the goal position. We maintain the fully decentralised and communication-free nature of the original strategy. In three sets of simulated experiments, we demonstrate that our proposed approach allows a swarm of sufficient size to successfully transport objects around obstacles that occlude the goal from the starting position of the object, that this strategy is robust to different object shapes, and that our proposed changes do not reduce the performance of the strategy in environments where obstacles are not present.




### State Machine

picture here, probably


![purple-divider](https://user-images.githubusercontent.com/7065401/52071927-c1cd7100-2562-11e9-908a-dde91ba14e59.png)



### Results / Conclusion 

words here 

![purple-divider](https://user-images.githubusercontent.com/7065401/52071927-c1cd7100-2562-11e9-908a-dde91ba14e59.png)

### References

- MacRae, D., Queiroz, B.C. (2023) _Occlusion-Based Object Transportation Around Obstacles With a Swarm of Multiple Miniature Robots_  
<span style="color:red">TBD</span>.
- Chen, J., Gauci, M., Li, W., Kolling, A., Groß, R. (2015). _Occlusion-based cooperative transport with a swarm of miniature mobile robots._ IEEE Transactions on Robotics, 31 , 307-321, https://doi.org/10.1109/TRO.2015.2400731


