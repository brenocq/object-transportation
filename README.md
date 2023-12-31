# Occlusion-Based Object Transportation Around Obstacles With a Swarm of Multiple Miniature Robots
<p align="center">
    <img src="https://github.com/brenocq/object-transportation/assets/17342434/e2ac83ec-ddfb-4cd6-96aa-63b75f64dda1"/>
</p>

This repository contains the code used in MacRae & Queiroz (2023) to perform our experiments utilising a swarm of miniature occlusion-based robots to transport objects in complex environments. An extension of Chen (2015), we augment the purely vision-based finite state machine to enable the robots to form "sub-goals" that allows observing robots to trace a path around obstacles; allowing the robots to successfully complete the object transportation task, while maintaining the decentralised, communication-free and vision-based implementation of the original Chen (2015) strategy.

### Execution
The simulation was implemented with [Atta](https://github.com/brenocq/atta), and it is necessary to install its dependencies to execute it. These are the dependencies for Ubuntu, if you are using another Linux distribution, or another operating system, follow the steps in the [Atta README](https://github.com/brenocq/atta).

```bash
sudo apt-get install cmake xorg-dev curl
```

After that, it is necessary to install Atta, we are using the version `458906d4`.
```bash
git clone git@github.com:brenocq/atta.git
cd atta
git checkout 458906d4
./build.sh ---install
```

Finally, we can execute the simulation
```bash
git clone git@github.com:brenocq/object-transportation.git
cd object-transportation/simulation
atta object-transportation.atta
```

### Abstract
Swarm robotics utilises decentralised self-organising systems to form complex collective behaviours built from the bottom-up using individuals that have simple, yet limited, capabilities. Previous works have explored the use of swarm robotics for the task of transporting objects to a goal position. In this paper, we propose an improvement to an occlusion-based strategy for such a task that enables the strategy to work in environments where obstacles block the line-of-sight between the object and the goal. Our approach allows the robots to form sub-goals; allowing any member of the swarm to establish a wider range of visibility of the goal, ultimately forming a chain of sub-goals between the object and the goal position. We maintain the fully decentralised and communication-free nature of the original strategy. In three sets of simulated experiments, we demonstrate that our proposed approach allows a swarm of sufficient size to successfully transport objects around obstacles that occlude the goal from the starting position of the object, that this strategy is robust to different object shapes, and that our proposed changes do not reduce the performance of the strategy in environments where obstacles are not present.


### State Machine
<p align="center">
    <img src="https://github.com/brenocq/object-transportation/assets/17342434/5564f711-1fe5-4f17-8aae-505be9bc703e" height="400"/>
</p>

The states `S1: Search For Object`, `S2: Approach Object`, `S3: Push Object`, and `S4: Move Around Object` are similar to the states proposed by Chen (2015). We show that, with the addition of the `S5: Be a Goal`, and by allowing the robots to change color, it is possible to extend the swarms capabilities to transpost objects in environments with obstacles. When a robots enter the state `S5`, its color changes to green, which will lead other robots will detect it as a goal, and push the object torwards it.

### Results / Conclusion
The proposed stated state machine was tested on the four maps presented below.

| Reference | Middle | Corner | 2-Corners |
|-----------|--------|--------|-----------|
| ![Reference](https://github.com/brenocq/object-transportation/assets/17342434/b2b7a269-d5da-4f09-b5f4-5eead43f9d50) | ![Middle](https://github.com/brenocq/object-transportation/assets/17342434/ca71da3a-206c-432d-b932-8407d02d0b0c) | ![Corner](https://github.com/brenocq/object-transportation/assets/17342434/10f7087a-0ce8-4872-a604-6d47d6abea8c) | ![2-Corners](https://github.com/brenocq/object-transportation/assets/17342434/0a641696-6ad6-4a7e-afa4-1bcda4493c06) |

Because the approach proposed by Chen (2015) can only transport the object to the goal when there is no occlusion between the object and the goal, it can only sucessfully complete the `reference` map. The proposed state machine, however, can complete all four maps. The Figure below presents the task completion time for each map with varying number of robots, and it is possible to see that incrising the number of robots reduces the completion time. Additionally, the path efficiency presents the ratio between the object trajectory length and the optimal possible trajectory length (1.0 meaning they are the same, and 0.5 meaning the trajectory is twice longer than the optimal).

<p align="center">
    <img src="https://github.com/brenocq/object-transportation/assets/17342434/051aa958-90b3-4bcc-bd7a-29ef78d7711f" height="250"/>
</p>

Please refer to the paper for more in-depth evaluation of the proposed state machine.

### References
- MacRae, D., Queiroz, B.C. (2023) _Occlusion-Based Object Transportation Around Obstacles With a Swarm of Multiple Miniature Robots_  
<span style="color:red">TBD</span>.
- Chen, J., Gauci, M., Li, W., Kolling, A., Groß, R. (2015). _Occlusion-based cooperative transport with a swarm of miniature mobile robots._ IEEE Transactions on Robotics, 31 , 307-321, https://doi.org/10.1109/TRO.2015.2400731


