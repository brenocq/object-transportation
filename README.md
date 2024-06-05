# Occlusion-Based Object Transportation Around Obstacles With a Swarm of Multiple Miniature Robots
<p align="center">
    <img src="https://github.com/brenocq/object-transportation/assets/17342434/4cbb3ed4-8312-40ad-bd78-a8689183c28c" alt="reference" height="150"/>
    <img src="https://github.com/brenocq/object-transportation/assets/17342434/666ced7e-674a-41ea-b504-932712852404" alt="2-corners" height="150"/>
    <img src="https://github.com/brenocq/object-transportation/assets/17342434/ef436352-b0ef-49ca-817f-83aa66fb96ba" alt="corner" height="150"/>
    <img src="https://github.com/brenocq/object-transportation/assets/17342434/0e251481-4094-42f1-a914-2df768f6217b" alt="middle" height="150"/>
</p>

This repository contains the code used in Queiroz & MacRae (2024) to perform our experiments utilising a swarm of miniature occlusion-based robots to transport objects in complex environments. An extension of Chen (2015), we augment the purely vision-based finite state machine to enable the robots to form "sub-goals" that allows observing robots to trace a path around obstacles; allowing the robots to successfully complete the object transportation task, while maintaining the decentralised, communication-free and vision-based implementation of the original Chen (2015) strategy.

### Execution
The simulation was implemented with [Atta](https://github.com/brenocq/atta), and it is necessary to install its dependencies to execute it. These are the dependencies for Ubuntu, if you are using another Linux distribution, or another operating system, follow the steps in the [Atta README](https://github.com/brenocq/atta).

```bash
sudo apt-get install cmake xorg-dev curl
```

After that, it is necessary to install Atta, we are using the version `458906d4`.
```bash
git clone git@github.com:brenocq/atta.git
cd atta
git checkout object-transportation-project-fixes
./build.sh ---install
```

Finally, we can execute the simulation
```bash
git clone git@github.com:brenocq/object-transportation.git
cd object-transportation/simulation
atta object-transportation.atta
```

### Abstract
Swarm robotics utilises decentralised self-organising systems to form complex collective behaviours built from the bottom-up using individuals that have limited capabilities. Previous work has shown that simple occlusion-based strategies can be effective in using swam robotics for the task of transporting objects to a goal position. However, this strategy requires a clear line-of-sight between object and goal. In this paper, we extend this strategy by allowing robots to form sub-goals; enabling any member of the swarm to establish a wider range of visibility of the goal, ultimately forming a chain of sub-goals between the object and the goal position. We do so while maintaining the fully decentralised and communication-free nature of the original strategy. In three sets of simulated experiments, we demonstrate that our proposed approach allows a swarm of sufficient size to successfully transport objects around obstacles that occlude the goal from the starting position of the object, that this strategy is robust to different object shapes, and that our proposed changes do not reduce the performance of the strategy in environments where obstacles are not present.


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
- Queiroz, B.C., MacRae, D. (2024) _Occlusion-Based Object Transportation Around Obstacles With a Swarm of Multiple Miniature Robots._
<span style="color:red">TBD</span>.
- Chen, J., Gauci, M., Li, W., Kolling, A., Groß, R. (2015). _Occlusion-based cooperative transport with a swarm of miniature mobile robots._ IEEE Transactions on Robotics, 31 , 307-321, https://doi.org/10.1109/TRO.2015.2400731


