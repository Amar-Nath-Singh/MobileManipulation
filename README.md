# Catching Moving Targets Using a Mobile Manipulator

## Abstract

This project focuses on catching moving targets using a mobile manipulator, specifically employing a Kuka Youbot in the CoppeliaSim simulation environment. The project is unique due to its dynamic simulation environment, where the map changes with each run, altering the positions of moving targets and stationary obstacles. This variability allows for testing the efficacy of planning algorithms across diverse scenarios.

The experiments conducted involve both known and random paths for the target, aiming to coordinate planning for both the manipulator and the base simultaneously. A proposed solution leverages the Probabilistic Roadmap (PRM) to adapt dynamically as both the bot and the target move. Constant replanning at each time step is necessary to effectively intercept the target.

By integrating PRM planning into the dynamic environment, the project aims to optimize mobile manipulation strategies for efficiently capturing moving targets, paving the way for advancements in real-world applications of robotic systems.



## Methodology

The project comprises four key components:

### 1. Environment Setup

The environment setup involves configuring CoppeliaSim with the necessary elements: the target, walls, and the Kuka Youbot. A random map is created by spawning a fixed number of walls in random locations. Two types of walls can be chosen at random by the algorithm: the ‘L’ shaped wall and the straight wall.

### 2. Base Planning

The base planning section involves using various algorithms such as RRT, RRT*, and PRM. Four primary sampling modes were used:

1. Random Sampling
2. Goal-Biased Sampling
3. Towards Goal Sampling
4. APF-Based Sampling

PRM was ultimately chosen as the most effective algorithm for base planning in a continuously changing environment. The bot’s path is planned using A* search on the roadmap created, with the target moving on a random path chosen at the start of the simulation. A basic PID controller ensures the robot does not collide with walls in CoppeliaSim.
![image](https://github.com/user-attachments/assets/eab291b1-6700-4a55-bff1-9e8cb50dc34c)

### 3. Arm Planning

The arm planning process begins with formulating the forward kinematics using DH parameters. Three major approaches were used for arm planning:

1. **Inverse Kinematics:** This approach sampled points between start and end points, but inaccuracies in inverse kinematics affected the results.
2. **Sampling in Configuration Space:** This approach involved sampling around 5000 points in configuration space, with forward kinematics used to find the task space corresponding to all samples.
3. **APF-RRT-Based Planning:** The algorithm for APF-RRT-based planning was used to generate new poses, avoid collisions, and move toward the goal.

The APF-RRT-based planning algorithm is detailed as follows:

1. Initialize reached node indexes as an empty list.
2. Initialize poses array with node poses.
3. Iterate from 0 to max iterations.
4. Inside loop:
   - Generate random pose.
   - Find index of nearest neighbor.
   - Get nearest node.
   - Generate new pose based on APF Local Planner.
   - Check collision.
     - If collision-free:
       - Create new node.
       - Append to nodes.
       - Update poses.
       - If star:
         - Update neighbors’ costs and parents.
       - Check if goal reached.
       - If goal reached and conditions met, add index to reached node indexes.
       - Break loop if goal reached and conditions met.
   - Increment iteration counter.
5. If no nodes reach goal, return None.
6. Otherwise:
   - Find node with minimum cost among reached nodes.
   - Generate path from node with minimum cost.
   - Return path.
![image](https://github.com/user-attachments/assets/7e1f2b66-6725-4257-8161-8eba3f09db45)
## Results

The results demonstrate that dynamic planning is possible in continuously changing environments. Generalizing the results suggests applicability across diverse environments and for various applications.

## Final Notes

The final video of the project will showcase the integration of base and arm planning, highlighting the success of the dynamic planning strategy in CoppeliaSim.
