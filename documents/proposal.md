## Academic Reference

The algorithm we are proposing to implement is Steven M. LaValle's Rapidly-exploring Random Tree (RRT) published in October, 1998. (https://msl.cs.uiuc.edu/~lavalle/papers/Lav98c.pdf).

For a more in-depth understanding, we will be referencing the Wikipedia article on RRT
(https://en.wikipedia.org/wiki/Rapidly-exploring_random_tree) and supplementary materials from various robotics textbooks available
in our library.

Since this algorithm has been widely implemented in the robotics field, we've decided to build our project based on Carnegie Mellon University's (CMU) homework question that given a certain Degree of Freedom (DoF) robotics arm, how can it reach a destination point. We'll refer to this document, (https://github.com/nrahnemoon/PlanningHW2/blob/master/hw2_16782_fall17.pdf).

A YouTube video that give great intuition to RRT/RRT* has also been helpful to our understanding and implementation (https://www.youtube.com/watch?v=_aqwJBx2NFk).

## Algorithm Summary

Algorithm Summary The RRT algorithm is a randomized data structure designed for a broad range of pathfinding tasks in robotics.
Given a start and a goal, the algorithm incrementally builds a tree rooted at the starting point. In each iteration, the algorithm
randomly samples a point in the configuration space, then finds the nearest node in the tree to this sample, and attempts to add a
new node in the direction of the sample. The presence of obstacles, as defined in the grid as integer 1, will be taken into account when extending the tree.

### RRT Pseudocode

![RRT Pseudocode](https://github.com/RiceMiceA/cs225_RRT_project/blob/main/documents/pseudo_code.png)
[3]


## Function I/O
Function I/O Our RRT algorithm will mainly consist of four functions:

1. `tuple<double*, int, int> loadMap(string filepath)`
```
@param filepath -- A txt file that contains a 2D binary matrix map with 0 represents empty space and 1 represents obstacle.
@return A tuple that contains a "double" type array, with map width and map height.
```

This loadMap function is initializing the map into a 1D array that contains all the points from columns to columns. In this phase, we'll also store startPosition and endPosition as the starting node and destination node.

For our initial tests test(1) and test(2), we will ensure that the tree starts and expands correctly without colliding with obstacles by checking if the edges between vertex is not overlapping with any obstacle. What's more, we also check if the new positions are within the boundaries of the 2D map. In future subsequent tests, we will check the algorithm's efficiency and how it handles more complex environments.

2. `Status Extend(Tree& T, double* q, int numofDOFs, double epsilon, double* map, int x_size, int y_size, double*& q_new)`
```
@param T -- the RRT tree that contains all the tree operations
@param q -- the current newly generated tree node
@param numofDOFs -- the amount of new tree nodes or robotic arm joints that we need to randomly create.
@param epsilon -- the maximum edge length value
@param map -- passing the map to help generate new node
@param x_size -- x direction boundary value
@param y_size -- y direction boundary value
@param q_new -- a reference to our new nodes array that stores DOF number of new nodes.
@return A boolean that shows this extension operation's result such like succedded or not.
```
This extension function has 4 main procedures:

-Nearest Neighbor Search: First, after passing into our newly generated tree node, the Extending function identifies the nearest node in the current RRT tree to the randomly sampled configuration, which was showcased inside NearestNeighbor() function. This nearest neighbor serves as the starting point for extending the tree toward the random configuration.

-Local Planning: Once the nearest neighbor is found, a local planning algorithm is used to determine how to extend the tree from the nearest neighbor toward the random configuration. This often involves calculating a path or trajectory that connects the nearest neighbor to the random configuration while avoiding obstacles in the configuration space.

-Node Addition: If the path from the nearest neighbor to the random configuration is collision-free, a new node is added to the RRT tree, and this new node is connected to the nearest neighbor. This node represents a new state in the exploration of the configuration space.

-Termination: The RRT algorithm typically continues to perform these steps until a predefined termination condition is met. This condition might include reaching the goal configuration, a maximum number of iterations, or other criteria.

To test on the accuracy of this function, we rely on test case (2) that uses IsValidArmConfiguratio() function to test if any parameter within this extension is agasinsting the map restrictions such like plotting through an obstacle.

3. `static void RRT(double* map, int x_size, int y_size, double* armstart_anglesV_rad, double* armgoal_anglesV_rad, int numofDOFs, double*** plan, int* planlength, int K, double epsilon)`
```
@param map -- passing the map to help generate new node
@param x_size -- x direction boundary value
@param y_size -- y direction boundary value
@param armstart_anglesV_rad -- starting node positions parameters
@param armgoal_anglesV_rad -- destination node positions parameters
@param numofDOFs -- the amount of new tree nodes or robotic arm joints that we need to randomly create.
@param plan -- the array that will store all the nodes on the path that we found using RRT
@param planlength -- the length of our plan array
@param K -- number of iterations
@param epsilon -- the maximum edge length value
```
This is our main algorithm application for RRT. It contains two main parts: Planning and Path Extraction. For the first part Planning, it utilizes all the helper functions that satisfy the following tasks: Map Initialization, Nodes Random Sampling and Path Extension. Once the termination condition is satisfied, if the goal configuration has been reached or is within a certain tolerance, a path from the start configuration to the goal configuration can be extracted by tracing the tree from the goal (leaf) node back to the root node.

For test cases (4) and (5), we will be finished with a complete execution of finding a path bewteen starting point and destination point and will used a equalDoubleArrays() function to determine if our robotics arm with its DOF number of nodes are at the destination locations.


4. **Special Visualizing Testing Approach** `visualizer.py`

This is an extra way of vividly getting the result and verify it with a visualizer python file that prints out all the stages of the path planning use the data from planner.cpp. Using matlab, we'd like to have a gif file that can even animate the process so that we can asure no collisions or out-of-bounds results are created.

## Data Description

The primary dataset for our testing will be various sizes of grid with cells marked as 1s (obstacles) and 0s (free space) that we generated manually but we'll strive to use a script to automate this process. This grid will be used to simulate a robotic environment where the algorithm will be tasked to find a feasible path from a start to a goal position, avoiding the obstacles.

Additionally, we might introduce variations in the dataset by altering the obstacle density or introducing dynamic obstacles. These
variations can help us better evaluate the robustness and adaptability of our implementation.

In terms of preprocessing, given that the dataset is a simple binary grid, we might only need to ensure that the start and goal nodes are not initialized within obstacles. We'll develop scripts, if required, to automate this preprocessing step, ensuring the consistency and integrity of our test cases.