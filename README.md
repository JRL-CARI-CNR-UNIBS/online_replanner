# online_replanner

The repository contains the implementation of an anytime path re-planner and optimizer. The re-planner avoids un-predicted obstacles and/or optimizes the current path switching between pre-computed paths. It is based on Moveit! to get information about the environment.

## Build/Installation
The software can be installed with the following [rosinstall file](online_replanner.rosinstall).

## Packages
## **graph_replanning**
It contains three classes:
 1. replanner: given the current robot configuration and a set of pre-computed paths, it searches for a new path that avoids obstacles and/or optimize the current one.
 2. replanner_manager: it executes the robot trajectory, updates information about obstacles and continuously calls the replanner to avoid them or to optimize the current path.
 3. trajectory: it provides some useful methods to compute paths and trajectories.

## **graph_replanning_examples**
It contains some useful examples of usage of the replanner and replanner manager. It provides also a base on which run the replanner and/or the replanner manager on your robotic cell.

##Examples/Usage
The package **graph_replanning_examples** provides two examples of usage, one for the replanner and one for the replanner_manager.

## **replanner example**
example_replanner_cartesian.cpp is a ready-to-use example of the replanner functioning. In this example, the robot is a simple point moving in the three-dimensional Cartesian space, in which there is a large grey obstacle. Initially, a user-defined number of paths is computed considering the static grey obstacle and they are displayed on Rviz; then, a new obstacle (a red box) is placed on the current path (the dark green one). Finally, the replanner is called to find a new path, which will be displayed in yellow. The current robot configuration is represented as a pink sphere.
To run the example, type in the terminal:
'roslaunch graph_replanning_examples example_replanner_cartesian.launch'

In graph_replanning_example/config you can find a file named example_replanner_cartesian.yaml, in which there are some parameters you can choose.
These parameters are related to the robot definition, do not change them in this example:
```yaml
group_name: "cartesian_arm"
base_link: "world"
last_link: "end_effector"
```
These other parameters can be modified:
```yaml
display_step_by_step: false
number_paths: 4
max_time_for_replanning: 0.1
start_configuration: [0.0, 0.0, 0.0]
stop_configuration: [0.8, 0.8, 0.8]
```
If 'display_step_by_step' is true, you can see the replanner algorithm execution step by step, its evolution and the intermediate solutions. You can advance in each step by pressing the button <kbd>Next</kbd> on Rviz. Note that if this parameter is true, no time constraints are given to the replanner.
'number_paths' defines the number of pre-comuted which the replanner will exploits to find a new free path.

If you want to run the example with your robotic cell, copy the file config/example_replanner.yaml, change its name and do the same with the file launch/example_replanner.launch, and modify the fields marked with CHANGE IT.



![](Documentation/example_replanner.png)
