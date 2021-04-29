# online_replanner

The repository contains the implementation of an anytime path re-planner and optimizer. The re-planner avoids un-predicted obstacles and/or optimizes the current path switching between pre-computed paths. It is based on Moveit! to get information about the environment.

## Build/Installation
The software can be installed with the following [rosinstall file](online_replanner.rosinstall).

## Packages
### **graph_replanning**
It contains three classes:
 1. [replanner](https://github.com/JRL-CARI-CNR-UNIBS/online_replanner/blob/devel/graph_replanning/include/graph_replanning/replanner.h): given the current robot configuration and a set of pre-computed paths, it searches for a new path that avoids obstacles and/or optimize the current one.
 2. [replanner_manager](https://github.com/JRL-CARI-CNR-UNIBS/online_replanner/blob/devel/graph_replanning/include/graph_replanning/replanner_manager.h): it interpolates the robot trajectory and sends the new robot state, updates information about obstacles and continuously calls the replanner to avoid them or to optimize the current path.
 3. [trajectory](https://github.com/JRL-CARI-CNR-UNIBS/online_replanner/blob/devel/graph_replanning/include/graph_replanning/trajectory.h): it provides some useful methods to compute paths and trajectories.

### **graph_replanning_examples**
It contains some useful examples of usage of the replanner and replanner manager. It provides also a base on which run the replanner and/or the replanner manager on your robotic cell.
