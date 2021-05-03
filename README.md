# online_replanner

The repository contains the implementation of an anytime informed path replanner and optimizer (**AIPRO**). The re-planner avoids un-predicted obstacles and/or optimizes the current path switching between pre-computed paths. It is based on Moveit! to get information about the environment.

You can read technical specifications about this replanner [here](https://arxiv.org/abs/2103.13245).

## Build/Installation
The software can be installed using rosinstall files.

1. Install ros: follow the steps described in http://wiki.ros.org/ROS/Installation
2. Install wstool and initialize the workspace: follow the steps described in http://wiki.ros.org/wstool
3. Install and configure rosdep: follow the steps described in http://wiki.ros.org/rosdep

Then, download and merge the rosinstall file:
```
cd ~/catkin_ws
wget https://raw.githubusercontent.com/JRL-CARI-CNR-UNIBS/online_replanner/master/online_replanner.rosinstall

cd ~/catkin_ws
wstool merge -t src ./online_replanner.rosinstall
```
Now, do the same with the dependencies required:
```
cd ~/catkin_ws
wget https://bitbucket.org/iras-ind/human_aware_motion_planners/raw/b1d545049aa78ab35e3918e4d30fbc395416ad40/human_aware.rosinstall

cd ~/catkin_ws
wstool merge -t src ./human_aware.rosinstall

cd ~/catkin_ws
wget https://raw.githubusercontent.com/CNR-STIIMA-IRAS/rosdyn/master/rosdyn.rosinstall

cd ~/catkin_ws
wstool merge -t src ./rosdyn.rosinstall
```
Download and install the packages specified in the rosinstall file and the other system dipendencies:
```
cd ~/catkin_ws
wstool update -t src
rosdep install --from-paths src --ignore-src -r -y
```
## Packages
### **graph_replanning [see README](https://github.com/JRL-CARI-CNR-UNIBS/online_replanner/blob/master/graph_replanning/README.md)**
It contains two main classes:
 1. [replanner](https://github.com/JRL-CARI-CNR-UNIBS/online_replanner/blob/master/graph_replanning/include/graph_replanning/replanner.h)
 2. [replanner_manager](https://github.com/JRL-CARI-CNR-UNIBS/online_replanner/blob/master/graph_replanning/include/graph_replanning/replanner_manager.h)

 replanner_manager executes the robot motion and, in the meanwhile, it continuously replans, calling several times the replanner.

### **graph_replanning_examples [see README](https://github.com/JRL-CARI-CNR-UNIBS/online_replanner/blob/master/graph_replanning_examples/README.md)**
It contains some useful examples of usage of the replanner and replanner manager. It provides also a base on which run the replanner and/or the replanner manager on your robotic cell.

## Work in progress
AIPRO is continuosly evolving. If you find errors or if you have some suggestions, [please let us know](https://github.com/JRL-CARI-CNR-UNIBS/online_replanner/issues).

## Developer Contact
### **Authors**
- Cesare Tonola (<mailto::c.tonola001@unibs.it>)
- Manuel Beschi (<mailto::manuel.beschi@stiima.cnr.it>)

## Acknowledgements
AIPRO is developed by CNR-STIIMA (http://www.stiima.cnr.it/)

***

![EC-H2020](Documentation/Sharework.png) [ShareWork webpage](https://sharework-project.eu/)

![EC-H2020](Documentation/flag_yellow.jpg)

This project has received funding from the European Union’s Horizon 2020 research and innovation programme under grant agreement No. 820807.
This website reflects only the author’s view and the European Commission is not responsible for any use that may be made of the information it contains.
