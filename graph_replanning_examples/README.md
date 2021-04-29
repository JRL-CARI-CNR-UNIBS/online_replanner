# **graph_replanning_examples**
The package **graph_replanning_examples** provides two usage examples, one for the replanner and one for the replanner_manager. It also provides the necessary files to use as a basis for running the examples with your robotic cell.

## **Replanner**
### **Example**
[example_replanner_cartesian.cpp](https://github.com/JRL-CARI-CNR-UNIBS/online_replanner/blob/devel/graph_replanning_examples/src/example_replanner.cpp) is a ready-to-use example of the replanner functioning. In this example, the robot is a simple pink sphere moving in the three-dimensional Cartesian space, in which there is a large grey obstacle. Initially, a user-defined number of paths is computed considering the static grey obstacle and they are displayed on Rviz; then, a new obstacle (a red box) is placed on the current path (the dark green one). Finally, the replanner is called to find a new path, which will be displayed in yellow.
To run the example, type in the terminal:

`roslaunch graph_replanning_examples example_replanner_cartesian.launch`

[example_replanner_cartesian.yaml](https://github.com/JRL-CARI-CNR-UNIBS/online_replanner/blob/devel/graph_replanning_examples/config/example_replanner_cartesian.yaml) contains some parameters which can be changed.
These parameters are related to the robot definition, do not change them in this example:
```yaml
group_name: "cartesian_arm"
base_link: "world"
last_link: "end_effector"
```
Instead, these parameters can be modified:
```yaml
display_step_by_step: false
number_paths: 4
max_time_for_replanning: 0.1 #seconds
start_configuration: [0.0, 0.0, 0.0]
stop_configuration: [0.8, 0.8, 0.8]
```
- If `display_step_by_step` is true, you can see the replanner algorithm execution step by step, its evolution and the intermediate solutions. You can advance in each step by pressing the button <kbd>Next</kbd> on Rviz (see Figure). Note that if this parameter is true, no time constraints are given to the replanner.
- `number_paths` defines the number of paths which will computed before launching the replanner and which the replanner will exploit to find a new free path. Recommended value 3-6 paths. NOTE: these paths should explore different parts of the workspace in order to make the replanning more effective.
- `max_time_for_replanning` defines the maximum time that the replanner can use to replan and optimize the solution. The higher the value, the better the solutions found. Complex cells require a bigger amount of time.

### **Usage**
In order to use the replanner with your robotic cell, you need to do the following preliminary steps:

> Create a moveit package (see <a href="http://docs.ros.org/kinetic/api/moveit_tutorials/html/doc/setup_assistant/setup_assistant_tutorial.html">tutorial</a>)

> Copy and change the name of the files [example_replanner.launch](https://github.com/JRL-CARI-CNR-UNIBS/online_replanner/blob/devel/graph_replanning_examples/launch/example_replanner.launch) and [example_replanner.yaml](https://github.com/JRL-CARI-CNR-UNIBS/online_replanner/blob/devel/graph_replanning_examples/config/example_replanner.yaml) and modify the fields marked with CHANGE IT

> in RViz:

>> click on *Panels* tag, then *add new panel*, select *RvizVisualToolsGui*

>> check on *Add* in the Display widgets, add a *Marker* visualization

>> In the *Marker* visualization, select as Marker topic */marker_visualization_topic*


Now you should be able to run your example.

![](https://github.com/JRL-CARI-CNR-UNIBS/online_replanner/blob/devel/Documentation/example_replanner.png)

## **Replanner manager**
### **Example**
[example_replanner_manager_cartesian.cpp](https://github.com/JRL-CARI-CNR-UNIBS/online_replanner/blob/devel/graph_replanning_examples/src/example_replanner_manager.cpp) is a ready-to-use example of the replanner manager functioning. In this example, the robot is a simple pink sphere moving in the three-dimensional Cartesian space, in which there is a large grey obstacle. Initially, a user-defined number of paths is computed considering the static grey obstacle and they are displayed on Rviz; then, the robot starts moving along a yellow path and during the motion two new obstacles (red boxes) appear on the robot current path. The replanner manager will notify the replanner, which will find a new feasible path.
To run the example, type in the terminal:

`roslaunch graph_replanning_examples example_replanner_manager_cartesian.launch`

[example_replanner_manager_cartesian.yaml](https://github.com/JRL-CARI-CNR-UNIBS/online_replanner/blob/devel/graph_replanning_examples/config/example_replanner_manager_cartesian.yaml) contains some parameters which can be changed.
These parameters are related to the robot definition, do not change them in this example:
```yaml
group_name: "cartesian_arm"
base_link: "world"
last_link: "end_effector"
```
These parameters define the frequencies used to send the new robot states and to update the planning scene. They can be modified, but carefully and without deviating much from the reference values:
```yaml
trj_execution_thread_frequency: 500
collision_checker_thread_frequency: 30
```
Instead, these parameter can be chosen by the user:
```yaml
read_safe_scaling: false
spawn_objs: true
number_paths: 4
dt_replan_restricted: 0.050 #seconds
dt_replan_relaxed: 0.100 #seconds
scaling: 1.0
start_configuration: [0.0, 0.0, 0.0]
stop_configuration: [0.8, 0.8, 0.8]
```
- If `spawn_objs` is true, two red boxes will appear on the robot path during the motion, forcing the replanner to find a new feasible path.
- `number_paths` defines the number of paths which will be computed before launching the replanner manager and which the replanner will exploit to find a new free path. Recommended value 3-6 paths. NOTE: these paths should explore different parts of the workspace in order to make the replanning more effective.
- `dt_replan_restricted` and `dt_replan_relaxed` defines the maximum time used by the replanner to find a solution when, respectively, there is an obstacle on the current path (replan to avoid obstacles) and when the current path is free (replan to optimize the current path). `dt_replan_restricted` should be a value lower than `dt_replan_relaxed` because when an obstacle is obstructing the current path, the priority is to find a solution quickly, to the detriment of its quality. Instead, when the current path is free, the aim is to optimize it, so the replanner can use much more time. These values ​​depend on the complexity of the robotic cell. For complex industrial cells, reference values for `dt_replan_restricted` are 0.100-0.150 seconds, while 0.150-0.200 seconds for `dt_replan_relaxed`.
- `scaling` is a user-defined override of the velocity of the robot. It is a value between 0.0 (robot static) and 1.0 (nominal velocity).
- If `read_safe_scaling` is true, `scaling` is not considered, but the overrides published on the topics `\speed_ovr` and `\safe_ovr_1` (values between 0 and 100) are considered.

### **Usage**
In order to use the replanner manager with your robotic cell, you need to do the following preliminary steps:

> Create a moveit package (see <a href="http://docs.ros.org/kinetic/api/moveit_tutorials/html/doc/setup_assistant/setup_assistant_tutorial.html">tutorial</a>)

> Copy and change the name of the files [example_replanner_manager.launch](https://github.com/JRL-CARI-CNR-UNIBS/online_replanner/blob/devel/graph_replanning_examples/launch/example_replanner_manager.launch) and [example_replanner_manager.yaml](https://github.com/JRL-CARI-CNR-UNIBS/online_replanner/blob/devel/graph_replanning_examples/config/example_replanner_manager.yaml) and modify the fields marked with CHANGE IT

> in RViz:

>> check on *Add* in the Display widgets, add a *Marker* visualization

>> In the *Marker* visualization, select as Marker topic */marker_visualization_topic*


Now you should be able to run your example.
