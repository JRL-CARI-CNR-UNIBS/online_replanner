# **graph_replanning**

The package **graph_replanning** implements an anytime informed path replanner and optimizer (AIPRO).
It contains two main classes:
 1. [replanner](https://github.com/JRL-CARI-CNR-UNIBS/online_replanner/blob/devel/graph_replanning/include/graph_replanning/replanner.h)
 2. [replanner_manager](https://github.com/JRL-CARI-CNR-UNIBS/online_replanner/blob/devel/graph_replanning/include/graph_replanning/replanner_manager.h)

 replanner_manager executes the robot motion and, in the meanwhile, it continuously replan, calling several times the replanner.

## Replanner
Given the current robot configuration and a set of pre-computed paths, it searches for a new path that avoids obstacles and/or optimize the current one. The main method of this class is the following:
```cpp
bool informedOnlineReplanning(const double &max_time  = std::numeric_limits<double>::infinity());
```
which menages the entire replanning procedure from a specific current robot configuration.

This is a brief explanation to create a replanner object.
You need to include:
```cpp
#include <graph_replanning/replanner.h>
#include <graph_replanning/trajectory.h>
```
Define the robot model and the planning scene:
```cpp
moveit::planning_interface::MoveGroupInterface move_group(group_name);
robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
planning_scene::PlanningScenePtr planning_scene = std::make_shared<planning_scene::PlanningScene>(kinematic_model);

const robot_state::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup(group_name);
std::vector<std::string> joint_names = joint_model_group->getActiveJointModelNames();

unsigned int dof = joint_names.size();
Eigen::VectorXd lb(dof);
Eigen::VectorXd ub(dof);

for (unsigned int idx = 0; idx < dof; idx++)
{
  const robot_model::VariableBounds& bounds = kinematic_model->getVariableBounds(joint_names.at(idx));
  if (bounds.position_bounded_)
  {
    lb(idx) = bounds.min_position_;
    ub(idx) = bounds.max_position_;
  }
}
```

Then create a set of paths:
```cpp
pathplan::MetricsPtr metrics = std::make_shared<pathplan::Metrics>();
pathplan::CollisionCheckerPtr checker = std::make_shared<pathplan::ParallelMoveitCollisionChecker>(planning_scene, group_name);
pathplan::Trajectory trajectory = pathplan::Trajectory(nh,planning_scene,group_name);

std::vector<pathplan::PathPtr> path_vector;
Eigen::VectorXd start_conf;  //NB: INITIALIZE THIS VALUE WITH THE PATH START CONFIGURATION
Eigen::VectorXd goal_conf;   //NB: INITIALIZE THIS VALUE WITH THE PATH STOP CONFIGURATION

for (unsigned int i =0; i<n_paths; i++)
{
  pathplan::SamplerPtr sampler = std::make_shared<pathplan::InformedSampler>(start_conf, goal_conf, lb, ub);
  pathplan::BiRRTPtr solver = std::make_shared<pathplan::BiRRT>(metrics, checker, sampler);
  pathplan::PathPtr solution = trajectory.computePath(start_conf, goal_conf,solver,1);
  path_vector.push_back(solution);
}

pathplan::PathPtr current_path = path_vector.front();
std::vector<pathplan::PathPtr> other_paths;
other_paths.insert(other_paths.end(),path_vector.begin()+1,path_vector.end());
```
Define the current robot configuration, for example:
```cpp
 Eigen::VectorXd current_configuration = current_path->getConnections.at(0)->getChild()->getConfiguration();
```
Finally, create the replanner and replan:
```cpp
pathplan::Replanner replanner = pathplan::Replanner(current_configuration, current_path, other_paths, solver, metrics, checker, lb, ub);
bool success =  replanner.informedOnlineReplanning(time_replanning);
pathplan::PathPtr replanned_path = replanner.getReplannedPath();
```
You can find a complete example code [here](https://github.com/JRL-CARI-CNR-UNIBS/online_replanner/blob/devel/graph_replanning_examples/src/example_replanner.cpp).

## Replanner manager
The replanner manager manages the whole motion from a starting robot configuration to a goal configuration in a dynamic environment. It interpolates the robot trajectory, sends the new robot states and continuously calls the replanner to avoid obstacles or to optimize the current path. To do this, it uses three threads:
- `trajectory execution thread`: to interpolate and execute the trajectory.
- `collision checking thread`: continuously updates the planning scene to check which paths are colliding and to provide updated information about the state of the environment to the replanner.
- `replanning thread`: it continuously tries to find new paths from the robot current configuration to avoid obstacles or to optimizes the current one. It contains a replanner which calls the method `informedOnlineReplanning` at each iteration from the current robot configuration.

The replanning framework listens to two speed overrides topics, `/speed_ovr` and `/safe_ovr_1`, in which the overrides must be published as values between 0 (robot static) and 100 (nominal velocity).

It subscribes a service (`moveit_msgs::GetPlanningScene`) to update the information about the planning scene.
It continuously interpolates the trajectory computed starting from the replanned path and sends the new robot state, publishing a `sensor_msgs::JointState` message on topic `/joint_target`, and the unscaled state on topic `/unscaled_joint_target`.

This is a brief explanation to create a replanner manager object.
You need to include:
```cpp
#include <graph_replanning/replanner_manager.h>
```
Define the robot model and the planning scene:
```cpp
moveit::planning_interface::MoveGroupInterface move_group(group_name);
robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
planning_scene::PlanningScenePtr planning_scene = std::make_shared<planning_scene::PlanningScene>(kinematic_model);

const robot_state::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup(group_name);
std::vector<std::string> joint_names = joint_model_group->getActiveJointModelNames();

unsigned int dof = joint_names.size();
Eigen::VectorXd lb(dof);
Eigen::VectorXd ub(dof);

for (unsigned int idx = 0; idx < dof; idx++)
{
  const robot_model::VariableBounds& bounds = kinematic_model->getVariableBounds(joint_names.at(idx));
  if (bounds.position_bounded_)
  {
    lb(idx) = bounds.min_position_;
    ub(idx) = bounds.max_position_;
  }
}
```

Then create a set of paths:
```cpp
pathplan::MetricsPtr metrics = std::make_shared<pathplan::Metrics>();
pathplan::CollisionCheckerPtr checker = std::make_shared<pathplan::ParallelMoveitCollisionChecker>(planning_scene, group_name);
pathplan::Trajectory trajectory = pathplan::Trajectory(nh,planning_scene,group_name);

std::vector<pathplan::PathPtr> path_vector;
Eigen::VectorXd start_conf;  //NB: INITIALIZE THIS VALUE WITH THE PATH START CONFIGURATION
Eigen::VectorXd goal_conf;   //NB: INITIALIZE THIS VALUE WITH THE PATH STOP CONFIGURATION

for (unsigned int i =0; i<n_paths; i++)
{
  pathplan::SamplerPtr sampler = std::make_shared<pathplan::InformedSampler>(start_conf, goal_conf, lb, ub);
  pathplan::BiRRTPtr solver = std::make_shared<pathplan::BiRRT>(metrics, checker, sampler);
  pathplan::PathPtr solution = trajectory.computePath(start_conf, goal_conf,solver,1);
  path_vector.push_back(solution);
}

pathplan::PathPtr current_path = path_vector.front();
std::vector<pathplan::PathPtr> other_paths;
other_paths.insert(other_paths.end(),path_vector.begin()+1,path_vector.end());
```
Finally, create the replanner manager and start the execution:
```cpp
pathplan::ReplannerManagerPtr replanner_manager = std::make_shared<pathplan::ReplannerManager>(current_path, other_paths, nh);
replanner_manager->start();
```
You can find a complete example code [here](https://github.com/JRL-CARI-CNR-UNIBS/online_replanner/blob/devel/graph_replanning_examples/src/example_replanner_manager.cpp).
