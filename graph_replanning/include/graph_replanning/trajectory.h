#ifndef TRAJECTORY_8_1_2021_H__
#define TRAJECTORY_8_1_2021_H__

#include <ros/ros.h>
#include <graph_core/metrics.h>
#include <graph_core/occupancy_metrics.h>
#include <graph_core/graph/node.h>
#include <graph_core/graph/path.h>
#include <graph_core/graph/tree.h>
#include <graph_core/solvers/rrt_connect.h>
#include <graph_core/solvers/birrt.h>
#include <graph_core/solvers/multigoal.h>
#include <graph_core/solvers/rrt_star.h>
#include <graph_core/solvers/path_solver.h>
#include <graph_core/moveit_collision_checker.h>
#include <graph_core/local_informed_sampler.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit/robot_state/conversions.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <rviz_visual_tools/rviz_visual_tools.h>
#include <eigen_conversions/eigen_msg.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>
#include <moveit_planning_helper/spline_interpolator.h>
#include <graph_replanning/moveit_utils.h>

#define COMMENT(...) ROS_LOG(::ros::console::levels::Debug, ROSCONSOLE_DEFAULT_NAME, __VA_ARGS__);

namespace pathplan
{
class Trajectory;
typedef std::shared_ptr<Trajectory> TrajectoryPtr;

class Trajectory: public std::enable_shared_from_this<Trajectory>
{
protected:

  robot_trajectory::RobotTrajectoryPtr trj_;
  PathPtr path_;
  ros::NodeHandle nh_;
  robot_model::RobotModelConstPtr kinematic_model_;
  planning_scene::PlanningScenePtr planning_scene_;
  std::string group_name_;
  MoveitUtilsPtr moveit_utils_;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Trajectory(const pathplan::PathPtr path,
             const ros::NodeHandle& nh,
             const planning_scene::PlanningScenePtr& planning_scene,
             const std::string& group_name);

  Trajectory(const ros::NodeHandle& nh,
             const planning_scene::PlanningScenePtr& planning_scene,
             const std::string& group_name);

  TrajectoryPtr pointer()
  {
    return shared_from_this();
  }

  void setPath(const pathplan::PathPtr path)
  {
    path_ = path;
  }

  pathplan::PathPtr getPath()
  {
    if(path_ == NULL) ROS_ERROR("Path not computed");
    return path_;
  }

  robot_trajectory::RobotTrajectoryPtr getTrj()
  {
    if(trj_ == NULL) ROS_ERROR("Trj not computed");
    return trj_;
  }

  // Compute a path then it is optimized if optimizePath==1 . nh is the NodeHandler of a ros node.
  PathPtr computePath(const Eigen::VectorXd &start_conf, const Eigen::VectorXd &goal_conf, const TreeSolverPtr& solver, const bool& optimizePath);

  //To trasform a path to a RobotTrajectory with or without initial condition
  robot_trajectory::RobotTrajectoryPtr fromPath2Trj(const trajectory_msgs::JointTrajectoryPointPtr& pnt = NULL);
  robot_trajectory::RobotTrajectoryPtr fromPath2Trj(const trajectory_msgs::JointTrajectoryPoint& pnt);

  double getTimeFromPositionOnTrj(const Eigen::VectorXd &joints_value, double step = 0.01);  //da sistemare

};
}

#endif // TRAJECTORY_H
