#include <ros/ros.h>
#include <graph_core/metrics.h>
#include <graph_core/occupancy_metrics.h>
#include <graph_core/graph/node.h>
#include <graph_core/graph/path.h>
#include <graph_core/graph/tree.h>
#include <graph_replanning/trajectory.h>
#include <graph_core/graph/graph_display.h>
#include <graph_core/solvers/birrt.h>
#include <graph_core/solvers/rrt_star.h>
#include <graph_core/solvers/path_solver.h>
#include <graph_core/moveit_collision_checker.h>
#include <graph_core/parallel_moveit_collision_checker.h>
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
#include <object_loader_msgs/AddObjects.h>
#include <rosparam_utilities/rosparam_utilities.h>


int main(int argc, char **argv)
{
  unsigned int n_paths = 1;

  ros::init(argc, argv, "node_test_path_validity");
  ros::AsyncSpinner spinner(4);
  spinner.start();

  ros::NodeHandle nh;
  // /////////////////////////////////UPLOADING THE ROBOT ARM/////////////////////////////////////////////////////////////
  std::string test;
  nh.getParam("test",test);

  std::vector<double> start;
  nh.getParam("start",start);

  std::vector<double> goal;
  nh.getParam("goal",goal);

  std::string group_name;
  nh.getParam("group_name",group_name);

  std::string base_link;
  nh.getParam("base_link",base_link);

  std::string last_link;
  nh.getParam("last_link",last_link);

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
    const robot_model::VariableBounds& bounds = kinematic_model->getVariableBounds(joint_names.at(idx));  //bounds dei joints definito in urdf e file joints limit
    if (bounds.position_bounded_)
    {
      lb(idx) = bounds.min_position_;
      ub(idx) = bounds.max_position_;
    }
  }

  // ///////////////////////////////////UPDATING THE PLANNING STATIC SCENE////////////////////////////////////
  ros::ServiceClient ps_client=nh.serviceClient<moveit_msgs::GetPlanningScene>("/get_planning_scene");

  if (!ps_client.waitForExistence(ros::Duration(10)))
  {
    ROS_ERROR("unable to connect to /get_planning_scene");
    return 1;
  }

  moveit_msgs::GetPlanningScene ps_srv;

  if (!ps_client.call(ps_srv))
  {
    ROS_ERROR("call to srv not ok");
    return 1;
  }

  if (!planning_scene->setPlanningSceneMsg(ps_srv.response.scene))
  {
    ROS_ERROR("unable to update planning scene");
    return 1;
  }

  // /////////////////////////////////////////////DEFINING START AND GOAL///////////////////////////////////////////////////////

  Eigen::VectorXd start_conf(dof);
  Eigen::VectorXd goal_conf(dof);

  for(unsigned int d=0;d<dof;d++)
  {
    start_conf[d]=start.at(d);
    goal_conf[d]=goal.at(d);
  }

  pathplan::MetricsPtr metrics = std::make_shared<pathplan::Metrics>();
  pathplan::CollisionCheckerPtr checker = std::make_shared<pathplan::MoveitCollisionChecker>(planning_scene, group_name,0.001);
  pathplan::CollisionCheckerPtr checker_parallel = std::make_shared<pathplan::ParallelMoveitCollisionChecker>(planning_scene, group_name,5,0.001);

  // //////////////////////////////////////////PATH PLAN & VISUALIZATION////////////////////////////////////////////////////////
  pathplan::Display disp = pathplan::Display(planning_scene,group_name,last_link);
  pathplan::Trajectory trajectory = pathplan::Trajectory(nh,planning_scene,group_name);

  ros::Duration(2).sleep();

  for(unsigned int j=0; j<1;j++)
  {
    std::vector<pathplan::PathPtr> path_vector;

    for (unsigned int i =0; i<n_paths; i++)
    {
      pathplan::SamplerPtr sampler = std::make_shared<pathplan::InformedSampler>(start_conf, goal_conf, lb, ub);
      pathplan::BiRRTPtr solver = std::make_shared<pathplan::BiRRT>(metrics, checker, sampler);

      pathplan::PathPtr solution = trajectory.computePath(start_conf,goal_conf,solver, 1);
      path_vector.push_back(solution);
      ros::Duration(0.1).sleep();

      std::vector<double> marker_color;
      if(i==0) marker_color = {0.5,0.5,0.0,1.0};
      if(i==1) marker_color = {0.0f,0.0f,1.0,1.0};
      if(i==2) marker_color = {1.0,0.0f,0.0f,1.0};

      disp.displayPathAndWaypoints(solution,"pathplan",marker_color);
    }

    pathplan::PathPtr current_path = path_vector.front();
    int idx = 0;//current_path->getConnections().size()/2;

    pathplan::SamplerPtr samp = std::make_shared<pathplan::InformedSampler>(start_conf, goal_conf, lb, ub);
    pathplan::BiRRTPtr solver = std::make_shared<pathplan::BiRRT>(metrics, checker, samp);
    solver->config(nh);

    Eigen::VectorXd current_configuration = (current_path->getConnections().at(idx)->getChild()->getConfiguration() + current_path->getConnections().at(idx)->getParent()->getConfiguration())/2.0;

    // ///////////////////////////////////////// VISUALIZATION OF CURRENT NODE ///////////////////////////////////////////////////////////
    std::vector<double> marker_color_sphere_actual = {1.0,0.0,1.0,1.0};
    disp.displayNode(std::make_shared<pathplan::Node>(current_configuration),"pathplan",marker_color_sphere_actual);

    // //////////////////////////////////////// ADDING A MOBILE OBSTACLE ////////////////////////////////////////////////////////////////
    ros::ServiceClient add_obj=nh.serviceClient<object_loader_msgs::AddObjects>("add_object_to_scene");

    if (!add_obj.waitForExistence(ros::Duration(10)))
    {
      ROS_FATAL("srv not found");
      return 1;
    }

    object_loader_msgs::AddObjects srv;
    object_loader_msgs::Object obj;
    obj.object_type="scatola";

    int obj_conn_pos = idx; // current_path->getConnections().size()/2;
    pathplan::ConnectionPtr obj_conn = current_path->getConnections().at(obj_conn_pos);
    pathplan::NodePtr obj_parent = obj_conn->getParent();
    pathplan::NodePtr obj_child = obj_conn->getChild();
    Eigen::VectorXd obj_pos = obj_parent->getConfiguration() + (0.8)*(obj_child->getConfiguration()-obj_parent->getConfiguration());

    pathplan::MoveitUtils moveit_utils(planning_scene,group_name);
    moveit::core::RobotState obj_pos_state = moveit_utils.fromWaypoints2State(obj_pos);
    tf::poseEigenToMsg(obj_pos_state.getGlobalLinkTransform(last_link),obj.pose.pose);
    obj.pose.header.frame_id="world";

    srv.request.objects.push_back(obj);
    if (!add_obj.call(srv))
    {
      ROS_ERROR("call to srv not ok");
      return 1;
    }
    if (!srv.response.success)
    {
      ROS_ERROR("srv error");
      return 1;

    }
    // ///////////////////////////////////UPDATING THE PLANNING SCENE WITH THE NEW OBSTACLE ////////////////////////////////////////

    if (!ps_client.waitForExistence(ros::Duration(10)))
    {
      ROS_ERROR("unable to connect to /get_planning_scene");
      return 1;
    }

    moveit_msgs::GetPlanningScene ps_srv;

    if (!ps_client.call(ps_srv))
    {
      ROS_ERROR("call to srv not ok");
      return 1;
    }


    if (!planning_scene->setPlanningSceneMsg(ps_srv.response.scene))
    {
      ROS_ERROR("unable to update planning scene");
      return 1;
    }

    // ///////////////////////////////////////////////////PATH CHECKING & REPLANNING/////////////////////////////////////////////////////
    checker_parallel->setPlanningSceneMsg(ps_srv.response.scene);

    bool valid;
    /*valid =current_path->isValid();
    ROS_INFO_STREAM("current path valid: "<<valid);
    valid =current_path->isValidFromConn(current_path->getConnections().at(obj_conn_pos+1));
    ROS_INFO_STREAM("obj conn valid: "<<valid);
    valid =current_path->isValidFromConf(current_configuration);
    ROS_INFO_STREAM("conf valid: "<<valid);
    valid =current_path->isValidFromConf(current_path->getConnections().at(obj_conn_pos)->getParent()->getConfiguration());
    ROS_INFO_STREAM("conf parent valid: "<<valid);
    pathplan::ConnectionPtr last_conn = current_path->getConnections().back();
    Eigen::VectorXd conf = (last_conn->getParent()->getConfiguration()+last_conn->getChild()->getConfiguration())/2.0;
    valid =current_path->isValidFromConf(conf);
    ROS_INFO_STREAM("conf last valid: "<<valid);*/

    //valid =current_path->isValidFromConf(current_configuration);


    pathplan::PathPtr current_path_parallel = current_path->clone();
    valid =current_path->isValidFromConf(current_configuration,checker);
    double cost = current_path->cost();
    ROS_INFO_STREAM("BASE path valid from conf: "<<valid <<" cost: "<<cost);

    valid =current_path_parallel->isValidFromConf(current_configuration,checker_parallel);
    cost = current_path_parallel->cost();
    ROS_INFO_STREAM("PARALLEL path valid from conf: "<<valid <<" cost: "<<cost);

    valid = checker_parallel->checkConnection(current_path->getConnections().at(0));
    ROS_INFO_STREAM("PARALLEL check first conn: "<<valid);
    valid = checker_parallel->checkConnFromConf(current_path->getConnections().at(0),current_configuration);
    ROS_INFO_STREAM("PARALLEL check first conn from conf: "<<valid);

    valid = checker->checkConnection(current_path->getConnections().at(0));
    ROS_INFO_STREAM("BASE check first conn: "<<valid);
    valid = checker->checkConnFromConf(current_path->getConnections().at(0),current_configuration);
    ROS_INFO_STREAM("BASE check first conn from conf: "<<valid);

    ros::Duration(2).sleep();
  }
  return 0;
}

