#include <ros/ros.h>
#include <graph_core/parallel_moveit_collision_checker.h>
#include <graph_replanning/trajectory.h>
#include <moveit/robot_state/robot_state.h>
#include <graph_replanning/replanner.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "node_test_replanner");
  ros::AsyncSpinner spinner(4);
  spinner.start();

  ros::NodeHandle nh;

  // ////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  std::string group_name = "cartesian_arm";
  std::string last_link = "end_effector";
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

  // //////////////////////////////////////////UPDATING PLANNING SCENE////////////////////////////////////////////////////////
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
    return 0;
  }

  // //////////////////////////////////////////PATH PLAN & VISUALIZATION////////////////////////////////////////////////////////

  pathplan::MetricsPtr metrics = std::make_shared<pathplan::Metrics>();
  pathplan::CollisionCheckerPtr checker = std::make_shared<pathplan::ParallelMoveitCollisionChecker>(planning_scene, group_name);

  pathplan::DisplayPtr disp = std::make_shared<pathplan::Display>(planning_scene,group_name,last_link);
  disp->clearMarkers();
  ros::Duration(1).sleep();
  pathplan::Trajectory trajectory = pathplan::Trajectory(nh,planning_scene,group_name);

  std::vector<pathplan::PathPtr> path_vector;
  Eigen::VectorXd start_conf(3);
  start_conf << 0.0,0.0,0.0;
  Eigen::VectorXd goal_conf(3);
  goal_conf << 0.8,0.8,0.8;

  int id=100;
  int id_wp = 1000;
  for (unsigned int i =0; i<2; i++)
  {
    pathplan::SamplerPtr sampler = std::make_shared<pathplan::InformedSampler>(start_conf, goal_conf, lb, ub);
    pathplan::BiRRTPtr solver = std::make_shared<pathplan::BiRRT>(metrics, checker, sampler);
    pathplan::PathPtr solution = trajectory.computePath(start_conf, goal_conf,solver,1);
    path_vector.push_back(solution);

    std::vector<double> marker_color;
    if(i==0) marker_color = {0.5,0.5,0.0,1.0};
    if(i==1) marker_color = {0.0f,0.0f,1.0,1.0};

    disp->displayPathAndWaypoints(solution,id,id_wp,"pathplan",marker_color);
    id +=1;
    id_wp +=50;

    ros::Duration(0.5).sleep();
  }

  pathplan::PathPtr current_path = path_vector.front();
  std::vector<pathplan::PathPtr> other_paths = {path_vector.at(1)};

  Eigen::VectorXd parent = current_path->getConnections().at(2)->getParent()->getConfiguration();
  Eigen::VectorXd child = current_path->getConnections().at(2)->getChild()->getConfiguration();

  Eigen::VectorXd current_configuration = parent + (child-parent)*0.5;

  current_path->getConnections().at(3)->setCost(std::numeric_limits<double>::infinity());
  current_path->cost();

  // ///////////////////////////////////////// VISUALIZATION OF CURRENT NODE ///////////////////////////////////////////////////////////
  std::vector<double> marker_color_sphere_actual = {1.0,0.0,1.0,1.0};
  disp->displayNode(std::make_shared<pathplan::Node>(current_configuration),id,"pathplan",marker_color_sphere_actual);
  id++;
  // //////////////////////////////////////// REPLANNING & TEST ////////////////////////////////////////////////////////////////
  pathplan::SamplerPtr samp = std::make_shared<pathplan::InformedSampler>(start_conf, goal_conf, lb, ub);
  pathplan::BiRRTPtr solver = std::make_shared<pathplan::BiRRT>(metrics, checker, samp);
  solver->config(nh);

  pathplan::Replanner replanner = pathplan::Replanner(current_configuration, current_path, other_paths, solver, metrics, checker, lb, ub);

  double time_repl = 1.0;
  bool success =  replanner.informedOnlineReplanning(time_repl);

  std::vector<double> marker_color = {1.0,1.0,0.0,1.0};
  std::vector<double> marker_color_sphere_new_curr_conf = {0.0,0.0,0.0,1.0};
  int id_new_curr_conf = 1678;

  if(success)
  {
    ROS_WARN("SUCCESS");

    std::vector<double> marker_scale(3,0.01);
    disp->changeConnectionSize(marker_scale);
    disp->displayPath(replanner.getReplannedPath(),id,"pathplan",marker_color);
  }
  else
  {
    return 0;
  }

  int idx_replanned_path_start;
  pathplan::PathPtr replanned_path = replanner.getReplannedPath()->clone();
  pathplan::ConnectionPtr replanned_path_conn = current_path->findConnection(replanned_path->getWaypoints().at(0),idx_replanned_path_start);
  pathplan::NodePtr parent_node,child_node;

  Eigen::VectorXd new_current_configuration;

  //
  disp->nextButton("Starting replanned path from conf1 -> press NEXT");
  parent_node = current_path->getConnections().at(idx_replanned_path_start)->getParent();
  child_node = current_path->getConnections().at(idx_replanned_path_start)->getChild();

  new_current_configuration = parent_node->getConfiguration()+0.3*(child_node->getConfiguration()-parent_node->getConfiguration());

  replanner.startReplannedPathFromNewCurrentConf(new_current_configuration);

  disp->displayPath(replanner.getReplannedPath(),id,"pathplan",marker_color);
  disp->displayNode(std::make_shared<pathplan::Node>(new_current_configuration),id_new_curr_conf,"pathplan",marker_color_sphere_new_curr_conf);

  replanner.setReplannedPath(replanned_path->clone());
  //
  disp->nextButton("Starting replanned path from conf2 -> press NEXT");
  parent_node = current_path->getConnections().at(0)->getParent();
  child_node = current_path->getConnections().at(0)->getChild();

  new_current_configuration = current_path->getWaypoints().at(0)+0.3*(current_path->getWaypoints().at(1)-current_path->getWaypoints().at(0));

  replanner.startReplannedPathFromNewCurrentConf(new_current_configuration);

  disp->displayPath(replanner.getReplannedPath(),id,"pathplan",marker_color);
  disp->displayNode(std::make_shared<pathplan::Node>(new_current_configuration),id_new_curr_conf,"pathplan",marker_color_sphere_new_curr_conf);

  replanner.setReplannedPath(replanned_path->clone());
  //
  disp->nextButton("Starting replanned path from conf3 -> press NEXT");
  parent_node = replanned_path_conn->getParent();
  child_node = replanned_path_conn->getChild();

  new_current_configuration = parent_node->getConfiguration()+0.3*(child_node->getConfiguration()-parent_node->getConfiguration());

  replanner.startReplannedPathFromNewCurrentConf(new_current_configuration);

  disp->displayPath(replanner.getReplannedPath(),id,"pathplan",marker_color);
  disp->displayNode(std::make_shared<pathplan::Node>(new_current_configuration),id_new_curr_conf,"pathplan",marker_color_sphere_new_curr_conf);

  replanner.setReplannedPath(replanned_path->clone());
  //
  disp->nextButton("Starting replanned path from conf4 -> press NEXT");
  parent_node = replanned_path->getConnections().at(1)->getParent();
  child_node = replanned_path->getConnections().at(1)->getChild();

  new_current_configuration = parent_node->getConfiguration()+0.3*(child_node->getConfiguration()-parent_node->getConfiguration());

  replanner.startReplannedPathFromNewCurrentConf(new_current_configuration);

  disp->displayPath(replanner.getReplannedPath(),id,"pathplan",marker_color);
  disp->displayNode(std::make_shared<pathplan::Node>(new_current_configuration),id_new_curr_conf,"pathplan",marker_color_sphere_new_curr_conf);

  replanner.setReplannedPath(replanned_path->clone());
  //
  disp->nextButton("Starting replanned path from conf5 -> press NEXT");
  parent_node = current_path->getConnections().at(idx_replanned_path_start+1)->getParent();
  child_node = current_path->getConnections().at(idx_replanned_path_start+1)->getChild();

  new_current_configuration = parent_node->getConfiguration()+0.3*(child_node->getConfiguration()-parent_node->getConfiguration());

  replanner.startReplannedPathFromNewCurrentConf(new_current_configuration);

  disp->displayPath(replanner.getReplannedPath(),id,"pathplan",marker_color);
  disp->displayNode(std::make_shared<pathplan::Node>(new_current_configuration),id_new_curr_conf,"pathplan",marker_color_sphere_new_curr_conf);

  return 0;
}

