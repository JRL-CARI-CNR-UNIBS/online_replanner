#include <ros/ros.h>
#include <graph_core/parallel_moveit_collision_checker.h>
#include <graph_replanning/trajectory.h>
#include <moveit/robot_state/robot_state.h>
#include <graph_replanning/replanner.h>
#include <object_loader_msgs/AddObjects.h>
#include <object_loader_msgs/RemoveObjects.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "node_example_replanner");
  ros::AsyncSpinner spinner(4);
  spinner.start();

  ros::NodeHandle nh;

  // //////////////////////////////////////// GETTING ROS PARAM ///////////////////////////////////////////////
  int n_paths;
  if (!nh.getParam("number_paths",n_paths))
  {
    ROS_INFO("number of paths not set, set  3");
    n_paths = 3;
  }

  bool display_step_by_step;
  if (!nh.getParam("display_step_by_step",display_step_by_step))
  {
    ROS_INFO("display_step_by_step not set, set false");
    display_step_by_step=false;
  }

  double time_replanning;
  if (!nh.getParam("max_time_for_replanning",time_replanning))
  {
    ROS_INFO("max_time_for_replanning not set, set inf");
    time_replanning=std::numeric_limits<double>::infinity();
  }

  std::string group_name;
  if (!nh.getParam("group_name",group_name))
  {
    ROS_ERROR("group_name not set, exit");
    return 0;
  }

  std::string base_link;
  if (!nh.getParam("base_link",base_link))
  {
    ROS_ERROR("base_link not set, exit");
    return 0;
  }

  std::string last_link;
  if (!nh.getParam("last_link",last_link))
  {
    ROS_ERROR("last_link not set, exit");
    return 0;
  }

  std::vector<double> start_configuration;
  if (!nh.getParam("start_configuration",start_configuration))
  {
    ROS_ERROR("start_configuration not set, exit");
    return 0;
  }

  std::vector<double> stop_configuration;
  if (!nh.getParam("stop_configuration",stop_configuration))
  {
    ROS_ERROR("stop_configuration not set, exit");
    return 0;
  }
  // /////////////////////////////////UPLOADING THE ROBOT ARM/////////////////////////////////////////////////////////////

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

  // //////////////////////////////////////////UPDATING THE PLANNING SCENE////////////////////////////////////////////////////////
  ros::ServiceClient ps_client=nh.serviceClient<moveit_msgs::GetPlanningScene>("/get_planning_scene");
  moveit_msgs::GetPlanningScene ps_srv;

  if (!ps_client.call(ps_srv))
  {
    ROS_ERROR("call to srv not ok");
    return 0;
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
  Eigen::VectorXd start_conf = Eigen::Map<Eigen::VectorXd>(start_configuration.data(), start_configuration.size());
  Eigen::VectorXd goal_conf = Eigen::Map<Eigen::VectorXd>(stop_configuration.data(), stop_configuration.size());

  int id=100;
  int id_wp = 1000;
  for (unsigned int i =0; i<n_paths; i++)
  {
    pathplan::SamplerPtr sampler = std::make_shared<pathplan::InformedSampler>(start_conf, goal_conf, lb, ub);
    pathplan::BiRRTPtr solver = std::make_shared<pathplan::BiRRT>(metrics, checker, sampler);
    pathplan::PathPtr solution = trajectory.computePath(start_conf, goal_conf,solver,1);
    path_vector.push_back(solution);

    std::vector<double> marker_color;
    int j = i%5;
    if(j==0) marker_color = {0.5,0.5,0.0,1.0};
    if(j==1) marker_color = {0.0,0.0,1.0,1.0};
    if(j==2) marker_color = {1.0,0.0,0.0,1.0};
    if(j==3) marker_color = {0.0,1.0,0.0,1.0};
    if(j==4) marker_color = {0.7,0.7,0.7,1.0};

    disp->displayPathAndWaypoints(solution,id,id_wp,"pathplan",marker_color);
    id +=1;
    id_wp +=50;

    ros::Duration(0.5).sleep();
  }

  pathplan::PathPtr current_path = path_vector.front();
  std::vector<pathplan::PathPtr> other_paths;
  other_paths.insert(other_paths.end(),path_vector.begin()+1,path_vector.end());

  int idx_current_conn = 1;

  int size = current_path->getConnections().size();
  if(idx_current_conn>size-1)
  {
    ROS_WARN("number_of_connection_on_which_current_conf_is (%d) is bigger than the current path number of connections (%d), set equal to the last conn number",idx_current_conn,(size-1));
    idx_current_conn = size-1;
  }

  Eigen::VectorXd parent = current_path->getConnections().at(idx_current_conn)->getParent()->getConfiguration();
  Eigen::VectorXd child = current_path->getConnections().at(idx_current_conn)->getChild()->getConfiguration();

  Eigen::VectorXd current_configuration = parent + (child-parent)*0.3;

  // //////////////////////////////////////ADDING THE MOBILE OBSTACLE/////////////////////////////////////
  object_loader_msgs::AddObjects add_srv;
  object_loader_msgs::RemoveObjects remove_srv;
  ros::ServiceClient add_obj=nh.serviceClient<object_loader_msgs::AddObjects>("add_object_to_scene");
  ros::ServiceClient remove_obj=nh.serviceClient<object_loader_msgs::RemoveObjects>("remove_object_from_scene");

  if (!add_obj.waitForExistence(ros::Duration(10)))
  {
    ROS_FATAL("srv not found");
    return 0;
  }

  object_loader_msgs::Object obj;
  obj.object_type="scatola";

  std::srand(time(NULL));
  int obj_conn_pos = (rand() % (size-idx_current_conn)) + idx_current_conn;
  pathplan::ConnectionPtr obj_conn = current_path->getConnections().at(obj_conn_pos);
  pathplan::NodePtr obj_parent = obj_conn->getParent();
  pathplan::NodePtr obj_child = obj_conn->getChild();
  Eigen::VectorXd obj_pos = obj_parent->getConfiguration()+(obj_child->getConfiguration()-obj_parent->getConfiguration())*0.8;

  pathplan::MoveitUtils moveit_utils(planning_scene,group_name);
  moveit::core::RobotState obj_pos_state = moveit_utils.fromWaypoints2State(obj_pos);
  tf::poseEigenToMsg(obj_pos_state.getGlobalLinkTransform(last_link),obj.pose.pose);
  obj.pose.header.frame_id="world";

  add_srv.request.objects.push_back(obj);
  if (!add_obj.call(add_srv))
  {
    ROS_ERROR("call to srv not ok");
    return 0;
  }
  if (!add_srv.response.success)
  {
    ROS_ERROR("srv error");
    return 1;
  }
  else
  {
    remove_srv.request.obj_ids.clear();
    for (const std::string& str: add_srv.response.ids)
    {
      remove_srv.request.obj_ids.push_back(str);
    }
  }
  // ///////////////////////////////////UPDATING THE PLANNING SCENE WITH THE NEW OBSTACLE ////////////////////////////////////////
  if (!ps_client.call(ps_srv))
  {
    ROS_ERROR("call to srv not ok");
    return 0;
  }

  if (!planning_scene->setPlanningSceneMsg(ps_srv.response.scene))
  {
    ROS_ERROR("unable to update planning scene");
    return 0;
  }

  checker->setPlanningSceneMsg(ps_srv.response.scene);
  // ///////////////////////////////////////////////////PATH CHECKING/////////////////////////////////////////////////////

  for(unsigned int i=0; i<n_paths;i++)
  {
    bool valid;
    valid =path_vector.at(i)->isValid();
    if(valid)
    {
      ROS_INFO_STREAM("path "<<i<<" is free");
    }
    else
    {
      ROS_INFO_STREAM("path "<<i<<" is obstructed");

    }
  }

  pathplan::SamplerPtr samp = std::make_shared<pathplan::InformedSampler>(start_conf, goal_conf, lb, ub);
  pathplan::BiRRTPtr solver = std::make_shared<pathplan::BiRRT>(metrics, checker, samp);
  solver->config(nh);

  // ///////////////////////////////////////// VISUALIZATION OF CURRENT NODE ///////////////////////////////////////////////////////////
  std::vector<double> marker_color_sphere_actual = {1.0,0.0,1.0,1.0};
  disp->displayNode(std::make_shared<pathplan::Node>(current_configuration),id,"pathplan",marker_color_sphere_actual);
  id++;
  // //////////////////////////////////////// REPLANNING ////////////////////////////////////////////////////////////////
  bool success;

  pathplan::Replanner replanner = pathplan::Replanner(current_configuration, current_path, other_paths, solver, metrics, checker, lb, ub);

  if(display_step_by_step)
  {
    replanner.setInformedOnlineReplanningDisp(disp);
    replanner.setPathSwitchDisp(disp);
  }

  ros::WallTime tic = ros::WallTime::now();
  success =  replanner.informedOnlineReplanning(time_replanning);
  ros::WallTime toc = ros::WallTime::now();

  ROS_INFO_STREAM("DURATION: "<<(toc-tic).toSec()<<" success: "<<success<< " n sol: "<<replanner.getReplannedPathVector().size());
  ros::Duration(0.01).sleep();

  if(success)
  {
    ROS_WARN("SUCCESS");

    std::vector<int> marker_id; marker_id.push_back(-101);
    std::vector<double> marker_color;
    marker_color = {1.0,1.0,0.0,1.0};

    std::vector<double> marker_scale(3,0.01);
    disp->changeConnectionSize(marker_scale);
    disp->displayPath(replanner.getReplannedPath(),id,"pathplan",marker_color);
  }
  else
  {
    ROS_WARN("NOT SUCCESS");
  }

  ros::Duration(10).sleep();

  // ////////////////////////////////////////// REMOVING MOBILE OBSTACLE //////////////////////////////////////////////////
  if (!remove_obj.waitForExistence(ros::Duration(10)))
  {
    ROS_FATAL("srv not found");
  }
  if (!remove_obj.call(remove_srv))
  {
    ROS_ERROR("call to srv not ok");
  }
  if (!remove_srv.response.success)
  {
    ROS_ERROR("srv error");
  }

  ROS_INFO("--------------------------------");

  return 0;
}

