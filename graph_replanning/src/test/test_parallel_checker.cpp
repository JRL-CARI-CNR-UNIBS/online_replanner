#include <ros/ros.h>
#include <graph_replanning/trajectory.h>
#include <graph_replanning/moveit_utils.h>
#include <moveit/robot_state/robot_state.h>
#include <graph_replanning/replanner.h>
#include <object_loader_msgs/AddObjects.h>
#include <object_loader_msgs/RemoveObjects.h>
#include <graph_core/parallel_moveit_collision_checker.h>

int main(int argc, char **argv)
{
  unsigned int n_paths = 3;

  ros::init(argc, argv, "node_parallel_check");
  ros::AsyncSpinner spinner(4);
  spinner.start();

  ros::NodeHandle nh;

  // //////////////////////////////////////// GETTING ROS PARAM ///////////////////////////////////////////////

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

  int n_test;
  if (!nh.getParam("n_test",n_test))
  {
    ROS_ERROR("n_test not set, set 1");
    n_test = 1;
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
    const robot_model::VariableBounds& bounds = kinematic_model->getVariableBounds(joint_names.at(idx));  //bounds dei joints definito in urdf e file joints limit
    if (bounds.position_bounded_)
    {
      lb(idx) = bounds.min_position_;
      ub(idx) = bounds.max_position_;
    }
  }

  ros::ServiceClient add_obj=nh.serviceClient<object_loader_msgs::AddObjects>("add_object_to_scene");
  ros::ServiceClient remove_obj=nh.serviceClient<object_loader_msgs::RemoveObjects>("remove_object_from_scene");
  object_loader_msgs::AddObjects add_srv;
  object_loader_msgs::RemoveObjects remove_srv;

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

  // //////////////////////////////////////////PATH PLAN & VISUALIZATION////////////////////////////////////////////////////////

  ROS_WARN("CREAZIONE PATH");

  pathplan::MetricsPtr metrics = std::make_shared<pathplan::Metrics>();
  pathplan::CollisionCheckerPtr checker = std::make_shared<pathplan::ParallelMoveitCollisionChecker>(planning_scene, group_name, 5, 0.001);

  pathplan::Display disp = pathplan::Display(planning_scene,group_name,last_link);
  disp.clearMarkers();
  ros::Duration(1).sleep();
  pathplan::Trajectory trajectory = pathplan::Trajectory(nh,planning_scene,group_name);

  for(unsigned int j=0; j<n_test;j++)
  {
    std::vector<pathplan::PathPtr> path_vector;
    Eigen::VectorXd start_conf = Eigen::Map<Eigen::VectorXd>(start_configuration.data(), start_configuration.size());
    Eigen::VectorXd goal_conf = Eigen::Map<Eigen::VectorXd>(stop_configuration.data(), stop_configuration.size());

    int id=0;
    int id_wp = 1000;
    for (unsigned int i =0; i<n_paths; i++)
    {
      pathplan::SamplerPtr sampler = std::make_shared<pathplan::InformedSampler>(start_conf, goal_conf, lb, ub);
      pathplan::BiRRTPtr solver = std::make_shared<pathplan::BiRRT>(metrics, checker, sampler);

      pathplan::PathPtr solution = trajectory.computePath(start_conf,goal_conf,solver,0);
      path_vector.push_back(solution);
      ros::Duration(0.1).sleep();

      std::vector<double> marker_color;
      if(i==0) marker_color = {0.5,0.5,0.0,1.0};
      if(i==1) marker_color = {0.0f,0.0f,1.0,1.0};
      if(i==2) marker_color = {1.0,0.0f,0.0f,1.0};

      disp.displayPathAndWaypoints(solution,id,id_wp,"pathplan",marker_color);
      id++;
      id_wp +=50;
    }

    pathplan::PathPtr current_path = path_vector.front();
    std::vector<pathplan::PathPtr> other_paths = {path_vector.at(1),path_vector.at(2)};

    int idx = 0;
    pathplan::SamplerPtr samp = std::make_shared<pathplan::InformedSampler>(start_conf, goal_conf, lb, ub);
    pathplan::BiRRTPtr solver = std::make_shared<pathplan::BiRRT>(metrics, checker, samp);
    solver->config(nh);

    Eigen::VectorXd current_configuration = (current_path->getConnections().at(idx)->getChild()->getConfiguration() + current_path->getConnections().at(idx)->getParent()->getConfiguration())/2.0;

    // ///////////////////////////////////////// VISUALIZATION OF CURRENT NODE ///////////////////////////////////////////////////////////
    std::vector<double> marker_color_sphere_actual = {1.0,0.0,1.0,1.0};
    disp.displayNode(std::make_shared<pathplan::Node>(current_configuration),id,"pathplan",marker_color_sphere_actual);
    id++;

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

    int obj_conn_pos = idx; //current_path->getConnections().size()/2;
    pathplan::ConnectionPtr obj_conn = current_path->getConnections().at(obj_conn_pos);
    pathplan::NodePtr obj_parent = obj_conn->getParent();
    pathplan::NodePtr obj_child = obj_conn->getChild();
    Eigen::VectorXd obj_pos = obj_parent->getConfiguration() + 0.8*(obj_child->getConfiguration()-obj_parent->getConfiguration());//(obj_child->getConfiguration()+obj_parent->getConfiguration())/2;

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
    else
    {
      remove_srv.request.obj_ids.clear();
      for (const std::string& str: srv.response.ids)
      {
        remove_srv.request.obj_ids.push_back(str);
      }
    }
    // ///////////////////////////////////UPDATING THE PLANNING SCENE WITH THE NEW OBSTACLE ////////////////////////////////////////

    if (!ps_client.waitForExistence(ros::Duration(10)))
    {
      ROS_ERROR("unable to connect to /get_planning_scene");
      return 1;
    }

    if (!ps_client.call(ps_srv))
    {
      ROS_ERROR("call to srv not ok");
      return 1;
    }

    checker->setPlanningSceneMsg(ps_srv.response.scene);
    /*if (!planning_scene->setPlanningSceneMsg(ps_srv.response.scene))
    {
      ROS_ERROR("unable to update planning scene");
      return 1;
    }*/


    // ///////////////////////////////////////////////////PATH CHECKING & REPLANNING//////////////////////////////////////////////////
    pathplan::CollisionCheckerPtr checker_base = std::make_shared<pathplan::MoveitCollisionChecker>(planning_scene, group_name, 0.001);

    bool valid;
    valid =current_path->isValid(checker);
    ROS_INFO_STREAM("current path valid: "<<valid);

    valid =current_path->isValidFromConf(current_configuration,checker);
    ROS_INFO_STREAM("current path valid from conf: "<<valid);

    valid =current_path->isValid(checker_base);
    ROS_INFO_STREAM("BASE current path valid: "<<valid);

    valid =current_path->isValidFromConf(current_configuration,checker_base);
    ROS_INFO_STREAM("BASE current path valid from conf: "<<valid);

    valid = other_paths.at(0)->isValid(checker);
    ROS_INFO_STREAM("path2 valid: "<<valid);

    valid = other_paths.at(0)->isValid(checker_base);
    ROS_INFO_STREAM("BASE path2 valid: "<<valid);

    valid = other_paths.at(1)->isValid(checker);
    ROS_INFO_STREAM("path3 valid: "<<valid);

    valid = other_paths.at(1)->isValid(checker_base);
    ROS_INFO_STREAM("BASE path3 valid: "<<valid);

    ros::Duration(5).sleep();

    //Removing mobile obs
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

    ROS_INFO("--------------------------------");
  }
  return 0;
}

