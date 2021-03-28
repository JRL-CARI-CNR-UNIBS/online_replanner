#include <ros/ros.h>
#include <graph_replanning/replanner_manager.h>
//#include <configuration_msgs/StartConfiguration.h>
//#include <configuration_msgs/StopConfiguration.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "node_replanner_manager_real");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(4);
  spinner.start();

  //ros::ServiceClient configuration_client=nh.serviceClient<configuration_msgs::StartConfiguration>("/configuration_manager/start_configuration");
  //ros::ServiceClient stop_configuration_client=nh.serviceClient<configuration_msgs::StopConfiguration>("/configuration_manager/stop_configuration");
  ros::ServiceClient ps_client=nh.serviceClient<moveit_msgs::GetPlanningScene>("/get_planning_scene");
  ros::ServiceClient add_obj=nh.serviceClient<object_loader_msgs::AddObjects>("add_object_to_scene");
  ros::ServiceClient remove_obj=nh.serviceClient<object_loader_msgs::RemoveObjects>("remove_object_from_scene");


  bool optimize_path;
  if (!nh.getParam("opt_path", optimize_path))
  {
    ROS_INFO("optimize_path not set, used 1");
    optimize_path = 1;
  }

  int init_test;
  if (!nh.getParam("init_test",init_test))
  {
    ROS_INFO("init_test not set, use 0");
    init_test=0;
  }

  int end_test;
  if (!nh.getParam("end_test",end_test))
  {
    ROS_INFO("end_test not set, use 10");
    end_test=10;
  }

  std::string test_name;
  if (!nh.getParam("test_name", test_name)) //"/binary_logger/test_name"
  {
    ROS_INFO("test_name not set, used no_name");
    test_name = "no_name";
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

  std::string test;
  if (!nh.getParam("test_name",test))
  {
    ROS_INFO("test_name not set, use noTest");
    test="noTest";
  }

  bool get_real_start_pos;
  if(!nh.getParam("get_real_start_pos",get_real_start_pos))
  {
    get_real_start_pos = false;
  }

  std::vector<double> start_configuration;
  if(!get_real_start_pos)
  {
    if (!nh.getParam("start_configuration",start_configuration))
    {
      ROS_ERROR("start_configuration not set, exit");
      return 0;
    }
  }

  std::vector<double> stop_configuration;
  if (!nh.getParam("stop_configuration",stop_configuration))
  {
    ROS_ERROR("stop_configuration not set, exit");
    return 0;
  }

  double checker_resolution;
  if (!nh.getParam("checker_resolution",checker_resolution))
  {
    ROS_ERROR("checker_resolution not set, set 0.05");
    checker_resolution = 0.05;
  }

  for(int n_iter = init_test; n_iter<end_test; n_iter++)
  {
    if(!ros::ok()) break;

    ROS_WARN("ITER n: %d",n_iter+1);

    //configuration_msgs::StartConfiguration srv_start_conf;
    //srv_start_conf.request.start_configuration="feedforward";
    //srv_start_conf.request.strictness=1;
    //configuration_client.call(srv_start);
    //ros::Duration(2).sleep();
    //configuration_msgs::StopConfiguration srv_stop_conf;
    //srv_stop_conf.request.strictness=1;

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

    Eigen::VectorXd start_conf;
    if(get_real_start_pos) move_group.getCurrentState()->copyJointGroupPositions(joint_model_group,start_configuration);
    else
    {
      moveit::planning_interface::MoveGroupInterface::Plan plan;

      move_group.setStartState(*move_group.getCurrentState());
      move_group.setJointValueTarget(start_configuration);

      bool success = (move_group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
      if(!success)
      {
        ROS_ERROR("Planning to start configuration failed");
        return 0;
      }
      move_group.execute(plan);
    }
    start_conf = Eigen::Map<Eigen::VectorXd>(start_configuration.data(), start_configuration.size());

    Eigen::VectorXd goal_conf = Eigen::Map<Eigen::VectorXd>(stop_configuration.data(), stop_configuration.size());
    pathplan::NodePtr start_node = std::make_shared<pathplan::Node>(start_conf);

    pathplan::MetricsPtr metrics = std::make_shared<pathplan::Metrics>();
    pathplan::CollisionCheckerPtr checker = std::make_shared<pathplan::MoveitCollisionChecker>(planning_scene, group_name, checker_resolution);

    // ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    moveit_msgs::GetPlanningScene ps_srv;
    object_loader_msgs::AddObjects add_srv;
    object_loader_msgs::RemoveObjects remove_srv;

    if(test_name == "sharework")
    {
      std::vector<double> starting_obs_pos;
      if (!nh.getParam("starting_obs_pos",starting_obs_pos))
      {
        starting_obs_pos = {1.0,0.0,1.5};
      }

      if (!add_obj.waitForExistence(ros::Duration(10)))
      {
        ROS_FATAL("srv not found");
        return 1;
      }
      object_loader_msgs::Object obj;
      obj.object_type="big_box_sharework";

      obj.pose.pose.position.x = starting_obs_pos.at(0);
      obj.pose.pose.position.y = starting_obs_pos.at(1);
      obj.pose.pose.position.z = starting_obs_pos.at(2);

      obj.pose.pose.orientation.x = 0.0;
      obj.pose.pose.orientation.y = 0.0;
      obj.pose.pose.orientation.z = 0.0;
      obj.pose.pose.orientation.w = 1.0;

      obj.pose.header.frame_id="world";

      add_srv.request.objects.push_back(obj);
      if (!add_obj.call(add_srv))
      {
        ROS_ERROR("call to srv not ok");
        return 1;
      }
      if (!add_srv.response.success)
      {
        ROS_ERROR("srv error");
        return 1;
      }
      else
      {
        for (const std::string& str: add_srv.response.ids)
        {
          remove_srv.request.obj_ids.push_back(str);
        }
      }
    }

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

    if (!planning_scene->setPlanningSceneMsg(ps_srv.response.scene))
    {
      ROS_ERROR("unable to update planning scene");
      return 1;
    }

    // //////////////////////////////////////PATH PLAN//////////////////////////////////////////////////////////////////////////
    pathplan::PathPtr path = NULL;
    pathplan::TrajectoryPtr trajectory = std::make_shared<pathplan::Trajectory>(path,nh,planning_scene,group_name,base_link,last_link);

    std::vector<pathplan::PathPtr> path_vector;

    for (unsigned int i =0; i<4; i++)
    {
      pathplan::NodePtr goal_node = std::make_shared<pathplan::Node>(goal_conf);
      pathplan::PathPtr solution = trajectory->computeBiRRTPath(start_node, goal_node, lb, ub, metrics, checker, optimize_path);
      path_vector.push_back(solution);
    }

    // ///////////////////////////////////////////////////////////////////////////
    if(test_name == "sharework")
    {
      if(!remove_obj.waitForExistence(ros::Duration(10)))
      {
        ROS_FATAL("srv not found");
      }
      if(!remove_obj.call(remove_srv))
      {
        ROS_ERROR("call to srv not ok");
      }
      if(!remove_srv.response.success)
      {
        ROS_ERROR("srv error");
      }

      moveit_msgs::GetPlanningScene ps_srv;

      if(!ps_client.call(ps_srv))
      {
        ROS_ERROR("call to srv not ok");
        return 1;
      }

      if(!planning_scene->setPlanningSceneMsg(ps_srv.response.scene))
      {
        ROS_ERROR("unable to update planning scene");
        return 1;
      }
    }

    if(!ps_client.waitForExistence(ros::Duration(10)))
    {
      ROS_ERROR("unable to connect to /get_planning_scene");
      return 1;
    }

    if(!ps_client.call(ps_srv))
    {
      ROS_ERROR("call to srv not ok");
      return 1;
    }

    if(!planning_scene->setPlanningSceneMsg(ps_srv.response.scene))
    {
      ROS_ERROR("unable to update planning scene");
      return 1;
    }
    // ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


    pathplan::PathPtr current_path = path_vector.front();
    std::vector<pathplan::PathPtr> other_paths = {path_vector.at(1),path_vector.at(2),path_vector.at(3)};

    ROS_INFO_STREAM("cost current path: "<<current_path->cost());
    for(const pathplan::PathPtr& path:other_paths) ROS_INFO_STREAM("cost path: "<<path->cost());

    pathplan::ReplannerManagerPtr replanner_manager = std::make_shared<pathplan::ReplannerManager>(current_path, other_paths, nh);
    replanner_manager->trajectoryExecutionThread();
  }

  return 0;
}
