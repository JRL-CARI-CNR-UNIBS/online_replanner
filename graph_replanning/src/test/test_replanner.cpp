#include <ros/ros.h>
#include <graph_core/parallel_moveit_collision_checker.h>
#include <graph_replanning/trajectory.h>
#include <moveit/robot_state/robot_state.h>
#include <graph_replanning/replanner.h>
#include <object_loader_msgs/AddObjects.h>
#include <object_loader_msgs/RemoveObjects.h>

int main(int argc, char **argv)
{
  unsigned int n_paths = 3;

  ros::init(argc, argv, "node_test_replanner");
  ros::AsyncSpinner spinner(4);
  spinner.start();

  ros::NodeHandle nh;

  // //////////////////////////////////////// GETTING ROS PARAM ///////////////////////////////////////////////
  std::string test_name;
  if (!nh.getParam("test_name",test_name))
  {
    ROS_INFO("test_name not set, use no_test_name");
    test_name = "no_test_name";
  }

  bool mobile_obstacle;
  if (!nh.getParam("moving_obs",mobile_obstacle))
  {
    ROS_INFO("moving_obs not set, use false");
    mobile_obstacle=false;
  }

  bool verbose_time;
  if (!nh.getParam("time_verb",verbose_time))
  {
    ROS_INFO("time_verb not set, use false");
    verbose_time=false;
  }

  bool verbose_disp_informed;
  if (!nh.getParam("disp_verb_informed",verbose_disp_informed))
  {
    ROS_INFO("disp_verb_informed not set, use false");
    verbose_disp_informed=false;
  }

  bool verbose_disp_pathSwitch;
  if (!nh.getParam("disp_verb_pathSwitch",verbose_disp_pathSwitch))
  {
    ROS_INFO("disp_verb_pathSwitch not set, use false");
    verbose_disp_pathSwitch=false;
  }

  double time;
  if (!nh.getParam("time_for_repl",time))
  {
    ROS_INFO("time_for_repl not set, use inf");
    time=std::numeric_limits<double>::infinity();
  }

  bool optimize_path;
  if(!nh.getParam("opt_path",optimize_path))
  {
    ROS_INFO("opt_path not set, use true");
    optimize_path = true;
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

  int n_test;
  if (!nh.getParam("n_test",n_test))
  {
    ROS_ERROR("n_test not set, set 1");
    n_test = 1;
  }

  std::vector<double> static_obs_pos;
  if(test_name == "sharework")
  {
    if(!nh.getParam("starting_obs_pos",static_obs_pos))
    {
      static_obs_pos = {1.0,0.0,1.5};
    }
    ROS_ERROR("pos obs static not defined");
  }

  ROS_INFO_STREAM("test: "<<test_name<<" mobile ob: "<<mobile_obstacle);

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

  if(test_name == "sharework")
  {
    if (!add_obj.waitForExistence(ros::Duration(10)))
    {
      ROS_FATAL("srv not found");
      return 1;
    }
    object_loader_msgs::Object obj;
    obj.object_type="scatola";

    obj.pose.pose.position.x = static_obs_pos[0];
    obj.pose.pose.position.y = static_obs_pos[1];
    obj.pose.pose.position.z = static_obs_pos[2];

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

  pathplan::MetricsPtr metrics = std::make_shared<pathplan::Metrics>();
  pathplan::CollisionCheckerPtr checker = std::make_shared<pathplan::MoveitCollisionChecker>(planning_scene, group_name);
  pathplan::CollisionCheckerPtr checker_parallel = std::make_shared<pathplan::ParallelMoveitCollisionChecker>(planning_scene, group_name);

  pathplan::DisplayPtr disp = std::make_shared<pathplan::Display>(planning_scene,group_name,last_link);
  disp->clearMarkers();
  ros::Duration(1).sleep();
  pathplan::Trajectory trajectory = pathplan::Trajectory(nh,planning_scene,group_name);

  ros::Duration(5).sleep();

  for(unsigned int j=0; j<n_test;j++)
  {
    std::vector<pathplan::PathPtr> path_vector;
    Eigen::VectorXd start_conf = Eigen::Map<Eigen::VectorXd>(start_configuration.data(), start_configuration.size());
    Eigen::VectorXd goal_conf = Eigen::Map<Eigen::VectorXd>(stop_configuration.data(), stop_configuration.size());

    int id=100;
    int id_wp = 1000;
    for (unsigned int i =0; i<n_paths; i++)
    {
      pathplan::SamplerPtr sampler = std::make_shared<pathplan::InformedSampler>(start_conf, goal_conf, lb, ub);
      pathplan::BiRRTPtr solver = std::make_shared<pathplan::BiRRT>(metrics, checker, sampler);
      pathplan::PathPtr solution = trajectory.computePath(start_conf, goal_conf,solver,optimize_path);
      path_vector.push_back(solution);

      std::vector<double> marker_color;
      if(i==0) marker_color = {0.5,0.5,0.0,1.0};
      if(i==1) marker_color = {0.0f,0.0f,1.0,1.0};
      if(i==2) marker_color = {1.0,0.0f,0.0f,1.0};

      disp->displayPathAndWaypoints(solution,id,id_wp,"pathplan",marker_color);
      id +=1;
      id_wp +=50;

      ros::Duration(0.5).sleep();
    }

    pathplan::PathPtr current_path = path_vector.front();
    std::vector<pathplan::PathPtr> other_paths = {path_vector.at(1),path_vector.at(2)};

    Eigen::VectorXd parent = current_path->getConnections().front()->getParent()->getConfiguration();
    Eigen::VectorXd child = current_path->getConnections().front()->getChild()->getConfiguration();

    Eigen::VectorXd current_configuration = parent + (child-parent)*0.3;

    // ///////////////////////////////////////////////////////////////////////////
    if(test_name == "sharework")
    {
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
    }

    if(mobile_obstacle)
    {
      ros::ServiceClient add_obj=nh.serviceClient<object_loader_msgs::AddObjects>("add_object_to_scene");

      if (!add_obj.waitForExistence(ros::Duration(10)))
      {
        ROS_FATAL("srv not found");
        return 1;
      }

      object_loader_msgs::AddObjects srv;
      object_loader_msgs::Object obj;
      obj.object_type="scatola";

      int obj_conn_pos = current_path->getConnections().size()-3;
      pathplan::ConnectionPtr obj_conn = current_path->getConnections().at(obj_conn_pos);
      pathplan::NodePtr obj_parent = obj_conn->getParent();
      pathplan::NodePtr obj_child = obj_conn->getChild();
      Eigen::VectorXd obj_pos = (obj_child->getConfiguration()+obj_parent->getConfiguration())/2;
      //Eigen::VectorXd obj_pos = obj_parent->getConfiguration()+(obj_child->getConfiguration()-obj_parent->getConfiguration())*0.8;

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

    }
    else
    {
      std::vector<pathplan::ConnectionPtr> conn_v = current_path->getConnections();
      if(conn_v.size()-3 > 0)
      {
        conn_v.at(conn_v.size()-3)->setCost(std::numeric_limits<double>::infinity());
      }
      else
      {
        conn_v.at(conn_v.size()-2)->setCost(std::numeric_limits<double>::infinity());
      }

      conn_v.back()->setCost(std::numeric_limits<double>::infinity());
      current_path->setConnections(conn_v);

      /*conn_v = path_vector.at(1)->getConnections();
      conn_v.back()->setCost(std::numeric_limits<double>::infinity());
      path_vector.at(1)->setConnections(conn_v);

      conn_v = path_vector.at(2)->getConnections();
      conn_v.back()->setCost(std::numeric_limits<double>::infinity());
      path_vector.at(2)->setConnections(conn_v);*/
    }


    // ///////////////////////////////////////////////////PATH CHECKING & REPLANNING/////////////////////////////////////////////////////

    bool valid_checker = checker->checkConnFromConf(current_path->getConnections().at(0),current_configuration);
    bool valid_checker_parallel = checker_parallel->checkConnFromConf(current_path->getConnections().at(0),current_configuration);
    bool valid_checker_parallel1 = checker_parallel->checkConnection(current_path->getConnections().at(0));

    ROS_INFO_STREAM("checker: "<<valid_checker<<" checker parallel: "<<valid_checker_parallel<<" check1: "<<valid_checker_parallel1);

    bool valid;
    valid =current_path->isValid();
    ROS_INFO_STREAM("current path valid: "<<valid);

    valid = other_paths.at(0)->isValid();
    ROS_INFO_STREAM("path2 valid: "<<valid);

    valid = other_paths.at(1)->isValid();
    ROS_INFO_STREAM("path3 valid: "<<valid);

    pathplan::SamplerPtr samp = std::make_shared<pathplan::InformedSampler>(start_conf, goal_conf, lb, ub);
    pathplan::BiRRTPtr solver = std::make_shared<pathplan::BiRRT>(metrics, checker, samp);
    solver->config(nh);

    // ///////////////////////////////////////// VISUALIZATION OF CURRENT NODE ///////////////////////////////////////////////////////////
    std::vector<double> marker_color_sphere_actual = {1.0,0.0,1.0,1.0};
    disp->displayNode(std::make_shared<pathplan::Node>(current_configuration),id,"pathplan",marker_color_sphere_actual);
    id++;
    // //////////////////////////////////////// ADDING A MOBILE OBSTACLE ////////////////////////////////////////////////////////////////

    bool success;

    //other_paths.at(0)->getConnections().back()->setCost(std::numeric_limits<double>::infinity());
    //other_paths.at(1)->getConnections().back()->setCost(std::numeric_limits<double>::infinity());

    pathplan::Replanner replanner = pathplan::Replanner(current_configuration, current_path, other_paths, solver, metrics, checker_parallel, lb, ub);

    if(verbose_time)
    {
      replanner.setInformedOnlineReplanningVerbose();
      replanner.setPathSwitchVerbose();
    }
    if(verbose_disp_informed)
    {
      replanner.setInformedOnlineReplanningDisp(disp);
    }
    if(verbose_disp_pathSwitch)
    {
      replanner.setPathSwitchDisp(disp);
    }

    ros::Duration(5).sleep();

    double time_repl = time;
    ros::WallTime tic = ros::WallTime::now();
    success =  replanner.informedOnlineReplanning(time_repl);
    ros::WallTime toc = ros::WallTime::now();
    if((toc-tic).toSec()>time) ROS_ERROR("TIME OUT");
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
      return 0;
    }

    disp->nextButton("press NEXT");

    current_path = replanner.getReplannedPath();

    ros::Duration(10).sleep();

    if(mobile_obstacle)
    {
      ros::ServiceClient add_obj=nh.serviceClient<object_loader_msgs::AddObjects>("add_object_to_scene");

      if (!add_obj.waitForExistence(ros::Duration(10)))
      {
        ROS_FATAL("srv not found");
        return 1;
      }

      object_loader_msgs::AddObjects srv;
      object_loader_msgs::Object obj;
      obj.object_type="scatola";

      int obj_conn_pos = current_path->getConnections().size()-2;
      pathplan::ConnectionPtr obj_conn = current_path->getConnections().at(obj_conn_pos);
      pathplan::NodePtr obj_parent = obj_conn->getParent();
      pathplan::NodePtr obj_child = obj_conn->getChild();
      Eigen::VectorXd obj_pos = (obj_child->getConfiguration()+obj_parent->getConfiguration())/2;
      //Eigen::VectorXd obj_pos = obj_parent->getConfiguration()+(obj_child->getConfiguration()-obj_parent->getConfiguration())*0.8;

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
    }

    replanner.setCurrentPath(current_path);
    success =  replanner.informedOnlineReplanning(time_repl);

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

    ROS_INFO("--------------------------------");
  }
  return 0;
}

