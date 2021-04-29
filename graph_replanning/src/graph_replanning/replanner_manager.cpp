#include "graph_replanning/replanner_manager.h"

namespace pathplan
{
ReplannerManager::ReplannerManager(PathPtr &current_path,
                                   std::vector<PathPtr> &other_paths,
                                   ros::NodeHandle &nh)
{
  current_path_ = current_path;
  other_paths_ = other_paths;
  nh_ = nh;

  subscribeTopicsAndServices();
  fromParam();
  attributeInitialization();
}


// /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void ReplannerManager::fromParam()
{
  if(!nh_.getParam("trj_execution_thread_frequency",trj_exec_thread_frequency_)) throw  std::invalid_argument("trj_execution_thread_frequency not set");
  if(!nh_.getParam("collision_checker_thread_frequency",collision_checker_thread_frequency_)) throw  std::invalid_argument("collision_checker_thread_frequency not set");
  if(!nh_.getParam("dt_replan_restricted",dt_replan_restricted_)) throw  std::invalid_argument("dt_replan_restricted not set");
  if(!nh_.getParam("dt_replan_relaxed",dt_replan_relaxed_)) throw  std::invalid_argument("dt_replan_relaxed not set");
  if(!nh_.getParam("group_name",group_name_)) throw  std::invalid_argument("group_name not set");
  if(!nh_.getParam("base_link",base_link_)) throw  std::invalid_argument("base_link not set");
  if(!nh_.getParam("last_link",last_link_)) throw  std::invalid_argument("last_link not set");
  if(!nh_.getParam("spawn_objs",spawn_objs_)) spawn_objs_ = false;
  if(!nh_.getParam("scaling",scaling_from_param_)) scaling_from_param_ = 1.0;
  if(!nh_.getParam("checker_resolution",checker_resol_)) checker_resol_ = 0.05;
  if(!nh_.getParam("display_timing_warning",display_timing_warning_)) display_timing_warning_ = false;
  if(!nh_.getParam("display_replanning_success",display_replanning_success_)) display_replanning_success_ = false;
  if(!nh_.getParam("read_safe_scaling",read_safe_scaling_)) read_safe_scaling_ = false;
}

void ReplannerManager::attributeInitialization()
{
  stop_ = false;
  replanning_ = false;
  replanning_thread_frequency_ = 1/dt_replan_restricted_;
  real_time_ = 0.0;
  t_ = 0.0;
  dt_ = 1/trj_exec_thread_frequency_;
  replan_offset_ = (dt_replan_restricted_-dt_)*K_OFFSET;
  t_replan_ = t_+replan_offset_;
  n_conn_ = 0;
  first_replan_ = true;
  replan_relaxed_ = false;
  path_obstructed_ = false;
  computing_avoiding_path_ = false;
  current_path_changed_ = false;
  pos_closest_obs_from_goal_check_ = -1;
  pos_closest_obs_from_goal_repl_ = pos_closest_obs_from_goal_check_;

  moveit::planning_interface::MoveGroupInterface move_group(group_name_);
  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
  kinematic_state_ = robot_state::RobotStatePtr (new robot_state::RobotState(kinematic_model));
  planning_scn_ = std::make_shared<planning_scene::PlanningScene>(kinematic_model);
  planning_scn_replanning_ = std::make_shared<planning_scene::PlanningScene>(kinematic_model);

  moveit_msgs::GetPlanningScene ps_srv;
  if (!plannning_scene_client_.call(ps_srv))
  {
    ROS_ERROR("call to srv not ok");
  }

  if (!planning_scn_->setPlanningSceneMsg(ps_srv.response.scene))
  {
    ROS_ERROR("unable to update planning scene");
  }

  if (!planning_scn_replanning_->setPlanningSceneMsg(ps_srv.response.scene))
  {
    ROS_ERROR("unable to update planning scene");
  }

  const robot_state::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup(group_name_);
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

  pathplan::MetricsPtr metrics = std::make_shared<pathplan::Metrics>();
  checker_thread_cc_ = std::make_shared<pathplan::ParallelMoveitCollisionChecker>(planning_scn_, group_name_,5, checker_resol_);
  checker_ = std::make_shared<pathplan::ParallelMoveitCollisionChecker>(planning_scn_replanning_, group_name_,5, checker_resol_);
  //checker_thread_cc_ = std::make_shared<pathplan::MoveitCollisionChecker>(planning_scn_, group_name_, checker_resol_);
  //checker_ = std::make_shared<pathplan::MoveitCollisionChecker>(planning_scn_replanning_, group_name_, checker_resol_);

  current_path_->setChecker(checker_);  //To synchronize path checker with the related one to the planning scene that will be used by the replanner, which will be different from the planning scene used by the collision checking thread
  for(const PathPtr &path:other_paths_) path->setChecker(checker_);

  pathplan::SamplerPtr samp = std::make_shared<pathplan::InformedSampler>(current_path_->getWaypoints().front(), current_path_->getWaypoints().back(), lb, ub);
  pathplan::BiRRTPtr solver = std::make_shared<pathplan::BiRRT>(metrics, checker_, samp);
  solver->config(nh_);

  trajectory_ = std::make_shared<pathplan::Trajectory>(current_path_,nh_,planning_scn_replanning_,group_name_);
  robot_trajectory::RobotTrajectoryPtr trj= trajectory_->fromPath2Trj();
  moveit_msgs::RobotTrajectory tmp_trj_msg;
  trj->getRobotTrajectoryMsg(tmp_trj_msg);
  interpolator_.setTrajectory(tmp_trj_msg);
  interpolator_.setSplineOrder(1);

  Eigen::VectorXd point2project(dof);
  interpolator_.interpolate(ros::Duration(t_replan_),pnt_replan_);
  for(unsigned int i=0; i<pnt_replan_.positions.size();i++) point2project[i] = pnt_replan_.positions.at(i);
  configuration_replan_ = current_path_->projectOnClosestConnection(point2project);
  current_configuration_ = current_path_->getWaypoints().front();

  n_conn_ = 0;

  //PathPtr current_path = current_path_->clone();
  //std::vector<PathPtr> other_paths;
  //for(const PathPtr& path:other_paths_)other_paths.push_back(path->clone());

  replanner_ = std::make_shared<pathplan::Replanner>(configuration_replan_, current_path_, other_paths_, solver, metrics, checker_, lb, ub);

  interpolator_.interpolate(ros::Duration(t_),pnt_);
  new_joint_state_.position = pnt_.positions;
  new_joint_state_.velocity = pnt_.velocities;
  new_joint_state_.name = joint_names;
  new_joint_state_.header.frame_id = kinematic_model->getModelFrame();
  new_joint_state_.header.stamp=ros::Time::now();
  //target_pub_.publish(new_joint_state_);
}

void ReplannerManager::subscribeTopicsAndServices()
{
  speed_ovr_sub_ =  std::make_shared<ros_helper::SubscriptionNotifier<std_msgs::Int64>>(nh_,"/speed_ovr",1);
  safe_ovr_1_sub_ =  std::make_shared<ros_helper::SubscriptionNotifier<std_msgs::Int64>>(nh_,"/safe_ovr_1",1);
  safe_ovr_2_sub_ =  std::make_shared<ros_helper::SubscriptionNotifier<std_msgs::Int64>>(nh_,"/safe_ovr_2",1);
  target_pub_= nh_.advertise<sensor_msgs::JointState>("/joint_target",1);
  unscaled_target_pub_= nh_.advertise<sensor_msgs::JointState>("/unscaled_joint_target",1);
  plannning_scene_client_ = nh_.serviceClient<moveit_msgs::GetPlanningScene>("/get_planning_scene");

  if (!plannning_scene_client_.waitForExistence(ros::Duration(10)))
  {
    ROS_ERROR("unable to connect to /get_planning_scene");
  }

  time_pub_= nh_.advertise<std_msgs::Float64>("/time_topic",1);
  current_norm_pub_ = nh_.advertise<std_msgs::Float64>("/current_norm_topic", 1000);
  new_norm_pub_ = nh_.advertise<std_msgs::Float64>("/new_norm_topic", 1000);
  time_replanning_pub_ = nh_.advertise<std_msgs::Float64>("/time_replanning_topic", 1000);
  obs_current_norm_pub_ = nh_.advertise<std_msgs::Float64>("/obs_current_norm_topic", 1000);
  obs_new_norm_pub_ = nh_.advertise<std_msgs::Float64>("/obs_new_norm_topic", 1000);
  obs_time_replanning_pub_ = nh_.advertise<std_msgs::Float64>("/obs_time_replanning_topic", 1000);
  start_log_ = nh_.serviceClient<std_srvs::Empty> ("/start_log");
  stop_log_  = nh_.serviceClient<std_srvs::Empty> ("/stop_log");

  add_obj_ = nh_.serviceClient<object_loader_msgs::AddObjects>("/add_object_to_scene");
  remove_obj_ = nh_.serviceClient<object_loader_msgs::RemoveObjects>("/remove_object_from_scene");
  if (!add_obj_.waitForExistence(ros::Duration(10)))
  {
    ROS_ERROR("unable to connect to /add_object_to_scene");
  }
  if (!remove_obj_.waitForExistence(ros::Duration(10)))
  {
    ROS_ERROR("unable to connect to /remove_object_to_scene");
  }

}

void ReplannerManager::replanningThread()
{
  ros::Rate lp(replanning_thread_frequency_);

  bool replan = true;
  bool success = 0;
  bool old_path_obstructed = path_obstructed_;
  int n_conn_replan = 0;
  double past_abscissa;
  double abscissa;
  double time_informedOnlineRepl;
  std::string string_dt;
  Eigen::VectorXd goal = replanner_->getCurrentPath()->getConnections().back()->getChild()->getConfiguration();
  Eigen::VectorXd point2project(pnt_replan_.positions.size());
  Eigen::VectorXd projection;
  Eigen::VectorXd past_configuration_replan = configuration_replan_;
  PathPtr path2project_on;
  pathplan::PathPtr current_path_copy;
  std::vector<pathplan::PathPtr> other_paths_copy;

  while (!stop_ && ros::ok())
  {
    ros::WallTime tic_tot=ros::WallTime::now();

    trj_mtx_.lock();
    t_replan_ = t_+replan_offset_;

    interpolator_.interpolate(ros::Duration(t_replan_),pnt_replan_);
    for(unsigned int i=0; i<pnt_replan_.positions.size();i++) point2project[i] = pnt_replan_.positions.at(i);
    trj_mtx_.unlock();

    replan =  (point2project-goal).norm()>1e-03;

    if(replan)
    {
      scene_mtx_.lock();
      moveit_msgs::PlanningScene scn;
      planning_scn_->getPlanningSceneMsg(scn); //updated by checker_thread_cc_->setPlanningSceneMsg(ps_srv.response.scene);
      checker_->setPlanningSceneMsg(scn);
      scene_mtx_.unlock();

      replanner_mtx_.lock();
      past_configuration_replan = configuration_replan_;
      path2project_on = replanner_->getCurrentPath();   //CHIEDI SE PER RISPARMIARE TEMPO DEVO FARE UN CLONE O SE GIÀ COSÌ È OK (puntano all stesso oggetto)
      replanner_mtx_.unlock();

      past_abscissa = abscissa;
      projection= path2project_on->projectOnClosestConnectionKeepingCurvilinearAbscissa(point2project,past_configuration_replan,abscissa,past_abscissa,n_conn_replan);

      replanner_mtx_.lock();
      configuration_replan_ = projection;
      replanner_mtx_.unlock();

      paths_mtx_.lock();
      current_path_copy = current_path_->clone();
      other_paths_copy.clear();
      for(const PathPtr path:other_paths_) other_paths_copy.push_back(path->clone());
      paths_mtx_.unlock();

      checker_mtx_.lock();
      pos_closest_obs_from_goal_repl_ = pos_closest_obs_from_goal_check_;

      replanner_mtx_.lock();
      emergency_stop_ = false;  //reset emergency stop (if at the previous iteration it was set, informedOnlineRepl was stopped and a this new iteration can be reset)

      if(current_path_copy->findConnection(configuration_replan_) == NULL)
      {
        trj_mtx_.lock();
        configuration_replan_ = current_configuration_;  //può essere che non venga trovata la repl conf e rimanga ferma per qualche ciclo, intanto la current conf va avanti, la supera e il replanned path partirà da questa e non includera la repl conf
        trj_mtx_.unlock();
      }
      replanner_->setCurrentConf(configuration_replan_);
      replanner_->setCurrentPath(current_path_copy);
      replanner_->setOtherPaths(other_paths_copy);

      if(path_obstructed_)
      {
        replan_offset_ = (dt_replan_restricted_-dt_)*K_OFFSET;
        time_informedOnlineRepl = 0.90*dt_replan_restricted_;
        string_dt = " reduced dt";
        computing_avoiding_path_ = true;
        replan_relaxed_ = true;
      }
      else
      {
        replan_offset_ = (dt_replan_relaxed_-dt_)*K_OFFSET;
        time_informedOnlineRepl = 0.90*dt_replan_relaxed_;
        string_dt = " relaxed dt";
        replan_relaxed_ = false;
      }

      if(old_path_obstructed != path_obstructed_)
      {
        abscissa = 0;
        past_abscissa = 0;
      }
      old_path_obstructed = path_obstructed_;

      replanner_mtx_.unlock();
      checker_mtx_.unlock();

      planning_mtx_.lock();

      replanning_ = true;
      ros::WallTime tic_rep=ros::WallTime::now();
      success =  replanner_->informedOnlineReplanning(time_informedOnlineRepl);
      ros::WallTime toc_rep=ros::WallTime::now();
      replanning_ = false;

      planning_mtx_.unlock();

      if((toc_rep-tic_rep).toSec()>=time_informedOnlineRepl/0.9 && display_timing_warning_) ROS_WARN("replanning duration: %f",(toc_rep-tic_rep).toSec());
      if(display_replanning_success_) ROS_INFO_STREAM("success: "<< success << string_dt << ": " << (toc_rep-tic_rep).toSec());

      ros::WallTime tic_trj;
      ros::WallTime toc_trj;

      if(success && !stop_)
      {
        if(first_replan_)
        {
          first_replan_ = false;

          replanner_mtx_.lock();
          replanner_->addOtherPath(replanner_->getCurrentPath());
          replanner_mtx_.unlock();

          paths_mtx_.lock();
          other_paths_.push_back(current_path_);
          paths_mtx_.unlock();
        }

        std_msgs::Float64 current_norm;
        std_msgs::Float64 new_norm;
        std_msgs::Float64 time_replanning;

        replanner_mtx_.lock();
        trj_mtx_.lock();
        ros::WallTime tic = ros::WallTime::now();
        //ROS_INFO("DENTRO REPL");

        ros::WallTime tic1 = ros::WallTime::now();
        replanner_->startReplannedPathFromNewCurrentConf(current_configuration_);
        ros::WallTime toc1 = ros::WallTime::now();
        replanner_->simplifyReplannedPath(0.01);  //usa var locale e poi assegna a curr path
        ros::WallTime toc2 = ros::WallTime::now();

        double duration1 = (toc1-tic1).toSec();
        double duration2 = (toc2-toc1).toSec();

        current_norm.data = replanner_->getCurrentPath()->getNormFromConf(current_configuration_);
        new_norm.data = replanner_->getReplannedPath()->cost();
        time_replanning.data = (toc_rep-tic_rep).toSec();

        replanner_->setCurrentPath(replanner_->getReplannedPath());

        ros::WallTime tic3 = ros::WallTime::now();
        paths_mtx_.lock();
        current_path_ = replanner_->getReplannedPath();
        current_path_changed_ = true;  //to warn the collision check thread that the current path is changed
        paths_mtx_.unlock();
        ros::WallTime toc3 = ros::WallTime::now();
        double duration3 = (toc3-tic3).toSec();

        tic_trj=ros::WallTime::now();
        moveit_msgs::RobotTrajectory tmp_trj_msg;
        trajectory_->setPath(current_path_);
        robot_trajectory::RobotTrajectoryPtr trj= trajectory_->fromPath2Trj(pnt_);
        trj->getRobotTrajectoryMsg(tmp_trj_msg);
        toc_trj=ros::WallTime::now();
        interpolator_.setTrajectory(tmp_trj_msg);
        interpolator_.setSplineOrder(1);

        t_=0;
        t_replan_=0;
        n_conn_ = 0;
        n_conn_replan = 0;
        abscissa = 0;
        past_abscissa = 0;

        trj_mtx_.unlock();
        ros::WallTime toc = ros::WallTime::now();
        double duration = (toc-tic).toSec();
        //ROS_INFO_STREAM("FUORI REPL: "<<duration<<" dur1: "<<duration1<<" dur2: "<<duration2<<" dur3"<<duration3);
        replanner_mtx_.unlock();

        checker_mtx_.lock();
        replanner_mtx_.lock();
        if(computing_avoiding_path_)
        {
          obs_current_norm_pub_.publish(current_norm);
          obs_new_norm_pub_.publish(new_norm);
          obs_time_replanning_pub_.publish(time_replanning);
        }
        else
        {
          current_norm_pub_.publish(current_norm);
          new_norm_pub_.publish(new_norm);
          time_replanning_pub_.publish(time_replanning);
        }

        path_obstructed_ = false;
        computing_avoiding_path_ = false;

        replanner_mtx_.unlock();
        checker_mtx_.unlock();
      }
      ros::WallTime toc_tot=ros::WallTime::now();
      double duration = (toc_tot-tic_tot).toSec();

      if(display_timing_warning_ && duration>(time_informedOnlineRepl/0.9))
      {
        ROS_WARN("Replanning thread time expired: duration-> %f",duration);
        ROS_WARN("replanning time-> %f",(toc_rep-tic_rep).toSec());
        if(success) ROS_WARN("trj computation time-> %f",(toc_trj-tic_trj).toSec());
      }
    }
    lp.sleep();
  }
}

void ReplannerManager::collisionCheckThread()
{
  ros::Rate lp(collision_checker_thread_frequency_);

  int other_paths_size;
  int pos_closest_obs_from_goal_check_copy;
  Eigen::VectorXd current_configuration_copy;
  PathPtr current_path_copy;
  std::vector<PathPtr> other_paths_copy;
  moveit_msgs::GetPlanningScene ps_srv;

  while (!stop_ && ros::ok())
  {
    ros::WallTime tic_tot = ros::WallTime::now();

    scene_mtx_.lock();
    ros::WallTime tic_pln_call = ros::WallTime::now();
    if (!plannning_scene_client_.call(ps_srv))
    {
      ROS_ERROR("call to srv not ok");
    }
    ros::WallTime toc_pln_call = ros::WallTime::now();
    checker_thread_cc_->setPlanningSceneMsg(ps_srv.response.scene);  //this function update the planning scene
    scene_mtx_.unlock();

    paths_mtx_.lock();
    current_path_copy = current_path_->clone();
    other_paths_copy.clear();
    for(const PathPtr& path: other_paths_) other_paths_copy.push_back(path->clone());
    current_path_changed_ = false;
    paths_mtx_.unlock();

    trj_mtx_.lock();
    current_configuration_copy = current_configuration_;
    trj_mtx_.unlock();

    ros::WallTime tic_check = ros::WallTime::now();
    bool path_obstructed = !(current_path_copy->isValidFromConf(current_configuration_copy,pos_closest_obs_from_goal_check_copy,checker_thread_cc_));
    for(const pathplan::PathPtr& path: other_paths_copy) path->isValid(checker_thread_cc_);
    ros::WallTime toc_check = ros::WallTime::now();

    checker_mtx_.lock();
    pos_closest_obs_from_goal_check_ = pos_closest_obs_from_goal_check_copy;
    checker_mtx_.unlock();

    paths_mtx_.lock();
    if(!current_path_changed_)  //if changed, it is useless checking current_path_copy
    {
      for(unsigned int z=0;z<current_path_->getConnections().size();z++)
      {
        current_path_->getConnections().at(z)->setCost(current_path_copy->getConnections().at(z)->getCost());
      }
    }

    other_paths_size = std::min(other_paths_.size(),other_paths_copy.size());  //it may happen that between other_paths_ cloning and this point a replanning has succeed so the other_path_ vector has 1 path more respect to other_paths_copy, only for this iteration
    for(unsigned int j=0; j<other_paths_size; j++)
    {
      for(unsigned int t=0;t<other_paths_.at(j)->getConnections().size();t++)
      {
        other_paths_.at(j)->getConnections().at(t)->setCost(other_paths_copy.at(j)->getConnections().at(t)->getCost());
      }
    }
    paths_mtx_.unlock();

    checker_mtx_.lock();
    replanner_mtx_.lock();

    if(!computing_avoiding_path_) path_obstructed_ = path_obstructed;
    if(!emergency_stop_ && replanning_ && path_obstructed)
    {
      if(replan_relaxed_ || (!replan_relaxed_ && (pos_closest_obs_from_goal_check_>pos_closest_obs_from_goal_repl_)))  //replanning relaxed or not relaxed but a new closer obstacle is spawned..
      {
        //replanner_->setEmergencyStop();
        emergency_stop_ = true;
        //ROS_WARN("EMERGENCY STOP!");
      }
    }
    replanner_mtx_.unlock();
    checker_mtx_.unlock();

    ros::WallTime toc_tot=ros::WallTime::now();
    double duration = (toc_tot-tic_tot).toSec();

    if(duration>(1/collision_checker_thread_frequency_) && display_timing_warning_)
    {
      ROS_WARN("Collision checking thread time expired: duration-> %f",duration);
      ROS_WARN("t scn call %f, t check %f",(toc_pln_call-tic_pln_call).toSec(),(toc_check-tic_check).toSec());
    }

    lp.sleep();
  }
}

bool ReplannerManager::start()
{
  start_log_.call(srv_log_);

  target_pub_.publish(new_joint_state_);
  unscaled_target_pub_.publish(new_joint_state_);

  ROS_WARN("Launching threads..");
  display_thread_ = std::thread(&ReplannerManager::displayThread, this);  //it must be the first one launched, otherwise the first paths will be not displayed in time
  if(spawn_objs_) spawn_obj_thread_ = std::thread(&ReplannerManager::spawnObjects, this);
  ros::Duration(0.1).sleep();
  replanning_thread_ = std::thread(&ReplannerManager::replanningThread, this);
  col_check_thread_ = std::thread(&ReplannerManager::collisionCheckThread, this);

  bool success = trajectoryExecutionThread();

  if(replanning_thread_.joinable()) replanning_thread_.join();
  if(col_check_thread_.joinable()) col_check_thread_.join();
  if(display_thread_.joinable()) display_thread_.join();
  if(spawn_objs_)
  {
    if(spawn_obj_thread_.joinable()) spawn_obj_thread_.join();
  }

  // BINARY LOGGER SALVA FINO A I-1 ESIMO DATO PUBBLICATO, QUESTO AIUTA A SALVARLI TUTTI
  std_msgs::Float64 fake_data;
  sensor_msgs::JointState joint_fake;
  joint_fake.position = pnt_.positions;
  joint_fake.velocity = pnt_.velocities;
  joint_fake.name = new_joint_state_.name;
  joint_fake.header.frame_id = new_joint_state_.header.frame_id;
  joint_fake.header.stamp=ros::Time::now();
  fake_data.data = 0.0;
  obs_current_norm_pub_.publish(fake_data);
  obs_new_norm_pub_.publish(fake_data);
  obs_time_replanning_pub_.publish(fake_data);
  current_norm_pub_.publish(fake_data);
  new_norm_pub_.publish(fake_data);
  time_replanning_pub_.publish(fake_data);
  time_pub_.publish(fake_data);
  target_pub_.publish(joint_fake);

  stop_log_.call(srv_log_);

  return success;
}

bool ReplannerManager::startWithoutReplanning()
{
  start_log_.call(srv_log_);

  target_pub_.publish(new_joint_state_);
  unscaled_target_pub_.publish(new_joint_state_);

  ROS_WARN("Launching threads..");
  display_thread_ = std::thread(&ReplannerManager::displayThread, this);

  bool success = trajectoryExecutionThread();

  if(display_thread_.joinable()) display_thread_.join();

  // BINARY LOGGER SALVA FINO A I-1 ESIMO DATO PUBBLICATO, QUESTO AIUTA A SALVARLI TUTTI
  std_msgs::Float64 fake_data;
  sensor_msgs::JointState joint_fake;
  joint_fake.position = pnt_.positions;
  joint_fake.velocity = pnt_.velocities;
  joint_fake.name = new_joint_state_.name;
  joint_fake.header.frame_id = new_joint_state_.header.frame_id;
  joint_fake.header.stamp=ros::Time::now();
  fake_data.data = 0.0;
  obs_current_norm_pub_.publish(fake_data);
  current_norm_pub_.publish(fake_data);
  time_pub_.publish(fake_data);
  target_pub_.publish(joint_fake);

  stop_log_.call(srv_log_);

  return success;
}

bool ReplannerManager::trajectoryExecutionThread()
{
  Eigen::VectorXd past_current_configuration = current_configuration_;
  Eigen::VectorXd goal_conf = current_path_->getWaypoints().back();
  PathPtr path2project_on;
  trajectory_msgs::JointTrajectoryPoint pnt;

  ros::WallTime tic;
  ros::WallTime toc;
  double duration1,duration2,duration3,duration4,duration5;

  ros::Rate lp(trj_exec_thread_frequency_);

  while(!stop_ && ros::ok())
  {
    ros::WallTime tic_tot = ros::WallTime::now();
    real_time_ += dt_;

    tic = ros::WallTime::now();
    trj_mtx_.lock();
    toc = ros::WallTime::now();
    duration1 = (toc-tic).toSec();
    tic = ros::WallTime::now();

    double scaling = 1.0;
    if(read_safe_scaling_)
    {
      scaling_ = ((double) speed_ovr_sub_->getData().data/100.0)*((double) safe_ovr_1_sub_->getData().data/100.0)*((double) safe_ovr_2_sub_->getData().data/100);
    }
    else
    {
      scaling_ = scaling_from_param_;
    }

    t_+= scaling_*dt_;

    toc = ros::WallTime::now();
    duration2 = (toc-tic).toSec();
    tic = ros::WallTime::now();

    interpolator_.interpolate(ros::Duration(t_),pnt_);
    pnt = pnt_;
    Eigen::VectorXd point2project(pnt_.positions.size());
    for(unsigned int i=0; i<pnt_.positions.size();i++) point2project[i] = pnt_.positions.at(i);
    trj_mtx_.unlock();
    toc = ros::WallTime::now();
    duration3 = (toc-tic).toSec();

    paths_mtx_.lock();
    path2project_on = current_path_->clone();   //CHIEDI SE BASTA O DEVO FARNE LA COPIA (puntano allo stesso oggetto)  //se no usa replanner_->getCurrentPath()
    paths_mtx_.unlock();

    tic = ros::WallTime::now();
    trj_mtx_.lock();
    past_current_configuration = current_configuration_;
    current_configuration_ = path2project_on->projectOnClosestConnectionKeepingPastPrj(point2project,past_current_configuration,n_conn_);
    trj_mtx_.unlock();
    toc = ros::WallTime::now();
    duration4 = (toc-tic).toSec();

    if((point2project-goal_conf).norm()<1e-3) stop_ = true;

    new_joint_state_.position = pnt.positions;
    new_joint_state_.velocity = pnt.velocities;
    new_joint_state_.header.stamp=ros::Time::now();
    unscaled_target_pub_.publish(new_joint_state_);

    new_joint_state_.position = pnt.positions;
    for(unsigned int i=0;i<pnt.velocities.size(); i++) new_joint_state_.velocity[i] = pnt.velocities[i]*scaling;
    new_joint_state_.header.stamp=ros::Time::now();
    target_pub_.publish(new_joint_state_);

    std_msgs::Float64 time_msg;
    time_msg.data = real_time_;
    time_pub_.publish(time_msg);

    ros::WallTime toc_tot = ros::WallTime::now();
    double duration = (toc_tot-tic_tot).toSec();

    if(duration>(1/trj_exec_thread_frequency_) && display_timing_warning_)
    {
      ROS_WARN("Trj execution thread time expired: duration-> %f",duration);
      ROS_INFO_STREAM("dur1: "<<duration1<<" dur2: "<<duration2<<" dur3: "<<duration3<<" dur4: "<<duration4);
    }

    lp.sleep();
  }

  ROS_ERROR("STOP");
  stop_ = true;

  return 1;
}

bool ReplannerManager::sendRobotStateThread()
{
  /*double send_state_thread_frequency = 500.0;
  double dt = 1/send_state_thread_frequency;
  double t = t_;
  double t_trj_thread = t_;
  Eigen::VectorXd goal_conf = current_path_->getWaypoints().back();
  trajectory_msgs::JointTrajectoryPoint pnt;

  ros::Rate lp(send_state_thread_frequency);
  while(!stop_ && ros::ok())
  {
    real_time_ += dt;

    send_robot_mtx_.lock();
    double scaling = scaling_;

    if(t_trj_thread == t_)
    {
      t+= scaling*dt;
    }
    else          //the trj_exec_thread cycle is changed
    {
      t_trj_thread = t_;
      t = t_;
    }

    //pnt = pnt_;
    //if((current_configuration_-goal_conf).norm()<1e-3) stop_ = true;
    //send_robot_mtx_.unlock();

    fast_interpolator_.interpolate(ros::Duration(t),real_pnt_);
    pnt = real_pnt_;
    Eigen::VectorXd point2project(pnt.positions.size());
    for(unsigned int i=0; i<pnt.positions.size();i++) point2project[i] = pnt.positions.at(i);
    real_pos_ = point2project;
    send_robot_mtx_.unlock();

    if((point2project-goal_conf).norm()<1e-3) stop_ = true;

    new_joint_state_.position = pnt.positions;
    new_joint_state_.velocity = pnt.velocities;
    new_joint_state_.header.stamp=ros::Time::now();
    unscaled_target_pub_.publish(new_joint_state_);

    new_joint_state_.position = pnt.positions;
    for(unsigned int i=0;i<pnt.velocities.size(); i++) new_joint_state_.velocity[i] = pnt.velocities[i]*scaling;
    new_joint_state_.header.stamp=ros::Time::now();
    target_pub_.publish(new_joint_state_);

    std_msgs::Float64 time_msg;
    time_msg.data = real_time_;
    time_pub_.publish(time_msg);

    lp.sleep();
  }

  ROS_ERROR("STOP");
  stop_ = true;

  return 1;*/
  while(ros::ok())
  {
    ros::Duration(0.00001).sleep();
  }
  stop_ = true;
  return 1;
}

void ReplannerManager::displayThread()
{
  pathplan::DisplayPtr disp = std::make_shared<pathplan::Display>(planning_scn_,group_name_,last_link_);
  ros::Duration(0.5).sleep();

  std::vector<double> marker_color;
  std::vector<double> marker_scale;
  std::vector<double> marker_scale_sphere(3,0.02);

  disp->clearMarkers();

  double display_thread_frequency = 0.75*trj_exec_thread_frequency_;
  ros::Rate lp(display_thread_frequency);

  while(!stop_ && ros::ok())
  {
    paths_mtx_.lock();
    pathplan::PathPtr current_path = current_path_->clone();
    std::vector<pathplan::PathPtr> other_paths;
    for(const pathplan::PathPtr path:other_paths_) other_paths.push_back(path->clone());
    paths_mtx_.unlock();

    replanner_mtx_.lock();
    trj_mtx_.lock();
    Eigen::VectorXd current_configuration = current_configuration_;
    Eigen::VectorXd configuration_replan = configuration_replan_;
    trajectory_msgs::JointTrajectoryPoint pnt = pnt_;
    trajectory_msgs::JointTrajectoryPoint pnt_replan = pnt_replan_;
    trj_mtx_.unlock();
    replanner_mtx_.unlock();

    int path_id = 10;
    int node_id = 1000;
    int wp_id = 10000;

    marker_scale = {0.01,0.01,0.01};
    marker_color =  {1.0,1.0,0.0,1.0};
    disp->changeConnectionSize(marker_scale);
    disp->displayPathAndWaypoints(current_path,path_id,wp_id,"pathplan",marker_color);
    disp->defaultConnectionSize();

    for(unsigned int i=0; i<other_paths.size();i++)
    {
      if(i==0) marker_color = {0.0f,0.0f,1.0,1.0};
      if(i==1) marker_color = {1.0,0.0f,0.0f,1.0};
      if(i==2) marker_color = {0.0,0.5,0.5,1.0};
      if(i==3) marker_color = {0.5,0.5,0.0,1.0};
      if(i==4) marker_color = {0.45,0.0,1.0,1.0};
      if(i==5) marker_color = {0.75,0.25,0.0,1.0};
      if(i==6) marker_color = {1.0,0.85,0.35,1.0};
      if(i==7) marker_color = {0.0,1.0,0.5,1.0};
      if(i==8) marker_color = {0.98,0.85,0.87,1.0};
      if(i==9) marker_color = {0.27,0.35,0.27,1.0};

      path_id += 1;
      wp_id += 1000;
      disp->displayPathAndWaypoints(other_paths.at(i),path_id,wp_id,"pathplan",marker_color);
    }

    disp->changeNodeSize(marker_scale_sphere);
    marker_color = {1.0,0.0,1.0,1.0};
    disp->displayNode(std::make_shared<pathplan::Node>(current_configuration),node_id,"pathplan",marker_color);

    Eigen::VectorXd point2project(pnt.positions.size());
    for(unsigned int i=0; i<pnt.positions.size();i++) point2project[i] = pnt.positions.at(i);
    node_id +=1;
    marker_color = {0.0,1.0,0.0,1.0};
    disp->displayNode(std::make_shared<pathplan::Node>(point2project),node_id,"pathplan",marker_color);

    node_id +=1;
    marker_color = {0.0,0.0,0.0,1.0};
    disp->displayNode(std::make_shared<pathplan::Node>(configuration_replan),node_id,"pathplan",marker_color);

    for(unsigned int i=0; i<pnt_replan.positions.size();i++) point2project[i] = pnt_replan.positions.at(i);
    node_id +=1;
    marker_color = {0.5,0.5,0.5,1.0};
    disp->displayNode(std::make_shared<pathplan::Node>(point2project),node_id,"pathplan",marker_color);

    disp->defaultNodeSize();

    lp.sleep();
  }
}

void ReplannerManager::spawnObjects()
{
  object_loader_msgs::AddObjects srv_add_object;
  object_loader_msgs::RemoveObjects srv_remove_object;
  MoveitUtils moveit_utils(planning_scn_,group_name_);

  ros::Rate lp(0.5*trj_exec_thread_frequency_);

  bool object_spawned = false;
  bool second_object_spawned = true;
  bool third_object_spawned = true;
  while (!stop_ && ros::ok())
  {
    // ////////////////////////////////////////////SPAWNING THE OBJECT/////////////////////////////////////////////
    if(real_time_>=1.10 && !third_object_spawned)  //1.5
    {
      third_object_spawned = true;
      object_spawned = false;
    }

    if(real_time_>=1.0 && !second_object_spawned)
    {
      second_object_spawned = true;
      object_spawned = false;
    }

    if(!object_spawned && real_time_>=0.50)
    {
      if (!add_obj_.waitForExistence(ros::Duration(10)))
      {
        ROS_FATAL("srv not found");
      }
      object_loader_msgs::Object obj;
      obj.object_type="scatola";

      int obj_conn_pos;
      int idx_current_conn;

      replanner_mtx_.lock();
      trj_mtx_.lock();
      replanner_->getCurrentPath()->findConnection(current_configuration_,idx_current_conn);
      trj_mtx_.unlock();
      replanner_mtx_.unlock();

      if(third_object_spawned)
      {
        obj_conn_pos = idx_current_conn;

        replanner_mtx_.lock();
        int size = replanner_->getCurrentPath()->getConnections().size();
        replanner_mtx_.unlock();
        obj_conn_pos = size/2;
      }
      else
      {
        replanner_mtx_.lock();
        int size = replanner_->getCurrentPath()->getConnections().size();
        replanner_mtx_.unlock();

        std::srand(time(NULL));
        obj_conn_pos = (rand() % (size-idx_current_conn)) + idx_current_conn;

        if(obj_conn_pos == idx_current_conn) obj_conn_pos +=1;  //ELIMINA

      }
      pathplan::ConnectionPtr obj_conn;
      pathplan::NodePtr obj_parent;
      pathplan::NodePtr obj_child;
      Eigen::VectorXd obj_pos;
      if(obj_conn_pos == idx_current_conn)
      {
        replanner_mtx_.lock();
        trj_mtx_.lock();
        obj_conn = replanner_->getCurrentPath()->getConnections().at(obj_conn_pos);
        obj_child = obj_conn->getChild();
        obj_pos = obj_child->getConfiguration();

        if((obj_pos-configuration_replan_).norm()<0.20 && ((obj_conn_pos+1)<replanner_->getCurrentPath()->getConnections().size()))
        {
          //ROS_WARN("Shifting the object..");
          obj_conn = replanner_->getCurrentPath()->getConnections().at(obj_conn_pos+1);
          obj_parent = obj_conn->getParent();
          obj_child = obj_conn->getChild();

          Eigen::VectorXd conn_vect = obj_child->getConfiguration()-obj_parent->getConfiguration();
          obj_pos = obj_parent->getConfiguration()+0.5*conn_vect;
        }
        trj_mtx_.unlock();
        replanner_mtx_.unlock();
      }
      else
      {
        replanner_mtx_.lock();
        obj_conn = replanner_->getCurrentPath()->getConnections().at(obj_conn_pos);
        obj_parent = obj_conn->getParent();
        obj_child = obj_conn->getChild();
        obj_pos =  (obj_child->getConfiguration() +  obj_parent->getConfiguration())/2;
        //obj_pos =  obj_parent->getConfiguration()+(obj_child->getConfiguration() -  obj_parent->getConfiguration())*0.2;
        replanner_mtx_.unlock();
      }

      moveit::core::RobotState obj_pos_state = moveit_utils.fromWaypoints2State(obj_pos);

      tf::poseEigenToMsg(obj_pos_state.getGlobalLinkTransform(last_link_),obj.pose.pose);

      obj.pose.header.frame_id="world";

      srv_add_object.request.objects.clear();
      srv_add_object.request.objects.push_back(obj);

      scene_mtx_.lock();
      if (!add_obj_.call(srv_add_object))
      {
        ROS_ERROR("call to srv not ok");
      }
      if (!srv_add_object.response.success)
      {
        ROS_ERROR("srv error");
      }
      else
      {
        for (const std::string& str: srv_add_object.response.ids)
        {
          srv_remove_object.request.obj_ids.push_back(str);   //per rimuovere gli oggetti alla fine
        }
      }
      scene_mtx_.unlock();

      object_spawned = true;
      ROS_WARN("OBJECT SPAWNED");
    }

    lp.sleep();
  }

  ros::Duration(5).sleep();

  scene_mtx_.lock();
  if (!remove_obj_.waitForExistence(ros::Duration(10)))
  {
    ROS_FATAL("srv not found");
  }
  if (!remove_obj_.call(srv_remove_object))
  {
    ROS_ERROR("call to srv not ok");
  }
  if (!srv_remove_object.response.success)
  {
    ROS_ERROR("srv error");
  }
  scene_mtx_.unlock();
}

}
