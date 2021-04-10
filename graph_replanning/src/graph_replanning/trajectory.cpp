#include "graph_replanning/trajectory.h"

namespace pathplan
{


Trajectory::Trajectory(const PathPtr path,
                       const ros::NodeHandle &nh,
                       const planning_scene::PlanningScenePtr &planning_scene,
                       const std::string &group_name)
{
  trj_ = NULL;
  path_ = path;
  nh_ = nh;
  kinematic_model_ = planning_scene->getRobotModel();
  planning_scene_ = planning_scene;
  group_name_ = group_name;
  moveit_utils_ = std::make_shared<MoveitUtils>(planning_scene,group_name);
}

Trajectory::Trajectory(const ros::NodeHandle &nh,
                       const planning_scene::PlanningScenePtr &planning_scene,
                       const std::string &group_name)
{
  trj_ = NULL;
  path_ = NULL;
  nh_ = nh;
  kinematic_model_ = planning_scene->getRobotModel();
  planning_scene_ = planning_scene;
  group_name_ = group_name;
  moveit_utils_ = std::make_shared<MoveitUtils>(planning_scene,group_name);
}

PathPtr Trajectory::computePath(const TreeSolverPtr& solver, const bool& optimizePath)
{
  CollisionCheckerPtr checker = solver->getChecker();
  SamplerPtr sampler = solver->getSampler();
  MetricsPtr metrics = solver->getMetrics();
  Eigen::VectorXd lb = sampler->getLB();
  Eigen::VectorXd ub = sampler->getUB();
  Eigen::VectorXd start_conf = sampler->getStartConf();
  Eigen::VectorXd goal_conf = sampler->getStopConf();
  NodePtr start_node = std::make_shared<Node>(start_conf);
  NodePtr goal_node = std::make_shared<Node>(goal_conf);

  //pathplan::SamplerPtr sampler = std::make_shared<pathplan::InformedSampler>(start_node->getConfiguration(), goal_node->getConfiguration(), lb, ub);
  //pathplan::BiRRT solver(metrics, checker, sampler);

  //pathplan::MultigoalSolver solver(metrics, checker, sampler);
  solver->config(nh_);
  solver->addStart(start_node);
  solver->addGoal(goal_node);

  pathplan::PathPtr solution;
  if (!solver->solve(solution, 10000))
  {
    ROS_INFO("No solutions found");
    assert(0);
  }

  if(optimizePath)
  {
    pathplan::PathLocalOptimizer path_solver(checker, metrics);
    path_solver.config(nh_);

    solution->setTree(solver->getStartTree());
    path_solver.setPath(solution);
    path_solver.solve(solution);

    sampler->setCost(solution->cost());
    solver->getStartTree()->addBranch(solution->getConnections());

    pathplan::LocalInformedSamplerPtr local_sampler = std::make_shared<pathplan::LocalInformedSampler>(start_conf, goal_conf, lb, ub);

    for (unsigned int isol = 0; isol < solution->getConnections().size() - 1; isol++)
    {
      pathplan::ConnectionPtr conn = solution->getConnections().at(isol);
      local_sampler->addBall(conn->getChild()->getConfiguration(), solution->cost() * 0.1);
    }
    local_sampler->setCost(solution->cost());

    pathplan::RRTStar opt_solver(metrics, checker, local_sampler);
    opt_solver.addStartTree(solver->getStartTree());
    opt_solver.addGoal(goal_node);
    opt_solver.config(nh_);

    std::vector<pathplan::NodePtr> white_list;
    white_list.push_back(goal_node);

    ros::Duration max_time(3);
    ros::Time t0 = ros::Time::now();

    int stall_gen = 0;
    int max_stall_gen = 200;

    std::mt19937 gen;
    std::uniform_int_distribution<> id = std::uniform_int_distribution<>(0, max_stall_gen);

    for (unsigned int idx = 0; idx < 1000; idx++)
    {
      if (ros::Time::now() - t0 > max_time)
        break;

      if (opt_solver.update(solution))
      {
        stall_gen = 0;
        path_solver.setPath(solution);
        solution->setTree(opt_solver.getStartTree());

        local_sampler->setCost(solution->cost());
        sampler->setCost(solution->cost());
        opt_solver.getStartTree()->purgeNodes(sampler, white_list, true);

        local_sampler->clearBalls();
        for (unsigned int isol = 0; isol < solution->getConnections().size() - 1; isol++)
        {
          pathplan::ConnectionPtr conn = solution->getConnections().at(isol);

          local_sampler->addBall(conn->getChild()->getConfiguration(), solution->cost() * 0.1);
        }
      }
      else
      {
        opt_solver.getStartTree()->purgeNodes(sampler, white_list, false);
        stall_gen++;
      }

      if (idx % 10 == 0)

        if (id(gen) < stall_gen)
          opt_solver.setSampler(sampler);
        else
          opt_solver.setSampler(local_sampler);

      if (stall_gen >= max_stall_gen)
        break;
    }

    path_solver.setPath(solution);
    path_solver.solve(solution);
  }

  return solution;
}

robot_trajectory::RobotTrajectoryPtr Trajectory::fromPath2Trj(const trajectory_msgs::JointTrajectoryPoint &pnt)
{
  trajectory_msgs::JointTrajectoryPoint::Ptr pnt_ptr(new trajectory_msgs::JointTrajectoryPoint());
  pnt_ptr->positions = pnt.positions;
  pnt_ptr->velocities = pnt.velocities;
  pnt_ptr->accelerations = pnt.accelerations;
  pnt_ptr->effort = pnt.effort;
  pnt_ptr->time_from_start = pnt.time_from_start;

  return Trajectory::fromPath2Trj(pnt_ptr);
}


robot_trajectory::RobotTrajectoryPtr Trajectory::fromPath2Trj(const trajectory_msgs::JointTrajectoryPointPtr &pnt)
{
  if(path_ == NULL)
  {
    throw std::invalid_argument("Path not assigned");
  }

  std::vector<Eigen::VectorXd> waypoints=path_->getWaypoints();
  std::vector<moveit::core::RobotState> wp_state_vector = moveit_utils_->fromWaypoints2State(waypoints);

  //Definizione della traiettoria, noti i waypoints del path
  trj_ = std::make_shared<robot_trajectory::RobotTrajectory>(kinematic_model_,group_name_);
  for(unsigned int j=0; j<waypoints.size();j++)
  {
    if (j==0 && pnt != NULL)
    {
      wp_state_vector.at(j).setJointGroupPositions(group_name_,pnt->positions);
      wp_state_vector.at(j).setJointGroupVelocities(group_name_,pnt->velocities);
      wp_state_vector.at(j).setJointGroupAccelerations(group_name_,pnt->accelerations);
    }
    trj_->addSuffixWayPoint(wp_state_vector.at(j),0); //time parametrization
  }

  trajectory_processing::IterativeParabolicTimeParameterization iptp;
  //trajectory_processing::IterativeParabolicTimeParameterization iptp(100,0.001);
  //trajectory_processing::TimeOptimalTrajectoryGeneration iptp(0.1,0.1,0.001);
  iptp.computeTimeStamps(*trj_);

  return trj_;
}

double Trajectory::getTimeFromPositionOnTrj(const Eigen::VectorXd &joints_value, double step)
{
  double t = -1.0;
  if(trj_ == NULL)
  {
    ROS_ERROR("Trj not computed");
    throw std::invalid_argument("trj not computed");
  }
  else
  {
    double t1,t2,dist,dist1,dist2,min_dist,t_min,t_max,final_time;
    dist = std::numeric_limits<double>::infinity();
    min_dist = dist;
    t_min = 0;
    t1 = t_min;
    t2 = t1+step;
    final_time = trj_->getDuration();


    moveit_msgs::RobotTrajectory tmp_trj_msg;
    trj_->getRobotTrajectoryMsg(tmp_trj_msg);

    trajectory_processing::SplineInterpolator interpolator;
    interpolator.setTrajectory(tmp_trj_msg);
    interpolator.setSplineOrder(1);

    for(unsigned int i=0;i<2;i++)
    {
      while(t2<=final_time)
      {
        trajectory_msgs::JointTrajectoryPoint pnt1;
        trajectory_msgs::JointTrajectoryPoint pnt2;

        interpolator.interpolate(ros::Duration(t1),pnt1);
        interpolator.interpolate(ros::Duration(t2),pnt2);

        Eigen::VectorXd pos1(pnt1.positions.size());
        Eigen::VectorXd pos2(pnt2.positions.size());

        for(unsigned int i=0; i<pnt1.positions.size();i++)
        {
          pos1[i] = pnt1.positions.at(i);
          pos2[i] = pnt2.positions.at(i);
        }

        dist1 = (joints_value-pos1).norm();
        dist2 = (joints_value-pos2).norm();
        dist = dist1+dist2;

        if(dist<min_dist)
        {
          min_dist = dist;
          t_min = t1;
          t_max = t2;

          if(i==2)
          {
            t = t1+step*(dist1)/(dist1+dist2);
          }
        }

        t1 += step;
        t2 = t1+step;
      }

      step = step/100;
      t1 = t_min;
      t2 = t1+step;
      final_time = t_max;
    }
  }

  return t;
}


}
