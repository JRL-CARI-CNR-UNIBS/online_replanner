#include "graph_replanning/replanner.h"

namespace pathplan
{
Replanner::Replanner(Eigen::VectorXd& current_configuration,
                     PathPtr& current_path,
                     std::vector<PathPtr>& other_paths,
                     const TreeSolverPtr &solver,
                     const MetricsPtr& metrics,
                     const CollisionCheckerPtr& checker,
                     const Eigen::VectorXd& lb,
                     const Eigen::VectorXd& ub)
{
  current_configuration_ = current_configuration;
  current_path_ = current_path;
  other_paths_ = other_paths;

  assert(solver);

  solver_ = solver;
  metrics_ = metrics;
  checker_ = checker;
  lb_ = lb;
  ub_ = ub;

  admissible_other_paths_ = other_paths_;
  available_time_ =  std::numeric_limits<double>::infinity();
  pathSwitch_cycle_time_mean_ = std::numeric_limits<double>::infinity();
  //informedOnlineReplanning_cycle_time_mean_ = std::numeric_limits<double>::infinity();
  time_percentage_variability_ = 0.7;

  success_ = false;
  emergency_stop_ = false;
  an_obstacle_ = false;

  informedOnlineReplanning_disp_ = false;
  pathSwitch_disp_ = false;

  informedOnlineReplanning_verbose_ = false;
  pathSwitch_verbose_ = false;

}

bool Replanner::checkPathValidity(const CollisionCheckerPtr &this_checker)
{
  CollisionCheckerPtr checker = checker_;
  if(this_checker != NULL) checker = this_checker;

  bool validity = true;

  if(!current_path_->isValidFromConf(current_configuration_,checker)) validity = false;

  for(const PathPtr& path: other_paths_)
  {
    if(!path->isValid(checker)) validity = false;
  }

  return validity;
}

bool Replanner::simplifyReplannedPath(const double& distance)
{
  ros::WallTime tic = ros::WallTime::now();
  int count = 0;

  bool simplify = false;
  bool simplified = false;
  do
  {
    simplify = replanned_path_->simplify(distance);
    if(simplify)
    {
      count +=1;
      simplified = true;
    }
  }
  while(simplify);

  ros::WallTime toc = ros::WallTime::now();
  //ROS_INFO_STREAM("n simpl: "<<count<<" time: "<<(toc-tic).toSec());

  return simplified;
}

void Replanner::startReplannedPathFromNewCurrentConf(Eigen::VectorXd &configuration)
{

  std::vector<pathplan::ConnectionPtr> path_connections;
  std::vector<pathplan::ConnectionPtr> connections;

  pathplan::PathPtr path = std::make_shared<pathplan::Path>(replanned_path_->getConnections(),metrics_,checker_);
  pathplan::NodePtr current_node = std::make_shared<pathplan::Node>(configuration);
  pathplan::NodePtr path_start = path->getConnections().front()->getParent();

  for(const Eigen::VectorXd wp:path->getWaypoints())
  {
    if(wp == configuration)
    {
      if(wp == path->getWaypoints().back()) assert(0);
      replanned_path_ = path->getSubpathFromNode(current_node);
      return;
    }
  }

  ROS_INFO("QUI0");
  int idx_current_conf, idx_path_start;
  double abscissa_current_conf = current_path_->curvilinearAbscissaOfPoint(configuration,idx_current_conf);
  ROS_INFO("QUI1");

  double abscissa_path_start = current_path_->curvilinearAbscissaOfPoint(path_start->getConfiguration(),idx_path_start);
  ROS_INFO("QUI2");

  if(abscissa_current_conf == abscissa_path_start)
  {
    ROS_INFO("QUI22");
    return;  //the start of the replanned path is the current configuration
  }
  else if(abscissa_current_conf < abscissa_path_start)  //the replanned path starts from a position after the current one
  {
    ROS_INFO("QUI23");
    if(idx_current_conf == idx_path_start)
    {
      //Directly connect the current configuration with the start of the replanned path
      ConnectionPtr conn = std::make_shared<Connection>(current_node, path_start);
      double cost_conn = metrics_->cost(configuration,path_start->getConfiguration());
      conn->setCost(cost_conn);
      conn->add();

      connections.push_back(conn);
    }
    if(idx_current_conf < idx_path_start)
    {
      ROS_INFO("QUI3");

      NodePtr child = current_path_->getConnections().at(idx_current_conf)->getChild();
      ROS_INFO("QUI4");

      if(child->getConfiguration() != configuration)
      {
        ConnectionPtr conn = std::make_shared<Connection>(current_node, child);
        double cost_conn = metrics_->cost(configuration,child->getConfiguration());
        conn->setCost(cost_conn);
        conn->add();

        connections.push_back(conn);
      }

      //Adding the connections between the two configurations
      for(unsigned int z = idx_current_conf+1; z<idx_path_start; z++) connections.push_back(current_path_->getConnections().at(z));

      if(path_start->getConfiguration() == current_path_->getConnections().at(idx_path_start)->getChild()->getConfiguration())
      {
        connections.push_back(current_path_->getConnections().at(idx_path_start));
      }
      else
      {
        NodePtr parent = current_path_->getConnections().at(idx_path_start)->getParent();
        if(parent->getConfiguration() != path_start->getConfiguration())
        {
          ConnectionPtr conn = std::make_shared<Connection>(parent,path_start);
          double cost_conn = metrics_->cost(parent->getConfiguration(),path_start->getConfiguration());
          conn->setCost(cost_conn);
          conn->add();

          connections.push_back(conn);
        }
      }
    }

    for(const ConnectionPtr& replanned_connection:path->getConnections()) connections.push_back(replanned_connection);
    path->setConnections(connections);
    replanned_path_ = path;

    return;
  }
  else //the replanned path starts from a position before the current configuration
  {
    ROS_INFO("QUI5");

    pathplan::NodePtr node;

    int idx_conn;
    if(path->findConnection(configuration,idx_conn) != NULL)
    {
      ROS_INFO("QUI6");
      node = path->getConnections().at(idx_conn)->getChild();

      if((path->getConnections().size()-1) > idx_conn)
      {
        for(unsigned int i=idx_conn+1; i<path->getConnections().size();i++) path_connections.push_back(path->getConnections().at(i));
      }
      for(unsigned int i=0; i<=idx_conn; i++) path->getConnections().at(i)->remove();

      if(current_node->getConfiguration() != node->getConfiguration())
      {
        pathplan::ConnectionPtr conn = std::make_shared<pathplan::Connection>(current_node,node);
        double cost = metrics_->cost(current_node,node);
        conn->setCost(cost);
        conn->add();

        connections.push_back(conn);
      }

      ROS_INFO("QUI7");
      if((path->getConnections().size()-1) > idx_conn) connections.insert(connections.end(),path_connections.begin(),path_connections.end());
      ROS_INFO("QUI8");

      path->setConnections(connections);
    }
    else
    {
      ROS_INFO("QUA");

      ConnectionPtr conn = current_path_->getConnections().at(idx_path_start);

      ROS_INFO("QUA1");

      int idx = idx_path_start;
      if(path_start->getConfiguration() == current_path_->getConnections().at(idx_path_start)->getChild()->getConfiguration()) idx = idx_path_start + 1;

      ROS_INFO("QUA2");

      int j = 0;
      int j_save = -2;
      for(unsigned int i=idx; i<current_path_->getConnections().size();i++)
      {
        if(path->getConnections().at(j)->getChild()->getConfiguration() == current_path_->getConnections().at(i)->getChild()->getConfiguration())
        {
          node = path->getConnections().at(j)->getChild();
          j_save = j;
        }
        else break;
        j+=1;
      }

      ROS_INFO("QUA3");

      bool add_conn = false;
      if(j_save != -2)
      {
        ROS_INFO("QUA4");
        for(unsigned int i=j_save+1; i<path->getConnections().size();i++) path_connections.push_back(path->getConnections().at(i));

        ROS_INFO("QUA5");
        for(unsigned int i=0; i<=j_save; i++) path->getConnections().at(i)->remove();
      }
      else
      {
        ROS_INFO("QUA6");
        if((conn->getParent()->getConfiguration() == path_start->getConfiguration()) || (conn->getChild()->getConfiguration() == path_start->getConfiguration()))
        {
          ROS_INFO("QUA7");
          node = path_start;
          path_connections = path->getConnections();
        }
        else
        {
          ROS_INFO("QUA8");
          if(idx_current_conf == idx_path_start) node = current_node;
          else node = current_path_->getConnections().at(idx_path_start)->getChild();
          path_connections = path->getConnections();
          add_conn = true;
        }
        ROS_INFO("QUA9");
      }

      bool connected = false;
      if(add_conn && idx_path_start == idx_current_conf) connected = true;  //if the current conf is near after the pat start, you only have to connect these two confs

      int t = idx_current_conf;
      pathplan::NodePtr child;
      pathplan::NodePtr parent;

      while(!connected)
      {
        if(t == idx_current_conf) child = current_node;
        else child = std::make_shared<pathplan::Node>(current_path_->getConnections().at(t)->getChild()->getConfiguration());

        parent = std::make_shared<pathplan::Node>(current_path_->getConnections().at(t)->getParent()->getConfiguration());

        if(child->getConfiguration() != node->getConfiguration())
        {
          pathplan::ConnectionPtr conn = std::make_shared<pathplan::Connection>(child,parent); //you re moving backwards
          double cost = metrics_->cost(child,parent);
          conn->setCost(cost);
          conn->add();

          connections.push_back(conn);
        }
        else connected = true;

        t-=1;
      }

      if(add_conn)
      {
        pathplan::ConnectionPtr conn = std::make_shared<pathplan::Connection>(node,path_start); //you are moving backwards
        double cost = metrics_->cost(node,path_start);
        conn->setCost(cost);
        conn->add();

        connections.push_back(conn);
      }
      connections.insert(connections.end(),path_connections.begin(),path_connections.end());
      path->setConnections(connections);
    }

    replanned_path_ = path;
  }
}

bool Replanner::connect2goal(const PathPtr& current_path, const NodePtr& node, PathPtr &new_path)
{
  std::vector<double> marker_color = {1.0,1.0,0.0,1.0};
  std::vector<double> marker_scale(3,0.01);

  ros::WallTime tic=ros::WallTime::now();
  ros::WallTime toc;
  if(pathSwitch_disp_) pathSwitch_max_time_ = std::numeric_limits<double>::infinity();
  else pathSwitch_max_time_ = available_time_;
  double time = pathSwitch_max_time_;
  std::vector<double> time_vector;

  if(pathSwitch_verbose_) ROS_INFO_STREAM("Connect2Goal cycle starting mean: "<<pathSwitch_cycle_time_mean_);
  if(pathSwitch_cycle_time_mean_ != std::numeric_limits<double>::infinity()) time_vector.push_back(pathSwitch_cycle_time_mean_);

  if(!pathSwitch_disp_)
  {
    if(pathSwitch_cycle_time_mean_ == std::numeric_limits<double>::infinity() || an_obstacle_)
    {
      if(time<=0.0) return false;
    }
    else
    {
      if(time<time_percentage_variability_*pathSwitch_cycle_time_mean_) return false;
    }
  }

  bool success = 0;

  Eigen::VectorXd first_free_point;
  //VERIFICA CHE LA FUNZIONE SI CHIAMI ANCORA COSÌ
  checker_->checkPath(current_path->getConnections().back()->getChild()->getConfiguration(),current_path->getConnections().back()->getParent()->getConfiguration(),first_free_point); //with inverted child-parent the free conf is the first one after the obstacle
  if(first_free_point.size() == 0)
  {
    ROS_ERROR("Goal obstructed");
    return false;
  }
  NodePtr first_free_node = std::make_shared<Node>(first_free_point);

  bool before_goal = false;
  if(first_free_point != current_path->getConnections().back()->getChild()->getConfiguration()) before_goal = true;

  PathPtr subpath1 = current_path->getSubpathFromNode(node);
  std::vector<ConnectionPtr> subpath2_conn;
  double subpath2_cost = 0;

  if(before_goal)
  {
    NodePtr goal = current_path->getConnections().back()->getChild();

    ConnectionPtr conn = std::make_shared<Connection>(first_free_node,goal);
    subpath2_cost = metrics_->cost(first_free_node,goal);
    conn->setCost(subpath2_cost);
    conn->add();

    subpath2_conn.push_back(conn);
  }

  double diff_subpath_cost = subpath1->cost()-subpath2_cost;
  double distance_path_node = (node->getConfiguration()-first_free_point).norm();

  if(diff_subpath_cost > 0 && distance_path_node < diff_subpath_cost)   //always verified..(?)
  {
    PathPtr connecting_path;

    NodePtr node_fake = std::make_shared<Node>(node->getConfiguration());
    NodePtr free_node_fake = std::make_shared<Node>(first_free_point);

    bool directly_connected = false;
    bool solver_has_solved = computeConnectingPath(node_fake, free_node_fake, diff_subpath_cost, tic, tic, connecting_path,directly_connected); //tyc_cycle = tic

    if (solver_has_solved)
    {
      if(!directly_connected)
      {
        //double opt_time = maxSolverTime(tic,tic);
        //optimizePath(connecting_path,opt_time);
      }

      double new_solution_cost = connecting_path->cost()+subpath2_cost;
      if(pathSwitch_verbose_ || pathSwitch_disp_) ROS_INFO_STREAM("solution cost: "<<new_solution_cost);

      if(new_solution_cost<subpath1->cost())
      {
        std::vector<ConnectionPtr>  connecting_path_conn = connecting_path->getConnections();
        new_path = concatConnectingPathAndSubpath2(connecting_path_conn,subpath2_conn,node,first_free_node);

        success = 1;
        an_obstacle_ = false;
      }
      else
      {
        if(pathSwitch_verbose_ || pathSwitch_disp_) ROS_INFO("It is not a better solution");
      }

      toc = ros::WallTime::now();
      if(pathSwitch_verbose_) ROS_INFO_STREAM("SOLVED->cycle time: "<<(toc-tic).toSec());

      if(!directly_connected)  //not directly connected, usually it is very fast and it would alterate the mean value
      {
        time_vector.push_back((toc-tic).toSec());
        pathSwitch_cycle_time_mean_ = std::accumulate(time_vector.begin(), time_vector.end(),0.0)/((double) time_vector.size());
        if(pathSwitch_verbose_) ROS_INFO_STREAM("cycle time mean updated: "<<pathSwitch_cycle_time_mean_);
      }
      else
      {
        if(pathSwitch_verbose_) ROS_INFO_STREAM("cycle time mean not updated");
      }
    }
    else
    {
      if(!an_obstacle_ && pathSwitch_cycle_time_mean_ != std::numeric_limits<double>::infinity())
      {
        pathSwitch_cycle_time_mean_ = 1.2*pathSwitch_cycle_time_mean_;
        if(pathSwitch_verbose_) ROS_INFO_STREAM("cycle time mean increased of 20%: "<<pathSwitch_cycle_time_mean_);
      }

      if(pathSwitch_disp_) ROS_INFO("Not solved");
    }

    //Regardless if you have found a solution or not, delete the fake nodes
    node_fake->disconnect();
    free_node_fake->disconnect();
  }
  else
  {
    if(pathSwitch_verbose_ || pathSwitch_disp_) ROS_INFO_STREAM("It would not be a better solution");
  }


  if(pathSwitch_verbose_) ROS_INFO_STREAM("cycle time mean: "<<pathSwitch_cycle_time_mean_);

  if(pathSwitch_verbose_ || pathSwitch_disp_)
  {
    if(pathSwitch_verbose_) ROS_INFO_STREAM("Connect2Goal duration: "<<(ros::WallTime::now()-tic).toSec());
    if(success)
    {
      ROS_INFO_STREAM("Connect2Goal has found a solution with cost: " << new_path->cost());
    }
    else
    {
      ROS_INFO_STREAM("Connect2Goal has NOT found a solution");
    }
  }

  /*///////////////////////////////Visualization//////////////////////////////////*/
  if(pathSwitch_disp_)
  {
    disp_->clearMarker(pathSwitch_path_id_);

    disp_->changeConnectionSize(marker_scale);
    pathSwitch_path_id_ = disp_->displayPath(new_path,"pathplan",marker_color);
    disp_->defaultConnectionSize();
    disp_->nextButton("Press \"next\" to execute the next step of Connect2Goal");
  }
  /*/////////////////////////////////////////////////////////////////////////////*/

  return success;
}

std::vector<PathPtr>  Replanner::addAdmissibleCurrentPath(const int &idx_current_conn, PathPtr& admissible_current_path)
{
  std::vector<PathPtr> reset_other_paths;
  admissible_current_path = NULL;

  if(current_path_->getCostFromConf(current_configuration_) == std::numeric_limits<double>::infinity())
  {
    if(current_path_->getConnections().back()->getCost() == std::numeric_limits<double>::infinity()) return other_paths_;
    else
    {
      int z = current_path_->getConnections().size()-2;  //penultimate connection (last connection is at end-1)
      ConnectionPtr conn;

      while(z>=idx_current_conn) //to find the savable part of current_path, the subpath after the connection obstruced by the obstacle
      {
        conn = current_path_->getConnections().at(z);
        if(conn->getCost() == std::numeric_limits<double>::infinity())
        {
          admissible_current_path = current_path_->getSubpathFromNode(conn->getChild());
          break;
        }
        z -= 1;
      }
    }
    // adding the savable subpath of the current_path to the set of available paths
    if(admissible_current_path != NULL) reset_other_paths.push_back(admissible_current_path);
    reset_other_paths.insert(reset_other_paths.end(),other_paths_.begin(),other_paths_.end());
    return reset_other_paths;
  }
  else
  {
    if(idx_current_conn<current_path_->getConnections().size()-1)
    {
      admissible_current_path = current_path_->getSubpathFromNode(current_path_->getConnections().at(idx_current_conn)->getChild());
      reset_other_paths.push_back(admissible_current_path);
      reset_other_paths.insert(reset_other_paths.end(),other_paths_.begin(),other_paths_.end());
    }
    else reset_other_paths = other_paths_;

    return reset_other_paths;
  }
}

std::vector<PathPtr> Replanner::sortPathsOnDistance(const NodePtr& node)
{
  std::multimap<double,PathPtr> path_map;
  for(unsigned int j = 0; j< admissible_other_paths_.size(); j++)
  {
    double distance;
    admissible_other_paths_.at(j)->findCloserNode(node,distance);
    path_map.insert(std::pair<double,PathPtr>(distance,admissible_other_paths_.at(j)));
  }

  std::vector<PathPtr> ordered_paths;
  for(const std::pair<double,PathPtr>& p: path_map)
  {
    ordered_paths.push_back(p.second);
  }

  return ordered_paths;
}

std::vector<NodePtr> Replanner::nodes2connect2(const PathPtr& path, const NodePtr& this_node)
{
  std::vector<NodePtr> path_node_vector;

  std::vector<ConnectionPtr> path_conn = path->getConnections();
  /*if(path_conn.size()>1)
  {*/
  std::multimap<double,NodePtr> node_map;

  double distance;
  for(unsigned int i=0; i<path_conn.size();i++)
  {
    distance = (this_node->getConfiguration()-path_conn.at(i)->getParent()->getConfiguration()).norm();
    node_map.insert(std::pair<double,NodePtr>(distance,path_conn.at(i)->getParent()));
  }

  distance = (this_node->getConfiguration()-path_conn.back()->getChild()->getConfiguration()).norm();
  node_map.insert(std::pair<double,NodePtr>(distance,path_conn.back()->getChild()));

  for(const std::pair<double,NodePtr>& p: node_map)
  {
    path_node_vector.push_back(p.second);
  }
  /*}
  else
  {
    path_node_vector.push_back(path_conn.at(0)->getParent());
  }*/

  return path_node_vector;
}

double Replanner::maxSolverTime(const ros::WallTime& tic, const ros::WallTime& tic_cycle)
{
  double time;
  ros::WallTime toc = ros::WallTime::now();

  if(pathSwitch_disp_) time = std::numeric_limits<double>::infinity();
  else if(pathSwitch_cycle_time_mean_ == std::numeric_limits<double>::infinity() || an_obstacle_) time = pathSwitch_max_time_-(toc-tic).toSec(); //when there is an obstacle or when the cycle time mean has not been defined yets
  else time = (2-time_percentage_variability_)*pathSwitch_cycle_time_mean_-(toc-tic_cycle).toSec();

  return time;
}

PathPtr Replanner::concatConnectingPathAndSubpath2(const std::vector<ConnectionPtr>& connecting_path_conn,
                                                   const std::vector<ConnectionPtr>& subpath2,
                                                   const NodePtr& path1_node,
                                                   const NodePtr& path2_node)
{
  std::vector<ConnectionPtr> new_connecting_path_conn;

  if(connecting_path_conn.size()>1)
  {
    NodePtr node1 = connecting_path_conn.front()->getChild();
    NodePtr node2 = connecting_path_conn.back()->getParent();

    double conn1_cost = metrics_->cost(path1_node,node1);
    double conn2_cost = metrics_->cost(node2,path2_node);

    ConnectionPtr conn1 = std::make_shared<Connection>(path1_node,node1);
    ConnectionPtr conn2 = std::make_shared<Connection>(node2,path2_node);

    conn1->setCost(conn1_cost);
    conn2->setCost(conn2_cost);

    conn1->add();
    conn2->add();

    new_connecting_path_conn.push_back(conn1);
    if(connecting_path_conn.size()>2) new_connecting_path_conn.insert(new_connecting_path_conn.end(), connecting_path_conn.begin()+1, connecting_path_conn.end()-1);
    new_connecting_path_conn.push_back(conn2);

    connecting_path_conn.front()->remove();
    connecting_path_conn.back()->remove();
  }
  else
  {
    ConnectionPtr conn1 = std::make_shared<Connection>(path1_node,path2_node);
    double conn1_cost =  metrics_->cost(path1_node,path2_node);
    conn1->setCost(conn1_cost);
    conn1->add();

    new_connecting_path_conn.push_back(conn1);
    connecting_path_conn.front()->remove();
  }

  if(!subpath2.empty())
  {
    new_connecting_path_conn.insert(new_connecting_path_conn.end(),subpath2.begin(),subpath2.end());
  }

  return std::make_shared<Path>(new_connecting_path_conn, metrics_, checker_);
}

bool Replanner::computeConnectingPath(const NodePtr& path1_node_fake, const NodePtr& path2_node_fake, const double& diff_subpath_cost, const ros::WallTime& tic, const ros::WallTime& tic_cycle, PathPtr& connecting_path, bool& directly_connected)
{
  SamplerPtr sampler = std::make_shared<InformedSampler>(path1_node_fake->getConfiguration(), path2_node_fake->getConfiguration(), lb_, ub_,diff_subpath_cost); //the ellipsoide determined by diff_subpath_cost is used. Outside from this elipsoid, the nodes create a connecting_path not convenient

  solver_->setSampler(sampler);
  solver_->resetProblem();
  solver_->addStart(path1_node_fake);

  double solver_time = maxSolverTime(tic,tic_cycle);

  if(pathSwitch_verbose_) ROS_INFO_STREAM("Searching for a direct connection...max time: "<<solver_time);

  ros::WallTime tic_directConnection = ros::WallTime::now();
  solver_->addGoal(path2_node_fake,solver_time);
  ros::WallTime toc_directConnection = ros::WallTime::now();

  directly_connected = solver_->solved();

  if(pathSwitch_verbose_)
  {
    if(directly_connected) ROS_INFO("Solved->direct connection found");
    else ROS_INFO("Direct connection NOT found");

    ROS_INFO_STREAM("Time to directly connect: "<<(toc_directConnection-tic_directConnection).toSec());
  }

  bool solver_has_solved;
  ros::WallTime tic_solver;
  ros::WallTime toc_solver;

  if(directly_connected)
  {
    connecting_path = solver_->getSolution();
    solver_has_solved = true;
  }
  else
  {
    solver_time = maxSolverTime(tic,tic_cycle);

    if(pathSwitch_verbose_) ROS_INFO_STREAM("solving...max time: "<<solver_time);

    tic_solver = ros::WallTime::now();
    solver_has_solved = solver_->solve(connecting_path,10000,solver_time);
    toc_solver = ros::WallTime::now();

    if(pathSwitch_verbose_)
    {
      if(solver_has_solved) ROS_INFO_STREAM("solved in time: "<<(toc_solver-tic_solver).toSec());
      else ROS_INFO_STREAM("not solved, time: "<<(toc_solver-tic_solver).toSec());
    }
  }

  return solver_has_solved;
}

void Replanner::optimizePath(PathPtr& path, const double& max_time)
{
  //PathLocalOptimizer path_solver(checker_, metrics_);
  //path_solver.setPath(connecting_path);
  //double opt_time = maxSolverTime(tic,tic_cycle);

  //  ros::WallTime tic_simplify = ros::WallTime::now();
  //  if(path->getConnections().at(0)->norm()<0.1)
  //  {
  //    //simplify solo della prima connessione
  //  }

  //  ros::WallTime toc_simplify = ros::WallTime::now();

  //  if(max_time<=0.0) return;
    ros::WallTime tic_opt = ros::WallTime::now();
    path->warp(0.1,max_time);
    ros::WallTime toc_opt = ros::WallTime::now();

  //  if(pathSwitch_verbose_)
  //  {
  //    ROS_INFO_STREAM("max opt time: "<<max_time<<" used time: "<<(toc_opt-tic_opt).toSec());
  //  }
}

bool Replanner::pathSwitch(const PathPtr &current_path,
                           const NodePtr &node,
                           PathPtr &new_path,
                           PathPtr &subpath_from_path2,
                           int &connected2path_number)
{
  ros::WallTime tic=ros::WallTime::now();
  ros::WallTime toc, tic_cycle, toc_cycle;
  if(pathSwitch_disp_) pathSwitch_max_time_ = std::numeric_limits<double>::infinity();
  else pathSwitch_max_time_ = available_time_;
  double time = pathSwitch_max_time_;
  std::vector<double> time_vector;
  bool time_out = false;

  if(pathSwitch_verbose_) ROS_INFO_STREAM("PathSwitch cycle starting mean: "<<pathSwitch_cycle_time_mean_);
  if(pathSwitch_cycle_time_mean_ != std::numeric_limits<double>::infinity()) time_vector.push_back(pathSwitch_cycle_time_mean_);

  if(!pathSwitch_disp_)
  {
    if(pathSwitch_cycle_time_mean_ == std::numeric_limits<double>::infinity() || an_obstacle_)
    {
      if(time<=0.0) return false;
    }
    else
    {
      if(time<time_percentage_variability_*pathSwitch_cycle_time_mean_) return false;
    }
  }

  int new_node_id;
  std::vector<int> node_id_vector;
  std::vector<double> marker_scale_sphere(3,0.025);
  std::vector<double> marker_color_sphere = {0.5,0.5,0.5,1.0};

  std::vector<double> marker_color = {1.0,0.5,0.0,1.0};
  std::vector<double> marker_scale(3,0.01);

  bool success = false;

  // Identifying the subpath of current_path starting from node
  NodePtr path1_node = node;
  PathPtr path1_node2goal;

  path1_node2goal = current_path->getSubpathFromNode(path1_node);
  double subpath1_cost = path1_node2goal->cost();
  double candidate_solution_cost = subpath1_cost;

  std::vector<PathPtr> ordered_paths = sortPathsOnDistance(path1_node);
  for(const PathPtr& path2:ordered_paths)
  {
    std::vector<NodePtr> path2_node_vector = nodes2connect2(path2,path1_node);
    for(const NodePtr& path2_node : path2_node_vector)
    {
      if(!emergency_stop_)
      {
        tic_cycle = ros::WallTime::now();

        PathPtr path2_node2goal = NULL;
        std::vector<ConnectionPtr> subpath2_conn;
        double subpath2_cost = 0.0;
        if(path2_node->getConfiguration() != current_path->getConnections().back()->getChild()->getConfiguration())
        {
          path2_node2goal = path2->getSubpathFromNode(path2_node);
          subpath2_conn = path2_node2goal->getConnections();
          subpath2_cost = path2_node2goal->cost();
        }

        double diff_subpath_cost = candidate_solution_cost - subpath2_cost; //it is the maximum cost to make the connecting_path convenient
        double distance_path_node = (path1_node->getConfiguration()-path2_node->getConfiguration()).norm(); //the Euclidean distance is the minimum cost that the connecting_path can have

        if(pathSwitch_disp_)
        {
          int node_n;
          for(unsigned int r=0; r<path2->getWaypoints().size();r++)
          {
            if(path2_node->getConfiguration() == path2->getWaypoints().at(r))
            {
              node_n = r;
            }
          }
          ROS_INFO_STREAM("candidate_solution_cost: "<<candidate_solution_cost<<" subpath2_cost: "<<subpath2_cost);
          ROS_INFO_STREAM("node n: " <<node_n<< " diff_subpath_cost: "<< diff_subpath_cost<<" distance: " << distance_path_node);
        }

        if(diff_subpath_cost > 0 && distance_path_node < diff_subpath_cost) //if the Euclidean distance between the two nodes is bigger than the maximum cost for the connecting_path to be convenient, it is useless to calculate a connecting_path because it surely will not be convenient
        {
          PathPtr connecting_path;

          NodePtr path1_node_fake = std::make_shared<Node>(path1_node->getConfiguration());
          NodePtr path2_node_fake = std::make_shared<Node>(path2_node->getConfiguration());

          bool directly_connected = false;
          bool solver_has_solved = computeConnectingPath(path1_node_fake, path2_node_fake, diff_subpath_cost, tic, tic_cycle, connecting_path, directly_connected);

          if(solver_has_solved)
          {
            if(!directly_connected)
            {
              //double opt_time = maxSolverTime(tic,tic_cycle);
              //optimizePath(connecting_path,opt_time);
            }

            double new_solution_cost = subpath2_cost + connecting_path->cost();
            if(pathSwitch_verbose_ || pathSwitch_disp_) ROS_INFO_STREAM("solution cost: "<<new_solution_cost);

            if(new_solution_cost<candidate_solution_cost)
            {
              std::vector<ConnectionPtr> connecting_path_conn = connecting_path->getConnections();
              new_path = concatConnectingPathAndSubpath2(connecting_path_conn,subpath2_conn,path1_node,path2_node);
              candidate_solution_cost = new_path->cost();

              success = 1;
              an_obstacle_ = false;
              subpath_from_path2 = path2_node2goal;
              for(unsigned int z=0; z<admissible_other_paths_.size();z++)
              {
                if(path2 == admissible_other_paths_.at(z)) connected2path_number = z;
              }

              /*//////////Visualization//////////////////*/
              if(pathSwitch_disp_)
              {
                disp_->clearMarker(pathSwitch_path_id_);
                disp_->changeConnectionSize(marker_scale);
                pathSwitch_path_id_ = disp_->displayPath(new_path,"pathplan",marker_color);
                disp_->defaultConnectionSize();
              }
              /*//////////////////////////////////////*/
            }
            else
            {
              if(pathSwitch_verbose_ || pathSwitch_disp_) ROS_INFO_STREAM("It is not a better solution");

              /*//////////Visualization//////////////////*/
              if(pathSwitch_disp_)
              {
                disp_->changeNodeSize(marker_scale_sphere);
                new_node_id = disp_->displayNode(path2_node,"pathplan",marker_color_sphere);
                disp_->defaultNodeSize();

                node_id_vector.push_back(new_node_id);
              }
              /*//////////////////////////////////////*/
            }

            toc_cycle = ros::WallTime::now();
            if(pathSwitch_verbose_) ROS_INFO_STREAM("SOLVED->cycle time: "<<(toc_cycle-tic_cycle).toSec());

            if(!directly_connected)  //not directly connected, usually it is very fast and it would alterate the mean value
            {
              time_vector.push_back((toc_cycle-tic_cycle).toSec());
              pathSwitch_cycle_time_mean_ = std::accumulate(time_vector.begin(), time_vector.end(),0.0)/((double) time_vector.size());
              if(pathSwitch_verbose_) ROS_INFO_STREAM("cycle time mean updated: "<<pathSwitch_cycle_time_mean_);
            }
            else
            {
              if(pathSwitch_verbose_) ROS_INFO_STREAM("cycle time mean not updated");
            }

            if(pathSwitch_disp_) disp_->nextButton("Press \"next\" to execute the next PathSwitch step");
          }
          else
          {
            if(!an_obstacle_ && pathSwitch_cycle_time_mean_ != std::numeric_limits<double>::infinity())
            {
              pathSwitch_cycle_time_mean_ = 1.2*pathSwitch_cycle_time_mean_;
              if(pathSwitch_verbose_) ROS_INFO_STREAM("cycle time mean increased of 20%: "<<pathSwitch_cycle_time_mean_);
            }

            /*//////////Visualization//////////////////*/
            if(pathSwitch_disp_)
            {
              ROS_INFO("Not solved");

              disp_->changeNodeSize(marker_scale_sphere);
              new_node_id = disp_->displayNode(path2_node,"pathplan",marker_color_sphere);
              disp_->defaultNodeSize();

              node_id_vector.push_back(new_node_id);

              disp_->nextButton("Press \"next\" to execute the next PathSwitch step");
            }
            /*//////////////////////////////////////*/
          }

          //Regardless if you have found a solution or not, delete the fake nodes
          path1_node_fake->disconnect();
          path2_node_fake->disconnect();
        }
        else
        {
          if(pathSwitch_verbose_ || pathSwitch_disp_) ROS_INFO_STREAM("It would not be a better solution");

          /*//////////Visualization//////////////////*/
          if(pathSwitch_disp_)
          {
            disp_->changeNodeSize(marker_scale_sphere);
            new_node_id = disp_->displayNode(path2_node,"pathplan",marker_color_sphere);
            disp_->defaultNodeSize();

            node_id_vector.push_back(new_node_id);

            disp_->nextButton("Press \"next\" to execute the next PathSwitch step");
          }
          /*//////////////////////////////////////*/
        }

        if(pathSwitch_verbose_)
        {
          toc=ros::WallTime::now();
          time = pathSwitch_max_time_ - (toc-tic).toSec();
          ROS_INFO_STREAM("cycle time mean: "<<pathSwitch_cycle_time_mean_<<" -> available time: "<< time);
        }

        toc=ros::WallTime::now();
        time = pathSwitch_max_time_ - (toc-tic).toSec();
        if((!an_obstacle_ && time<time_percentage_variability_*pathSwitch_cycle_time_mean_ && pathSwitch_cycle_time_mean_ != std::numeric_limits<double>::infinity()) || time<=0.0)  //if there is an obstacle, you should use the entire available time to find a feasible solution
        {
          time_out = true;
          break;
        }

      }
      else break;
    }
    if(time_out)
    {
      if(pathSwitch_verbose_) ROS_INFO_STREAM("TIME OUT! max time: "<<pathSwitch_max_time_<<", time_available: "<<time<<", time needed for a new cycle: "<<time_percentage_variability_*pathSwitch_cycle_time_mean_);
      break;
    }
  }

  if(pathSwitch_verbose_ || pathSwitch_disp_)
  {
    if(pathSwitch_verbose_) ROS_INFO_STREAM("PathSwitch duration: "<<(ros::WallTime::now()-tic).toSec());
    if(pathSwitch_disp_)
    {
      for(const int& id_to_delete:node_id_vector) disp_->clearMarker(id_to_delete);
    }
    if(success)
    {
      ROS_INFO_STREAM("PathSwitch has found a solution with cost: " << new_path->cost());
    }
    else
    {
      ROS_INFO_STREAM("PathSwitch has NOT found a solution");
    }
  }

  return success;
}

std::vector<NodePtr> Replanner::startingNodesForPathSwitch(const std::vector<ConnectionPtr>& subpath1_conn, const NodePtr& current_node, const double& current2child_conn_cost, const int& idx,  bool& available_nodes)
{
  std::vector<NodePtr> path1_node_vector;

  if(current2child_conn_cost == std::numeric_limits<double>::infinity() || idx == current_path_->getConnections().size()-1) //if the obstacle is obstructing the current connection, the replanning must start from the current configuration
  {
    available_nodes = 0;
    path1_node_vector.push_back(current_node);
  }
  else      //if the current connection is free, all the nodes between the current child to the parent of the connection obstructed are considered as starting points for the replanning
  {
    int subpath1_size =  subpath1_conn.size();
    for(unsigned int i=0; i<subpath1_size; i++)
    {
      if(i==subpath1_size-1)
      {
        if(subpath1_conn.at(i)->getCost() ==  std::numeric_limits<double>::infinity())      // if the path is free, you can consider all the nodes but it is useless to consider the last one before the goal (it is already connected to the goal with a straight line)
        {
          path1_node_vector.push_back(subpath1_conn.at(i)->getParent());
        }
      }
      else
      {
        path1_node_vector.push_back(subpath1_conn.at(i)->getParent());
        if(subpath1_conn.at(i)->getCost() ==  std::numeric_limits<double>::infinity()) break;
      }
    }
    available_nodes = 1;
  }

  return path1_node_vector;
}

void Replanner::simplifyAdmissibleOtherPaths(const bool& no_available_paths, const PathPtr& confirmed_subpath_from_path2, const int& confirmed_connected2path_number, const NodePtr& starting_node_of_pathSwitch, const std::vector<PathPtr>& reset_other_paths)
{
  if(confirmed_subpath_from_path2 != NULL && !no_available_paths)
  {
    std::vector<Eigen::VectorXd> node_vector = confirmed_subpath_from_path2->getWaypoints();
    node_vector.pop_back();  // removing the goal from the vector

    int pos = -1;
    for (unsigned int k=0; k<node_vector.size(); k++)
    {
      if(starting_node_of_pathSwitch->getConfiguration() == node_vector.at(k))
      {
        pos = k;
        break;
      }
    }

    if(pos>=0)
    {
      if(confirmed_connected2path_number<admissible_other_paths_.size()-1)
      {
        admissible_other_paths_.clear();

        admissible_other_paths_.insert(admissible_other_paths_.begin(),reset_other_paths.begin(),reset_other_paths.begin()+confirmed_connected2path_number);
        admissible_other_paths_.push_back(confirmed_subpath_from_path2->getSubpathFromNode(node_vector.at(pos)));
        admissible_other_paths_.insert(admissible_other_paths_.end(),reset_other_paths.begin()+confirmed_connected2path_number+1,reset_other_paths.end());
      }
      else
      {
        admissible_other_paths_.clear();

        admissible_other_paths_.insert(admissible_other_paths_.begin(),reset_other_paths.begin(),reset_other_paths.begin()+confirmed_connected2path_number);
        admissible_other_paths_.push_back(confirmed_subpath_from_path2->getSubpathFromNode(node_vector.at(pos)));
      }
    }
    else
    {
      admissible_other_paths_ = reset_other_paths;
    }
  }
  else
  {
    admissible_other_paths_ = reset_other_paths;
  }
}

bool Replanner::informedOnlineReplanning(const double &max_time)
{

  /*TO DO:
  - valuta se mettere condizioni tempo medio ciclo informed o guardare solo se t<0
  - valuta se guardare tempo medio impiegato da PS prima di lanciarlo o se far fare direttamente a lui
  - valuta se settare valore default media pari a 0
*/

  ros::WallTime tic=ros::WallTime::now();
  ros::WallTime toc, tic_cycle, toc_cycle;
  double MAX_TIME;
  if(informedOnlineReplanning_disp_) MAX_TIME= std::numeric_limits<double>::infinity();
  else MAX_TIME = max_time;
  available_time_ = MAX_TIME;
  std::vector<double> time_vector;
  const double TIME_LIMIT = 0.85*MAX_TIME; //seconds
  const int CONT_LIMIT = 5;

  //if(informedOnlineReplanning_cycle_time_mean_ != std::numeric_limits<double>::infinity()) time_vector.push_back(informedOnlineReplanning_cycle_time_mean_);
  if(!informedOnlineReplanning_disp_ && available_time_<=0.0) return false;

  std::vector<double> marker_scale_sphere(3,0.03);
  std::vector<double> marker_color_sphere = {0.0,0.0,0.0,1.0};
  std::vector<double> marker_color_sphere_analizing = {1.0,0.5,0.0,1.0};

  std::vector<double> marker_color = {1.0,1.0,0.0,1.0};
  std::vector<double> marker_scale(3,0.01);
  int replanned_path_id;
  int node_id;

  PathPtr new_path;
  PathPtr replanned_path;
  PathPtr subpath_from_path2;
  PathPtr confirmed_subpath_from_path2 = NULL;
  PathPtr admissible_current_path = NULL;
  PathPtr subpath1;
  std::vector<PathPtr> replanned_path_vector;
  std::vector<PathPtr> reset_other_paths;
  std::vector<NodePtr> path1_node_vector;
  std::vector<ConnectionPtr> subpath1_conn;
  bool exit = false;
  bool success = false;
  bool solved = false;
  bool first_sol = true;
  bool available_nodes;
  bool no_available_paths = true;
  int confirmed_connected2path_number;
  int connected2path_number;
  unsigned int cont = 0;  //to count the number of replanning without significant improvement in the final solution
  double replanned_path_cost = std::numeric_limits<double>::infinity();
  double previous_cost = current_path_->getCostFromConf(current_configuration_);

  examined_nodes_.clear();
  success_ = false;
  an_obstacle_ = false;

  int current_conn_idx; //to save the index of the connection on which the current configuration is
  ConnectionPtr current_conn = current_path_->findConnection(current_configuration_,current_conn_idx);

  if(current_conn_idx<0) // ////DA ELIMINARE/////////////////////////////////////////////////
  {
    double dist1 = (current_configuration_ - current_path_->getWaypoints().at(1)).norm();
    double dist2 = (current_path_->getWaypoints().at(0) - current_path_->getWaypoints().at(1)).norm();
    ROS_INFO_STREAM("current conf: "<< current_configuration_.transpose()<< " dist1: "<<dist1);
    ROS_INFO_STREAM("start conf: "<< current_path_->getWaypoints().at(0).transpose() << " lenght_conn: "<<dist2);
    ROS_INFO_STREAM("DIFF: "<<std::abs(dist1-dist2));
    ROS_INFO_STREAM("idx: "<<current_conn_idx);

    for(Eigen::VectorXd wp:current_path_->getWaypoints()) ROS_INFO_STREAM("WP: "<<wp.transpose());
    throw std::invalid_argument("idx < 0");
  } // //////////////////////////////////////////////////////////////////////////

  admissible_other_paths_.clear();
  reset_other_paths = addAdmissibleCurrentPath(current_conn_idx, admissible_current_path);
  admissible_other_paths_ = reset_other_paths;

  for(const PathPtr& path: admissible_other_paths_)
  {
    if(path->getConnections().back()->getCost() != std::numeric_limits<double>::infinity()) no_available_paths = false;  //if there is a path with the last connection free it means that there is almost an available path to connect to
  }

  NodePtr parent = current_conn->getParent();
  NodePtr current_node = std::make_shared<Node>(current_configuration_);
  NodePtr child = current_conn->getChild();

  ConnectionPtr current2child_conn = NULL;
  double current2child_conn_cost = 0;

  std::vector<ConnectionPtr> conn;

  if(current_conn_idx<current_path_->getConnections().size()-1)
  {
    subpath1 = current_path_->getSubpathFromNode(child);
    subpath1_conn =  subpath1->getConnections();
    if(subpath1->cost() == std::numeric_limits<double>::infinity()) an_obstacle_ = true;
  }

  if(current_configuration_ != child->getConfiguration() && current_configuration_ != parent->getConfiguration())
  {
    current2child_conn = std::make_shared<Connection>(current_node,child);

    if(current_conn->getCost() == std::numeric_limits<double>::infinity())
    {
      if(!checker_->checkConnection(current2child_conn))
      {
        current2child_conn_cost = std::numeric_limits<double>::infinity();
        an_obstacle_ = true;
      }
      else current2child_conn_cost = metrics_->cost(current_node,child);
    }
    else current2child_conn_cost = metrics_->cost(current_node,child);

    current2child_conn->setCost(current2child_conn_cost);
    current2child_conn->add();

    conn.push_back(current2child_conn);
    if(current_conn_idx<current_path_->getConnections().size()-1)
    {
      conn.insert(conn.end(),subpath1_conn.begin(),subpath1_conn.end());
    }
  }
  else if(current_configuration_ == parent->getConfiguration())  //e.g., if the current_conf is equal to START
  {
    current2child_conn = current_conn;
    conn.push_back(current2child_conn);
    if(current_conn_idx<current_path_->getConnections().size()-1)
    {
      conn.insert(conn.end(),subpath1_conn.begin(),subpath1_conn.end());
    }
  }
  else
  {
    if(current_conn_idx<current_path_->getConnections().size()-1)
    {
      conn = subpath1_conn;
    }
    else
    {
      success_ = false;
      return success_;
    }
  }

  replanned_path = std::make_shared<Path>(conn,metrics_,checker_); // at the start, the replanned path is initialized with the subpath of the current path from the current config to GOAL
  replanned_path_cost = replanned_path->cost();

  path1_node_vector = startingNodesForPathSwitch(subpath1_conn,current_node,current2child_conn_cost,current_conn_idx,available_nodes);
  int j = path1_node_vector.size()-1;

  while(j>=0 && !emergency_stop_)
  {
    if(informedOnlineReplanning_verbose_ || informedOnlineReplanning_disp_) ROS_INFO_STREAM("j: "<<j);
    if(informedOnlineReplanning_disp_)
    {
      /*////////////////////////Visualization of analyzed nodes //////////////////////////////////////*/
      disp_->changeNodeSize(marker_scale_sphere);
      node_id = disp_->displayNode(path1_node_vector.at(j),"pathplan",marker_color_sphere_analizing);
      disp_->defaultNodeSize();
      /*/////////////////////////////////////////////////////////////////////////////////////////////*/
    }

    tic_cycle = ros::WallTime::now();

    simplifyAdmissibleOtherPaths(no_available_paths,confirmed_subpath_from_path2,confirmed_connected2path_number,path1_node_vector.at(j),reset_other_paths);

    if(pathSwitch_cycle_time_mean_ >= 0.8*max_time) pathSwitch_cycle_time_mean_ = std::numeric_limits<double>::infinity();  //reset

    toc = ros::WallTime::now();
    available_time_ = MAX_TIME - (toc-tic).toSec();
    double min_time_pathSwitch;
    if(informedOnlineReplanning_disp_) min_time_pathSwitch = std::numeric_limits<double>::infinity();
    else if(an_obstacle_ || pathSwitch_cycle_time_mean_ == std::numeric_limits<double>::infinity()) min_time_pathSwitch = 0.0;
    else min_time_pathSwitch = time_percentage_variability_*pathSwitch_cycle_time_mean_;

    if(informedOnlineReplanning_verbose_) ROS_INFO_STREAM("available time: "<<available_time_<<", min required time to call PathSwitch: "<<min_time_pathSwitch);

    if(available_time_>=min_time_pathSwitch && !emergency_stop_)
    {
      if(no_available_paths)
      {
        if(informedOnlineReplanning_verbose_ || informedOnlineReplanning_disp_) ROS_INFO_STREAM("Launching Connect2Goal...");
        solved = connect2goal(replanned_path,path1_node_vector.at(j),new_path);
      }
      else
      {
        if(informedOnlineReplanning_verbose_ || informedOnlineReplanning_disp_) ROS_INFO_STREAM("Launching PathSwitch...");
        solved = pathSwitch(replanned_path, path1_node_vector.at(j), new_path, subpath_from_path2, connected2path_number);
      }
    }
    else
    {
      solved = false;
      exit = true;
      if(informedOnlineReplanning_verbose_) ROS_INFO_STREAM("Not eanough time to call PathSwitch or Connect2Goal, available time: "<<available_time_<<" min time to call ps: "<<min_time_pathSwitch);
    }

    path1_node_vector.at(j)->setAnalyzed(1);  //to set as ANALYZED the node just analyzed. In this way, it will not be analyzed again in this replanning procedure
    examined_nodes_.push_back(path1_node_vector.at(j));  //to save the analyzed nodes

    if(informedOnlineReplanning_verbose_ || informedOnlineReplanning_disp_) ROS_INFO_STREAM("Solved: "<<solved);
    if(informedOnlineReplanning_disp_)
    {
      /*////////////////////////Visualization of analyzed nodes //////////////////////////////////////*/
      disp_->clearMarker(node_id);
      disp_->changeNodeSize(marker_scale_sphere);
      node_id = disp_->displayNode(examined_nodes_.back(),"pathplan",marker_color_sphere);
      disp_->defaultNodeSize();
      /*/////////////////////////////////////////////////////////////////////////////////////////////*/
    }

    if(solved)
    {
      PathPtr path;
      PathPtr subpath;
      std::vector<ConnectionPtr> path_conn;

      if(available_nodes) //calculating the cost of the replanned path found
      {
        if(path1_node_vector.at(j)->getConfiguration() != child->getConfiguration() && path1_node_vector.at(j)->getConfiguration() != parent->getConfiguration())
        {
          if(current2child_conn != NULL) path_conn.push_back(current2child_conn);  //connection between the current config and the child of the current conn

          try
          {
            subpath =  subpath1->getSubpathToNode(path1_node_vector.at(j));  //path between the current connection child and the node analyzed now
          }
          catch(std::invalid_argument)
          {
            ROS_INFO_STREAM("curr conf: "<<current_configuration_.transpose()<<" child: "<<child->getConfiguration().transpose()<<" parent: "<<parent->getConfiguration().transpose());
          }

          std::vector<ConnectionPtr> conn_sup;
          conn_sup = subpath->getConnections();
          path_conn.insert(path_conn.end(),conn_sup.begin(),conn_sup.end());

          conn_sup.clear();
          conn_sup = new_path->getConnections();
          path_conn.insert(path_conn.end(),conn_sup.begin(),conn_sup.end());

          path = std::make_shared<Path>(path_conn,metrics_,checker_);
        }
        else if(path1_node_vector.at(j)->getConfiguration() == parent->getConfiguration())
        {
          path = new_path;

          if(current_configuration_ != parent->getConfiguration())
          {
            throw std::invalid_argument("curr conf dovrebbe essere = al parent");
          }
        }
        else    //the node analyzed is the child of the current connection
        {
          if(current2child_conn != NULL) path_conn.push_back(current2child_conn);

          std::vector<ConnectionPtr> conn_sup =  new_path->getConnections();
          path_conn.insert(path_conn.end(),conn_sup.begin(),conn_sup.end());

          path = std::make_shared<Path>(path_conn,metrics_,checker_);
        }
      }
      else
      {
        path = new_path;
      }

      if(path->cost()<replanned_path_cost) //if the cost of the new solution found is better than the cost of the best solution found so far
      {
        if(informedOnlineReplanning_verbose_ || informedOnlineReplanning_disp_) ROS_INFO_STREAM("new path found, cost: " << path->cost() <<" previous cost: " << replanned_path_cost);

        if(first_sol)
        {
          toc = ros::WallTime::now();
          time_first_sol_ = (toc - tic).toSec();
          time_replanning_ = time_first_sol_;
          first_sol = 0;  //0 when the time of the first solution found has been already saved
        }

        previous_cost = replanned_path_cost;
        replanned_path = path;
        replanned_path_cost = path->cost();

        if(!no_available_paths)  //PathSwitch called and not Connect2Goal
        {
          confirmed_connected2path_number = connected2path_number;  //to remember the vector index of the path to which the algoithm has created a connection
          confirmed_subpath_from_path2 = subpath_from_path2;        //to save the subpath of the path to which the algoritm has created a connection
        }

        success = 1;
        an_obstacle_ = false;

        if(replanned_path->cost() == std::numeric_limits<double>::infinity()) throw std::invalid_argument("the cost of the path found should be finite");

        if(replanned_path_vector.size()<10) //the algorithm gives as output the vector of the best 10 solutions found
        {
          replanned_path_vector.push_back(replanned_path);
        }
        else
        {
          std::vector<PathPtr> support_vector;
          support_vector.insert(support_vector.begin(),replanned_path_vector.begin()+1,replanned_path_vector.end());
          support_vector.push_back(replanned_path);

          replanned_path_vector = support_vector;
        }

        if(available_nodes == 0 && replanned_path->getConnections().size()>1) // when actual conn is obstructed and a path has been found -> PathSwitch will be called from the nodes of the new path found
        {
          current2child_conn = replanned_path->getConnections().at(0);
          child = current2child_conn->getChild();
          available_nodes = 1;
        }

        if(informedOnlineReplanning_disp_)
        {
          /*//////////////////////////Visualization////////////////////////////////////*/
          disp_->clearMarker(replanned_path_id);
          disp_->changeConnectionSize(marker_scale);
          replanned_path_id = disp_->displayPath(replanned_path,"pathplan",marker_color);
          disp_->defaultConnectionSize();
          /*/////////////////////////////////////////////////////////////////////////*/
        }

        toc = ros::WallTime::now();
        if((toc-tic).toSec()>TIME_LIMIT && cont >= CONT_LIMIT)
        {
          j = -1;
          break;
        }
        else
        {
          if((previous_cost-replanned_path_cost)<0.05*previous_cost) cont = cont+1;
          else cont = 0;
        }
      }
      else
      {
        if(informedOnlineReplanning_verbose_ || informedOnlineReplanning_disp_) ROS_INFO_STREAM("NO better path found, cost: " << path->cost() <<" previous cost: " << replanned_path_cost);
      }

      toc_cycle = ros::WallTime::now();
      //time_vector.push_back((toc_cycle-tic_cycle).toSec());
      //informedOnlineReplanning_cycle_time_mean_ = std::accumulate(time_vector.begin(), time_vector.end(),0.0)/((double) time_vector.size());
      if(informedOnlineReplanning_verbose_) ROS_INFO_STREAM("Solution with cost "<<replanned_path_cost<<" found!->Informed cycle duration: "<<(toc_cycle-tic_cycle).toSec());
    }

    if(success && j == 0)
    {
      if(child->getConfiguration() != current_path_->getWaypoints().back())
      {
        subpath1 = replanned_path->getSubpathFromNode(child);
        path1_node_vector.clear();
        for(unsigned int r=0; r<subpath1->getConnections().size()-1; r++)
        {
          if(subpath1->getConnections().at(r)->getParent()->getAnalyzed() == 0 && subpath1->getConnections().at(r)->getParent()->getNonOptimal() == 0) //Analyzed to check if they have been already analyzed (if 0 not not analyzed), nonOptimal to check if they are useful to improve the replanning solution (if 0, maybe they can improve the solution)
          {
            // the nodes of the new solution found are added to the set of the nodes to be analyzed
            path1_node_vector.push_back(subpath1->getConnections().at(r)->getParent());
          }
        }
        j = path1_node_vector.size();  //then, j=j-1

        if(informedOnlineReplanning_verbose_ || informedOnlineReplanning_disp_) ROS_INFO_STREAM("NEW J: "<<j-1);
      }
    }

    //if(informedOnlineReplanning_verbose_) ROS_INFO_STREAM("informed cycle time mean: "<< informedOnlineReplanning_cycle_time_mean_);

    toc = ros::WallTime::now();
    available_time_ = MAX_TIME-(toc-tic).toSec();

    //if((!an_obstacle_ && available_time_<time_percentage_variability_*informedOnlineReplanning_cycle_time_mean_ && informedOnlineReplanning_cycle_time_mean_ != std::numeric_limits<double>::infinity()) || available_time_<=0.0)
    if(exit || j==0)
    {
      if(informedOnlineReplanning_verbose_) ROS_INFO_STREAM("TIME OUT! available time: "<<available_time_<<", time needed for a new cycle: "<<min_time_pathSwitch);
      //if(informedOnlineReplanning_verbose_) ROS_INFO_STREAM("TIME OUT! available time: "<<available_time_<<", time needed for a new cycle: "<<time_percentage_variability_*informedOnlineReplanning_cycle_time_mean_);
      if(informedOnlineReplanning_disp_)
      {
        ROS_INFO("Optimizing...");
        disp_->nextButton();
      }

      double cost_pre_opt = replanned_path->cost();
      ros::WallTime tic_warp = ros::WallTime::now();
      if(success) optimizePath(replanned_path,available_time_*0.95);
      ros::WallTime toc_warp = ros::WallTime::now();
      double cost_opt = replanned_path->cost();

      if(informedOnlineReplanning_verbose_) ROS_INFO_STREAM("Path optimization, max time: "<<available_time_<<" time used: "<<(toc_warp-tic_warp).toSec()<<" previous cost: "<<cost_pre_opt<<" new cost: "<<cost_opt);

      j = -1;
      break;
    }

    j -= 1;

    if(emergency_stop_)
    {
      ROS_WARN("EMERGENCY STOP HANDLED");   //ELIMINA
      emergency_stop_ = false;
      break;
    }

    if(informedOnlineReplanning_verbose_ || informedOnlineReplanning_disp_) ROS_INFO("------------------------------------------");
    if(informedOnlineReplanning_disp_) disp_->nextButton("Press \"next\" to execute the next InformedOnlineReplanning step");
  }

  for(unsigned int x=0; x<examined_nodes_.size();x++) examined_nodes_.at(x)->setAnalyzed(0);

  if(success)
  {
    replanned_path_ = replanned_path;
    std::reverse(replanned_path_vector.begin(),replanned_path_vector.end());  //ordered with growing cost
    replanned_paths_vector_ = replanned_path_vector;
    success_ = true;

    toc = ros::WallTime::now();
    time_replanning_ = (toc - tic).toSec();

    if(informedOnlineReplanning_verbose_ || informedOnlineReplanning_disp_) ROS_INFO_STREAM("InformedOnlineReplanning has found a solution with cost: " <<replanned_path_->cost() << " in "<< time_replanning_ << "seconds. Number of sol: " << replanned_path_vector.size());
  }
  else
  {
    success_ = false;
    if(informedOnlineReplanning_verbose_ || informedOnlineReplanning_disp_) ROS_INFO_STREAM("InformedOnlineReplanning has NOT found a solution");
  }

  toc = ros::WallTime::now();
  available_time_ = MAX_TIME-(toc-tic).toSec();

  return success;
}

}
