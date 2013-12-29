
#include <local_planner/local_planner.h>

namespace local_planner
{

LocalPlanner::LocalPlanner() : robot_linear_vel_(0.0), robot_angular_vel_(0.0)
{

}

LocalPlanner::~LocalPlanner()
{

}

void LocalPlanner::initialise(base_local_planner::LocalPlannerUtil & local_planner_util, costmap_2d::Costmap2DROS * costmap_ros)
{

  obs_cost_func_ptr_ = boost::shared_ptr<base_local_planner::ObstacleCostFunction>(new base_local_planner::ObstacleCostFunction(local_planner_util.getCostmap()));
  obs_cost_func_ptr_->setFootprint(costmap_ros->getRobotFootprint());

  critics_.push_back(obs_cost_func_ptr_);

  prefer_straight_cost_func_ptr_ = boost::shared_ptr<local_planner::PreferStraightCostFunction>(new local_planner::PreferStraightCostFunction());

  critics_.push_back(prefer_straight_cost_func_ptr_);

  prefer_straight_cost_func_ptr_->setScale(0.0);

  nearest_scan_cost_func_ptr_ = boost::shared_ptr<wall_following::NearestScanCostFunction>(new wall_following::NearestScanCostFunction());

  critics_.push_back(nearest_scan_cost_func_ptr_);

  nearest_scan_cost_func_ptr_->setTargetDistanceToScan(1.0);
  nearest_scan_cost_func_ptr_->setMaxDistanceToScan(5.0);
  nearest_scan_cost_func_ptr_->setScale(10.0);

}

void LocalPlanner::parseParams(ros::NodeHandle & node_handle)
{

  param_utils::ParamHelper param_helper(node_handle, "local_planner");

  param_helper.getParamWithInfo("linear_vel_min", linear_vel_min_, linear_vel_min_);
  param_helper.getParamWithInfo("linear_vel_max", linear_vel_max_, linear_vel_max_);
  param_helper.getParamWithInfo("linear_vel_samples", linear_vel_samples_, linear_vel_samples_);

  param_helper.getParamWithInfo("angular_vel_max", angular_vel_max_, angular_vel_max_);
  angular_vel_min_ = -angular_vel_max_;
  param_helper.getParamWithInfo("angular_vel_samples", angular_vel_samples_, angular_vel_samples_);

  param_helper.getParamWithInfo("linear_acc_max", linear_acc_max_, linear_acc_max_);
  param_helper.getParamWithInfo("angular_acc_max", angular_acc_max_, angular_acc_max_);

  param_helper.getParamWithInfo("sim_time", sim_time_, sim_time_);
  param_helper.getParamWithInfo("sim_dt", sim_dt_, sim_dt_);

  trajs_pub_ = node_handle.advertise<nav_msgs::Path>("trajectories", 1);
  best_traj_pub_ = node_handle.advertise<nav_msgs::Path>("best_trajectory", 1);

}

bool LocalPlanner::computeVelocityCommands(geometry_msgs::Twist & cmd_vel)
{

  // Generate the linear velocities to sampmle.
  std::vector<double> achievable_vels = generateAchievableLinearVelocities();

  double min_ach_ang_vel = std::max(robot_angular_vel_ - angular_acc_max_*sim_dt_, angular_vel_min_);
  double max_ach_ang_vel = std::min(robot_angular_vel_ + angular_acc_max_*sim_dt_, angular_vel_max_);
  //double min_ach_ang_vel = angular_vel_min_;
  //double max_ach_ang_vel = angular_vel_max_;

  // Initialise the trajectory generator.
  traj_gen_.setAngularVelMin(min_ach_ang_vel);
  traj_gen_.setAngularVelMax(max_ach_ang_vel);
  traj_gen_.setAngularVelSamples(angular_vel_samples_);
  traj_gen_.setRobotPose(robot_x_, robot_y_, robot_th_);
  traj_gen_.setAngularVelResamples(2);
  traj_gen_.setSimTime(sim_time_);
  traj_gen_.setSimDt(sim_dt_);

  for (std::vector<double>::iterator ach_vel = achievable_vels.begin(); ach_vel != achievable_vels.end(); ++ach_vel)
  {

    std::vector<cmd_vel_traj_t> cmd_vel_trajs;

    traj_gen_.setLinearVel(*ach_vel);
    traj_gen_.generateTrajectories(cmd_vel_trajs);
    publishTrajectories(cmd_vel_trajs);

    bool found_valid_traj = findBestTrajectory(cmd_vel_trajs, cmd_vel);

    if (found_valid_traj)
    {
      break;
    }

  }

  // Save commanded values for next time.
  robot_linear_vel_ = cmd_vel.linear.x;
  robot_angular_vel_ = cmd_vel.angular.z;

  return true;

}

bool LocalPlanner::findBestTrajectory(std::vector<cmd_vel_traj_t> & cmd_vel_trajs, cmd_vel_t & cmd_vel)
{

  double best_traj_cost = std::numeric_limits<double>::max();
  base_local_planner::Trajectory best_traj;

  for (std::vector<cost_ptr_t>::iterator critic = critics_.begin(); critic != critics_.end(); ++critic)
  {
    (*critic)->prepare();
  }

  for (std::vector<cmd_vel_traj_t>::iterator cmd_vel_traj = cmd_vel_trajs.begin(); cmd_vel_traj != cmd_vel_trajs.end(); ++cmd_vel_traj)
  {

    double traj_cost = 0.0;

    for (std::vector<cost_ptr_t>::iterator critic = critics_.begin(); critic != critics_.end(); ++critic)
    {

      double critic_cost = (*critic)->getScale()*(*critic)->scoreTrajectory(cmd_vel_traj->second);

      if (critic_cost < 0.0)
      {
        traj_cost = -1.0;
        break;
      }
      else
      {
        traj_cost += critic_cost;
      }

    }

    if (traj_cost >= 0.0 && traj_cost < best_traj_cost)
    {
      best_traj_cost = traj_cost;
      cmd_vel = cmd_vel_traj->first;
      best_traj = cmd_vel_traj->second;
    }

  }
  
  publishBestTrajectory(best_traj);

  if (best_traj_cost != std::numeric_limits<double>::max())
  {
    return true;
  }
  else
  {
    return false;
  }

}

std::vector<double> LocalPlanner::generateAchievableLinearVelocities()
{

  double min_ach_vel = robot_linear_vel_ - linear_acc_max_*sim_dt_;
  min_ach_vel = std::max(min_ach_vel, linear_vel_min_);

  double max_ach_vel = robot_linear_vel_ + linear_acc_max_*sim_dt_;
  max_ach_vel = std::min(max_ach_vel, linear_vel_max_);

  std::vector<double> ach_vels;

  double linear_vel_delta;

  if (linear_vel_samples_ == 1)
  {
    linear_vel_delta = 0.0; 
  }
  else
  {
    linear_vel_delta = (max_ach_vel - min_ach_vel)/(double)(linear_vel_samples_ - 1);
  }

  double linear_vel = max_ach_vel;
  for (int i = 0; i < linear_vel_samples_; ++i)
  {
    linear_vel = linear_vel - (double)i*linear_vel_delta;
    ach_vels.push_back(linear_vel); 
  }

  return ach_vels; 

}

void LocalPlanner::publishTrajectories(std::vector<cmd_vel_traj_t> & cmd_vel_trajs)
{

  nav_msgs::Path trajs;
  trajs.header.frame_id = "/odom";

  for (std::vector<cmd_vel_traj_t>::iterator cmd_vel_traj = cmd_vel_trajs.begin(); cmd_vel_traj != cmd_vel_trajs.end(); ++cmd_vel_traj)
  {

    for (int i = 0; i < cmd_vel_traj->second.getPointsSize(); ++i)
    {

      double x, y, th;
      cmd_vel_traj->second.getPoint(i, x, y, th);

      geometry_msgs::PoseStamped pose;
      pose.header.frame_id = "/odom";
      pose.pose.position.x = x;
      pose.pose.position.y = y;
      trajs.poses.push_back(pose);

    }

    for (int i = cmd_vel_traj->second.getPointsSize() - 1; i >= 0; --i)
    {

      double x, y, th;
      cmd_vel_traj->second.getPoint(i, x, y, th);

      geometry_msgs::PoseStamped pose;
      pose.header.frame_id = "/odom";
      pose.pose.position.x = x;
      pose.pose.position.y = y;
      trajs.poses.push_back(pose);

    }

  }

  trajs_pub_.publish(trajs);

}

void LocalPlanner::publishBestTrajectory(traj_t & traj)
{

  nav_msgs::Path best_traj;
  best_traj.header.frame_id = "/odom";

  for (int i = 0; i < traj.getPointsSize(); ++i)
  {

    double x, y, th;
    traj.getPoint(i, x, y, th);

    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = "/odom";
    pose.pose.position.x = x;
    pose.pose.position.y = y;
    pose.pose.position.z = 0.1;
    best_traj.poses.push_back(pose);

  }

  best_traj_pub_.publish(best_traj); 

}

} // namespace local_planner

