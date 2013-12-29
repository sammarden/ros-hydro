
#ifndef LOCAL_PLANNER_H_
#define LOCAL_PLANNER_H_

#include <nav_msgs/Path.h>

#include <base_local_planner/local_planner_util.h>
#include <base_local_planner/obstacle_cost_function.h>
#include <base_local_planner/trajectory_cost_function.h>

#include <local_planner/constant_linear_velocity_trajectory_generator.h>
#include <local_planner/prefer_straight_cost_function.h>

#include <wall_following/nearest_scan_cost_function.h>

#include <param_utils/param_utils.h>

namespace local_planner
{

class LocalPlanner
{

typedef geometry_msgs::Twist cmd_vel_t;
typedef base_local_planner::Trajectory traj_t;
typedef std::pair<cmd_vel_t, traj_t> cmd_vel_traj_t;
typedef base_local_planner::TrajectoryCostFunction cost_t;
typedef boost::shared_ptr<cost_t> cost_ptr_t;

public:

  LocalPlanner();

  ~LocalPlanner();

  void initialise(base_local_planner::LocalPlannerUtil & local_planner_util, costmap_2d::Costmap2DROS * costmap_ros);

  void parseParams(ros::NodeHandle & node_handle);

  bool computeVelocityCommands(geometry_msgs::Twist & cmd_vel);

  bool findBestTrajectory(std::vector<cmd_vel_traj_t> & cmd_vel_trajs, cmd_vel_t & cmd_vel);

  inline void setRobotPose(double x, double y, double th)
  {
    robot_x_ = x;
    robot_y_ = y;
    robot_th_ = th; 
  }

private:

  std::vector<double> generateAchievableLinearVelocities();

  void publishTrajectories(std::vector<cmd_vel_traj_t> & cmd_vel_trajs);

  void publishBestTrajectory(traj_t & traj);

  ros::Publisher trajs_pub_;

  ros::Publisher best_traj_pub_;

  local_planner::ConstantLinearVelocityTrajectoryGenerator traj_gen_;

  boost::shared_ptr<base_local_planner::ObstacleCostFunction> obs_cost_func_ptr_;

  boost::shared_ptr<local_planner::PreferStraightCostFunction> prefer_straight_cost_func_ptr_;

  boost::shared_ptr<wall_following::NearestScanCostFunction> nearest_scan_cost_func_ptr_;

  std::vector<cost_ptr_t> critics_;

  double robot_x_;

  double robot_y_;
  
  double robot_th_;

  double robot_linear_vel_;

  double robot_angular_vel_;

  double linear_vel_min_;

  double linear_vel_max_;

  int linear_vel_samples_;

  double angular_vel_min_;

  double angular_vel_max_;

  int angular_vel_samples_;

  double linear_acc_max_;

  double angular_acc_max_;

  double sim_time_;

  double sim_dt_;

};

} // namespace local_planner

#endif // #ifndef LOCAL_PLANNER_H_

