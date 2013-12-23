
#ifndef PREFER_FAST_COST_FUNCTION_H_
#define PREFER_FAST_COST_FUNCTION_H_

#include <ros/console.h>

#include <base_local_planner/trajectory_cost_function.h>

namespace wall_following
{

class PreferFastCostFunction : public base_local_planner::TrajectoryCostFunction
{

public:

  PreferFastCostFunction();

  virtual ~PreferFastCostFunction();

  bool prepare() { return true; }

  double scoreTrajectory(base_local_planner::Trajectory & traj);

  void setMaxLinearVel(double max_linear_vel) { max_linear_vel_ = max_linear_vel; }

  void setMaxAngularVel(double max_angular_vel) { max_angular_vel_ = max_angular_vel; }

  void setTrajTime(double traj_time) { traj_time_ = traj_time; }

private:

  double max_linear_vel_;

  double max_angular_vel_;

  double traj_time_; 

};

} // namespace wall_following

#endif // #ifndef PREFER_FAST_COST_FUNCTION_H_

