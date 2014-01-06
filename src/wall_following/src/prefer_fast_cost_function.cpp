
#include <wall_following/prefer_fast_cost_function.h>

namespace wall_following
{

PreferFastCostFunction::PreferFastCostFunction() : max_linear_vel_(-1.0), max_angular_vel_(-1.0), traj_time_(-1.0)
{

}

PreferFastCostFunction::~PreferFastCostFunction()
{

}

double PreferFastCostFunction::scoreTrajectory(base_local_planner::Trajectory & traj)
{

  if (max_linear_vel_ < 0.0)
  {
    ROS_ERROR("max_linear_vel not set");
    return 0.0;
  }

  if (max_angular_vel_ < 0.0)
  {
    ROS_ERROR("max_angular_vel not set");
    return 0.0;
  }

  if (traj_time_ < 0.0)
  {
    ROS_ERROR("traj_time not set");
    return 0.0;
  }

  double linear_dist = 0.0;
  double angular_dist = 0.0;
  double prev_traj_x, prev_traj_y, prev_traj_th;

  for (unsigned int i = 0; i < traj.getPointsSize(); ++i)
  {

    double traj_x, traj_y, traj_th;
    traj.getPoint(i, traj_x, traj_y, traj_th);

    if (i == 0)
    {
      prev_traj_x = traj_x;
      prev_traj_y = traj_y;
      prev_traj_th = traj_th;
      continue;
    }

    double dx = traj_x - prev_traj_x;
    double dy = traj_y - prev_traj_y;
    double dth = traj_th - prev_traj_th;

    linear_dist += sqrt(dx*dx + dy*dy);
    angular_dist += fabs(dth);

    prev_traj_x = traj_x;
    prev_traj_y = traj_y;
    prev_traj_th = traj_th;

  }

  double avg_linear_vel = linear_dist/traj_time_;
  double avg_angular_vel = angular_dist/traj_time_;

//  double score = std::min(1.0 - avg_linear_vel/max_linear_vel_, 1.0 - avg_angular_vel/max_angular_vel_);
  double score = 1.0 - avg_linear_vel/max_linear_vel_ + avg_angular_vel/max_angular_vel_;

  score = std::max(score, 0.0);

  ROS_INFO("%f %f %f %f", avg_linear_vel, max_linear_vel_, avg_angular_vel, score);

  return score;

}

} // namespace wall_following

