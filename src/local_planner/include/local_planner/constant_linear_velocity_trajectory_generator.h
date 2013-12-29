
#ifndef CONSTANT_LINEAR_VELOCITY_TRAJECTORY_GENERATOR_H_
#define CONSTANT_LINEAR_VELOCITY_TRAJECTORY_GENERATOR_H_

#include <base_local_planner/trajectory.h>

#include <local_planner/cmd_vel_sampler.h>

namespace local_planner
{

class ConstantLinearVelocityTrajectoryGenerator
{

typedef geometry_msgs::Twist cmd_vel_t;
typedef base_local_planner::Trajectory traj_t;
typedef std::pair<cmd_vel_t, traj_t> cmd_vel_traj_t;

public:

  ConstantLinearVelocityTrajectoryGenerator();

  ~ConstantLinearVelocityTrajectoryGenerator();

  void generateTrajectories(std::vector<cmd_vel_traj_t> & cmd_vel_trajs);

  void forwardSimulateTrajectory(traj_t & traj, const cmd_vel_t & cmd_vel);

  inline void setAngularVelResamples(int angular_vel_resamples) { angular_vel_resamples_ = angular_vel_resamples; }

  inline void setLinearVel(double linear_vel) { linear_vel_ = linear_vel; }

  inline void setAngularVelMin(double angular_vel_min) { angular_vel_min_ = angular_vel_min; }

  inline void setAngularVelMax(double angular_vel_max) { angular_vel_max_ = angular_vel_max; }

  inline void setAngularVelSamples(int angular_vel_samples) { angular_vel_samples_ = angular_vel_samples; }

  inline void setSimTime(double sim_time) { sim_time_ = sim_time; }

  inline void setSimDt(double sim_dt) { sim_dt_ = sim_dt; }

  inline void setRobotPose(double x, double y, double th)
  {
    robot_x_ = x;
    robot_y_ = y;
    robot_th_ = th;
  }

private:

  std::vector<CmdVelSampler> cmd_vel_samplers_;

  int angular_vel_resamples_;

  double linear_vel_;

  double angular_vel_min_;

  double angular_vel_max_;

  int angular_vel_samples_;

  double sim_time_;

  double sim_dt_;

  double robot_x_;

  double robot_y_;

  double robot_th_;

};

} // namespace local_planner

#endif // ifndef CONSTANT_LINEAR_VELOCITY_TRAJECTORY_GENERATOR_H_

