
#include <local_planner/constant_linear_velocity_trajectory_generator.h>

namespace local_planner
{

ConstantLinearVelocityTrajectoryGenerator::ConstantLinearVelocityTrajectoryGenerator()
{

}

ConstantLinearVelocityTrajectoryGenerator::~ConstantLinearVelocityTrajectoryGenerator()
{

}

void ConstantLinearVelocityTrajectoryGenerator::generateTrajectories(std::vector<cmd_vel_traj_t> & cmd_vel_trajs)
{

  cmd_vel_samplers_.resize(angular_vel_resamples_ + 1);

  for (std::vector<CmdVelSampler>::iterator cmd_vel_sampler = cmd_vel_samplers_.begin(); cmd_vel_sampler != cmd_vel_samplers_.end(); ++cmd_vel_sampler)
  {
    cmd_vel_sampler->reset(linear_vel_, linear_vel_, 1, angular_vel_min_, angular_vel_max_, angular_vel_samples_);
  }

  cmd_vel_t start_cmd_vel;
  traj_t traj;
  int i = 0;

  std::vector<traj_t> traj_tree(angular_vel_resamples_ + 1);

  while (true)
  {

    // At the end of the tree.
    if (i == cmd_vel_samplers_.size())
    {
      cmd_vel_trajs.push_back(cmd_vel_traj_t(start_cmd_vel, traj));
      //traj.resetPoints();
      --i;
      traj = traj_tree[i];
    }
    // This sampler is empty.
    else if (cmd_vel_samplers_[i].isEmpty())
    {

      // We've finished.
      if (i == 0)
      {
        break;
      }

      cmd_vel_samplers_[i].reset(linear_vel_, linear_vel_, 1, angular_vel_min_, angular_vel_max_, angular_vel_samples_);
      traj = traj_tree[i-1];
      --i; 

    }
    // We can simulate.
    else
    {
      cmd_vel_t cmd_vel = cmd_vel_samplers_[i].next(); 
      if (i == 0)
      {
        start_cmd_vel = cmd_vel;
      }
      traj_tree[i] = traj;
      forwardSimulateTrajectory(traj, cmd_vel);
      ++i;
    }
   
  }

}

void ConstantLinearVelocityTrajectoryGenerator::forwardSimulateTrajectory(traj_t & traj, const cmd_vel_t & cmd_vel)
{

  double x, y, th;
  // If the trajectory is not empty, simulate the trajectory from where it current ends.
  if (traj.getPointsSize() > 0)
  {
    traj.getPoint(traj.getPointsSize() - 1, x, y, th);
  }
  else
  {
    x = robot_x_;
    y = robot_y_;
    th = robot_th_;
  }

  int num_steps = (int)ceil(sim_time_/sim_dt_)/(angular_vel_resamples_ + 1);

  for (int i = 0; i < num_steps; ++i)
  {

    x += cmd_vel.linear.x*cos(th)*sim_dt_;
    y += cmd_vel.linear.x*sin(th)*sim_dt_; 
    th += cmd_vel.angular.z*sim_dt_;

    traj.addPoint(x, y, th);

  }

}

} // namespace local_planner

