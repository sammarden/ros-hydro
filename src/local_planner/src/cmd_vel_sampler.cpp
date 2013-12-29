
#include <local_planner/cmd_vel_sampler.h>

namespace local_planner
{

CmdVelSampler::CmdVelSampler()
{

}

CmdVelSampler::~CmdVelSampler()
{

}


void CmdVelSampler::reset(double linear_min, double linear_max, int linear_samples, double angular_min, double angular_max, int angular_samples)
{

  cmd_vel_queue_.clear();

  double linear_delta;
  double angular_delta;

  if (linear_samples > 1)
  {
    linear_delta = (linear_max - linear_min)/(double)(linear_samples - 1);
  }
  else
  {
    linear_delta = 0.0;
  }

  if (angular_samples > 1)
  {
    angular_delta = (angular_max - angular_min)/(double)(angular_samples - 1);
  }
  else
  {
    angular_delta = 0.0;
  }

  for (int i = 0; i < linear_samples; ++i)
  {
    for (int j = 0; j < angular_samples; ++j)
    {

      geometry_msgs::Twist cmd_vel;
      cmd_vel.linear.x = linear_min + (double)i*linear_delta; 
      cmd_vel.angular.z = angular_min + (double)j*angular_delta; 

      cmd_vel_queue_.push_back(cmd_vel);

    }
  }

}

bool CmdVelSampler::isEmpty()
{

  return cmd_vel_queue_.empty();

}

geometry_msgs::Twist CmdVelSampler::next()
{

  geometry_msgs::Twist cmd_vel = cmd_vel_queue_.front();
  cmd_vel_queue_.pop_front();

  return cmd_vel;

}

} // namespace local_planner

