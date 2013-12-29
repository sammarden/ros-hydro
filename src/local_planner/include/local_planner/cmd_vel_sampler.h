
#ifndef CMD_VEL_SAMPLER_H_
#define CMD_VEL_SAMPLER_H_

#include <deque>

#include <geometry_msgs/Twist.h>

namespace local_planner
{

class CmdVelSampler
{

public:

  CmdVelSampler();

  ~CmdVelSampler();

  void reset(double linear_min, double linear_max, int linear_samples, double angular_min, double angular_max, int angular_samples);

  bool isEmpty();

  geometry_msgs::Twist next();

private:

  std::deque<geometry_msgs::Twist> cmd_vel_queue_;

};

} // namespace local_planner

#endif // #ifndef CMD_VEL_SAMPLER_H_

