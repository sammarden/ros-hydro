
#include <cmd_vel_filters/exponential_moving_average_filter.h>

ExponentialMovingAverageFilter::~ExponentialMovingAverageFilter()
{

}

bool ExponentialMovingAverageFilter::configure()
{

  moving_avg_linear_x_ = 0.0;
  moving_avg_angular_z_ = 0.0;

  alpha_ = 0.9;
  getParam("alpha", alpha_);

  return true;

}

bool ExponentialMovingAverageFilter::update(const geometry_msgs::Twist & input_cmd_vel, geometry_msgs::Twist & filtered_cmd_vel)
{

  moving_avg_linear_x_ = alpha*input_cmd_vel.linear.x + (1.0 - alpha)*moving_avg_linear_x_;
  moving_avg_angular_z_ = alpha*input_cmd_vel.angular.z + (1.0 - alpha)*moving_avg_angular_z_;

  filtered_cmd_vel.linear.x = moving_avg_linear_x_;
  filtered_cmd_vel.angular.z = moving_avg_angular_z_;

  return true;

}

