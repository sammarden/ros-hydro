
#ifndef EXPONENTIAL_MOVING_AVERAGE_FILTER_H_
#define EXPONENTIAL_MOVING_AVERAGE_FILTER_H_

#include <geometry_msgs/Twist.h>

#include <filters/filter_base.h>

namespace cmd_vel_filters
{

/** \brief Implements an exponential moving average for cmd_vel drive commands.
  *
  * \author Sam Marden
  */
class ExponentialMovingAverageFilter : public filters::FilterBase<geometry_msgs::Twist>
{

public:

  virtual ~ExponentialMovingAverageFilter();

  /** \brief Configure the filter. */
  bool configure();

  /** \brief Update the filter.
    *
    * \param input_cmd_vel Input command velocity.
    * \param filtered_cmd_vel Filtered command velocity.
    */
  bool update(const geometry_msgs::Twist & input_cmd_vel, geometry_msgs::Twist & filtered_cmd_vel);

private:

  /** \brief Exponential moving average of linear velocity commands. */
  double moving_avg_linear_x_;

  /** \brief Exponential moving average of angular velocity commands. */
  double moving_avg_angular_z_;

  /** \brief Filter smoothing factor. Higher values discount older observations faster. Defaults to 0.9. */
  double alpha_;

};

} // namespace cmd_vel_filters

#endif // #ifndef EXPONENTIAL_MOVING_AVERAGE_FILTER_H_

