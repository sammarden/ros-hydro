
#ifndef IMU_FEEDBACK_FILTER_H_
#define IMU_FEEDBACK_FILTER_H_

#include <ros/ros.h>

#include <geometry_msgs/Twist.h>

#include <filters/filter_base.h>

#include <boost/thread/thread.hpp>

namespace cmd_vel_filters
{

class IMUFeedbackFilter : public filters::FilterBase<geometry_msgs::Twist>
{

public:

  virtual ~IMUFeedbackFilter();

  bool configure();

  bool update(const geometry_msgs::Twist & input_cmd_vel, geometry_msgs::Twist & filtered_cmd_vel);

private:

  void spin();

  void imuCallback(const sensor_msgs::Imu::ConstPtr & imu);

  ros::NodeHandle node_handle_;

  ros::Subscriber imu_sub_;

  boost::shared_ptr<boost::thread> imu_thread_;

  boost::mutex imu_lock_;

  double angular_vel_;

  ros::Time prev_imu_time_;

  bool imu_thread_should_run_;

};

} // namespace cmd_vel_filters

#endif // #ifndef IMU_FEEDBACK_FILTER_H_

