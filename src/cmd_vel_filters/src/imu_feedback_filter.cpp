
#include <cmd_vel_filters/imu_feedback_filter.h>

PLUGINLIB_EXPORT_CLASS(cmd_vel_filters::IMUFeedbackFilter, filters::FilterBase<geometry_msgs::Twist>);

namespace cmd_vel_filters
{

IMUFeedbackFilter::~IMUFeedbackFilter()
{
  imu_thread_->join();
}

bool IMUFeedbackFilter::configure()
{

  std::string imu_topic;
  getParam("imu_topic", imu_topic);

  prev_angular_vel_ = 0.0;

  prev_imu_time_ = ros::Time(0, 0);

  imu_sub_ = node_handle_.subscribe(imu_topic, 1, &IMUFeedbackFilter::imuCallback, this);

  imu_thread_ = boost::shared_ptr<boost::thread>(new boost::thread(boost::bind(&IMUFeedbackFilter::spin, this)));

}

bool IMUFeedbackFilter::update(const geometry_msgs::Twist & input_cmd_vel, geometry_msgs::Twist & filtered_cmd_vel)
{

  ros::Time current_time = ros::Time::now();

  imu_lock_.lock();
  double angular_vel_error_ = input_cmd_vel.angular.z - angular_vel_;
  double dt = (current_time - prev_imu_time_).toSec();
  imu_lock_.unlock();

  filtered_cmd_vel.linear.x = input_cmd_vel.linear.x;

  if (dt < 0.1)
  {
    filtered_cmd_vel.angular.z = prev_angular_vel_ + angular_vel_error_; 
  }
  else
  {
    filtered_cmd_vel.angular.z = input_cmd_vel.angular.z;
  }

  prev_angular_vel_ = filtered_cmd_vel.angular.z;

}

void IMUFeedbackFilter::spin()
{

  while (ros::ok())
  {
    try
    {
      ros::spinOnce();
      ros::Rate(50.0).sleep();
    }
    catch (boost::thread_interrupted &)
    {
      return;
    }
  }

}

void IMUFeedbackFilter::imuCallback(const sensor_msgs::Imu::ConstPtr & imu)
{

  boost::mutex::scoped_lock l(imu_lock_);

  angular_vel_ = imu->angular_velocity.z;
  prev_imu_time_ = imu->header.stamp;

}

} // namespace cmd_vel_filters

