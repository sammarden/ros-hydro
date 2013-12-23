
#include <robot_state_estimation/base_robot_state_estimator.h>

namespace robot_state_estimation
{

BaseRobotStateEstimator::BaseRobotStateEstimator() : map_frame_("map"), robot_base_frame_("base_footprint"), tf_rate_(100.0), tf_x_(0.0), tf_y_(0.0), tf_z_(0.0), tf_roll_(0.0), tf_pitch_(0.0), tf_yaw_(0.0), tf_looping_(false)
{

}

BaseRobotStateEstimator::~BaseRobotStateEstimator()
{
  if (tf_looping_)
  {
    stopTFLoop();
  }
}

void BaseRobotStateEstimator::startTFLoop()
{
  tf_looping_ = true;
  tf_loop_thread_ptr_ = boost::shared_ptr<boost::thread>(new boost::thread(boost::bind(&BaseRobotStateEstimator::tfLoop, this)));
}

void BaseRobotStateEstimator::stopTFLoop()
{
  tf_looping_ = false;
  tf_loop_thread_ptr_->join();
}

void BaseRobotStateEstimator::setTFState(double x, double y, double z, double roll, double pitch, double yaw)
{

  tf_state_lock_.lock();

  tf_x_ = x;
  tf_y_ = y;
  tf_z_ = z;
  tf_roll_ = roll;
  tf_pitch_ = pitch;
  tf_yaw_ = yaw;

  tf_state_lock_.unlock();

}

void BaseRobotStateEstimator::tfLoop()
{

  while (tf_looping_)
  {

    broadcastTransform();

    ros::Rate(tf_rate_).sleep();

  }

}

void BaseRobotStateEstimator::broadcastTransform()
{

  tf::Transform transform;
  tf::Quaternion tf_quat;

  tf_state_lock_.lock();
  transform.setOrigin(tf::Vector3(tf_x_, tf_y_, tf_z_));
  tf_quat.setRPY(tf_roll_, tf_pitch_, tf_yaw_);
  transform.setRotation(tf_quat);
  tf_state_lock_.unlock();

  tf_broadcaster_.sendTransform(tf::StampedTransform(transform, ros::Time::now(), map_frame_, robot_base_frame_));

}

} // namespace robot_state_estimation

