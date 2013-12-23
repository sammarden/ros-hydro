
#ifndef EKF_2D_NODE_H_
#define EKF_2D_NODE_H_

#include <ros/ros.h>

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>

#include <tf/transform_broadcaster.h>

#include <Eigen/Dense>

#include <param_utils/param_utils.h>

namespace robot_state_estimation
{

class EKF2DNode
{

public:

  EKF2DNode();

  void spin();

private:

  void odomCallback(const nav_msgs::Odometry::ConstPtr & odom);

  void imuCallback(const sensor_msgs::Imu::ConstPtr & imu);

  void gpsCallback(const nav_msgs::Odometry::ConstPtr & gps);

  void predict();

  void broadcastTransforms();

  void publishState();

  ros::NodeHandle node_handle_;

  ros::Subscriber odom_sub_;

  ros::Subscriber imu_sub_;

  ros::Subscriber gps_sub_;

  ros::Publisher state_pub_;

  tf::TransformBroadcaster tf_broadcaster_;

  Eigen::Vector3d state_;

  Eigen::Matrix3d covariance_;

  std::string map_frame_;

  std::string odom_frame_;

  std::string robot_base_frame_;

  double linear_vel_;

  double angular_vel_;

  double dt_;

  ros::Time prev_predict_time_;

  ros::Time prev_odom_time_;

  bool predicted_once_;

};

}

#endif // #ifndef EKF_2D_NODE_H_

