
#include <robot_state_estimation/ekf_2d_node.h>

namespace robot_state_estimation
{

EKF2DNode::EKF2DNode() : map_frame_("/map"), odom_frame_("/odom"), robot_base_frame_("base_footprint"), linear_vel_(0.0), angular_vel_(0.0), predicted_once_(false)
{

  ros::NodeHandle private_node_handle("~");
  param_utils::ParamHelper param_helper(private_node_handle, ros::this_node::getName());

  param_helper.getParamWithInfo("map_frame", map_frame_, map_frame_);
  param_helper.getParamWithInfo("odom_frame", odom_frame_, odom_frame_);
  param_helper.getParamWithInfo("robot_base_frame", robot_base_frame_, robot_base_frame_);

  std::string odom_topic("odom");
  param_helper.getParamWithInfo("odom_topic", odom_topic, odom_topic);

  odom_sub_ = node_handle_.subscribe(odom_topic, 100, &EKF2DNode::odomCallback, this);

  std::string imu_topic("imu");
  param_helper.getParamWithInfo("imu_topic", imu_topic, imu_topic);

  imu_sub_ = node_handle_.subscribe(imu_topic, 100, &EKF2DNode::imuCallback, this);

  std::string gps_topic("gps");
  param_helper.getParamWithInfo("gps_topic", gps_topic, gps_topic);

  gps_sub_ = node_handle_.subscribe(gps_topic, 100, &EKF2DNode::gpsCallback, this);

  std::string state_topic("state");
  param_helper.getParamWithInfo("state_topic", state_topic, state_topic);

  state_pub_ = node_handle_.advertise<nav_msgs::Odometry>(state_topic, 100);

  state_ << 0.0,
            0.0,
            0.0;

}

void EKF2DNode::spin()
{

  while (ros::ok())
  {

    ros::spinOnce();

    broadcastTransforms();
    publishState();

    ros::Rate(50.0).sleep();

  }

}

void EKF2DNode::odomCallback(const nav_msgs::Odometry::ConstPtr & odom)
{

  if (!predicted_once_)
  {
    prev_predict_time_ = odom->header.stamp;
    predicted_once_ = true;
  }

  linear_vel_ = odom->twist.twist.linear.x;

  dt_ = (odom->header.stamp - prev_predict_time_).toSec();

  predict();

  prev_predict_time_ = odom->header.stamp;

}

void EKF2DNode::imuCallback(const sensor_msgs::Imu::ConstPtr & imu)
{

  if (!predicted_once_)
  {
    prev_predict_time_ = imu->header.stamp;
    predicted_once_ = true;
  }

  angular_vel_ = imu->angular_velocity.z;

  dt_ = (imu->header.stamp - prev_predict_time_).toSec();

  predict();

  prev_predict_time_ = imu->header.stamp;

}

void EKF2DNode::gpsCallback(const nav_msgs::Odometry::ConstPtr & gps)
{

}

void EKF2DNode::predict()
{

  Eigen::Vector3d delta_state;
  delta_state << linear_vel_*cos(state_(2))*dt_,
                 linear_vel_*sin(state_(2))*dt_,
                 angular_vel_*dt_;

  state_ = state_ + delta_state;

}

void EKF2DNode::broadcastTransforms()
{

  tf::Transform map_to_odom_transform;

  map_to_odom_transform.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
  
  tf::Quaternion map_to_odom_quat;
  map_to_odom_quat.setRPY(0.0, 0.0, 0.0);
  map_to_odom_transform.setRotation(map_to_odom_quat);

  tf_broadcaster_.sendTransform(tf::StampedTransform(map_to_odom_transform, prev_predict_time_, map_frame_, odom_frame_));

  tf::Transform odom_to_base_link_transform;

  odom_to_base_link_transform.setOrigin(tf::Vector3(state_(0), state_(1), 0.0));
  
  tf::Quaternion odom_to_base_link_quat;
  odom_to_base_link_quat.setRPY(0.0, 0.0, state_(2));
  odom_to_base_link_transform.setRotation(odom_to_base_link_quat);

  tf_broadcaster_.sendTransform(tf::StampedTransform(odom_to_base_link_transform, prev_predict_time_, odom_frame_, robot_base_frame_));

}

void EKF2DNode::publishState()
{

  nav_msgs::Odometry state;

  state.header.stamp = prev_predict_time_;
  state.header.frame_id = map_frame_;
  state.child_frame_id = robot_base_frame_;

  state.pose.pose.position.x = state_(0);
  state.pose.pose.position.y = state_(1);
  state.pose.pose.orientation = tf::createQuaternionMsgFromYaw(state_(2));

  state.twist.twist.linear.x = linear_vel_;
  state.twist.twist.angular.z = angular_vel_;

  state_pub_.publish(state);

}

} // namespace robot_state_estimation

int main(int argc, char ** argv)
{

  ros::init(argc, argv, "ekf_2d_node");

  robot_state_estimation::EKF2DNode ekf_2d_node;

  ekf_2d_node.spin();

  return 0;

}

