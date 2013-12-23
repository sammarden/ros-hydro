
#include <trajectory_tracking/pd_controller_node.h>

namespace trajectory_tracking
{

PDControllerNode::PDControllerNode() : k_p_lateral_(0.0), k_d_lateral_(0.0), k_p_angular_(0.0), k_d_angular_(0.0), valid_traj_(false)
{

  ros::NodeHandle private_node_handle("~");

  param_utils::ParamHelper param_helper(private_node_handle, ros::this_node::getName());

  std::string state_topic("state");
  param_helper.getParamWithInfo("state_topic", state_topic, state_topic);

  state_sub_ = node_handle_.subscribe(state_topic, 100, &PDControllerNode::stateCallback, this);

  std::string traj_topic("trajectory");
  param_helper.getParamWithInfo("trajectory_topic", traj_topic, traj_topic);

  traj_sub_ = node_handle_.subscribe(traj_topic, 10, &PDControllerNode::trajCallback, this);

  std::string cmd_vel_topic("cmd_vel");
  param_helper.getParamWithInfo("cmd_vel_topic", cmd_vel_topic, cmd_vel_topic);

  cmd_vel_pub_ = node_handle_.advertise<geometry_msgs::Twist>(cmd_vel_topic, 40);

  param_helper.getParamWithInfo("k_p_lateral", k_p_lateral_, k_p_lateral_);
  param_helper.getParamWithInfo("k_d_lateral", k_d_lateral_, k_d_lateral_);
  param_helper.getParamWithInfo("k_p_angular", k_p_angular_, k_p_angular_);
  param_helper.getParamWithInfo("k_d_angular", k_d_angular_, k_d_angular_);

}

void PDControllerNode::spin()
{

  while (ros::ok())
  {
  
    ros::spinOnce();

    calculateControlEffort();

    publishCmdVel();

    ros::Rate(100).sleep();

  }

}

void PDControllerNode::trajCallback(const nav_msgs::Path::ConstPtr & traj)
{

  if (traj->poses.size() != 2)
  {
    ROS_ERROR("[%s]: trajectory must be a path defined by 2 points, received %d points", ros::this_node::getName().c_str(), traj->poses.size());
    valid_traj_ = false;
  }
  else
  {

    Eigen::Vector2d point_1, point_2;
    point_1 << traj->poses[0].pose.position.x,
               traj->poses[0].pose.position.y;
    point_2 << traj->poses[1].pose.position.x,
               traj->poses[1].pose.position.y;

    Eigen::Vector2d line_origin = point_1; 

    Eigen::Vector2d line_direction = point_2 - point_1;
    line_direction.normalize();

    traj_line_ = Eigen::ParametrizedLine<double, 2>(line_origin, line_direction);

    traj_heading_ = atan2(line_direction(1), line_direction(0));

    valid_traj_ = true;

  }

}

void PDControllerNode::stateCallback(const nav_msgs::Odometry::ConstPtr & state)
{

  x_estimate_ = state->pose.pose.position.x;
  y_estimate_ = state->pose.pose.position.y;
  th_estimate_ = tf::getYaw(state->pose.pose.orientation);
  linear_vel_ = state->twist.twist.linear.x;
  angular_vel_ = state->twist.twist.angular.z;

}

void PDControllerNode::calculateControlEffort()
{

  if (valid_traj_)
  {

    double lateral_error, angular_error;
    calculateTrajErrors(lateral_error, angular_error);

    double lateral_error_derivative = linear_vel_*sin(angular_error);
    double angular_error_derivative = angular_vel_;

    ROS_INFO("%f %f %f %f", lateral_error, lateral_error_derivative, angular_error, angular_error_derivative);

    linear_control_effort_ = 1.0;

    angular_control_effort_ = k_p_lateral_*lateral_error + 
                              k_d_lateral_*lateral_error_derivative +
                              k_p_angular_*angular_error +
                              k_d_angular_*angular_error_derivative; 

  }
  else
  {
    linear_control_effort_ = 0.0;
    angular_control_effort_ = 0.0;
  }

}

void PDControllerNode::calculateTrajErrors(double & lateral_error, double & angular_error) const
{

  Eigen::Vector2d robot_pose;
  robot_pose << x_estimate_,
                y_estimate_;

  Eigen::Vector2d robot_pose_diff = robot_pose - traj_line_.origin();
  double cross_product_z = robot_pose_diff(1)*traj_line_.direction()(0) - robot_pose_diff(0)*traj_line_.direction()(1);
  double side_of_line = cross_product_z > 0 ? 1.0 : -1.0; 

  lateral_error = -traj_line_.distance(robot_pose)*side_of_line;

  ROS_INFO("%f %f", traj_heading_, th_estimate_);

  angular_error = traj_heading_ - th_estimate_;

  while (angular_error >= M_PI) { angular_error -= 2*M_PI; }
  while (angular_error < -M_PI) { angular_error += 2*M_PI; }

}

void PDControllerNode::publishCmdVel() const
{

  geometry_msgs::Twist cmd_vel;
  
  cmd_vel.linear.x = linear_control_effort_;
  cmd_vel.angular.z = angular_control_effort_;

  cmd_vel_pub_.publish(cmd_vel);

}

}

int main(int argc, char ** argv)
{

  ros::init(argc, argv, "pd_controller_node");

  trajectory_tracking::PDControllerNode pd_controller_node;

  pd_controller_node.spin();

  return 0;

}

