
#include <cmd_vel_filters/cmd_vel_filter_chain_node.h>

namespace cmd_vel_filters
{

CmdVelFilterChainNode::CmdVelFilterChainNode() : cmd_vel_filter_chain_("geometry_msgs::Twist")
{

  ros::NodeHandle private_node_handle("~");

  if (!cmd_vel_filter_chain_.configure("cmd_vel_filters", private_node_handle))
  {
    ROS_WARN("[cmd_vel_filter_chain_node]: unable to configure filter chain");
  }

  param_utils::ParamHelper param_helper(private_node_handle, ros::this_node::getName());

  std::string input_cmd_vel_topic("input_cmd_vel");
  param_helper.getParamWithInfo("input_cmd_vel_topic", input_cmd_vel_topic, input_cmd_vel_topic);

  std::string filtered_cmd_vel_topic("filtered_cmd_vel");
  param_helper.getParamWithInfo("filtered_cmd_vel_topic", filtered_cmd_vel_topic, filtered_cmd_vel_topic);

  input_cmd_vel_sub_ = node_handle_.subscribe(input_cmd_vel_topic, 1, &CmdVelFilterChainNode::cmdVelCallback, this);

  filtered_cmd_vel_pub_ = node_handle_.advertise<geometry_msgs::Twist>(filtered_cmd_vel_topic, 1);

}

void CmdVelFilterChainNode::spin()
{

  while (ros::ok())
  {
    ros::spinOnce();
    ros::Rate(50.0).sleep();
  }

}

void CmdVelFilterChainNode::cmdVelCallback(const geometry_msgs::Twist::ConstPtr & cmd_vel)
{

  geometry_msgs::Twist filtered_cmd_vel;

  cmd_vel_filter_chain_.update(*cmd_vel, filtered_cmd_vel);

  filtered_cmd_vel_pub_.publish(filtered_cmd_vel);

}

} // namespace cmd_vel_filters

int main(int argc, char ** argv)
{

  ros::init(argc, argv, "cmd_vel_filter_chain_node");

  cmd_vel_filters::CmdVelFilterChainNode cmd_vel_filter_chain_node;

  cmd_vel_filter_chain_node.spin();

  return 0;

}

