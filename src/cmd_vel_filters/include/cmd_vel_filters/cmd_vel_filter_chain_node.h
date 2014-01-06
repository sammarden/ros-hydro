
#ifndef CMD_VEL_FILTER_CHAIN_NODE_H_
#define CMD_VEL_FILTER_CHAIN_NODE_H_

#include <ros/ros.h>

#include <geometry_msgs/Twist.h>

#include <filters/filter_chain.h>

#include <param_utils/param_utils.h>

namespace cmd_vel_filters
{

class CmdVelFilterChainNode
{

public:

  CmdVelFilterChainNode();

  void spin();

private:

  void cmdVelCallback(const geometry_msgs::Twist::ConstPtr & cmd_vel);

  ros::NodeHandle node_handle_;

  ros::Subscriber input_cmd_vel_sub_;

  ros::Publisher filtered_cmd_vel_pub_;

  filters::FilterChain<geometry_msgs::Twist> cmd_vel_filter_chain_;

};

} // namespace cmd_vel_filters

#endif // #ifndef CMD_VEL_FILTER_CHAIN_NODE_H_

