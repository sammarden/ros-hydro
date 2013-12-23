
#include <ros/ros.h>

#include <nav_msgs/Path.h>

int main(int argc, char ** argv)
{

  ros::init(argc, argv, "test_trajectory_node");

  ros::NodeHandle node_handle;

  ros::Publisher traj_pub = node_handle.advertise<nav_msgs::Path>("traj", 1);

  nav_msgs::Path traj;

  geometry_msgs::PoseStamped pose_1, pose_2;

  pose_1.pose.position.x = 0.0;
  pose_1.pose.position.y = 10.0;

  pose_2.pose.position.x = 10.0;
  pose_2.pose.position.y = 0.0;

  traj.poses.push_back(pose_1);
  traj.poses.push_back(pose_2);

  while (ros::ok())
  {
    ros::Duration(1).sleep();
    traj_pub.publish(traj);
  }

  return 0;

}

