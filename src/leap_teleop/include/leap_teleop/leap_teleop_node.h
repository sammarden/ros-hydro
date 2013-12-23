
#ifndef LEAP_TELEOP_NODE_H_
#define LEAP_TELEOP_NODE_H_

#include <ros/ros.h>

#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Twist.h>

namespace leap_teleop
{

/** \brief Implements teleoperation using a Leap Motion device.
  *
  * Authors: Sam Marden
  */
class LeapTeleopNode
{

public:

  /** \brief Default constructor. Parses params and subscribes to and advertises topics. */
  LeapTeleopNode();

  /** \brief Destructor. */
  ~LeapTeleopNode();

  /** \brief Spins the node. */
  void spin();

private:

  /** \brief Callback for hands detected by Leap.
    *
    * \param hands PoseArray of detected hands.
    */
  void handsCallback(const geometry_msgs::PoseArray::ConstPtr & hands);

  /** \brief ROS node handle. */
  ros::NodeHandle node_handle_;
  
  /** \brief Subcriber for hands detected by Leap. */
  ros::Subscriber hands_sub_;

  /** \brief Publisher for drive command. */
  ros::Publisher cmd_vel_pub_;

}

} // namespace leap_teleop

#endif // #ifndef LEAP_TELEOP_NODE_H_

