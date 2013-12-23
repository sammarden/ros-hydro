
#ifndef PD_CONTROLLER_NODE_H_
#define PD_CONTROLLER_NODE_H_

#include <ros/ros.h>

#include <tf/transform_datatypes.h>

#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/Imu.h>

#include <Eigen/Geometry>

#include <param_utils/param_utils.h>

namespace trajectory_tracking
{

/* \brief Implements a basic PD controller for trajectory tracking.
 *
 * Authors: Sam Marden
 */
class PDControllerNode
{

public:

  /** \brief Default constructor, parses params and sets up subscribers and publishers. */
  PDControllerNode();

  /** \brief Spins the node, calculates and publishes the control effort. */
  void spin();

private:

  /** \brief Trajectory callback.
    *
    * \param traj Trajectory message, should only consist of 2 points.
    */
  void trajCallback(const nav_msgs::Path::ConstPtr & traj);

  /** \brief State callback.
    *
    * \param state State estimate message.
    */
  void stateCallback(const nav_msgs::Odometry::ConstPtr & state);

  /** \brief Calculate the PD control effort. */
  void calculateControlEffort();

  /** \brief Calculate the trajectory errors based on the line of the trajectory.
    *
    * \param lateral_error Perpendicular distance from the line of the trajectory to the pose of the robot.
    * \param angular_error Difference in angle between the angle of the line of the trajectory and the heading of the robot.
    */
  void calculateTrajErrors(double & lateral_error, double & angular_error) const;

  /** \brief Publish the calculated control effort as a cmd_vel message. */
  void publishCmdVel() const;

  /** \brief ROS node handle. */
  ros::NodeHandle node_handle_;

  /** \brief Subscriber for the state estimate. */
  ros::Subscriber state_sub_;

  /** \brief Subscriber for the trajectory to follow. */
  ros::Subscriber traj_sub_;

  /** \brief Publisher for the command velocity. */
  ros::Publisher cmd_vel_pub_;

  /** \brief Proportional constant for lateral trajectory error. */
  double k_p_lateral_;

  /** \brief Derivative constant for lateral trajectory error. */
  double k_d_lateral_;

  /** \brief Proportional constant for angular trajectory error. */
  double k_p_angular_;

  /** \brief Derivative constant for angular trajectory error. */
  double k_d_angular_;

  /** \brief The line of the trajectory. */
  Eigen::ParametrizedLine<double, 2> traj_line_; 

  /** \brief The angle of the line of the trajectory. */
  double traj_heading_;

  /** \brief Whether or not we have received a valid trajectory. */
  bool valid_traj_;

  /** \brief The most recent x estimate. */
  double x_estimate_;

  /** \brief The most recent y estimate. */
  double y_estimate_;

  /** \brief The most recent heading estimate. */
  double th_estimate_;

  /** \brief The most recent linear velocity. */
  double linear_vel_;

  /** \brief The most recent angular velocity. */
  double angular_vel_;

  /** \brief The most recently calculated linear control effort. */
  double linear_control_effort_;

  /** \brief The most recently calculated angular control effort. */
  double angular_control_effort_;

};

} // namespace trajectory_tracking

#endif // #ifndef PD_CONTROLLER_NODE_H_

