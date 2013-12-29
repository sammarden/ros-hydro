
#include <local_planner/local_planner_ros.h>

PLUGINLIB_EXPORT_CLASS(local_planner::LocalPlannerROS, nav_core::BaseLocalPlanner);

namespace local_planner
{

LocalPlannerROS::LocalPlannerROS()
{

}

LocalPlannerROS::~LocalPlannerROS()
{

}

void LocalPlannerROS::initialize(std::string name, tf::TransformListener * tf, costmap_2d::Costmap2DROS * costmap_ros)
{

  ros::NodeHandle private_node_handle("~/" + name);
  local_planner_.parseParams(private_node_handle);

  costmap_ros_ = costmap_ros;

  costmap_ros_->getRobotPose(current_pose_);

  local_planner_util_.initialize(tf, costmap_ros_->getCostmap(), costmap_ros_->getGlobalFrameID());

  local_planner_.initialise(local_planner_util_, costmap_ros_);

}

bool LocalPlannerROS::computeVelocityCommands(geometry_msgs::Twist & cmd_vel)
{

  costmap_ros_->getRobotPose(current_pose_);

  local_planner_.setRobotPose(current_pose_.getOrigin().getX(), current_pose_.getOrigin().getY(), tf::getYaw(current_pose_.getRotation()));

  local_planner_.computeVelocityCommands(cmd_vel);

  return true;

}

bool LocalPlannerROS::setPlan(const std::vector<geometry_msgs::PoseStamped> & orig_global_plan)
{
  return true;
}

bool LocalPlannerROS::isGoalReached()
{
  return false;
}

} // namespace local_planner

