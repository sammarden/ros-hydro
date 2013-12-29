
#ifndef LOCAL_PLANNER_ROS_H_
#define LOCAL_PLANNER_ROS_H_

#include <nav_core/base_local_planner.h>

#include <pluginlib/class_list_macros.h>

#include <local_planner/local_planner.h>

namespace local_planner
{

class LocalPlannerROS : public nav_core::BaseLocalPlanner
{

public:

  LocalPlannerROS();

  ~LocalPlannerROS();

  void initialize(std::string name, tf::TransformListener * tf, costmap_2d::Costmap2DROS * costmap_ros);

  bool computeVelocityCommands(geometry_msgs::Twist & cmd_vel);

  bool setPlan(const std::vector<geometry_msgs::PoseStamped> & orig_global_plan);

  bool isGoalReached();

private:

  tf::TransformListener * tf_;

  base_local_planner::LocalPlannerUtil local_planner_util_;

  costmap_2d::Costmap2DROS * costmap_ros_;

  tf::Stamped<tf::Pose> current_pose_;

  local_planner::LocalPlanner local_planner_;

};

}

#endif // $ifndef LOCAL_PLANNER_ROS_H_

