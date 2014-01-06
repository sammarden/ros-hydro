
#ifndef NEAREST_SCAN_COST_FUNCTION_H
#define NEAREST_SCAN_COST_FUNCTION_H

#include <ros/ros.h>

#include <tf/transform_listener.h>

#include <sensor_msgs/point_cloud_conversion.h>

#include <base_local_planner/trajectory_cost_function.h>

#include <laser_assembler/AssembleScans2.h>

#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>

#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/extract_indices.h>

#include <pcl_conversions/pcl_conversions.h>

#include <boost/thread/thread.hpp>

namespace wall_following
{

enum WallFollowingSide
{
  LEFT,
  RIGHT,
  NONE
};

class NearestScanCostFunction : public base_local_planner::TrajectoryCostFunction
{

public:

  NearestScanCostFunction();

  virtual ~NearestScanCostFunction();

  bool prepare();

  double scoreTrajectory(base_local_planner::Trajectory & traj);

  inline void setTargetDistanceToScan(double target_distance_to_scan)
  {
    target_distance_to_scan_ = target_distance_to_scan;
  }

  inline void setMaxDistanceToScan(double max_distance_to_scan)
  {
    max_distance_to_scan_ = max_distance_to_scan;
  }

  inline void setServiceTimeout(double service_timeout) { service_timeout_ = service_timeout; }

  inline void setSide(WallFollowingSide side) { side_ = side; }

  double minDistanceToPointCloud(double x, double y);

  double scoreDistanceToPointCloud(double dist);

  void initKdTree(const pcl::PointCloud<pcl::PointXYZ>::Ptr & pcl_cloud_ptr)
  {
    kd_tree_.setInputCloud(pcl_cloud_ptr);
  }

  void pointsOnOneSide(const sensor_msgs::PointCloud2 & ros_cloud, pcl::PointCloud<pcl::PointXYZ> & one_side_cloud);


private:

  bool init();

  bool assembleScansIntoPCLCloud(pcl::PointCloud<pcl::PointXYZ> & pcl_cloud);

  ros::NodeHandle node_handle_;

  ros::Publisher cloud_pub_;

  ros::ServiceClient assemble_scans_client_;

  tf::TransformListener tf_listener_;

  pcl::KdTreeFLANN<pcl::PointXYZ> kd_tree_;

  double target_distance_to_scan_;
  
  double max_distance_to_scan_; 

  WallFollowingSide side_;

  double service_timeout_;

  bool initialised_;

};

} // namespace wall_following

#endif //  #ifndef NEAREST_SCAN_COST_FUNCTION_H

