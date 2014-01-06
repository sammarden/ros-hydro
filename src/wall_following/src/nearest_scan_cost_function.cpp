
#include <wall_following/nearest_scan_cost_function.h>

namespace wall_following
{

NearestScanCostFunction::NearestScanCostFunction() : target_distance_to_scan_(-1.0), max_distance_to_scan_(-1.0), side_(NONE), service_timeout_(5.0), initialised_(false)
{

}

NearestScanCostFunction::~NearestScanCostFunction()
{

}

bool NearestScanCostFunction::prepare()
{
  
  if (!initialised_)
  {
    if (!init())
    {
      return false;
    }
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>());
  if (!assembleScansIntoPCLCloud(*pcl_cloud_ptr))
  {
    return false;
  }

  initKdTree(pcl_cloud_ptr);

  return true;

}

double NearestScanCostFunction::scoreTrajectory(base_local_planner::Trajectory & traj)
{
 
  double score = 0.0;
  int valid_points = 0;

  for (int i = 0; i < traj.getPointsSize(); ++i)
  {

    double traj_x, traj_y, traj_th;
    traj.getPoint(i, traj_x, traj_y, traj_th);

    double min_dist_to_cloud = minDistanceToPointCloud(traj_x, traj_y);

    if (min_dist_to_cloud > 0.0 && min_dist_to_cloud < max_distance_to_scan_)
    {
      score += scoreDistanceToPointCloud(min_dist_to_cloud);
      ++valid_points;
    }

  }

  if (valid_points > 0)
  {
    return score/(double)valid_points;
  }
  else
  {
    return 0.0;
  }

}

double NearestScanCostFunction::minDistanceToPointCloud(double x, double y)
{

  pcl::PointXYZ search_point;
  search_point.x = x;
  search_point.y = y;
  search_point.z = 0.0;

  std::vector<int> nn_indices;
  std::vector<float> nn_sq_dists;

  if (kd_tree_.nearestKSearch(search_point, 1, nn_indices, nn_sq_dists) > 0)
  {
    return sqrt(nn_sq_dists[0]);
  }
  else
  {
    return -1.0;
  }

}

double NearestScanCostFunction::scoreDistanceToPointCloud(double dist)
{
  double delta_dist = fabs(dist - target_distance_to_scan_);
  return delta_dist;
}

bool NearestScanCostFunction::init()
{
  
  if (target_distance_to_scan_ < 0.0)
  {
    ROS_ERROR_NAMED("wall_following::NearestScanCostFunction", "target_distance_to_scan not set");
    return false;
  }

  if (max_distance_to_scan_ < 0.0)
  {
    ROS_ERROR_NAMED("wall_following::NearestScanCostFunction", "max_distance_to_scan not set");
    return false;
  }

  if (side_ == NONE)
  {
    ROS_ERROR_NAMED("wall_following::NearestScanCostFunction", "side not set");
  }

  if (!ros::service::waitForService("assemble_scans2", ros::Duration(service_timeout_)))
  {
    ROS_ERROR_NAMED("wall_following::NearestScanCostFunction", "assemble_scans2 service not available");
    return false;
  }

  assemble_scans_client_ = node_handle_.serviceClient<laser_assembler::AssembleScans2>("assemble_scans2");
  ROS_INFO_NAMED("wall_following::NearestScanCostFunction", "assemble_scans2 service available");

  cloud_pub_ = node_handle_.advertise<pcl::PointCloud<pcl::PointXYZ> >("nearest_scan/cloud", 1);

  initialised_ = true;

  return true;

}

bool NearestScanCostFunction::assembleScansIntoPCLCloud(pcl::PointCloud<pcl::PointXYZ> & pcl_cloud)
{

  laser_assembler::AssembleScans2 assemble_scans_2_srv;

  assemble_scans_2_srv.request.end = ros::Time::now();
  assemble_scans_2_srv.request.begin = assemble_scans_2_srv.request.end - ros::Duration(10.0);

  if (assemble_scans_client_.call(assemble_scans_2_srv))
  {
    pointsOnOneSide(assemble_scans_2_srv.response.cloud, pcl_cloud);
    cloud_pub_.publish(pcl_cloud);
    return true;
  }
  else
  {
    ROS_ERROR_NAMED("wall_following::NearestScanCostFunction", "assemble_scans service call failed");
    return false;
  }

}

void NearestScanCostFunction::pointsOnOneSide(const sensor_msgs::PointCloud2 & ros_cloud, pcl::PointCloud<pcl::PointXYZ> & one_side_cloud)
{

  if (!tf_listener_.waitForTransform(ros_cloud.header.frame_id, "/base_link", ros_cloud.header.stamp, ros::Duration(0.5)))
  {
    ROS_ERROR("[NearestScanCostFunction]: unable to transform from %s to /base_link", ros_cloud.header.frame_id.c_str());
    return;
  }

  sensor_msgs::PointCloud2 local_ros_cloud;
  pcl_ros::transformPointCloud("/base_link", ros_cloud, local_ros_cloud, tf_listener_);

  pcl::PointCloud<pcl::PointXYZ> local_pcl_cloud;
  pcl::fromROSMsg(local_ros_cloud, local_pcl_cloud);

  pcl::PointIndices::Ptr indices_on_side(new pcl::PointIndices());
  for (int i = 0; i < local_pcl_cloud.points.size(); ++i)
  {
    if ((side_ == LEFT  && local_pcl_cloud.points[i].y >= 0.0) ||
        (side_ == RIGHT && local_pcl_cloud.points[i].y <= 0.0))
    {
      indices_on_side->indices.push_back(i);
    }
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::fromROSMsg(ros_cloud, *pcl_cloud_ptr);

  pcl::ExtractIndices<pcl::PointXYZ> one_side_filter;
  one_side_filter.setInputCloud(pcl_cloud_ptr);
  one_side_filter.setIndices(indices_on_side);
  one_side_filter.filter(one_side_cloud);

}

}

