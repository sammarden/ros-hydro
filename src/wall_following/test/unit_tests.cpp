
#include <wall_following/nearest_scan_cost_function.h>

#include <gtest/gtest.h>

TEST(NearestScanCostFunction, prepare)
{

  wall_following::NearestScanCostFunction nearest_scan_cost_function;

  EXPECT_FALSE(nearest_scan_cost_function.prepare());

  nearest_scan_cost_function.setTargetDistanceToScan(1.0);
  EXPECT_FALSE(nearest_scan_cost_function.prepare());

  nearest_scan_cost_function.setMaxDistanceToScan(1.0);
  nearest_scan_cost_function.setServiceTimeout(0.0);
  EXPECT_FALSE(nearest_scan_cost_function.prepare());

}

TEST(NearestScanCostFunction, minDistanceToPointCloud)
{

  wall_following::NearestScanCostFunction nearest_scan_cost_function;
  nearest_scan_cost_function.setMaxDistanceToScan(1.0);

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>());

  pcl::PointXYZ point_1;
  point_1.x = 0.0;
  point_1.y = 0.0;
  point_1.z = 0.0;

  pcl::PointXYZ point_2;
  point_2.x = 1.0;
  point_2.y = 1.0;
  point_2.z = 0.0;

  pcl::PointXYZ point_3;
  point_3.x = -10.0;
  point_3.y = 0.0;
  point_3.z = 0.0;

  cloud_ptr->points.push_back(point_1);
  cloud_ptr->points.push_back(point_2);

  nearest_scan_cost_function.initKdTree(cloud_ptr);

  double dist_1 = nearest_scan_cost_function.minDistanceToPointCloud(point_1.x, point_1.y);
  EXPECT_FLOAT_EQ(dist_1, 0.0);

  double dist_2 = nearest_scan_cost_function.minDistanceToPointCloud(point_2.x, point_2.y);
  EXPECT_FLOAT_EQ(dist_2, 0.0);

  double dist_3 = nearest_scan_cost_function.minDistanceToPointCloud(point_3.x, point_3.y);
  EXPECT_FLOAT_EQ(dist_3, 10.0);

  double dist_4 = nearest_scan_cost_function.minDistanceToPointCloud(-0.5, 0.0);
  EXPECT_FLOAT_EQ(dist_4, 0.5);

}

TEST(NearestScanCostFunction, scoreDistanceToPointCloud)
{

  wall_following::NearestScanCostFunction nearest_scan_cost_function;
  nearest_scan_cost_function.setTargetDistanceToScan(1.0);

  EXPECT_FLOAT_EQ(nearest_scan_cost_function.scoreDistanceToPointCloud(1.0), 0.0);
  EXPECT_FLOAT_EQ(nearest_scan_cost_function.scoreDistanceToPointCloud(0.0), 1.0);
  EXPECT_FLOAT_EQ(nearest_scan_cost_function.scoreDistanceToPointCloud(2.0), 1.0);

}

int main(int argc, char ** argv)
{

  ros::init(argc, argv, "wall_following_unit_tests");

  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();

}
