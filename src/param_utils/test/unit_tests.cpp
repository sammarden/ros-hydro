
#include <gtest/gtest.h>

#include <param_utils/waypoint_parser.h>

TEST(WaypointParser, getWaypoints)
{

  ros::NodeHandle private_node_handle("~");

  XmlRpc::XmlRpcValue xml_waypoints;
  private_node_handle.getParam("valid_waypoints", xml_waypoints);

  std::vector<nav_msgs::Odometry> waypoints;
  EXPECT_TRUE(param_utils::WaypointParser::getWaypoints(xml_waypoints, waypoints));

  EXPECT_FLOAT_EQ(waypoints[0].pose.pose.position.x, 1.0);
  EXPECT_FLOAT_EQ(waypoints[0].pose.pose.position.y, -1.0);
  EXPECT_FLOAT_EQ(waypoints[1].pose.pose.position.x, 2.0);
  EXPECT_FLOAT_EQ(waypoints[1].pose.pose.position.y, -2.0);

  private_node_handle.getParam("invalid_waypoints_1", xml_waypoints);
  EXPECT_FALSE(param_utils::WaypointParser::getWaypoints(xml_waypoints, waypoints));

  private_node_handle.getParam("invalid_waypoints_2", xml_waypoints);
  EXPECT_FALSE(param_utils::WaypointParser::getWaypoints(xml_waypoints, waypoints));

}

int main(int argc, char ** argv)
{

  testing::InitGoogleTest(&argc, argv);

  ros::init(argc, argv, "param_utils_unit_tests");

  return RUN_ALL_TESTS();

}
