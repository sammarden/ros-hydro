
#include <param_utils/waypoint_parser.h>

namespace param_utils
{

bool WaypointParser::getWaypoints(XmlRpc::XmlRpcValue & xml_waypoints, std::vector<nav_msgs::Odometry> & waypoints)
{

  if (!xml_waypoints.hasMember("frame_id"))
  {
    ROS_ERROR("[param_utils::WaypointParser]: missing <frame_id>");
    return false;
  }

  std::string frame_id = xml_waypoints["frame_id"];

  if (!xml_waypoints.hasMember("waypoint_list"))
  {
    ROS_ERROR("[param_utils::WaypointParser]: missing <waypoint_list>");
    return false;
  }

  XmlRpc::XmlRpcValue xml_waypoint_list = xml_waypoints["waypoint_list"];

  // Make sure the waypoints are an array.
  if (xml_waypoint_list.getType() != XmlRpc::XmlRpcValue::TypeArray)
  {
    ROS_ERROR("[param_utils::WaypointParser]: <waypoints> not an array");
    return false;
  }

  for (int i = 0; i < xml_waypoint_list.size(); ++i)
  {

    XmlRpc::XmlRpcValue xml_waypoint = xml_waypoint_list[i];

    if (xml_waypoint.getType() != XmlRpc::XmlRpcValue::TypeStruct)
    {
      ROS_ERROR("[param_utils::WaypointParser]: waypoint %d must be struct containing x/y or easting/northing", i);
      return false;
    }

    // Make sure we have only x and y or easting and northing.
    bool has_x = xml_waypoint.hasMember("x");
    bool has_y = xml_waypoint.hasMember("y");
    bool has_e = xml_waypoint.hasMember("easting");
    bool has_n = xml_waypoint.hasMember("northing");

    if (!(has_x && has_y))
    {
      ROS_ERROR("[param_utils::WaypointParser]: waypoint %d must have x/y", i);
      return false; 
    }

    if (has_e || has_n)
    {
      ROS_ERROR("[param_utils::WaypointParser]: waypoint %d has both x/y and easting/northing", i);
      return false;
    }

    nav_msgs::Odometry waypoint;
    waypoint.header.frame_id = frame_id;
    waypoint.pose.pose.position.x = xml_waypoint["x"];
    waypoint.pose.pose.position.y = xml_waypoint["y"];

    waypoints.push_back(waypoint);

  }
}

} // namespace param_utils

