
#ifndef WAYPOINT_PARSER_H_
#define WAYPOINT_PARSER_H_

#include <ros/ros.h>

#include <nav_msgs/Odometry.h>

namespace param_utils
{

class WaypointParser
{

public:

  static bool getWaypoints(XmlRpc::XmlRpcValue & xml_waypoints, std::vector<nav_msgs::Odometry> & waypoints);

};

} // namespace param_utils

#endif // ifndef WAYPOINT_PARSER_H_

