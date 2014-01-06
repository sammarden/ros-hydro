
#ifndef KEYBOARD_TELEOP_NODE_H_
#define KEYBOARD_TELEOP_NODE_H_

#include <ros/ros.h>

#include <geometry_msgs/Twist.h>

#include <SDL/SDL.h>

#include <param_utils/param_utils.h>

namespace keyboard_teleop
{

class KeyboardTeleopNode
{

public:
  
  KeyboardTeleopNode();
  void keyLoop();

  bool SDLOnInit();
  void SDLOnLoop();

private:
  
  ros::NodeHandle node_handle_;
  ros::Publisher cmd_vel_pub_;

  SDL_Surface * surf_display_;

  bool running_;

  double linear_;
  double angular_;
  double linear_scale_;
  double angular_scale_;

};

}

#endif

