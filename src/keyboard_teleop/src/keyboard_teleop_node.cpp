
#include <keyboard_teleop/keyboard_teleop_node.h>

namespace keyboard_teleop
{

KeyboardTeleopNode::KeyboardTeleopNode():
  linear_(0),
  angular_(0),
  linear_scale_(0.5),
  angular_scale_(0.5)
{
 
  ros::NodeHandle private_node_handle("~");
  param_utils::ParamHelper param_helper(private_node_handle, ros::this_node::getName());

  param_helper.getParamWithInfo("linear_scale", linear_scale_, linear_scale_);
  param_helper.getParamWithInfo("angular_scale", angular_scale_, angular_scale_);

  cmd_vel_pub_ = node_handle_.advertise<geometry_msgs::Twist>("keyboard_teleop/cmd_vel", 1);

}

void KeyboardTeleopNode::keyLoop()
{

  ROS_INFO("Initialising SDL.");

  if (SDLOnInit() == false)
  {
    exit(-1);
  }

  ROS_INFO("SDL Initialised.");

  ros::Rate r(10);

  SDL_Event event;

  while (ros::ok() && running_)
  {
    while (SDL_PollEvent(&event))
    {
    }

    SDLOnLoop();
    r.sleep();
  }

}

bool KeyboardTeleopNode::SDLOnInit()
{

  if (SDL_Init(SDL_INIT_EVERYTHING) < 0)
  {
    return false;
  }

  if ((surf_display_ = SDL_SetVideoMode(128, 72, 32, SDL_OPENGL)) == NULL)
  {
    return false;
  }

  return true;

}

void KeyboardTeleopNode::SDLOnLoop()
{

  Uint8 * key_state = SDL_GetKeyState(NULL);

  linear_ = 0;
  angular_ = 0;

  if (key_state['w'])
  {
    linear_ = 1.0;
  }
  if (key_state['a'])
  {
    angular_ = 1.0;
  }
  if (key_state['s'])
  {
    linear_ = -1.0;
  }
  if (key_state['d'])
  {
    angular_ = -1.0;
  }
  if (key_state[SDLK_ESCAPE])
  {
    running_ = false;
  }

  geometry_msgs::Twist msg;
  msg.linear.x = linear_scale_*linear_;
  msg.angular.z = angular_scale_*angular_;

  cmd_vel_pub_.publish(msg);

}
 
}

int main(int argc, char ** argv)
{

  ros::init(argc, argv, "keyboard_teleop_node");

  keyboard_teleop::KeyboardTeleopNode keyboard_teleop_node;

  keyboard_teleop_node.keyLoop();

  return 0;

}

