#include <ras_group8_cartesian_controller/Teleop.hpp>

// STD
#include <string>
#include <stdio.h>
#include <signal.h>

namespace ras_group8_cartesian_controller {

/* Setup global exit mechanism. There can only ever be on Teleop node at a time. */

static Teleop *instance;
static void quit(int sig)
{
  instance->restore();
  ros::shutdown();
  exit(0);
}

Teleop::Teleop(ros::NodeHandle& node_handle, int kfd)
    : node_handle_(node_handle)
{
  if (!readParameters()) {
    ROS_ERROR("Could not read parameters.");
    ros::requestShutdown();
    exit(0);
  }
    
  kfd_ = kfd;
  
  /* Setup publisher */
  twist_publisher_ =
    node_handle_.advertise<geometry_msgs::Twist>(twist_topic_, 1);

  ROS_INFO("Successfully launched node.");
}

Teleop::~Teleop()
{
}

void Teleop::restore()
{
  tcsetattr(kfd_, TCSANOW, &terminal_cooked_);
}

void Teleop::spin()
{
  char c;
  bool dirty = false;
  
  //ROS_ASSERT(NULL == instance);
  instance = this;
  
  signal(SIGINT, quit);
  
  /* Save the current terminal state and initialize a new, raw profile */
  tcgetattr(kfd_, &terminal_cooked_);
  std::memcpy(&terminal_raw_, &terminal_cooked_, sizeof(struct termios));
  
  terminal_raw_.c_lflag &= ~(ICANON | ECHO);
  terminal_raw_.c_cc[VEOL] = 1;
  terminal_raw_.c_cc[VEOF] = 2;
  
  tcsetattr(kfd_, TCSANOW, &terminal_raw_);
  
  for (;;) {
    if (read(kfd_, &c, 1) < 0)
    {
      ROS_ERROR("failed to read");
      exit(-1);
    }
    
    switch (c) {
      /* Increase the linear velocity */
      case KEYCODE_U:
        ROS_INFO("UP");
        twist_msg_.linear.x += linear_velocity_step_;
        dirty = true;
        break;
        
      /* Increase the linear velocity */
      case KEYCODE_D:
        ROS_INFO("DOWN");
        twist_msg_.linear.x -= linear_velocity_step_;
        dirty = true;
        break;
      
      /* Increase the angular velocity */
      case KEYCODE_R:
        ROS_INFO("RIGHT");
        twist_msg_.angular.z -= angular_velocity_step_;
        dirty = true;
        break;
        
      /* Decrease the angular velocity */
      case KEYCODE_L:
        ROS_INFO("LEFT");
        twist_msg_.angular.z += angular_velocity_step_;
        dirty = true;
        break;
      
      /* Reset */
      case KEYCODE_0:
        ROS_INFO("Resetting");
        twist_msg_.linear.x  = 0;
        twist_msg_.angular.z = 0;
        dirty = true;
        break;
    }
    
    if (dirty) {
      twist_publisher_.publish(twist_msg_);
      dirty = false;
    }
  }
}

bool Teleop::readParameters()
{
  if (!node_handle_.getParam("linear_velocity_step", linear_velocity_step_))
    return false;
  if (!node_handle_.getParam("angular_velocity_step", angular_velocity_step_))
    return false;
  if (!node_handle_.getParam("twist_topic", twist_topic_))
    return false;

  return true;
}


} /* namespace */