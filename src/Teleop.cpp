#include <ras_group8_cartesian_controller/Teleop.hpp>

// STD
#include <string>
#include <stdio.h>
#include <signal.h>

namespace ras_group8_cartesian_controller {

Teleop::Teleop(ros::NodeHandle& node_handle, int kfd,
               double linear_velocity_step,
               double angular_velocity_step,
               const std::string& twist_topic)
    : node_handle_(node_handle),
      linear_velocity_step_(linear_velocity_step),
      angular_velocity_step_(linear_velocity_step),
      kfd_(kfd)
{
  twist_publisher_ =
    node_handle_.advertise<geometry_msgs::Twist>(twist_topic, 1);

  ROS_INFO("Successfully launched node.");
}

Teleop::~Teleop()
{
  restore(); /* TODO: Verify that this needs to be here */
}

/* Quit
 * Static method gets called when a SIGINT is issued by the user.
 */
void Teleop::quit(int sig)
{
  getInstance().restore(); /* TODO: Verify that this needs to be here */
  ros::shutdown();
  exit(0);
}

/* Restore
 * Restore the terminal to its initial state.
 */
void Teleop::restore()
{
  tcsetattr(kfd_, TCSANOW, &terminal_cooked_);
}

/* Spin
 * Loop until the user sends a SIGINT to the process.
 */
void Teleop::spin()
{
  char c;
  bool dirty = false;
    
  /* Save the current terminal state and initialize a new, raw profile */
  tcgetattr(kfd_, &terminal_cooked_);
  std::memcpy(&terminal_raw_, &terminal_cooked_, sizeof(struct termios));
  
  terminal_raw_.c_lflag &= ~(ICANON | ECHO);
  terminal_raw_.c_cc[VEOL] = 1;
  terminal_raw_.c_cc[VEOF] = 2;
  
  tcsetattr(kfd_, TCSANOW, &terminal_raw_);
  
  signal(SIGINT, &Teleop::quit); /* TODO: Verify that this needs to be here */
  
  while (node_handle_.ok()) {
    if (read(kfd_, &c, 1) < 0)
    {
      ROS_ERROR("failed to read");
      exit(-1);
    }
    
    switch (c) {
      /* Increase the linear velocity */
      case KEYCODE_U:
        twist_msg_.linear.x += linear_velocity_step_;
        dirty = true;
        break;
        
      /* Increase the linear velocity */
      case KEYCODE_D:
        twist_msg_.linear.x -= linear_velocity_step_;
        dirty = true;
        break;
      
      /* Increase the angular velocity */
      case KEYCODE_R:
        twist_msg_.angular.z -= angular_velocity_step_;
        dirty = true;
        break;
        
      /* Decrease the angular velocity */
      case KEYCODE_L:
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
      ROS_INFO("^ %2.2f, < %2.2f", twist_msg_.linear.x, twist_msg_.angular.z);
      dirty = false;
    }
  }
  
  restore(); /* TODO: Verify that this needs to be here */
}

/* Load
 * Private static method that initializes a new instance using parameters loaded
 * from the parameter server.
 */
Teleop Teleop::load(ros::NodeHandle& n)
{
  int kfd;
  double linear_velocity_step;
  double angular_velocity_step;
  std::string twist_topic;
  
  /* Load params with defaults */
  kfd = n.param("kfd", 0);
  linear_velocity_step = n.param("linear_velocity_step", 0.05);
  angular_velocity_step = n.param("angular_velocity_step", 0.05);
  
  /* Load required params */
  if (!n.getParam("twist_topic", twist_topic))
    exit(-1);
  
  Teleop teleop(n, kfd, linear_velocity_step,
                angular_velocity_step, twist_topic);
  
  return teleop;
}


} /* namespace */