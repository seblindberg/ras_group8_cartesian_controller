#pragma once

#include <ros/ros.h>
#include <termios.h>
#include <geometry_msgs/Twist.h>

namespace ras_group8_cartesian_controller {
  
#define KEYCODE_R 0x43
#define KEYCODE_L 0x44
#define KEYCODE_U 0x41
#define KEYCODE_D 0x42
#define KEYCODE_Q 0x71
#define KEYCODE_0 48

class Teleop
{
public:
  static Teleop& getInstance()
  {
    static ros::NodeHandle n("~");
    static Teleop instance = load(n);
    return instance;
  }
  
  
  virtual ~Teleop();
  
  void spin();
  
  void restore();
  
  static void
    quit(int sig);

private:
  Teleop(ros::NodeHandle& node_handle, int kfd,
         double linear_velocity_step,
         double angular_velocity_step,
         const std::string& twist_topic);
  
  Teleop(Teleop const&);         // Don't Implement
  void operator=(Teleop const&); // Don't implement
  
  static Teleop
    load(ros::NodeHandle& n);  

  /* ROS Objects
   */
  ros::NodeHandle& node_handle_;
  
  ros::Publisher   twist_publisher_;

  /* Parameters
   */
  const double linear_velocity_step_;
  const double angular_velocity_step_;

  /* Variables
   */
  geometry_msgs::Twist twist_msg_;
  struct termios terminal_raw_;
  struct termios terminal_cooked_;
  
  int kfd_;
};

} /* namespace */