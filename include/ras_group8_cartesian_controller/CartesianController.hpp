#pragma once

#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Twist.h>

namespace ras_group8_cartesian_controller {

class CartesianController
{
public:
  CartesianController(ros::NodeHandle& node_handle,
                      std::string& linear_twist_topic,
                      std::string& motor_left_topic,
                      std::string& motor_right_topic,
                      // double wheel_radius,
                      double wheel_distance);
  virtual
    ~CartesianController();
  
  /* Updates the left and right wheel velocities */
  void
    update();
    
  void
    reset();
    
  static CartesianController
    load(ros::NodeHandle& n);

private:
  bool
    readParameters();
    
  void
    linearTwistCallback(const geometry_msgs::Twist& msg);

  /* ROS Objects
   */
  ros::NodeHandle& node_handle_;
  ros::Subscriber  linear_twist_subscriber_;
  ros::Publisher   motor_left_publisher_;
  ros::Publisher   motor_right_publisher_;
  
  /* Parameters
   */
  // const std::string linear_twist_topic_;
  // const std::string motor_left_topic_;
  // const std::string motor_right_topic_;
  
  const double wheel_distance_;
  
  /* Variables
   */
  std_msgs::Float32 motor_left_msg_;
  std_msgs::Float32 motor_right_msg_;
};

} /* namespace */