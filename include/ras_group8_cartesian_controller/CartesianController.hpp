#pragma once

#include <ros/ros.h>
#include <phidgets/motor_encoder.h>

namespace ras_group8_cartesian_controller {

class CartesianController
{
public:
  CartesianController(ros::NodeHandle& nodeHandle);
  virtual ~CartesianController();

private:
  bool readParameters();
  void topicCallback(const phidgets::motor_encoder& msg);

  /* ROS Objects
   */
  ros::NodeHandle& nodeHandle_;
  ros::Subscriber subscriber_;
  
  /* Parameters
   */
  std::string subscriberTopic_;
};

} /* namespace */