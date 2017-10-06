#include <ras_group8_cartesian_controller/CartesianController.hpp>

// STD
#include <string>

namespace ras_group8_cartesian_controller {

CartesianController::CartesianController(ros::NodeHandle& nodeHandle)
    : nodeHandle_(nodeHandle)
{
  if (!readParameters()) {
    ROS_ERROR("Could not read parameters.");
    ros::requestShutdown();
  }
  
  subscriber_ = nodeHandle_.subscribe(subscriberTopic_, 1,
                                      &CartesianController::topicCallback, this);

  ROS_INFO("Successfully launched node.");
}

CartesianController::~CartesianController()
{
}

bool CartesianController::readParameters()
{
  if (!nodeHandle_.getParam("subscriber_topic", subscriberTopic_)) return false;
  return true;
}

void CartesianController::topicCallback(const phidgets::motor_encoder& msg)
{
}


} /* namespace */