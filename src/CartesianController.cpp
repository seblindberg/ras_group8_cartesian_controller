#include <ras_group8_cartesian_controller/CartesianController.hpp>

// STD
#include <string>

namespace ras_group8_cartesian_controller {

CartesianController::CartesianController(ros::NodeHandle& node_handle)
    : node_handle_(node_handle)
{
  if (!readParameters()) {
    ROS_ERROR("Could not read parameters.");
    ros::requestShutdown();
  }
  
  linear_twist_subscriber_
    = node_handle_.subscribe(linear_twist_topic_, 1,
                            &CartesianController::linearTwistCallback, this);
  
  motor_left_publisher_
    = node_handle_.advertise<std_msgs::Float32>(motor_left_topic_, 1);
    
  motor_right_publisher_
    = node_handle_.advertise<std_msgs::Float32>(motor_right_topic_, 1);
    
  reset();

  ROS_INFO("Successfully launched node.");
}

CartesianController::~CartesianController()
{
}

/* Publish motor velocities to the motor controller topics.
 */
void CartesianController::update()
{
  motor_left_publisher_.publish(motor_left_msg_);
  motor_right_publisher_.publish(motor_right_msg_);
}

void CartesianController::reset()
{
  motor_left_msg_.data = 0.0;
  motor_right_msg_.data = 0.0;
}

bool CartesianController::readParameters()
{
  if (!node_handle_.getParam("linear_twist_topic", linear_twist_topic_))
    return false;
  if (!node_handle_.getParam("left_motor_topic", motor_left_topic_))
    return false;
  if (!node_handle_.getParam("right_motor_topic", motor_right_topic_))
    return false;
  if (!node_handle_.getParam("/platform/wheel_radius", wheel_radius_))
    return false;
  if (!node_handle_.getParam("/platform/wheel_distance", wheel_distance_))
    return false;
  return true;
}

/* Transform the linear and angular twist velocities into wheel velocities
 */
void CartesianController::linearTwistCallback(const geometry_msgs::Twist& msg)
{
  double v = msg.linear.x;
  double w = msg.angular.z;
  double c = wheel_distance_ / 2.0 * w;
  
  motor_left_msg_.data = (v - c) / wheel_radius_;
  motor_right_msg_.data = (v + c) / wheel_radius_;
}


} /* namespace */