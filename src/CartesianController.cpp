#include <ras_group8_cartesian_controller/CartesianController.hpp>

// STD
#include <string>

namespace ras_group8_cartesian_controller {

CartesianController::CartesianController(ros::NodeHandle& node_handle,
                                         std::string& linear_twist_topic,
                                         std::string& motor_left_topic,
                                         std::string& motor_right_topic,
                                         double wheel_distance)
    : node_handle_(node_handle),
      wheel_distance_(wheel_distance)
{
  linear_twist_subscriber_
    = node_handle_.subscribe(linear_twist_topic, 1,
                            &CartesianController::linearTwistCallback, this);
  
  motor_left_publisher_
    = node_handle_.advertise<std_msgs::Float32>(motor_left_topic, 1);
    
  motor_right_publisher_
    = node_handle_.advertise<std_msgs::Float32>(motor_right_topic, 1);
    
  reset();

  ROS_INFO("Successfully launched node.");
}

CartesianController::~CartesianController()
{
  linear_twist_subscriber_.shutdown();
  motor_left_publisher_.shutdown();
  motor_right_publisher_.shutdown();
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
  motor_right_msg_.data = 0.0;
  motor_left_msg_.data  = 0.0;
}

/* Transform the linear and angular twist velocities into wheel velocities
 */
void CartesianController::linearTwistCallback(const geometry_msgs::Twist& msg)
{
  double v = msg.linear.x;
  double w = msg.angular.z;
  double c = (wheel_distance_ / 2.0) * w;
  
  motor_right_msg_.data = v + c;
  motor_left_msg_.data  = v - c;
}


} /* namespace */