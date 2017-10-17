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
}

/* Update
 * Publish motor velocities to the motor controller topics.
 */
void CartesianController::update()
{
  motor_left_publisher_.publish(motor_left_msg_);
  motor_right_publisher_.publish(motor_right_msg_);
}

/* Reset
 * Sets the linear and angular velocity to 0.
 */
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

/* Load
 * Static method that loads parameters for the parameter server and creates a
 * CartesianController using those values.
 */
CartesianController CartesianController::load(ros::NodeHandle& n)
{
  std::string linear_twist_topic;
  std::string motor_left_topic;
  std::string motor_right_topic;
  
  double wheel_distance;
  
  if (!n.getParam("linear_twist_topic", linear_twist_topic))
    exit(-1);
  if (!n.getParam("left_motor_topic", motor_left_topic))
    exit(-1);
  if (!n.getParam("right_motor_topic", motor_right_topic))
    exit(-1);
  if (!n.getParam("/platform/wheel_distance", wheel_distance))
    exit(-1);
  
  CartesianController controller(n, linear_twist_topic,
                                    motor_left_topic,
                                    motor_right_topic,
                                    wheel_distance);
  
  return controller;
}


} /* namespace */