#include <ros/ros.h>
//#include <ras_group8_util/Reloadable.hpp>
#include <ras_group8_cartesian_controller/CartesianController.hpp>

ras_group8_cartesian_controller::CartesianController
  setup(ros::NodeHandle& nh)
{
  std::string linear_twist_topic;
  std::string motor_left_topic;
  std::string motor_right_topic;
  // double wheel_radius;
  double wheel_distance;

  if (!nh.getParam("linear_twist_topic", linear_twist_topic))
    exit(-1);
  if (!nh.getParam("left_motor_topic", motor_left_topic))
    exit(-1);
  if (!nh.getParam("right_motor_topic", motor_right_topic))
    exit(-1);
  // if (!nh.getParam("/platform/wheel_radius", wheel_radius))
  //   exit(-1);
  if (!nh.getParam("/platform/wheel_distance", wheel_distance))
    exit(-1);

  ras_group8_cartesian_controller::CartesianController
    controller(nh, linear_twist_topic,
                   motor_left_topic,
                   motor_right_topic,
                   // wheel_radius,
                   wheel_distance);

  return controller;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ras_group8_cartesian_controller");
  ros::NodeHandle node_handle("~");
  ros::Rate       loop_rate(10.0);
  
  // ras_group8_util::Reloadable<ras_group8_cartesian_controller::CartesianController> reloadable(node_handle, setup);
  
  
  ras_group8_cartesian_controller::CartesianController controller
    = setup(node_handle);
  
  while (ros::ok()) {
    ros::spinOnce();
    /* Publish motor control messages */
    controller.update();
    loop_rate.sleep();
  }
  
  return 0;
}