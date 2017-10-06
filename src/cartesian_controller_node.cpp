#include <ros/ros.h>
#include <ras_group8_cartesian_controller/CartesianController.hpp>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ras_group8_cartesian_controller");
  ros::NodeHandle nodeHandle("~");

  ras_group8_cartesian_controller::CartesianController mainObject(nodeHandle);

  ros::spin();
  return 0;
}