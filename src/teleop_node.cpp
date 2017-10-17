#include <ros/ros.h>
#include <ras_group8_cartesian_controller/Teleop.hpp>

using namespace ras_group8_cartesian_controller;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "teleop");
  ros::NodeHandle node_handle("~");
  
  Teleop teleop(node_handle, 0);

  teleop.spin();

  return 0;
}