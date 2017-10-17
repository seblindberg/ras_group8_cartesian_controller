#include <ros/ros.h>
#include <ras_group8_cartesian_controller/Teleop.hpp>

using namespace ras_group8_cartesian_controller;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "teleop");
  Teleop& teleop = Teleop::getInstance();

  teleop.spin();

  return 0;
}