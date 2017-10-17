#include <ros/ros.h>
#include <ras_group8_cartesian_controller/CartesianController.hpp>

using namespace ras_group8_cartesian_controller;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ras_group8_cartesian_controller");
  ros::NodeHandle node_handle("~");
  ros::Rate       loop_rate(10.0);
  
  // ras_group8_util::Reloadable<ras_group8_cartesian_controller::CartesianController> reloadable(node_handle, setup);
  
  CartesianController controller = CartesianController::load(node_handle);
  
  while (ros::ok()) {
    ros::spinOnce();
    controller.update();
    loop_rate.sleep();
  }
  
  return 0;
}