#include <ros/ros.h>
#include <ras_group8_cartesian_controller/CartesianController.hpp>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ras_group8_cartesian_controller");
  ros::NodeHandle node_handle("~");
  ros::Rate       loop_rate(10.0);
  
  ras_group8_cartesian_controller::CartesianController controller(node_handle);

  for (;;) {
    ros::spinOnce();
    /* Publish motor control messages */
    controller.update();
    loop_rate.sleep();
  }
  
  return 0;
}