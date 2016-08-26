#include "canyonero_hardware.h"
#include "controller_manager/controller_manager.h"
#include "ros/ros.h"
#include <unistd.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "canyonero_hardware_interface");

  ros::NodeHandle nh;

  Canyonero canyonero;
  controller_manager::ControllerManager cm(&canyonero, nh);

  ROS_INFO("Entering control loop");

  // Control loop
  while(ros::ok())
  {
    canyonero.read();
    cm.update(canyonero.get_time(), canyonero.get_period());
    canyonero.write();
    usleep(100000);
  }

  ROS_INFO("We exited the control loop");

  return 0;
}
