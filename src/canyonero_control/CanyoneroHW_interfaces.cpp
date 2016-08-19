#include "canyonero_hardware_interfaces.h"
#include "controller_manager/controller_manager.h"
#include "ros/ros.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "canyonero_hardware_interface");

  ros::NodeHandle n;

  Canyonero canyonero;
  controller_manager::ControllerManager cm(&canyonero);

  while(ros::ok())
  {
    canyonero.read();
    cm.update(canyonero.get_time(), canyonero.get_period());
    canyonero.write();
    sleep(20);
  }
}
