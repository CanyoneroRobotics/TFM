#ifndef CANYONERO_CONTROL_H
#define CANYONERO_CONTROL_H

#include <boost/bind.hpp>

#include <stdio.h>
#include <ros/ros.h>

#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int16MultiArray.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>

class CanyoneroControlNode {
    
 public:
  CanyoneroControlNode();
  ~CanyoneroControlNode();

  void controlPublish();
  std_msgs::Int16MultiArray controlMsg_;

 private:

  // subscribers
  ros::Subscriber joy_sub_;

  //publishers
  ros::Publisher control_pub_;

  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy_msg);
};

#endif // CANYONERO_CONTROL_H
