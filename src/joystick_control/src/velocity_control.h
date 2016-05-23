#ifndef VELOCITY_CONTROL_H
#define VELOCITY_CONTROL_H

#include <boost/bind.hpp>

#include <stdio.h>
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Int32.h>



class VelocityControlNode {
 public:
  VelocityControlNode();
  ~VelocityControlNode();

  void vel1Publish();


  std_msgs::Float32 vel1_;

 private:

  // subscribers
  ros::Subscriber joy_sub_;

  ros::Subscriber encoder1_sub_;
  ros::Publisher vel1_pub_;



  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy_msg);
  void Encoder1Callback(const std_msgs::Int32ConstPtr& encoder_msg);

};

#endif // VELOCITY_CONTROL_H
