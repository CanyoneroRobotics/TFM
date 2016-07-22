#ifndef VELOCITY_CONTROL_H
#define VELOCITY_CONTROL_H

#include <boost/bind.hpp>

#include <stdio.h>
#include <ros/ros.h>

#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int32MultiArray.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>

class VelocityControlNode {
 public:
  VelocityControlNode();
  ~VelocityControlNode();

  void vel1Publish();
  void vel2Publish();


  std_msgs::Float32 vel1_;
  std_msgs::Float32 vel2_;

  //Wheel Odom
  nav_msgs::Odometry wheelOdomMsg;
  tf::TransformBroadcaster odom_broadcaster;
  ros::Publisher odom_pub_;

  ros::Time current_time, last_time;

  double x;
  double y;
  double th;

  double vx;
  double vy;
  double vth;


  //Encoders
  int encodersValue[2], lastEncodersValue[2];

  bool isDouble_, isEncoder1_, isEncoder2_;

 private:

  // subscribers
  ros::Subscriber joy_sub_;

  ros::Subscriber encoder1_sub_;
  ros::Subscriber encoder2_sub_;

  //publishers
  ros::Publisher vel1_pub_;
  ros::Publisher vel2_pub_;

  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy_msg);
  void Encoder1Callback(const std_msgs::Int32ConstPtr& encoder1_msg);
  void Encoder2Callback(const std_msgs::Int32ConstPtr& encoder2_msg);

};

#endif // VELOCITY_CONTROL_H
