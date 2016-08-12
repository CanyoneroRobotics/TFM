#ifndef CANYONERO_ODOM_H
#define CANYONERO_ODOM_H

#include <boost/bind.hpp>

#include <stdio.h>
#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>



class CanyoneroOdomNode {
 public:
  CanyoneroOdomNode();
  ~CanyoneroOdomNode();

  //velocities of wheels
  double v1, v2;
  //position robot frame
  double x, y, th;
  //velocities robot frame
  double vx, vy, vth;

  bool hasWheelsVelocities;



  //Wheel Odom
  nav_msgs::Odometry wheelOdomMsg;
  ros::Publisher odom_pub_;
  tf::TransformBroadcaster odom_broadcaster;

  void odometryPublish();

 private:

  // subscribers
  ros::Subscriber motorsVel_sub_;
  void motorsVelocityCallback(const std_msgs::Float32MultiArray& msg);

};

#endif // CANYONERO_ODOM_H
