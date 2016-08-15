#include "canyonero_odom.h"

#define DISTANCEWHEELS 0.533
#define PI 3.14159265359

CanyoneroOdomNode::CanyoneroOdomNode(){

    ros::NodeHandle nh;

    //Motors velocity Subscriber
    motorsVel_sub_ = nh.subscribe("/motors_vel", 10, &CanyoneroOdomNode::motorsVelocityCallback, this);

    //Odomtry publisher
    odom_pub_ = nh.advertise<nav_msgs::Odometry>("/odometry", 10);

    v1 = 0.0, v2 = 0.0;
    x = 0.0, y = 0.0, th = 0.0;
    vx = 0.0, vy = 0.0, vth = 0.0;

    hasWheelsVelocities = false;

}

CanyoneroOdomNode::~CanyoneroOdomNode(){ }

void CanyoneroOdomNode::odometryPublish(){

    odom_pub_.publish(wheelOdomMsg);
}

void CanyoneroOdomNode::motorsVelocityCallback(const std_msgs::Float32MultiArray& msg){

    v1 = msg.data[0];
    v2 = msg.data[1];

    hasWheelsVelocities = true;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "canyonero_odom_node");

  CanyoneroOdomNode canyonero_odom;

  double tmp_x = 0.0, tmp_y = 0.0, tmp_th = 0.0;
  double dt = 0.0, last_time = 0.0, current_time = 0.0;
  double v_linear = 0.0, v_angular = 0.0;

  ROS_INFO_STREAM("Canyonero Odometry Started");

  while(ros::ok()){

      ros::spinOnce();

      if(canyonero_odom.hasWheelsVelocities){

          //compute velocity of robot frame
          v_angular = ((canyonero_odom.v1 - canyonero_odom.v2)/DISTANCEWHEELS);
          v_linear = (canyonero_odom.v1 + canyonero_odom.v2)/2;

          //Time between loops
          ros::Time ros_time = ros::Time::now();
          current_time = ros_time.toSec();
          dt = current_time - last_time;
          last_time = current_time;

          //compute x, y velocities and angular
          canyonero_odom.vx = v_linear * cos(canyonero_odom.th);
          canyonero_odom.vy = v_linear * sin(canyonero_odom.th);
          canyonero_odom.vth = v_angular;

          //Accumulate distance x, y and yaw
          tmp_th = canyonero_odom.th + canyonero_odom.vth * dt;
          tmp_x = canyonero_odom.x + (canyonero_odom.vx * cos(canyonero_odom.th) - canyonero_odom.vy * sin(canyonero_odom.th)) * dt;
          tmp_y = canyonero_odom.y + (canyonero_odom.vx * sin(canyonero_odom.th) + canyonero_odom.vy * cos(canyonero_odom.th)) * dt;

          //yaw restriction between [pi,-pi]
          if(tmp_th > PI) tmp_th = tmp_th - 2*PI;
          else if (tmp_th < -PI) tmp_th = tmp_th +2*PI;

          canyonero_odom.th = tmp_th;
          canyonero_odom.x = tmp_x;
          canyonero_odom.y = tmp_y;

          //compute quaternion from yaw
          geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(canyonero_odom.th);

          //first, we'll publish the transform over tf
          geometry_msgs::TransformStamped odom_trans;
          odom_trans.header.stamp = ros_time;
          odom_trans.header.frame_id = "wheels_odom";
          odom_trans.child_frame_id = "base_link";

          odom_trans.transform.translation.x = canyonero_odom.x;
          odom_trans.transform.translation.y = canyonero_odom.y;
          odom_trans.transform.translation.z = 0.0;
          odom_trans.transform.rotation = odom_quat;

          //send the transform
          canyonero_odom.odom_broadcaster.sendTransform(odom_trans);

          //create odom message
          canyonero_odom.wheelOdomMsg.header.stamp = ros_time;
          canyonero_odom.wheelOdomMsg.header.frame_id = "wheels_odom";

          //set the position
          canyonero_odom.wheelOdomMsg.pose.pose.position.x = canyonero_odom.x;
          canyonero_odom.wheelOdomMsg.pose.pose.position.y = canyonero_odom.y;
          canyonero_odom.wheelOdomMsg.pose.pose.position.z = 0.0;
          canyonero_odom.wheelOdomMsg.pose.pose.orientation = odom_quat;

          //set the velocity
          canyonero_odom.wheelOdomMsg.child_frame_id = "base_link";
          canyonero_odom.wheelOdomMsg.twist.twist.linear.x = canyonero_odom.vx;
          canyonero_odom.wheelOdomMsg.twist.twist.linear.y = canyonero_odom.vy;
          canyonero_odom.wheelOdomMsg.twist.twist.angular.z = canyonero_odom.vth;

          canyonero_odom.odometryPublish();

          canyonero_odom.hasWheelsVelocities = false;
      }
  }
  return 0;
}
