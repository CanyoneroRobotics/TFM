#include "velocity_control.h"


VelocityControlNode::VelocityControlNode(){

    ros::NodeHandle nh;

    vel1_.data = 50;

    joy_sub_  = nh.subscribe("/joy", 1,  &VelocityControlNode::joyCallback, this);

    encoder1_sub_  = nh.subscribe("/motor1_encoder", 1,  &VelocityControlNode::Encoder1Callback, this);
    vel1_pub_  =  nh.advertise<std_msgs::Float32>("/motor1_vel", 100);
}

VelocityControlNode::~VelocityControlNode(){ }

void VelocityControlNode::vel1Publish(){

    vel1_pub_.publish(vel1_);
}

void VelocityControlNode::joyCallback(const sensor_msgs::Joy::ConstPtr& joy_msg){

    //float lateraValueRaw = joy_msg->axes[0];

    if(joy_msg->buttons[1] != 1) vel1_.data = joy_msg->axes[1] * 255.0;

    else vel1_.data = 0.0;


    ROS_INFO_STREAM ("Motor 1 Velocity:  " <<  vel1_.data);

    vel1Publish();
}

void VelocityControlNode::Encoder1Callback(const std_msgs::Int32ConstPtr &encoder_msg){

    ROS_INFO_STREAM ("ENCODER MOTOR 1:  " << encoder_msg->data);
}


int main(int argc, char** argv) {
  ros::init(argc, argv, "velocity_control_node");

  VelocityControlNode velocity_node;

  ros::spin();

  return 0;
}
