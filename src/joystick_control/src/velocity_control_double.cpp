#include "velocity_control_double.h"


VelocityControlNode::VelocityControlNode(){

    ros::NodeHandle nh;
    isDouble_= true;

    vel1_.data = 0;
    vel2_.data = 0;

    joy_sub_  = nh.subscribe("/joy", 1,  &VelocityControlNode::joyCallback, this);

    //2 MOTORS
    encoder1_sub_  = nh.subscribe("/motor1_encoder", 1,  &VelocityControlNode::Encoder1Callback, this);
    vel1_pub_  =  nh.advertise<std_msgs::Float32>("/motor1_vel", 10);

    encoder2_sub_ = nh.subscribe("/motor2_encoder", 1, &VelocityControlNode::Encoder2Callback, this);
    vel2_pub_ = nh.advertise<std_msgs::Float32>("/motor2_vel", 10);
}

VelocityControlNode::~VelocityControlNode(){ }

void VelocityControlNode::vel1Publish(){

    vel1_pub_.publish(vel1_);
}

void VelocityControlNode::vel2Publish(){

    vel2_pub_.publish(vel2_);
}


void VelocityControlNode::joyCallback(const sensor_msgs::Joy::ConstPtr& joy_msg){

    //float lateraValueRaw = joy_msg->axes[0];

    if(!isDouble_){
        if(joy_msg->buttons[1] != 1) vel1_.data = joy_msg->axes[1] * 255.0;

        else vel1_.data = 0.0;

        vel1Publish();
    }else{

        if(joy_msg->buttons[1] != 1){

            //FORWARD || BACKWARD
            if(joy_msg->axes[1] != 0 && joy_msg->axes[0] == 0){

                vel1_.data = (joy_msg->axes[1] * 255.0);
                vel2_.data = (-joy_msg->axes[1] * 255.0);

            //FULL ROTATION
            }else if(joy_msg->axes[1] == 0 && joy_msg->axes[0] != 0){

                vel1_.data = (joy_msg->axes[0] * 255.0);
                vel2_.data = (joy_msg->axes[0] * 255.0);

            //TURNING 1
            }else if(joy_msg->axes[1] != 0 && joy_msg->axes[1] > 0){

                vel1_.data = (joy_msg->axes[0] * 255.0);
                vel2_.data = (joy_msg->axes[1] * 255.0);

            //TURNNG 2
            }else if(joy_msg->axes[1] != 0 && joy_msg->axes[1] < 0){

                vel1_.data = (joy_msg->axes[1] * 255.0);
                vel2_.data = (joy_msg->axes[0] * 255.0);

            }

        }else{

            vel1_.data = 0.0;
            vel2_.data = 0.0;
        }


        ROS_INFO_STREAM ("Motor 1 Velocity:  " <<  vel1_.data);
        ROS_INFO_STREAM ("Motor 2 Velocity:  " <<  vel2_.data);

        vel1Publish();
        vel2Publish();
    }


}
void VelocityControlNode::Encoder1Callback(const std_msgs::Int32ConstPtr &encoder1_msg){

    encodersValue[0] = encoder1_msg->data;

}

void VelocityControlNode::Encoder2Callback(const std_msgs::Int32ConstPtr& encoder2_msg){


    encodersValue[1] = encoder2_msg->data;
    ROS_INFO_STREAM ("ENCODER MOTOR 1:  " << encodersValue[0]);
    ROS_INFO_STREAM ("ENCODER MOTOR 2:  " << encodersValue[1]);

}


int main(int argc, char** argv) {
  ros::init(argc, argv, "velocity_control_node");

  VelocityControlNode velocity_node;

  ros::spin();

  return 0;
}
