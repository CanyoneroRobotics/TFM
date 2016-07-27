#include "velocity_control_double.h"

#define TICK 0.000473845
#define DISTANCEWHEELS 0.70

VelocityControlNode::VelocityControlNode(){

    ros::NodeHandle nh;


    isDouble_= true;

    vel1_.data = 0;
    vel2_.data = 0;
    x = 0.0;
    y = 0.0;
    th = 0.0;

    vx = 0.0;
    vy = 0.0;
    vth = 0.0;

    isEncoder1_ = false;
    isEncoder2_ = false;

    current_time = ros::Time::now();
    last_time = ros::Time::now();

    joy_sub_  = nh.subscribe("/joy", 1,  &VelocityControlNode::joyCallback, this);

    //2 MOTORS
    encoder1_sub_  = nh.subscribe("/motor1_encoder", 1,  &VelocityControlNode::Encoder1Callback, this);
    vel1_pub_  =  nh.advertise<std_msgs::Float32>("/motor1_vel", 10);

    encoder2_sub_ = nh.subscribe("/motor2_encoder", 1, &VelocityControlNode::Encoder2Callback, this);
    vel2_pub_ = nh.advertise<std_msgs::Float32>("/motor2_vel", 10);

    //Odomtry publisher
    odom_pub_ = nh.advertise<nav_msgs::Odometry>("/odometry", 25);
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

    lastEncodersValue[0] = encodersValue[0];
    encodersValue[0] = encoder1_msg->data;

    isEncoder1_ = true;

}

void VelocityControlNode::Encoder2Callback(const std_msgs::Int32ConstPtr& encoder2_msg){

    lastEncodersValue[1] = encodersValue[1];
    encodersValue[1] = encoder2_msg->data;

    isEncoder2_ = true;


    ROS_INFO_STREAM ("ENCODER MOTOR 1:  " << encodersValue[0]);
    ROS_INFO_STREAM ("ENCODER MOTOR 2:  " << encodersValue[1]);

}


int main(int argc, char** argv) {
  ros::init(argc, argv, "velocity_control_node");

  VelocityControlNode velocity_node;

  while(ros::ok()){

      ros::spinOnce();

      if(velocity_node.isEncoder1_ && velocity_node.isEncoder2_){

          velocity_node.current_time = ros::Time::now();
          double dt = (velocity_node.current_time - velocity_node.last_time).toSec();

          //compute wheels velocity and angular velocity
          double v1 = (((velocity_node.encodersValue[0]-velocity_node.lastEncodersValue[0])*TICK)/dt);
          double v2 = (((velocity_node.encodersValue[1]-velocity_node.lastEncodersValue[1])*TICK)/dt);

          //compute velocity center
          double vth = ((v1-v2)/DISTANCEWHEELS);
          double v = (v1+v2)/2;

          //compute linear velocities & odometry
          double delta_th = vth * dt;
          double vx = v*cos(velocity_node.th);
          double vy = v*sin(velocity_node.th);

          //double delta_x = (vx * cos(velocity_node.th) - vy * sin(velocity_node.th)) * dt;
          //double delta_y = (vx * sin(velocity_node.th) + vy * cos(velocity_node.th)) * dt;

          double delta_x = vx * dt;
          double delta_y = vy * dt;

          velocity_node.x += delta_x;
          velocity_node.y += delta_y;
          velocity_node.th += delta_th;

          //compute quaternion from yaw
          geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(velocity_node.th);

          //first, we'll publish the transform over tf
          geometry_msgs::TransformStamped odom_trans;
          odom_trans.header.stamp = velocity_node.current_time;
          odom_trans.header.frame_id = "odom";
          odom_trans.child_frame_id = "base_link";

          odom_trans.transform.translation.x = velocity_node.x;
          odom_trans.transform.translation.y = velocity_node.y;
          odom_trans.transform.translation.z = 0.0;
          odom_trans.transform.rotation = odom_quat;

          //send the transform
          velocity_node.odom_broadcaster.sendTransform(odom_trans);

          //create odom message
          velocity_node.wheelOdomMsg.header.stamp = velocity_node.current_time;
          velocity_node.wheelOdomMsg.header.frame_id = "odom";

          //set the position
          velocity_node.wheelOdomMsg.pose.pose.position.x = velocity_node.x;
          velocity_node.wheelOdomMsg.pose.pose.position.y = velocity_node.y;
          velocity_node.wheelOdomMsg.pose.pose.position.z = 0.0;
          velocity_node.wheelOdomMsg.pose.pose.orientation = odom_quat;

          //set the velocity
          velocity_node.wheelOdomMsg.child_frame_id = "base_link";
          velocity_node.wheelOdomMsg.twist.twist.linear.x = vx;
          velocity_node.wheelOdomMsg.twist.twist.linear.y = vy;
          velocity_node.wheelOdomMsg.twist.twist.angular.z = vth;

          velocity_node.odom_pub_.publish(velocity_node.wheelOdomMsg);

          velocity_node.isEncoder1_ = false;
          velocity_node.isEncoder2_ = false;
      }

  }

  return 0;
}
