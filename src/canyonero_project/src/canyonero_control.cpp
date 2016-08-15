#include "canyonero_control.h"

#define THRESHOLD 0.15

CanyoneroControlNode::CanyoneroControlNode(){

    ros::NodeHandle nh;

    controlMsg_.data.clear();

    //Joystick subscriber
    joy_sub_  = nh.subscribe("/joy", 1,  &CanyoneroControlNode::joyCallback, this);

    //Control motors publishers
    control_pub_ = nh.advertise<std_msgs::Int16MultiArray>("/motors_control", 1);
}

CanyoneroControlNode::~CanyoneroControlNode(){}

void CanyoneroControlNode::controlPublish(){
    control_pub_.publish(controlMsg_);
}

void CanyoneroControlNode::joyCallback(const sensor_msgs::Joy::ConstPtr& joy_msg){

    controlMsg_.data.clear();

    if(joy_msg->buttons[1] != 1 && joy_msg->buttons[0] != 1){

        //FORWARD || BACKWARD
        if((joy_msg->axes[1] > THRESHOLD || joy_msg->axes[1] < -THRESHOLD) && joy_msg->axes[0] == 0){

            controlMsg_.data.push_back(joy_msg->axes[1] * 255.0);
            controlMsg_.data.push_back((-1)*(joy_msg->axes[1] * 255.0));

        //FULL ROTATION
        }else if(joy_msg->axes[1] == 0 && (joy_msg->axes[0] > THRESHOLD || joy_msg->axes[0] < -THRESHOLD)){

            controlMsg_.data.push_back(joy_msg->axes[0] * 255.0);
            controlMsg_.data.push_back(joy_msg->axes[0] * 255.0);

        //TURNING 1
        }else if((joy_msg->axes[1] > THRESHOLD || joy_msg->axes[1] < -THRESHOLD) && joy_msg->axes[0] > THRESHOLD){

            controlMsg_.data.push_back(joy_msg->axes[0] * 255.0);
            controlMsg_.data.push_back(joy_msg->axes[1] * 255.0);

        //TURNNG 2
        }else if((joy_msg->axes[1] > THRESHOLD || joy_msg->axes[1] < -THRESHOLD) && joy_msg->axes[0] < -THRESHOLD){

            controlMsg_.data.push_back(joy_msg->axes[1] * 255.0);
            controlMsg_.data.push_back(joy_msg->axes[0] * 255.0);

        }else{

            controlMsg_.data.push_back(0.0);
            controlMsg_.data.push_back(0.0);
        }
    }else if(joy_msg->buttons[1] == 1){

        controlMsg_.data.push_back(0.0);
        controlMsg_.data.push_back(0.0);
    }else if(joy_msg->buttons[0] == 1){

        controlMsg_.data.push_back(1);
        controlMsg_.data.push_back(-1);
    }

    controlPublish();
}

int main(int argc, char** argv) {

  ros::init(argc, argv, "canyonero_control_node");

  CanyoneroControlNode control_node;


  ROS_INFO_STREAM("Canyonero Control Started");

  ros::Rate r(10);

  while(ros::ok()){

      ros::spinOnce();

  }

  return 0;
}
