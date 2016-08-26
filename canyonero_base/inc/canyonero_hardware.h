#include "hardware_interface/joint_command_interface.h"
#include "hardware_interface/joint_state_interface.h"
#include "hardware_interface/robot_hw.h"

#include "ros/time.h"
#include "ros/duration.h"

#include "ros/console.h"

#include "Arduino.h"

class Canyonero : public hardware_interface::RobotHW {
public:
Canyonero(){
        //register joint state interface
        hardware_interface::JointStateHandle state_handle_front_left_wheel("wheel_front_left_joint", &pos[0], &vel[0], &eff[0]);
        jnt_state_interface.registerHandle(state_handle_front_left_wheel);

        hardware_interface::JointStateHandle state_handle_front_right_wheel("wheel_front_right_joint", &pos[1], &vel[1], &eff[1]);
        jnt_state_interface.registerHandle(state_handle_front_right_wheel);

        hardware_interface::JointStateHandle state_handle_rear_left_wheel("wheel_rear_left_joint", &pos[2], &vel[2], &eff[2]);
        jnt_state_interface.registerHandle(state_handle_rear_left_wheel);

        hardware_interface::JointStateHandle state_handle_rear_right_wheel("wheel_rear_right_joint", &pos[3], &vel[3], &eff[3]);
        jnt_state_interface.registerHandle(state_handle_rear_right_wheel);

        registerInterface(&jnt_state_interface);

        //register joint velocity interface
        hardware_interface::JointHandle vel_handle_front_left_wheel(jnt_state_interface.getHandle("wheel_front_left_joint"), &cmd[0]);
        jnt_vel_interface.registerHandle(vel_handle_front_left_wheel);

        hardware_interface::JointHandle vel_handle_front_right_wheel(jnt_state_interface.getHandle("wheel_front_right_joint"), &cmd[1]);
        jnt_vel_interface.registerHandle(vel_handle_front_right_wheel);

        hardware_interface::JointHandle vel_handle_rear_left_wheel(jnt_state_interface.getHandle("wheel_rear_left_joint"), &cmd[2]);
        jnt_vel_interface.registerHandle(vel_handle_rear_left_wheel);

        hardware_interface::JointHandle vel_handle_rear_right_wheel(jnt_state_interface.getHandle("wheel_rear_right_joint"), &cmd[3]);
        jnt_vel_interface.registerHandle(vel_handle_rear_right_wheel);

        registerInterface(&jnt_vel_interface);

        //record current time
        last_time = ros::Time::now();

        arduino.open();
}

void read();
void write();
ros::Time get_time();

ros::Duration get_period();

private:
hardware_interface::JointStateInterface jnt_state_interface;
hardware_interface::VelocityJointInterface jnt_vel_interface;

ros::Time last_time;
ros::Duration period;
double cmd[4];
double pos[4];
double vel[4];
double eff[4];
Arduino arduino;
};

void Canyonero::read () {
        std::vector<int8_t> buffer;

        buffer = arduino.read();

        for (unsigned int i = 0; i<4 && i<buffer.size(); i++)
        {
                pos[i]=0;
                eff[i]=0;

                vel[i]=(double)buffer[i];
                ROS_INFO("Canyonero::read() -> vel[%d] = %f", i, vel[i]);
        }
}

void Canyonero::write(){
        std::vector<int8_t> buffer;

        for(unsigned int i = 0; i<4; i++)
        {
            ROS_INFO("Canyonero::write() -> cmd[%d] = %f", i, cmd[i]);
            buffer.push_back((int8_t)cmd[i]);
        }

        arduino.write(buffer);
}

ros::Time Canyonero::get_time(){
        return ros::Time::now();
}

ros::Duration Canyonero::get_period(){
        ros::Time current_time = ros::Time::now();
        ros::Duration period = current_time - last_time;
        last_time = current_time;
        return period;
}
