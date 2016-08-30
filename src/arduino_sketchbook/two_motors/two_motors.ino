#include <stdlib.h>

//MOTORS
#include <Wire.h>
#include <Adafruit_MotorShield.h>
//#include "utility/Adafruit_PWMServoDriver.h"

//ROS
#include <ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int16MultiArray.h>

#define TICK 1330
#define TICK1 2600
#define TICK2 3650
#define PERIMETER 0.62831853

Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *myMotor1 = AFMS.getMotor(1);
Adafruit_DCMotor *myMotor2 = AFMS.getMotor(2);
Adafruit_DCMotor *myMotor3 = AFMS.getMotor(3);
Adafruit_DCMotor *myMotor4 = AFMS.getMotor(4);

//ROS Variables
ros::NodeHandle nh;
float vel1;
float vel2;
float tmp;
//std_msgs::Int32 encoder1_msg;
//std_msgs::Int32 encoder2_msg;
std_msgs::Float32MultiArray wheelsVel_msg;

std_msgs::Int16 wheelVel1_msg;
std_msgs::Int16 wheelVel2_msg;

//ENCODERS Variables
volatile long encoder1Value = 0;
volatile long encoder2Value = 0;
volatile long lastEncoder1 = 0;
volatile long lastEncoder2 = 0;

//OUTPUT VELS
float wheel1_vel = 0;
float wheel2_vel = 0;

//TIME
unsigned long last_time = 0;
unsigned long current_time = 0;
unsigned long dt = 0;
 
//void motor1VelCallBack(const std_msgs::Float32& msg){
//  vel1 = msg.data;
//}
//void motor2VelCallBack(const std_msgs::Float32& msg){
//  vel2 = msg.data;
//}
void motorsControlCallBack(const std_msgs::Int16MultiArray& msg){
  
  vel1 = msg.data[0];
  vel2 = msg.data[1];

//encoders RESET. Triangle button of joystick. 
  if(vel1 == 1 && vel2 == -1){ 
    vel1 = 0;
    vel2 = 0;
    encoder1Value = 0;
    lastEncoder1 = 0;
    encoder2Value = 0;
    lastEncoder2 = 0;
  }
}

//http://bildr.org/2012/08/rotary-encoder-arduino/
void updateEncoder1(){
  
   // look for a low-to-high on channel A
  if (vel1 > 0) { 
      encoder1Value ++;         // CW
  }
  else if (vel1 < 0){   // must be a high-to-low edge on channel A                                       
      encoder1Value --;          // CCW
  }
}
void updateEncoder2(){

   // look for a low-to-high on channel B
  if (vel2 < 0) { 
      encoder2Value --;         // CW
  }
  // Look for a high-to-low on channel B
  else if (vel2 > 0){ 
      encoder2Value ++;          // CCW
  }
}

//ros::Subscriber<std_msgs::Float32> sub_vel1("/motor1_vel", &motor1VelCallBack);
//ros::Subscriber<std_msgs::Float32> sub_vel2("/motor2_vel", &motor2VelCallBack);
ros::Subscriber<std_msgs::Int16MultiArray> sub_control("/motors_control", &motorsControlCallBack);

//ros::Publisher encoder1_pub("/motor1_encoder", &encoder1_msg);
//ros::Publisher encoder2_pub("/motor2_encoder", &encoder2_msg);
ros::Publisher wheelsVel_pub("/wheels_vel", &wheelsVel_msg);
//ros::Publisher wheelVel1_pub("/wheel1_vel", &wheelVel1_msg);
//ros::Publisher wheelVel2_pub("/wheel2_vel", &wheelVel1_msg);

void setup() {
  // put your setup code here, to run once:
  vel1 = 0;
  vel2 = 0;
  //encoder1_msg.data = 0;
  //encoder2_msg.data = 0;
  
  wheelsVel_msg.data_length = 2;
  wheelsVel_msg.data[0] = 0;
  wheelsVel_msg.data[1] = 0;

  last_time = 0;

  //Mega2560
  // external interrupt int.0    int.1    int.2   int.3   int.4   int.5            
  // pin                  2         3      21      20      19      18
  pinMode(19, INPUT); 
  pinMode(18, INPUT);

  digitalWrite(19, HIGH); //turn pullup resistor on
  digitalWrite(18, HIGH); //turn pullup resistor on

  attachInterrupt(4, updateEncoder1, CHANGE); // Also LOW, RISING, FALLING
  attachInterrupt(5, updateEncoder2, CHANGE); // Also LOW, RISING, FALLING
  
  AFMS.begin(); // create with the default frequency 1.6KHz
  //AFMS.begin(1000); // OR with a different frequency, say 1KHz
  
  nh.getHardware()->setBaud(9600);
  nh.initNode();
  
  //nh.subscribe(sub_vel1);
  //nh.subscribe(sub_vel2);
  //nh.advertise(encoder1_pub);
  //nh.advertise(encoder2_pub);

  
  nh.subscribe(sub_control);
  nh.advertise(wheelsVel_pub);
  //nh.advertise(wheelVel1_pub);
  //nh.advertise(wheelVel2_pub);

}

void loop() {
  
  // put your main code here, to run repeatedly:
  if(vel1 >= 0){
    myMotor1->setSpeed(vel1);
    myMotor2->setSpeed(vel1);
    myMotor1->run(FORWARD);
    myMotor2->run(FORWARD);
  }else if(vel1 < 0){
    myMotor1->setSpeed(abs(vel1));
    myMotor2->setSpeed(abs(vel1));
    myMotor1->run(BACKWARD);
    myMotor2->run(BACKWARD);
  }

  if(vel2 >= 0){
    myMotor3->setSpeed(vel2);
    myMotor4->setSpeed(vel2);
    myMotor3->run(BACKWARD);
    myMotor4->run(BACKWARD);
  }else if(vel2 < 0){
    myMotor3->setSpeed(abs(vel2));
    myMotor4->setSpeed(abs(vel2));
    myMotor3->run(FORWARD);
    myMotor4->run(FORWARD);
  }

  //compute delta the time between loops
  current_time = micros();
  dt = (current_time - last_time);
  last_time = current_time;

  //compute wheels velocity
  wheel1_vel = (((encoder1Value-lastEncoder1)*(PERIMETER/TICK1))/dt);
  wheel2_vel = (-1)*(((encoder2Value-lastEncoder2)*(PERIMETER/TICK2))/dt);

  //wheel1_vel = (encoder1Value);// - lastEncoder1);
  //wheel2_vel = (encoder2Value);// - lastEncoder2);

  wheelsVel_msg.data[0] = wheel1_vel;
  wheelsVel_msg.data[1] = wheel2_vel;

  //wheelVel1_msg.data = wheel1_vel;
  //wheelVel2_msg.data = wheel2_vel;

  lastEncoder1 = encoder1Value;
  lastEncoder2 = encoder2Value;
  
  wheelsVel_pub.publish(&wheelsVel_msg);
  //wheelVel1_pub.publish(&wheelVel1_msg);
  //wheelVel2_pub.publish(&wheelVel2_msg);

  nh.spinOnce();
}
