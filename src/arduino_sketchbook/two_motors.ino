#include <stdlib.h>

//MOTORS
#include <Wire.h>
#include <Adafruit_MotorShield.h>
//#include "utility/Adafruit_PWMServoDriver.h"

//ROS
#include <ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>

Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *myMotor1 = AFMS.getMotor(1);
Adafruit_DCMotor *myMotor2 = AFMS.getMotor(3);

//ROS Variables
ros::NodeHandle nh;
float vel1;
float vel2;
std_msgs::Int32 encoder1_msg;
std_msgs::Int32 encoder2_msg;

//ENCODER 1 Variables
volatile long encoder1Value = 0;
volatile long encoder2Value = 0;
 
void motor1VelCallBack(const std_msgs::Float32& msg){
  vel1 = msg.data;
}
void motor2VelCallBack(const std_msgs::Float32& msg){
  vel2 = msg.data;
}

//http://bildr.org/2012/08/rotary-encoder-arduino/
void updateEncoder1(){
   // look for a low-to-high on channel A
  if (vel1 > 0) { 
   
      encoder1Value ++;         // CW
  }
  else if (vel1 < 0)   // must be a high-to-low edge on channel A                                       
  { 
      encoder1Value --;          // CCW

  }
  
   encoder1_msg.data = encoder1Value;
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
  
   encoder2_msg.data = encoder2Value;
}

ros::Subscriber<std_msgs::Float32> sub_vel1("/motor1_vel", &motor1VelCallBack);
ros::Subscriber<std_msgs::Float32> sub_vel2("/motor2_vel", &motor2VelCallBack);

ros::Publisher encoder1_pub("/motor1_encoder", &encoder1_msg);
ros::Publisher encoder2_pub("/motor2_encoder", &encoder2_msg);

void setup() {
  // put your setup code here, to run once:
  vel1 = 0;
  vel2 = 0;
  encoder1_msg.data = 0;
  encoder2_msg.data = 0;

  //Mega2560
  // external interrupt int.0    int.1    int.2   int.3   int.4   int.5            
  // pin                  2         3      21      20      19      18
  //pinMode(19, INPUT); 
  //pinMode(18, INPUT);
  //pinMode(22, INPUT); 
  //pinMode(23, INPUT);

  //digitalWrite(19, HIGH); //turn pullup resistor on
  //digitalWrite(18, HIGH); //turn pullup resistor on
  //digitalWrite(22, HIGH); //turn pullup resistor on
  //digitalWrite(23, HIGH); //turn pullup resistor on

  attachInterrupt(4, updateEncoder1, CHANGE); // Also LOW, RISING, FALLING
  attachInterrupt(5, updateEncoder2, CHANGE); // Also LOW, RISING, FALLING
  
  AFMS.begin(); // create with the default frequency 1.6KHz
  //AFMS.begin(1000); // OR with a different frequency, say 1KHz
  
  pinMode(13, OUTPUT);
  nh.initNode();
  nh.subscribe(sub_vel1);
  nh.subscribe(sub_vel2);
  nh.advertise(encoder1_pub);
  nh.advertise(encoder2_pub);

}

void loop() {
  // put your main code here, to run repeatedly:
  if(vel1 >= 0){
    myMotor1->setSpeed(vel1);
    myMotor1->run(FORWARD);
  }else if(vel1 < 0){
    myMotor1->setSpeed(abs(vel1));
    myMotor1->run(BACKWARD);
  }

  if(vel2 >= 0){
    myMotor2->setSpeed(vel2);
    myMotor2->run(BACKWARD);
  }else if(vel2 < 0){
    myMotor2->setSpeed(abs(vel2));
    myMotor2->run(FORWARD);
  }

  encoder1_pub.publish(&encoder1_msg);
  encoder2_pub.publish(&encoder2_msg);
  nh.spinOnce();
  
  delay(1);
}
