#include <stdlib.h>

//MOTORS
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include <Adafruit_PWMServoDriver.h>

//ROS
#include <ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int32.h>

Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *myMotor1 = AFMS.getMotor(1);

//ROS Variables
ros::NodeHandle nh;
float vel;
std_msgs::Int32 encoder1_msg;

//ENCODER 1 Variables
volatile long encoder1Value = 0;
 
void motor1VelCallBack(const std_msgs::Float32& msg){
  vel = msg.data;
}




//http://bildr.org/2012/08/rotary-encoder-arduino/
void updateEncoder1_A(){
   // look for a low-to-high on channel A
  if (digitalRead(18) == HIGH) { 
    // check channel B to see which way encoder is turning
    if (digitalRead(19) == LOW) {  
      encoder1Value = encoder1Value + 1;         // CW
    } 
    else {
      encoder1Value = encoder1Value - 1;         // CCW
    }
  }
  else   // must be a high-to-low edge on channel A                                       
  { 
    // check channel B to see which way encoder is turning  
    if (digitalRead(19) == HIGH) {   
      encoder1Value = encoder1Value + 1;          // CW
    } 
    else {
      encoder1Value = encoder1Value - 1;          // CCW
    }
  }
  
   encoder1_msg.data = encoder1Value;
}
void updateEncoder1_B(){
   // look for a low-to-high on channel B
  if (digitalRead(19) == HIGH) {   
   // check channel A to see which way encoder is turning
    if (digitalRead(18) == HIGH) {  
      encoder1Value = encoder1Value + 1;         // CW
    } 
    else {
      encoder1Value = encoder1Value - 1;         // CCW
    }
  }
  // Look for a high-to-low on channel B
  else { 
    // check channel B to see which way encoder is turning  
    if (digitalRead(18) == LOW) {   
      encoder1Value = encoder1Value + 1;          // CW
    } 
    else {
      encoder1Value = encoder1Value - 1;          // CCW
    }
  }
  
   encoder1_msg.data = encoder1Value;
}


ros::Subscriber<std_msgs::Float32> sub_vel1("/motor1_vel", &motor1VelCallBack);

ros::Publisher encoder1_pub("/motor1_encoder", &encoder1_msg);

void setup() {
  // put your setup code here, to run once:
  //Serial.begin(9600); // set up Serial library at 9600 bps
  //Serial.println("Adafruit Motorshield v2 - DC Motor test!");
  vel = 0.0;
  encoder1_msg.data = 0,


  //Mega2560
  // external interrupt int.0    int.1    int.2   int.3   int.4   int.5            
  // pin                  2         3      21      20      19      18
  pinMode(19, INPUT); 
  pinMode(18, INPUT);

  digitalWrite(19, HIGH); //turn pullup resistor on
  digitalWrite(18, HIGH); //turn pullup resistor on  
  
  attachInterrupt(4, updateEncoder1_B, CHANGE); // Also LOW, RISING, FALLING
  attachInterrupt(5, updateEncoder1_A, CHANGE); // Also LOW, RISING, FALLING

  AFMS.begin(); // create with the default frequency 1.6KHz
  //AFMS.begin(1000); // OR with a different frequency, say 1KHz
  
  nh.initNode();
  nh.subscribe(sub_vel1);
  nh.advertise(encoder1_pub);
}

void loop() {
  // put your main code here, to run repeatedly:
  //Serial.println("MOTOR ON");
  
  if(vel >= 0){
    myMotor1->setSpeed(vel);
    myMotor1->run(FORWARD);
  }else{
    myMotor1->setSpeed(abs(vel));
    myMotor1->run(BACKWARD);
  }
  
  encoder1_pub.publish(&encoder1_msg);
  
  nh.spinOnce();
  
  delay(1);
}
