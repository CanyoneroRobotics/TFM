#include <stdlib.h>

//MOTORS
#include <Wire.h>
#include <Adafruit_MotorShield.h>

Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *RL_motor = AFMS.getMotor(1);
Adafruit_DCMotor *RR_motor = AFMS.getMotor(3);

//ROS Variables
uint16_t RL_speed;
uint16_t RR_speed;
uint16_t FL_speed;
uint16_t FR_speed;

int8_t RL_rads;
int8_t RR_rads;
int8_t FL_rads;
int8_t FR_rads;

//ENCODER 1 Variables
volatile long RL_ticks = 0;
volatile long last_RL_ticks = 0;
volatile long delta_RL_ticks = 0;

volatile long RR_ticks = 0;
volatile long last_RR_ticks = 0;
volatile long delta_RR_ticks = 0;

#define SOP '{'
#define EOP '}'
#define WHEEL_DIAM_CM  (20) // centimeters
#define WHEEL_PERIM_CM (62) // centimeters
#define RL_TICKS  1300
#define RR_TICKS 840

int recv_bytes;

unsigned long last_time = 0;
unsigned long current_time = 0;
unsigned long dt = 0;

// As seen on http://bildr.org/2012/08/rotary-encoder-arduino/
void updateRLEncoder() {
  // look for a low-to-high on channel A
  if (RL_speed > 0)
  {
    RL_ticks++;         // CW
  }
  else if (RL_speed < 0)   // must be a high-to-low edge on channel A
  {
    RL_ticks--;          // CCW
  }
}

void updateRREncoder() {

  // look for a low-to-high on channel B
  if (RR_speed < 0)
  {
    RR_ticks--;         // CW
  }
  // Look for a high-to-low on channel B
  else if (RR_speed > 0)
  {
    RR_ticks++;          // CCW
  }
}

void writeToRoscontrol(void)
{
  dt = millis() - last_time;
  delta_RL_ticks = RL_ticks - last_RL_ticks;
  delta_RR_ticks = RR_ticks - last_RR_ticks;

  RL_speed = 10 * ((delta_RL_ticks) * WHEEL_PERIM_CM) / (RL_TICKS * dt); // cm/ms to m/s
  RR_speed = -10 * ((delta_RR_ticks) * WHEEL_PERIM_CM) / (RR_TICKS * dt); // cm/ms to m/s

  // We don't have encoders for the front wheels so just reuse rear speeds
  FL_speed = RL_speed;
  FR_speed = RR_speed;

  last_time = millis();
  last_RL_ticks = RL_speed;
  last_RR_ticks = RR_speed;

  // Start of Packet
  Serial.write('{');

  // Speeds
  Serial.write((uint8_t)FL_speed);
  Serial.write((uint8_t)FR_speed);
  Serial.write((uint8_t)RL_speed);
  Serial.write((uint8_t)RR_speed);

  // End of Packet
  Serial.write('}');
}

void readFromRoscontrol(void)
{
  char c = '\0';

  // Detect Start of Packet
  while (c != SOP)
  {
    c = Serial.read();
  }

  // Start of Packet detected
  FL_rads = (int8_t)Serial.read();
  FR_rads = (int8_t)Serial.read();
  RL_rads = (int8_t)Serial.read();
  RR_rads = (int8_t)Serial.read();

  // Detect End of Packet
  while (c != EOP)
  {
    c = Serial.read();
  }

  digitalWrite(13, HIGH);

  RL_speed = (RL_rads * WHEEL_DIAM_CM / 2) / 100; // rad/s to cm/s to m/s
  RR_speed = (RR_rads * WHEEL_DIAM_CM / 2) / 100; // rad/s to cm/s to m/s 

  if (RL_speed > 255) RL_speed = 255;
  else if (RL_speed < -255) RL_speed = -255;

  if (RR_speed > 255) RR_speed = 255;
  else if (RR_speed < -255) RR_speed = -255;

  if (RL_speed >= 0)
  {
    RL_motor->setSpeed(RL_speed);
    RL_motor->run(FORWARD);
  }
  else if (RL_speed < 0)
  {
    RL_motor->setSpeed(abs(RL_speed));
    RL_motor->run(BACKWARD);
  }

  if (RR_speed >= 0) {
    RR_motor->setSpeed(RR_speed);
    RR_motor->run(BACKWARD);
  }
  else if (RR_speed < 0) {
    RR_motor->setSpeed(abs(RR_speed));
    RR_motor->run(FORWARD);
  }

  digitalWrite(13, LOW);
}

void setup() {
  RL_ticks = 0;
  last_RL_ticks = 0;
  RR_ticks = 0;
  last_RR_ticks = 0;

  //Mega2560
  // external interrupt int.0    int.1    int.2   int.3   int.4   int.5
  // pin                  2         3      21      20      19      18
  pinMode(19, INPUT);
  pinMode(18, INPUT);

  digitalWrite(19, HIGH); //turn pullup resistor on
  digitalWrite(18, HIGH); //turn pullup resistor on

  attachInterrupt(4, updateRLEncoder, CHANGE); // Also LOW, RISING, FALLING
  attachInterrupt(5, updateRREncoder, CHANGE); // Also LOW, RISING, FALLING

  AFMS.begin(); // create with the default frequency 1.6KHz

  Serial.begin(9600);
  recv_bytes = 0;

  pinMode(13, OUTPUT);

  RL_motor->setSpeed(0);
  RL_motor->run(FORWARD);

  RR_motor->setSpeed(0);
  RR_motor->run(FORWARD);

  last_time = millis();
}

void loop() {

  writeToRoscontrol();
  readFromRoscontrol();

  delay(5);
}

