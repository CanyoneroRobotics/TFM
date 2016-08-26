#include <stdlib.h>

//MOTORS
#include <Wire.h>
#include <Adafruit_MotorShield.h>

Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *FL_motor = AFMS.getMotor(4);
Adafruit_DCMotor *RL_motor = AFMS.getMotor(3);
Adafruit_DCMotor *FR_motor = AFMS.getMotor(2);
Adafruit_DCMotor *RR_motor = AFMS.getMotor(1);

//ROS Variables
int16_t RL_speed;
int16_t RR_speed;
int16_t FL_speed;
int16_t FR_speed;

int8_t RL_rads;
int8_t RR_rads;
int8_t FL_rads;
int8_t FR_rads;

//ENCODER 1 Variables
long RL_ticks = 0;
long last_RL_ticks = 0;
long delta_RL_ticks = 0;

long RR_ticks = 0;
long last_RR_ticks = 0;
long delta_RR_ticks = 0;

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

  if (delta_RL_ticks < 0) delta_RL_ticks = abs(delta_RL_ticks);
  if (delta_RR_ticks < 0) delta_RR_ticks = abs(delta_RR_ticks);

  RL_speed = 10 * ((delta_RL_ticks) * WHEEL_PERIM_CM) / (RL_TICKS * dt); // cm/ms to m/s
  RR_speed = 10 * ((delta_RR_ticks) * WHEEL_PERIM_CM) / (RR_TICKS * dt); // cm/ms to m/s

  if (RL_speed > 127) RL_speed = 127;
  else if (RL_speed < -128) RL_speed = -128;
  if (RR_speed > 127) RR_speed = 127;
  else if (RR_speed < -128) RR_speed = -128;

  // We don't have encoders for the front wheels so just reuse rear speeds
  FL_speed = RL_speed;
  FR_speed = RR_speed;

  last_time = millis();
  last_RL_ticks = RL_ticks;
  last_RR_ticks = RR_ticks;

  // Start of Packet
  Serial.write('{');

  // Speeds
  Serial.write((int8_t)FL_speed);
  Serial.write((int8_t)FR_speed);
  Serial.write((int8_t)RL_speed);
  Serial.write((int8_t)RR_speed);

  // End of Packet
  Serial.write('}');
}

void readFromRoscontrol(void)
{
  char c = '\0';
  unsigned long timeout = millis();

  // Detect Start of Packet
  while (c != SOP && (timeout + 250) > millis())
  {
    c = Serial.read();
  }

  if (c == SOP)
  {
    // Start of Packet detected
    FL_rads = (int8_t)Serial.read();
    FR_rads = (int8_t)Serial.read();
    RL_rads = (int8_t)Serial.read();
    RR_rads = (int8_t)Serial.read();
  }
  else
  {
    FL_rads = 0;
    FR_rads = 0;
    RL_rads = 0;
    RR_rads = 0;
  }

  timeout = millis();
  // Detect End of Packet
  while (c != EOP && (timeout + 250) > millis())
  {
    c = Serial.read();
  }

  if (c == EOP)
  {
    if (RL_rads > 0)
    {
      digitalWrite(13, HIGH);
    }
    else
    {
      digitalWrite(13, LOW);
    }

    //RL_speed = (RL_rads * WHEEL_DIAM_CM / 2) / 100; // rad/s to cm/s to m/s
    //RR_speed = (RR_rads * WHEEL_DIAM_CM / 2) / 100; // rad/s to cm/s to m/s

    if (RL_rads > 10) RL_rads = 10;
    else if (RL_rads < -10) RL_rads = -10;

    if (RR_rads > 10) RR_rads = 10;
    else if (RR_rads < -10) RR_rads = -10;

    if (RL_rads != 0)
      RL_speed = map(RL_rads, -10, 10, -192, 192);
    else
      RL_speed = 0;

    if (RR_rads != 0)
      RR_speed = map(RR_rads, -10, 10, -192, 192);
    else
      RR_speed = 0;

    if (RL_speed > 192) RL_speed = 192;
    else if (RL_speed < -192) RL_speed = -192;

    if (RR_speed > 192) RR_speed = 192;
    else if (RR_speed < -192) RR_speed = -192;

    if (RL_speed >= 0)
    {
      RL_motor->setSpeed(RL_speed);
      RL_motor->run(FORWARD);
      FL_motor->setSpeed(RL_speed);
      FL_motor->run(FORWARD);
    }
    else if (RL_speed < 0)
    {
      RL_motor->setSpeed(-RL_speed);
      RL_motor->run(BACKWARD);
      FL_motor->setSpeed(-RL_speed);
      FL_motor->run(BACKWARD);
    }

    if (RR_speed >= 0) {
      RR_motor->setSpeed(RR_speed);
      RR_motor->run(FORWARD);
      FR_motor->setSpeed(RR_speed);
      FR_motor->run(FORWARD);
    }
    else if (RR_speed < 0) {
      RR_motor->setSpeed(-RR_speed);
      RR_motor->run(BACKWARD);
      FR_motor->setSpeed(-RR_speed);
      FR_motor->run(BACKWARD);
    }
  }
}

void setup() {
  RL_ticks = 0;
  last_RL_ticks = 0;
  RR_ticks = 0;
  last_RR_ticks = 0;
  RR_speed = 0;
  RL_speed = 0;

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

  FL_motor->setSpeed(0);
  FL_motor->run(FORWARD);

  FR_motor->setSpeed(0);
  FR_motor->run(FORWARD);

  last_time = millis();
}

void loop() {

  writeToRoscontrol();
  delay(50);
  readFromRoscontrol();
  delay(50);
}

