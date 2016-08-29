//MOTORS
#include <Wire.h>
#include <Adafruit_MotorShield.h>

Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *FL_motor = AFMS.getMotor(4);
Adafruit_DCMotor *RL_motor = AFMS.getMotor(3);
Adafruit_DCMotor *FR_motor = AFMS.getMotor(2);
Adafruit_DCMotor *RR_motor = AFMS.getMotor(1);

//ROS Variables
float RL_speed_ms;
float RR_speed_ms;
float FL_speed_ms;
float FR_speed_ms;

int8_t RL_speed_cmd;
int8_t RR_speed_cmd;
int8_t FL_speed_cmd;
int8_t FR_speed_cmd;

int16_t RL_speed_servo_cmd;
int16_t RR_speed_servo_cmd;

//ENCODER 1 Variables
volatile long RL_ticks = 0;
long last_RL_ticks = 0;
long delta_RL_ticks = 0;

volatile long RR_ticks = 0;
long last_RR_ticks = 0;
long delta_RR_ticks = 0;

#define SOP '{'
#define EOP '}'
#define WHEEL_DIAM_CM  (20.0) // centimeters
#define WHEEL_PERIM_CM (62.0) // centimeters
#define RL_TICKS       (1300.0)
#define RR_TICKS       (840.0)

unsigned long last_time = 0;
unsigned long current_time = 0;
unsigned long dt = 0;

// As seen on http://bildr.org/2012/08/rotary-encoder-arduino/
void updateRLEncoder() {
  // look for a low-to-high on channel A
  if (RL_speed_servo_cmd > 0)
  {
    RL_ticks++;         // CW
  }
  else if (RL_speed_servo_cmd < 0)   // must be a high-to-low edge on channel A
  {
    RL_ticks--;          // CCW
  }
}

void updateRREncoder() {

  // look for a low-to-high on channel B
  if (RR_speed_servo_cmd < 0)
  {
    RR_ticks--;         // CW
  }
  // Look for a high-to-low on channel B
  else if (RR_speed_servo_cmd > 0)
  {
    RR_ticks++;          // CCW
  }
}

void writeToRoscontrol(void)
{
  int8_t FL_speed_ms_aux;
  int8_t FR_speed_ms_aux;
  int8_t RL_speed_ms_aux;
  int8_t RR_speed_ms_aux;

  dt = millis() - last_time;
  delta_RL_ticks = RL_ticks - last_RL_ticks;
  delta_RR_ticks = RR_ticks - last_RR_ticks;

  if (delta_RL_ticks < 0) delta_RL_ticks = abs(delta_RL_ticks);
  if (delta_RR_ticks < 0) delta_RR_ticks = abs(delta_RR_ticks);

  RL_speed_ms = 10.0 * (((float)delta_RL_ticks) * WHEEL_PERIM_CM) / (RL_TICKS * (float)dt); // cm/ms to m/s
  RR_speed_ms = 10.0 * (((float)delta_RR_ticks) * WHEEL_PERIM_CM) / (RR_TICKS * (float)dt); // cm/ms to m/s

  // We don't have encoders for the front wheels so just reuse rear speeds
  FL_speed_ms = RL_speed_ms;
  FR_speed_ms = RR_speed_ms;

  last_time = millis();
  last_RL_ticks = RL_ticks;
  last_RR_ticks = RR_ticks;

  if (FL_speed_ms*100.0 > 127.0) FL_speed_ms_aux = 127;
  else if (FL_speed_ms*100.0 < -128) FL_speed_ms_aux = -128;
  else FL_speed_ms_aux = (int8_t)(FL_speed_ms*100.0);
  
  if (FR_speed_ms*100.0 > 127.0) FR_speed_ms_aux = 127;
  else if (FR_speed_ms*100.0 < -128) FR_speed_ms_aux = -128;
  else FR_speed_ms_aux = (int8_t)(FR_speed_ms*100.0);

  if (RL_speed_ms*100.0 > 127.0) RL_speed_ms_aux = 127;
  else if (RL_speed_ms*100.0 < -128) RL_speed_ms_aux = -128;
  else RL_speed_ms_aux = (int8_t)(RL_speed_ms*100.0);
  
  if (RR_speed_ms*100.0 > 127.0) RR_speed_ms_aux = 127;
  else if (RR_speed_ms*100.0 < -128) RR_speed_ms_aux = -128;
  else RR_speed_ms_aux = (int8_t)(RR_speed_ms*100.0);

  // Start of Packet
  Serial.write('{');

  // Speeds - sending float
  Serial.write(FL_speed_ms_aux);
  Serial.write(FR_speed_ms_aux);
  Serial.write(RL_speed_ms_aux);
  Serial.write(RR_speed_ms_aux);

  // End of Packet
  Serial.write('}');
}

void readFromRoscontrol(void)
{
  float RL_speed_cms;
  float RR_speed_cms;
  int RL_speed_ms_aux;
  int RR_speed_ms_aux;
  
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
    FL_speed_cmd = (int8_t)Serial.read();
    FR_speed_cmd = (int8_t)Serial.read();
    RL_speed_cmd = (int8_t)Serial.read();
    RR_speed_cmd = (int8_t)Serial.read();
  }
  else
  {
    FL_speed_cmd = 0;
    FR_speed_cmd = 0;
    RL_speed_cmd = 0;
    RR_speed_cmd = 0;
  }

  timeout = millis();
  // Detect End of Packet
  while (c != EOP && (timeout + 250) > millis())
  {
    c = Serial.read();
  }

  if (c == EOP) // End of packet detected, let's process the commands
  {
    digitalWrite(13, HIGH);
    
    RL_speed_cms = ((float)RL_speed_cmd * WHEEL_DIAM_CM / 2.0); // rad/s to cm/s
    RR_speed_cms = ((float)RR_speed_cmd * WHEEL_DIAM_CM / 2.0); // rad/s to cm/s

    // Max speed is 150.0 cm/s so saturate just in case
    if (RL_speed_cms > 150.0) RL_speed_cms = 150.0;
    else if (RL_speed_cms < -150.0) RL_speed_cms = -150.0;

    if (RR_speed_cms > 150.0) RR_speed_cms = 150.0;
    else if (RR_speed_cms < -150.0) RR_speed_cms = -150.0;

    // Remove decimal part for mapping: Max speed (1.5m/s) -> Max servo command (255)
    RL_speed_ms_aux = (int)RL_speed_cms;
    RR_speed_ms_aux = (int)RR_speed_cms;

    if (RL_speed_ms_aux != 0)
      RL_speed_servo_cmd = map(RL_speed_ms_aux, -150, 150, -255, 255);
    else
      RL_speed_servo_cmd = 0;

    if (RR_speed_ms_aux != 0)
      RR_speed_servo_cmd = map(RR_speed_ms_aux, -150, 150, -255, 255);
    else
      RR_speed_servo_cmd = 0;

    // Set front motors to same speed as rear motors
    if (RL_speed_servo_cmd >= 0)
    {
      RL_motor->setSpeed(RL_speed_servo_cmd);
      RL_motor->run(FORWARD);
      FL_motor->setSpeed(RL_speed_servo_cmd);
      FL_motor->run(FORWARD);
    }
    else if (RL_speed_servo_cmd < 0)
    {
      RL_motor->setSpeed(-RL_speed_servo_cmd);
      RL_motor->run(BACKWARD);
      FL_motor->setSpeed(-RL_speed_servo_cmd);
      FL_motor->run(BACKWARD);
    }

    if (RR_speed_servo_cmd >= 0) {
      RR_motor->setSpeed(RR_speed_servo_cmd);
      RR_motor->run(FORWARD);
      FR_motor->setSpeed(RR_speed_servo_cmd);
      FR_motor->run(FORWARD);
    }
    else if (RR_speed_servo_cmd < 0) {
      RR_motor->setSpeed(-RR_speed_servo_cmd);
      RR_motor->run(BACKWARD);
      FR_motor->setSpeed(-RR_speed_servo_cmd);
      FR_motor->run(BACKWARD);
    }
  }
}

void setup() {
  RL_ticks = 0;
  last_RL_ticks = 0;
  RR_ticks = 0;
  last_RR_ticks = 0;
  RR_speed_ms = 0;
  RL_speed_ms = 0;

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

