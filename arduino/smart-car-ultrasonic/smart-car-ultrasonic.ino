/*
   GSET 2019 Smart Car Protocols
   Date: 2019-07-03
   Author: Mihir Laud

   Purpose: To drive an autonomous Arduino car using line tracking and ultrasonic sensors.

*/

/*
   Includes:
    - Library for Ultrasonic Sensor HC-SR04
*/

#include <HCSR04.h>

/*
    Defined Constants:
     - Drive Motors
       - driveFL: Front left drive motor port
       - driveFR: Front right drive motor port
       - driveBL: Back left drive motor port
       - driveBR: Back right drive motor port
     - Ultrasonic Sensor Pins:
       - trigger_pin: Sensor trigger pin
       - echo_pin: Sensor echo pin
     - led_port: LED port
*/

#include "UCMotor.h"

UC_DCMotor leftMotor1(3, MOTOR34_64KHZ);
UC_DCMotor rightMotor1(4, MOTOR34_64KHZ);
UC_DCMotor leftMotor2(1, MOTOR34_64KHZ);
UC_DCMotor rightMotor2(2, MOTOR34_64KHZ);

#define trigger_pin A2
#define echo_pin A3

/*
   Global Variables
    - sensor: Reference to ultrasonic sensor
    - current_state: Refers to current 'distractedness' state
    - time_distracted: How long the driver has been distracted for
                       since the previous state change
    - cooldown: How long the driver has been atentive for since the
                last time they were distracted
    - now: The current system time
    - interval: The interval between LED blinks, as determined by
                the current_state
    - led: Whether or not the led is currently on
*/

//UltraSonicDistanceSensor sensor(trigger_pin, echo_pin);
int current_state;

int time_distracted;
int cooldown;
int last_check;
int now;
long interval;
boolean led;

#define SPEAKER_PIN 13

void setup() {
  //Set data rate for bluetooth input
  Serial.begin(115200);

  //Set all global variables for normal, attentive operation
  current_state = 0;
  time_distracted = 0;
  last_check = millis();
  led = false;

  pinMode(trigger_pin, OUTPUT);
  pinMode(echo_pin, INPUT);

  pinMode(SPEAKER_PIN, OUTPUT);

  now = millis();
}

int readPing()
{
  // establish variables for duration of the ping,
  // and the distance result in inches and centimeters:
  long duration, cm;
  // The PING))) is triggered by a HIGH pulse of 2 or more microseconds.
  // Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
  pinMode(trigger_pin, OUTPUT);
  digitalWrite(trigger_pin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigger_pin, HIGH);
  delayMicroseconds(3);
  digitalWrite(trigger_pin, LOW);

  pinMode(echo_pin, INPUT);
  duration = pulseIn(echo_pin, HIGH);

  // convert the time into a distance
  cm = microsecondsToCentimeters(duration);
  return cm ;
}

long microsecondsToCentimeters(long microseconds)
{
  // The speed of sound is 340 m/s or 29 microseconds per centimeter.
  // The ping travels out and back, so to find the distance of the
  // object we take half of the distance travelled.
  return microseconds / 29 / 2;
}


/*
   Autonomous Drive Function:

   Takes in information from line sensors and
   ultrasonic sensor and sends motor commands
   based on this.

   turn_mode - Turn mode from line_tracking function

*/

int target = 8;

int total_error = 0;
int previous_error = readPing();

void drive() {
  int current_value = readPing();
  
  int error = current_value - target;
  
  if(fabs(error) < 1.0) {
    total_error += error;
  } else {
    total_error = 0;  
  }

  int d_error = error - previous_error;
  
  //if(fabs(d_error) > 10)
  //  target = current_value;

  double kP = 0.1000;
  double kI = 0.0000;
  double kD = 0.0000;

  leftMotor1.run(0x01);
  leftMotor2.run(0x01);
  rightMotor1.run(0x01);
  rightMotor2.run(0x01);
  
  double base_command = kP * error + kI * total_error + kD * d_error;

  double left_command = 0.15 + base_command;
  double right_command = 0.15 - base_command;

  if(left_command > 1) left_command = 1;
  if(left_command < 0) left_command = 0;

  if(right_command > 1) right_command = 1;
  if(right_command < 0) right_command = 0;

  leftMotor1.setSpeed(255 * left_command);
  leftMotor2.setSpeed(255 * left_command);
  rightMotor1.setSpeed(255 * right_command);
  rightMotor2.setSpeed(255 * right_command);

  Serial.print(base_command);
  Serial.print("\t");
  Serial.print(left_command);
  Serial.print("\t");
  Serial.println(right_command);

}

void sound_buzzer(int level, boolean speaker) {

  int melody[] = {
    3500, 4978
  };

  if(speaker) {
    tone(SPEAKER_PIN, melody[level]);
  } else {
    noTone(SPEAKER_PIN);
  }
}

void pull_over() {
  leftMotor1.run(0x01);
  leftMotor2.run(0x01);
  rightMotor1.run(0x01);
  rightMotor2.run(0x01);
  
  for(double x = 0; x < 100; x++) {
    leftMotor1.setSpeed(128 + (-0.008 * pow(x - 50, 2) + 20));
    leftMotor2.setSpeed(128 + (-0.008 * pow(x - 50, 2) + 20));
    rightMotor1.setSpeed(128 +(0.008 * pow(x - 50, 2) - 20));
    rightMotor2.setSpeed(128 +(0.008 * pow(x - 50, 2) - 20));
    
    delay(20);
  }

  leftMotor1.setSpeed(0);
  leftMotor2.setSpeed(0);
  rightMotor1.setSpeed(0);
  rightMotor2.setSpeed(0);
  
}

int flag = 2;
boolean speaker = false;
void loop() {

  drive();

  if(Serial.available()) {
    flag = Serial.readString().toInt();
    Serial.println(flag);
  }

  switch(flag) {
    case 1:
      break;
    case 2:
      if(millis() - now > 200) {
        speaker = !speaker;
        sound_buzzer(0, speaker);
        now = millis();
      }
      break;
    case 3:
      pull_over();
      break;
  }

}
