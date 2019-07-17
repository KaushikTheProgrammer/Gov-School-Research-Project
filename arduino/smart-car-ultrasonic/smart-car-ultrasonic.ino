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

#define led_port 0

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

void setup() {
  //Set data rate for bluetooth input
  Serial.begin(38400);

  //Set all global variables for normal, attentive operation
  current_state = 0;
  time_distracted = 0;
  last_check = millis();
  led = false;

  //Set LED pin to output pin
  pinMode(led_port, OUTPUT);

  pinMode(trigger_pin, OUTPUT);
  pinMode(echo_pin, INPUT);
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

int target = 6;

int total_error = 0;
int previous_error = readPing();
double baseSpeed = 85.0;

void drive() {
  int current_value = readPing();
  
  int error = current_value - target;
  
  if(fabs(error) < 1.0) {
    total_error += error;
  } else {
    total_error = 0;  
  }

  int d_error = error - previous_error;

  double kP = 0.0800;
  double kI = 0.0000;
  double kD = 0.0000;

  // Each motor starts with y as the base power
  // and x is then either added or subtracted
  // based on which side of the car the motor is on.
  leftMotor1.run(0x01);
  leftMotor2.run(0x01);
  rightMotor1.run(0x01);
  rightMotor2.run(0x01);
  
  double base_command = kP * error + kI * total_error + kD * d_error;

  if (base_command < 0) {
    base_command *= 1;
    leftMotor1.run(0x02);
    leftMotor2.run(0x02);
    rightMotor1.run(0x02);
    rightMotor2.run(0x02);
  } else if (base_command > 1) {
    base_command = 1;
  } 

  base_command *= baseSpeed;

  double right_command = base_command + baseSpeed;
  double left_command = base_command - baseSpeed;

  Serial.print(base_command);
  Serial.print('\t');
  Serial.print(right_command);
  Serial.print('\t');
  Serial.println(left_command);
  
  
  leftMotor1.setSpeed(base_command += baseSpeed);
  leftMotor2.setSpeed(base_command += baseSpeed);
  rightMotor1.setSpeed(base_command -= baseSpeed);
  rightMotor2.setSpeed(base_command -= baseSpeed);
}

void sound_buzzer() {
  
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

String flag = "";
void loop() {

  //drive();

  if(Serial.available()) {
    flag = Serial.readString();
    Serial.println(flag);

    switch(flag) {
      case "1":
        break;
      case "2":
        sound_buzzer();
        break;
      case "3":
        pull_over();
        break;
    }
    
  }

}
