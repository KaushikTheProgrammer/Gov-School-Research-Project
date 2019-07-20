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

  double left_command = 0.20 + (kP * error + kI * total_error + kD * d_error);

  if(left_command > 1) left_command = 1;
  if(left_command < 0) left_command = 0;
  
  double right_command = 0.20 - (kP * error + kI * total_error + kD * d_error);

  if(right_command > 1) right_command = 1;
  if(right_command < 0) right_command = 0;

  leftMotor1.setSpeed(255 * left_command);
  leftMotor2.setSpeed(255 * left_command);
  rightMotor1.setSpeed(255 * right_command);
  rightMotor2.setSpeed(255 * right_command);
}

/*
   Change State Function:

   Changes the state of the LED blink interval
   based on the current_state of the program.
*/

int change_state() {
  switch (current_state) {
    case 0:
      // interval set to 1 * 10^6 s to make sure LED
      // does not blink during attentive operation
      interval = 1e6;
      break;
    case 1:
      // interval is 1 s during slight distraction
      interval = 1000;
      break;
    case 2:
      // interval is 0.5 s during moderate distraction
      interval = 500;
      break;
    case 3:
      // interval is 0 s during severe distractions
      // This ensures that the LED is constantly on
      interval = 0;
      break;
  }
}

/*
   Main Loop:

   Loop will proceed with the following structure

   1. Take in information from line_tracking function
   2. Send motor commands based on line tracking
      and distance sensing
   3. Read bluetooth input; if true, increment
      time_distracted; else, increment cooldown
   4. Change state if necessary based on time_distracted (a)
      and cooldown (b)
   5. Blink the LDE based on the determined interval
   6. If current_state is 4, start pulling over
*/

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

int t = 0;
int flag = 0;
void loop() {

  //Steps 1 and 2
  drive();

  //if(t == 0)
  //  pull_over();
  //t++;

  if(Serial.available()) {
    flag = Serial.read();  
  }

  //Step 3
  if (flag) {
    cooldown = 0;
    time_distracted++;
    flag = 0;
  } else {
    cooldown++;
  }

  //Step 4a
  if (time_distracted == 500) {
    time_distracted = 0;
    current_state++;
    change_state();
  }

  //Step 4b
  if (cooldown == 1000) {
    time_distracted = 0;
    cooldown = 0;
    current_state--;
    change_state();
  }

/*
  //Step 5
  now = millis();
  if (now > last_check + interval) {
    last_check = now;
    led = !led;
    if (led) {
      digitalWrite(led_port, HIGH);
    } else {
      digitalWrite(led_port, LOW);
    }
  }

  //Step 6
  if (current_state == 4) {
//    if (sensor.measureDistanceCm() > 20) {
      //TODO write pull over protocol
//    }
    exit(0);
  }
*/
}
