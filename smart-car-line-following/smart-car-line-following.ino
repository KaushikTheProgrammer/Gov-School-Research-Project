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

#define leftSensor    A0
#define middleSensor  A1
#define rightSensor   13

#define led_pin 10
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

int base_speed = 250;

void setup() {
  //Set data rate for bluetooth input
  Serial.begin(115200);

  //Set all global variables for normal, attentive operation
  current_state = 0;
  time_distracted = 0;
  now = millis();

  pinMode(trigger_pin, OUTPUT);
  pinMode(echo_pin, INPUT);

  pinMode(led_pin, OUTPUT);

  delay(1000);
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

  pinMode(led_pin, OUTPUT);

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
  int line1 = digitalRead(leftSensor);
  int line2 = digitalRead(middleSensor);
  int line3 = digitalRead(rightSensor);

  if(line1 == 1) {
    leftMotor1.run(0x04);
    leftMotor2.run(0x04);
    rightMotor1.run(0x04);
    rightMotor2.run(0x04);

  } else if(line3 == 1) {
    leftMotor1.run(0x03);
    leftMotor2.run(0x03);
    rightMotor1.run(0x03);
    rightMotor2.run(0x03);
    
  } else {
    leftMotor1.run(0x01);
    leftMotor2.run(0x01);
    rightMotor1.run(0x01);
    rightMotor2.run(0x01);

  }

  leftMotor1.setSpeed(base_speed);
  leftMotor2.setSpeed(base_speed);
  rightMotor1.setSpeed(base_speed);
  rightMotor2.setSpeed(base_speed);
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
  leftMotor1.run(0x03);
  leftMotor2.run(0x03);
  rightMotor1.run(0x03);
  rightMotor2.run(0x03);

  leftMotor1.setSpeed(150);
  leftMotor2.setSpeed(150);
  rightMotor1.setSpeed(150);
  rightMotor2.setSpeed(150);

  delay(375);

  leftMotor1.setSpeed(0);
  leftMotor2.setSpeed(0);
  rightMotor1.setSpeed(0);
  rightMotor2.setSpeed(0);

  delay(250);

  leftMotor1.run(0x02);
  leftMotor2.run(0x02);
  rightMotor1.run(0x02);
  rightMotor2.run(0x02);

  leftMotor1.setSpeed(90);
  leftMotor2.setSpeed(90);
  rightMotor1.setSpeed(90);
  rightMotor2.setSpeed(90);

  delay(750);

  leftMotor1.setSpeed(0);
  leftMotor2.setSpeed(0);
  rightMotor1.setSpeed(0);
  rightMotor2.setSpeed(0);

  delay(250);

  leftMotor1.run(0x04);
  leftMotor2.run(0x04);
  rightMotor1.run(0x04);
  rightMotor2.run(0x04);

  leftMotor1.setSpeed(150);
  leftMotor2.setSpeed(150);
  rightMotor1.setSpeed(150);
  rightMotor2.setSpeed(150);

  delay(375);

  leftMotor1.setSpeed(0);
  leftMotor2.setSpeed(0);
  rightMotor1.setSpeed(0);
  rightMotor2.setSpeed(0);

  delay(250);

  leftMotor1.run(0x01);
  leftMotor2.run(0x01);
  rightMotor1.run(0x01);
  rightMotor2.run(0x01);

  leftMotor1.setSpeed(75);
  leftMotor2.setSpeed(75);
  rightMotor1.setSpeed(75);
  rightMotor2.setSpeed(75);

  delay(600);

  leftMotor1.setSpeed(0);
  leftMotor2.setSpeed(0);
  rightMotor1.setSpeed(0);
  rightMotor2.setSpeed(0);

  delay(250);

}

int t = 0;
int flag = 0;
boolean led = false;
int c = 0;

boolean done = false;
boolean block = false;

void loop() {
  if(!done) {
    if(Serial.available() && !block) {
      flag = Serial.read();
    }


    if(flag == '1') {
      base_speed = 150;
      drive();
      digitalWrite(led_pin, LOW);
    } else if(flag == '2') {
      base_speed = 120;
      drive();
      if(millis() - now > 1000) {
        now = millis();
        led = !led;
        if(led)
          digitalWrite(led_pin, HIGH);
        else
          digitalWrite(led_pin, LOW);
      }
    } else if(flag == '3') {
      block = true;
      digitalWrite(led_pin, HIGH);
      base_speed = 100;
      drive();
      if(readPing() > 18) {
        c++;
        if(c > 300) {
          leftMotor1.setSpeed(0);
          leftMotor2.setSpeed(0);
          rightMotor1.setSpeed(0);
          rightMotor2.setSpeed(0);
          delay(1000);
          pull_over();
          done = true;
        }
      } else {
        c = 0;  
      }
    }
  }
}
