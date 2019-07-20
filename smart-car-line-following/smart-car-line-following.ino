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
#include <Servo.h>

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

#define speaker_pin 10
Servo servo;
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

int base_speed = 100;

void setup() {
  //Set data rate for bluetooth input
  Serial.begin(115200);

  //Set all global variables for normal, attentive operation
  current_state = 0;
  time_distracted = 0;
  now = millis();
  led = false;

  pinMode(trigger_pin, OUTPUT);
  pinMode(echo_pin, INPUT);
  servo.attach(speaker_pin);
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

  pinMode(speaker_pin, OUTPUT);

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

    base_speed = 255;
  } else if(line3 == 1) {
    leftMotor1.run(0x03);
    leftMotor2.run(0x03);
    rightMotor1.run(0x03);
    rightMotor2.run(0x03);
    
    base_speed = 255;
  } else {
    leftMotor1.run(0x01);
    leftMotor2.run(0x01);
    rightMotor1.run(0x01);
    rightMotor2.run(0x01);

    base_speed = 100;
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

void sound_buzzer(int level, boolean speaker) {

  int melody[] = {
    3500, 4978
  };

  if(speaker) {
    tone(speaker_pin, melody[level]);
    delay(10);
  } else {
    noTone(speaker_pin);
  }
}

int t = 0;

boolean speaker = false;

void loop() {

  //Steps 1 and 2
  //drive();

    int flag = Serial.readString().toInt();
    Serial.println(flag);
    if(flag == 1)
      servo.write(90);
    else
      servo.write(180);

//  switch(flag) {
//    case 1:
//      drive();
//      break;
//    case 2:
//      if(millis() - now > 1000) {
//        speaker = !speaker;
//        //sound_buzzer(0, speaker);
//        now = millis();
//      }
//      break;
//    case 3:
//      pull_over();
//      break;
//  }

/*
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
