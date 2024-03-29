/*
 * GSET 2019 Smart Car Protocols
 * Date: 2019-07-03
 * Author: Mihir Laud
 * 
 * Purpose: To drive an autonomous Arduino car using line tracking and ultrasonic sensors. 
 * 
 */


/*
 * Includes:
 *  - Library for Ultrasonic Sensor HC-SR04
 */
#include <HCSR04.h>

/*
 *  Defined Constants:
 *   - Drive Motors
 *     - driveFL: Front left drive motor port
 *     - driveFR: Front right drive motor port
 *     - driveBL: Back left drive motor port
 *     - driveBR: Back right drive motor port
 *   - Drive Modes:
 *     - keep_straight: Keep the car going straight
 *     - turn_right: Turn car right 
 *     - turn_left: Turn car left
 *   - TURNING_CONSTANT: Defines how sharply the robot will turn
 *                       when it determines it must
 *   - STOPPING_DISTANCE: Defines how far away in cm from an object
 *                        the car must stop if it must
 *   - Sensors
 *     - Left line sensor: 1
 *     - Right line sensor: 2
 *   - Ultrasonic Sensor Pins:
 *     - trigger_pin: Sensor trigger pin
 *     - echo_pin: Sensor echo pin
 *   - led_port: LED port
 */

#define driveFL 1
#define driveFR 2
#define driveBL 3
#define driveBR 4

#define keep_straight 0
#define turn_right 1
#define turn_left -1

#define TURNING_CONSTANT 50

#define STOPPING_DISTANCE 20

#define lineLeft 1
#define lineRight 2

#define trigger_pin 1
#define echo_pin 2

#define led_port 0

/*
 * Global Variables
 *  - sensor: Reference to ultrasonic sensor
 *  - current_state: Refers to current 'distractedness' state
 *  - time_distracted: How long the driver has been distracted for
 *                     since the previous state change
 *  - cooldown: How long the driver has been atentive for since the
 *              last time they were distracted
 *  - now: The current system time
 *  - interval: The interval between LED blinks, as determined by
 *              the current_state
 *  - led: Whether or not the led is currently on
 */

UltraSonicDistanceSensor sensor(trigger_pin, echo_pin);
int current_state;
int time_distracted;
int cooldown;
int last_check;
int now;
long interval;
boolean led;

void setup() {
  //Set data rate for bluetooth input
  Serial.begin(9600);

  //Set all global variables for normal, attentive operation
  current_state = 0;
  time_distracted = 0;
  last_check = millis();
  led = false;

  //Set line sensor pins to input pins
  pinMode(lineLeft, INPUT);
  pinMode(lineRight, INPUT);

  //Set drive motor pins to output pins
  pinMode(driveFL, OUTPUT);
  pinMode(driveFR, OUTPUT);
  pinMode(driveBL, OUTPUT);
  pinMode(driveBR, OUTPUT);

  //Set LED pin to output pin
  pinMode(led_port, OUTPUT);
}


/*
 * Autonomous Drive Function:
 * 
 * Takes in information from line sensors and
 * ultrasonic sensor and sends motor commands
 * based on this.
 * 
 * turn_mode - Turn mode from line_tracking function
 * 
 */

void drive(int turn_mode) {

  // Each motor starts with y as the base power
  // and x is then either added or subtracted
  // based on which side of the car the motor is on.
  analogWrite(driveFL, 127 + turn_mode * TURNING_CONSTANT);
  analogWrite(driveFR, 127 - turn_mode * TURNING_CONSTANT);
  analogWrite(driveBL, 127 + turn_mode * TURNING_CONSTANT);
  analogWrite(driveBR, 127 - turn_mode * TURNING_CONSTANT);
}

/*
 * Line Tracking Function:
 * 
 * Takes information from both line sensors and
 * determines whether to turn left, turn right,
 * or keep going straight.
 */

int line_tracking() {

  /*
   *  Line Sensor Truth Table:
   *  
   *   Left | Right | Result
   *  ------------------------ 
   *    0   |   0   |  Stop
   *    1   |   0   |  Left
   *    0   |   1   |  Right
   *    1   |   1   | Straight
   */
  
  if(digitalRead(lineLeft) && digitalRead(lineRight)) {
    return keep_straight;
  } else if(digitalRead(lineLeft) && !digitalRead(lineRight)) {
    return turn_right;
  } else if(!digitalRead(lineLeft) && digitalRead(lineRight)) {
    return turn_left;
  }
}

/*
 * Change State Function:
 * 
 * Changes the state of the LED blink interval
 * based on the current_state of the program.
 */

int change_state() {
  switch(current_state) {
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
 * Main Loop:
 * 
 * Loop will proceed with the following structure
 * 
 * 1. Take in information from line_tracking function
 * 2. Send motor commands based on line tracking 
 *    and distance sensing
 * 3. Read bluetooth input; if true, increment
 *    time_distracted; else, increment cooldown
 * 4. Change state if necessary based on time_distracted (a)
 *    and cooldown (b)
 * 5. Blink the LDE based on the determined interval
 * 6. If current_state is 4, start pulling over
 */

void loop() {

  //Steps 1 and 2
  drive(line_tracking());

  //Step 3
  if(Serial.read()) {
    time_distracted++;
  } else {
    cooldown++;
  }

  //Step 4a
  if(time_distracted == 500) {
    time_distracted = 0;
    current_state++;
    change_state();
  }

  //Step 4b
  if(cooldown == 1000) {
    time_distracted = 0;
    cooldown = 0;
    current_state--;
    change_state();
  }

  //Step 5
  now = millis();
  if(now > last_check + interval) {
    last_check = now;
    led = !led;
    if(led) {
      digitalWrite(led_port, HIGH);
    } else {
      digitalWrite(led_port, LOW);
    }
  }

  //Step 6
  if(current_state == 4) {
    if(sensor.measureDistanceCm() > 20) {
      //TODO write pull over protocol
    }
    exit(0);  
  }
  
}
