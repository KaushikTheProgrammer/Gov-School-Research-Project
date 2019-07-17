#include <Servo.h>

Servo test;


void setup() {
  Serial.begin(115200); //initialize serial COM at 9600 baudrate
  pinMode(LED_BUILTIN, OUTPUT); //make the LED pin (13) as output
  digitalWrite (LED_BUILTIN, LOW);
  test.attach(10);

  test.write(0);
}

void loop() {

  if (Serial.available()) {
    int servoPos = Serial.readString().toInt();
    test.write(servoPos);
  }


}
