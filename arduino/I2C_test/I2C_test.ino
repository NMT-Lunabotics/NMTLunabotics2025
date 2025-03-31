#include <Servo.h>

Servo myServo;  // Create a servo object

void setup() {
  myServo.attach(12);  // Attach the servo to pin 9
  Serial.begin(9600); // Start serial communication
  Serial.println("Servo Test Initialized");
}

void loop() {
  for (int pos = 0; pos <= 180; pos += 1) { // Sweep from 0 to 180 degrees
    myServo.write(pos);                    // Move the servo to the position
    delay(15);                             // Wait for the servo to reach the position
  }

  for (int pos = 180; pos >= 0; pos -= 1) { // Sweep back from 180 to 0 degrees
    myServo.write(pos);                     // Move the servo to the position
    delay(15);                              // Wait for the servo to reach the position
  }
}