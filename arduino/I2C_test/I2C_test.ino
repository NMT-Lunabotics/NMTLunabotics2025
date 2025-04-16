#include <Servo.h>

Servo myServo;  // Create a servo object

void setup() {
  myServo.attach(12);  // Attach the servo to pin 9
  Serial.begin(9600); // Start serial communication
  Serial.println("Servo Test Initialized");
}

void loop() {
  myServo.write(90);  // Move servo to 90 degrees
  delay(1000);        // Wait for 1 second
  myServo.write(0);   // Move servo to 0 degrees
  delay(1000);        // Wait for 1 second
}