#include "helpers.hpp"
#include "arduino_lib.hpp"

int max_vel = 30;
Motor leftMotor(OutPin(6), OutPin(9), max_vel, false);
Motor rightMotor(OutPin(3), OutPin(5), max_vel, true);

void setup() {
    // Initialize serial communication at 9600 baud
    Serial.begin(9600);
}

void loop() {
    for (int i = -max_vel; i <= max_vel; i++) {
        leftMotor.motor_ctrl(i);
        rightMotor.motor_ctrl(i);
        delay(100);
    }
    for (int i = max_vel; i >= -max_vel; i--) {
        leftMotor.motor_ctrl(i);
        rightMotor.motor_ctrl(i);
        delay(100);
    }
}