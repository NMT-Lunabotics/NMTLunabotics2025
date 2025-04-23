
#include "helpers.hpp"
#include "arduino_lib.hpp"

int max_vel = 30;
Motor test(OutPin(6), OutPin(9), max_vel, false);
void setup() {
    // Initialize serial communication at 9600 baud
    Serial.begin(9600);

}
void loop() {
    for (int i = -max_vel; i <= max_vel; i++) {
        test.motor_ctrl(i);
        delay(10);
    }
    for (int i = max_vel; i >= -max_vel; i--) {
        test.motor_ctrl(i);
        delay(10);
    }
}