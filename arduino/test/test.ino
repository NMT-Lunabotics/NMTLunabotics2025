
#include "helpers.hpp"
#include "arduino_lib.hpp"

OutPin test(9);

void setup() {
    // Initialize serial communication at 9600 baud
    Serial.begin(9600);

}

void loop() {
    for (int i = 0; i < 255; i++) {
        test.write_pwm_raw(i);
        delay(10);
    }
    for (int i = 255; i >= 0; i--) {
        test.write_pwm_raw(i);
        delay(10);
    }
}