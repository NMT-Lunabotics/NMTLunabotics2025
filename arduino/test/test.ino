#include "helpers.hpp"

//////// MOTORS ////////
const int DACL1_PIN = 6;
const int DACL2_PIN = 9;
const int DACR1_PIN = 3;
const int DACR2_PIN = 5;

int motor_max_vel = 30; //rpm

// Motor speeds
int mL_speed = 0;
int mR_speed = 0;

// Set up motors
OutPin motor_left_dac1(DACL1_PIN);
OutPin motor_left_dac2(DACL2_PIN);
OutPin motor_right_dac1(DACR1_PIN);
OutPin motor_right_dac2(DACR2_PIN);
Motor motor_left(motor_left_dac1, motor_left_dac2, motor_max_vel, false);
Motor motor_right(motor_right_dac1, motor_right_dac2, motor_max_vel, true);

// Timing
int update_rate = 10; //hz
int feedback_rate = 10; //hz
int reset_int_rate = 2; //hz
unsigned long last_update_time = 0;
unsigned long last_feedback_time = 0;
unsigned long last_reset_int_time = 0;
unsigned long current_time = 0;
const unsigned long estop_timeout = 1000; // 1 second timeout
unsigned long last_message_time = 0;
bool emergency_stop = false;

void processMessage(byte* data, int length);

void setup() {
    Serial.begin(115200);
    Serial.flush();

}

void loop() {
    if (Serial.available() > 0) {
        if (Serial.read() == 0x02) { // Start byte
            while (Serial.available() < 1) {} // Wait for type and length bytes
            int length = Serial.read();
            while (Serial.available() < length + 1) {} // Wait for the entire message
            byte data[length];
            Serial.readBytes(data, length);
            while (Serial.available() < 1) {} // Wait for end byte
            if (Serial.read() == 0x03) { // End byte
                emergency_stop = false;
                last_message_time = millis();
                processMessage(data, length);
            } else {
                Serial.println("End byte not found");
            }
        }
    }

    current_time = millis();

    if (current_time - last_message_time > estop_timeout) {
        emergency_stop = true;
    }

    if (emergency_stop) {
        motor_left.motor_ctrl(0);
        motor_right.motor_ctrl(0);
    }

    if (current_time - last_update_time >= 1000 / update_rate) {
        last_update_time = current_time;

        if (!emergency_stop) { //TODO if not doomsday
            //Run motors
            motor_left.motor_ctrl(mL_speed);
            motor_right.motor_ctrl(mR_speed);
        }
    }

    if (current_time - last_feedback_time >= 1000 / feedback_rate) {
        last_feedback_time = current_time;

        // Send feedback
        if (emergency_stop) {
            Serial.println("Estopped");
        }
        Serial.println("<F," + String(mL_speed) + "," + String(mR_speed) + ">");
    }

}

void processMessage(byte* data, int length) {
    char type = data[0];

    switch (type) {
        case 'M': { // Motor control
            mL_speed = (int8_t)data[1];  // Adjusted index to skip the type byte
            mR_speed = (int8_t)data[2];
            break;
        }
        default:
            Serial.println("Unknown message type");
            break;
    }
}