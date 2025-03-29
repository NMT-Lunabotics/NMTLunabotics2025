#include <Wire.h>
#include "helpers.hpp"

// Debug mode flag
bool debug_mode = false;
bool calibrate_actuators_flag = false;
//TODO check pins

//////// ACTUATORS ////////
//Left side is L, right side is R, both is LR, bucket is B
// I2C addresses for actuators
#define AL_I2C_ADDRESS 0x5A // B0
#define AR_I2C_ADDRESS 0x58 // B4
#define AB_I2C_ADDRESS 0x59 // B2

// I2C registers for actuators
#define SPEED_REG 0x02
#define DIR_REG 0x00

// Actuator potentiometer read pins
#define POTL_PIN A1
#define POTR_PIN A0
#define POTB_PIN A2

// Actuator info
// Actuator stroke in mm
#define ALR_STROKE 191
#define AB_STROKE 140

// Actuator Calibration
#define AL_POT_MIN 53
#define AL_POT_MAX 883
#define AR_POT_MIN 3
#define AR_POT_MAX 837
#define AB_POT_MIN 30
#define AB_POT_MAX 782

float act_max_vel = 25; //mm/s
float act_max_error = 3; // mm

// Actuator targets
int aL_speed = 0;
int aR_speed = 0;
int aB_speed = 0;

float aL_pos = 0;
float aR_pos = 0;
float aB_pos = 0;

int aLR_tgt = -1;
int aB_tgt = -1;

//////// MOTORS ////////
#define DACL1_PIN 3
#define DACL2_PIN 5
#define DACR1_PIN 6
#define DACR2_PIN 9

int motor_max_vel = 21; //rpm

// Motor speeds
int mL_speed = 0;
int mR_speed = 0;

//TODO implement servo logic
#define SERVO_PIN 12

// Servo
bool servo_state = false;

// LED's
// TODO implement
#define LEDR_PIN 2
#define LEDY_PIN 4
#define LEDG_PIN 7
#define LEDB_PIN 8

bool led_r = false;
bool led_y = false;
bool led_g = false;
bool led_b = false;

// Timing
int update_rate = 150; //hz
int feedback_rate = 10; //hz
int reset_int_rate = 2; //hz
unsigned long last_update_time = 0;
unsigned long last_feedback_time = 0;
unsigned long last_reset_int_time = 0;
unsigned long current_time = 0;
//const unsigned long estop_timeout = 1000; // 1 second timeout
//unsigned long last_message_time = 0;
bool emergency_stop = false;
bool doomsday = false;

// Set up PID controllers
PID pidL(2.2, 0.0022, 0.34, 2.0);
PID pidR(1.85, 0.0018, 0.31, 1.7);
PID pidB(3.0, 0.001, 0.4);
float vel_gain = 2.0;

// Set up actuators
Actuator act_left(AL_I2C_ADDRESS, SPEED_REG, DIR_REG, POTL_PIN, false, 
                    ALR_STROKE, AL_POT_MIN, AL_POT_MAX, act_max_vel, pidL);
Actuator act_right(AR_I2C_ADDRESS, SPEED_REG, DIR_REG, POTR_PIN, false, 
                    ALR_STROKE, AR_POT_MIN, AR_POT_MAX, act_max_vel, pidR);
Actuator act_bucket(AB_I2C_ADDRESS, SPEED_REG, DIR_REG, POTB_PIN, false, 
                    AB_STROKE, AB_POT_MIN, AB_POT_MAX, act_max_vel, pidB);

// Set up motors
Motor motor_left(DACL1_PIN, DACL2_PIN, motor_max_vel, false);
Motor motor_right(DACR1_PIN, DACR2_PIN, motor_max_vel, true);

// Set up LEDs
OutPin ledr_pin(LEDR_PIN);
OutPin ledy_pin(LEDY_PIN);
OutPin ledg_pin(LEDG_PIN);
OutPin ledb_pin(LEDB_PIN);

// Set up servo
OutPin servo_pin(SERVO_PIN);

void processMessage(byte* data, int length);

void setup(){
    Serial.begin(115200);
    Serial.flush();
    Wire.begin();
    // if (calibrate_actuators_flag) {
    //     calibrateActuators(act_left, act_right, act_bucket);
    // }
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

    if (emergency_stop || doomsday) {
        act_left.stop();
        act_right.stop();
    //     // act_bucket.stop(); TODO
        motor_left.motor_ctrl(0);
        motor_right.motor_ctrl(0);
    }

    current_time = millis();

    //if (current_time - last_message_time > estop_timeout) {
    //    emergency_stop = true;
    //}

    if (current_time - last_update_time >= 1000 / update_rate) {
        last_update_time = current_time;

        aL_pos = act_left.update_pos();
        aR_pos = act_right.update_pos();
        aB_pos = act_bucket.update_pos();

        if (abs(aL_pos - aR_pos) >= act_max_error) {
            doomsday = true;
        } else {
            doomsday = false;
        }

        //Run actuators
        aLR_tgt = constrain(aLR_tgt, -1, ALR_STROKE);

        if (!doomsday && !emergency_stop) {
            if (aLR_tgt >= 0) {
                act_left.tgt_ctrl(aLR_tgt, aR_pos);
                act_right.tgt_ctrl(aLR_tgt, aL_pos);
            } else {
                float factor = (aL_pos - aR_pos) * vel_gain;
                act_left.vel_ctrl(aL_speed - factor);
                act_right.vel_ctrl(aR_speed + factor);
            }

            // if (aB_tgt >= 0) {
            //     act_bucket.tgt_ctrl(aB_tgt);
            // } else {
            //     act_bucket.vel_ctrl(aB_speed);
            // } TODO

            // //Run servo
            // //TODO implement

            //Run motors
            motor_left.motor_ctrl(mL_speed);
            motor_right.motor_ctrl(mR_speed);
        }

        //Run LEDs
        ledr_pin.write(led_r);
        ledy_pin.write(led_y);
        ledg_pin.write(led_g);
        ledb_pin.write(led_b);
    }

    if (current_time - last_feedback_time >= 1000 / feedback_rate) {
        last_feedback_time = current_time;

        // Send feedback
        if (doomsday) {
            Serial.println("Doomsday");
        } else if (emergency_stop) {
            Serial.println("Estopped");
        }
        Serial.println("<F," + String(aL_pos) + "," + String(aR_pos) + "," + String(aB_pos)
            + "," + String(aL_speed) + "," + String(aR_speed) + ',' + String(aLR_tgt)
            + "," + String(mL_speed) + "," + String(mR_speed) + ">");
        // Serial.print("aL_pos:");
        // Serial.print(aL_pos);
        // Serial.print(",");
        // Serial.print("aR_pos:");
        // Serial.println(aR_pos);
    }

    if (current_time - last_reset_int_time >= 1000 / reset_int_rate) {
        last_reset_int_time = current_time;

        act_left.resetPIDIntegral();
        act_right.resetPIDIntegral();
        act_bucket.resetPIDIntegral();
    }
}

void processMessage(byte* data, int length) {
    char type = data[0];
    if (debug_mode) {
        Serial.print("Received message of type: ");
        Serial.println(type);
        Serial.print("Length: ");
        Serial.println(length);
        Serial.print("Data: ");
        for (int i = 0; i < length; i++) {
            Serial.print(data[i], HEX);
            Serial.print(" ");
        }
        Serial.println();
    }

    switch (type) {
        case 'A': { // Actuator control
            aLR_tgt = (int16_t)((data[1] << 8) | data[2]);  // Adjusted index to skip the type byte
            aB_tgt = (int16_t)((data[3] << 8) | data[4]);
            aL_speed = (int8_t)data[5];
            aR_speed = aL_speed;
            aB_speed = (int8_t)data[6];
            if (debug_mode) {
                Serial.print("Arm Position: ");
                Serial.println(aLR_tgt);
                Serial.print("Bucket Position: ");
                Serial.println(aB_tgt);
                Serial.print("Arm Velocity: ");
                Serial.println(aL_speed);
                Serial.print("Bucket Velocity: ");
                Serial.println(aB_speed);
            }
            break;
        }
        case 'M': { // Motor control
            mL_speed = (int8_t)data[1];  // Adjusted index to skip the type byte
            mR_speed = (int8_t)data[2];
            if (debug_mode) {
                Serial.print("Left Speed: ");
                Serial.println(mL_speed);
                Serial.print("Right Speed: ");
                Serial.println(mR_speed);
            }
            break;
        }
        case 'S': { // Servo control
            servo_state = data[1];  // Adjusted index to skip the type byte
            if (debug_mode) {
                Serial.print("Servo State: ");
                Serial.println(servo_state);
            }
            break;
        }
        case 'L': { // LED control
            led_r = data[1];  // Adjusted index to skip the type byte
            led_y = data[2];
            led_g = data[3];
            led_b = data[4];
            if (debug_mode) {
                Serial.print("Red: ");
                Serial.println(led_r);
                Serial.print("Yellow: ");
                Serial.println(led_y);
                Serial.print("Green: ");
                Serial.println(led_g);
                Serial.print("Blue: ");
                Serial.println(led_b);
            }
            break;
        }
        default:
            Serial.println("Unknown message type");
            break;
    }
}

// void calibrateActuators(Actuator act_left, Actuator act_right, Actuator act_bucket) {
//     act_left.set_speed(act_max_vel);
//     act_right.set_speed(act_max_vel);
//     act_bucket.set_speed(act_max_vel);
//     delay(10000);
//     act_left.calibrate_pot();
//     act_right.calibrate_pot();
//     act_bucket.calibrate_pot();
//     act_left.set_speed(-act_max_vel);
//     act_right.set_speed(-act_max_vel);
//     act_bucket.set_speed(-act_max_vel);
//     delay(10000);
//     Serial.println("Calibration complete: Biases set");
//     Serial.print("Left Actuator: ");
//     Serial.println(act_left.pos_mm());
//     Serial.print("Right Actuator: ");
//     Serial.println(act_right.pos_mm());
//     Serial.print("Bucket Actuator: ");
//     Serial.println(act_bucket.pos_mm());
//     Serial.println("Calibration Complete");
// }
