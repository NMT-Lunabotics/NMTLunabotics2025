#include <Wire.h>
#include "helpers.hpp"

// Debug mode flag
bool debug_mode = false;

//TODO check pins

//////// ACTUATORS ////////
//Left side is L, right side is R, both is LR, bucket is B
// I2C addresses for actuators
#define AL_I2C_ADDRESS 0x58 // B0
#define AR_I2C_ADDRESS 0x5A // B4
#define AB_I2C_ADDRESS 0x59 // B2

// I2C registers for actuators
#define SPEED_REG 0x02
#define DIR_REG 0x00

// Actuator potentiometer read pins
#define POTL_PIN A0
#define POTR_PIN A1
#define POTB_PIN A2

// Actuator info
// Actuator stroke in mm
#define ALR_STROKE 191
#define AB_STROKE 140

// Actuator Calibration
#define AL_POT_MIN 32
#define AL_POT_MAX 869
#define AR_POT_MIN 32
#define AR_POT_MAX 869
#define AB_POT_MIN 29
#define AB_POT_MAX 782

float act_max_vel = 25; //mm/s
float act_threshold = 0.5; //mm
float act_error_factor = .01;
float act_max_error = 5; // mm

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
int update_rate = 20; //hz
unsigned long last_update_time = 0;
unsigned long current_time = 0;
const unsigned long estop_timeout = 1000; // 1 second timeout
unsigned long last_message_time = 0;
bool emergency_stop = false;

// Set up actuators
Actuator act_left(AL_I2C_ADDRESS, SPEED_REG, DIR_REG, POTL_PIN, false, 
                    ALR_STROKE, AL_POT_MIN, AL_POT_MAX, act_max_vel);
Actuator act_right(AR_I2C_ADDRESS, SPEED_REG, DIR_REG, POTR_PIN, false, 
                    ALR_STROKE, AR_POT_MIN, AR_POT_MAX, act_max_vel);
Actuator act_bucket(AB_I2C_ADDRESS, SPEED_REG, DIR_REG, POTB_PIN, false, 
                    AB_STROKE, AB_POT_MIN, AB_POT_MAX, act_max_vel);

// Set up motors
Motor motor_left(DACL1_PIN, DACL2_PIN, motor_max_vel);
Motor motor_right(DACR1_PIN, DACR2_PIN, motor_max_vel);

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
                processMessage(data, length);
            } else {
                Serial.println("End byte not found");
            }
        }
    }
    // TODO put back
    // if (emergency_stop) {
    //     act_left.actuator_ctrl(0);
    //     act_right.actuator_ctrl(0);
    //     act_bucket.actuator_ctrl(0);
    //     motor_left.motor_ctrl(0);
    //     motor_right.motor_ctrl(0);
    //     Serial.println("Estopped");
    // }

    current_time = millis();

    if (current_time - last_message_time > estop_timeout) {
        emergency_stop = true;
    }

    if (current_time - last_update_time >= 1000 / update_rate) {
        last_update_time = current_time;

        int aL_pos = act_left.pos_mm();
        int aR_pos = act_right.pos_mm();
        int aB_pos = act_bucket.pos_mm();

        // Actuator control
        if (aB_tgt > 0) {
            float aB_error = aB_tgt - aB_pos;
            if (abs(aB_error) > act_threshold) {
                aB_speed = aB_error > 0 ? act_max_vel : -act_max_vel;
            } else {
                aB_speed = 0;
            }
        }
        
        if (aLR_tgt > 0 ) {
            float aL_error = aLR_tgt - aL_pos;
            float aR_error = aLR_tgt - aR_pos;
            
            
            if (abs(aL_error) > act_threshold) {
                aL_speed = aL_error > 0 ? act_max_vel : -act_max_vel;
            } else {
                aL_speed = 0;
            }
            
            if (abs(aR_error) > act_threshold) {
                aR_speed = aR_error > 0 ? act_max_vel : -act_max_vel;
            } else {
                aR_speed = 0;
            }
        }

        float aLR_error = aL_pos - aR_pos;
            
        if (aLR_error > act_max_error) {
            emergency_stop = true;
        }
        
        float factor = act_error_factor * aLR_error;
        // aL_speed -= factor;
        // aR_speed += factor;
        
        // Send feedback
        Serial.println("<F," + String(aL_pos) + "," + String(aR_pos) + "," + String(aB_pos)
            + "," + String(aL_speed) + "," + String(aR_speed) + ',' + String(factor) + ">");

        //Run motors
        motor_left.motor_ctrl(mL_speed);
        motor_right.motor_ctrl(mR_speed);

        //Run actuators
        act_left.actuator_ctrl(aL_speed);
        act_right.actuator_ctrl(aR_speed);
        // act_bucket.actuator_ctrl(aB_speed);

        // //Run servo
        // //TODO implement

        //Run LEDs
        ledr_pin.write(led_r);
        ledy_pin.write(led_y);
        ledg_pin.write(led_g);
        ledb_pin.write(led_b);
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