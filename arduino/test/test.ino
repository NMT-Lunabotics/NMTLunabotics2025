#include <Wire.h>
#include "helpers.hpp"

// bool debug_mode = false;

//////// ACTUATORS ////////
//Left side is L, right side is R, both is LR, bucket is B
// I2C addresses for actuators
// B0: 58
// B2: 59
// B4: 5A
#define AL_I2C_ADDRESS 0x5A // B4
#define AR_I2C_ADDRESS 0x59 // B2
#define AB_I2C_ADDRESS 0x58 // B0

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
float act_max_error = 10; // mm

// Actuator targets
int aL_speed = 0;
int aR_speed = 0;
int aB_speed = 0;

float aL_pos = 0;
float aR_pos = 0;
float aB_pos = 0;

int aLR_tgt = -1;
int aB_tgt = -1;

// Timing
int update_rate = 150; //hz
int feedback_rate = 10; //hz
int reset_int_rate = 10; //hz
unsigned long last_update_time = 0;
unsigned long last_feedback_time = 0;
unsigned long last_reset_int_time = 0;
unsigned long current_time = 0;
const unsigned long estop_timeout = 1000; // 1 second timeout
unsigned long last_message_time = 0;
bool emergency_stop = false;
bool doomsday = false;

// Set up PID controllers
PID pidL(2.2, 0.0022, 0.34, 2.0);
PID pidR(1.85, 0.0018, 0.31, 1.7);
PID pidB(3.0, 0.001, 0.4);
float vel_gain = 1.0;

// Set up actuators
Actuator act_left(AL_I2C_ADDRESS, SPEED_REG, DIR_REG, POTL_PIN, true, 
                    ALR_STROKE, AL_POT_MIN, AL_POT_MAX, act_max_vel, pidL);
Actuator act_right(AR_I2C_ADDRESS, SPEED_REG, DIR_REG, POTR_PIN, true, 
                    ALR_STROKE, AR_POT_MIN, AR_POT_MAX, act_max_vel, pidR);
Actuator act_bucket(AB_I2C_ADDRESS, SPEED_REG, DIR_REG, POTB_PIN, false, 
                    AB_STROKE, AB_POT_MIN, AB_POT_MAX, act_max_vel, pidB);

// void processMessage(byte* data, int length);

void setup(){
    Serial.begin(115200);
    Serial.flush();
    Wire.begin();
}

void loop() {
    // Get and print the position of each actuator
    float left_pos = act_left.update_pos();
    float right_pos = act_right.update_pos();

    Serial.print("Left Actuator Position: ");
    Serial.println(left_pos);
    Serial.print("Right Actuator Position: ");
    Serial.println(right_pos);

    act_left.vel_ctrl(-25);
    act_right.vel_ctrl(-25);
}
