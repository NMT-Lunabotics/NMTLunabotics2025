#include <Wire.h>
#include "helpers.hpp"
#define AL_I2C_ADDRESS 0x5A // B0
// #define AR_I2C_ADDRESS 0x58 // B4

// I2C registers for actuators
#define SPEED_REG 0x02
#define DIR_REG 0x00

// Actuator potentiometer read pins
#define POTL_PIN A1
// #define POTR_PIN A0

// Actuator stroke in mm
#define ALR_STROKE 191

// Actuator Calibration
#define AL_POT_MIN 49
#define AL_POT_MAX 888
// #define AR_POT_MIN 1
// #define AR_POT_MAX 834

float act_max_vel = 25; //mm/s

float aLR_tgt = 100;

float aL_pos = 0;

// Set up PID controllers
PID pidL(2.5, 0.00, 0.5, 0.2);
// PID pidR(2.5, 0.00, 0.5, 0.2);

Actuator act_left(AL_I2C_ADDRESS, SPEED_REG, DIR_REG, POTL_PIN, false, 
                    ALR_STROKE, AL_POT_MIN, AL_POT_MAX, act_max_vel, pidL);
// Actuator act_right(AR_I2C_ADDRESS, SPEED_REG, DIR_REG, POTR_PIN, false, 
                    // ALR_STROKE, AR_POT_MIN, AR_POT_MAX, act_max_vel, pidR);

void setup() {
  Serial.begin(115200);
  Wire.begin();
}

void loop () {
  act_left.vel_ctrl(-act_max_vel);
  aL_pos = act_left.update_pos();
  Serial.println(aL_pos);
  delay(10);
}