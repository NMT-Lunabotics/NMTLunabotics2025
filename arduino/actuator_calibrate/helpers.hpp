#ifndef HELPERS_H
#define HELPERS_H

#include "arduino_lib.hpp"

#define MEDIAN_SIZE 15 // Median filter window size for potentionmeter smoothing

class PID {
private:
  float error;
  float prev_error;
  float derivative;
  float integral;

  float p, i, d, s;

public:
  PID(float p, float i, float d, float s=0) : p(p), i(i), d(d), s(s) {
    error = 0;
    prev_error = 0;
    derivative = 0;
    integral = 0;
  }

  float update(float error, float rel_error=0) {
    derivative = error - prev_error;
    integral += error;
    prev_error = error;
    return p * error + i * integral + d * derivative + s * rel_error;
  }

  void resetIntegral() {
    integral = 0;
  }
};

class Median {
private:
  int history_size;
  int* history;
  int current_idx;

public:
  Median(int history_size) : history_size(history_size) {
    history = new int[history_size];
    current_idx = 0;
    memset(history, 0, sizeof(history));
  }

  ~Median() {
    delete[] history;
  }

  int update(int new_val) {
    history[current_idx] = new_val;
    current_idx++;
    current_idx %= history_size;

    int sorted[history_size];
    memcpy(sorted, history, history_size * sizeof(history[0]));

    qsort(sorted, history_size, sizeof(sorted[0]), [](const void *a, const void *b) {
      if (*(int *)a > *(int *)b)
        return 1;
      else if (*(int *)a < *(int *)b)
        return -1;
      return 0;
    });

    if (history_size % 2 == 1)
      return (sorted[history_size / 2] + sorted[history_size / 2 + 1]) / 2;
    else
      return sorted[history_size / 2];
  }
};

//////// Actuator Class ////////
// Takes in the I2C address, speed register, direction register, speed, potentiometer pin, and invert direction
// pos_mm returns the position of the actuator in mm
// set_speed() sets the speed of the actuator
// sendI2CCommand() sends an I2C command to the actuator
class Actuator {
  char i2c_address;
  char speed_reg;
  char dir_reg;
  float stroke;
  float pot_min;
  float pot_max;
  float act_max_vel;
  float pos_mm;
  PID pid;

  SmoothedInput<MEDIAN_SIZE> pot;

  bool invert_direction;

  float f_map(float x, float in_min, float in_max, float out_min, float out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
  }

public:
  Actuator(char i2c_address, char speed_reg, char dir_reg, InPin pot, bool invert_direction, float stroke, float pot_min, 
       float pot_max, float act_max_vel, PID pid)
    : i2c_address(i2c_address),  speed_reg(speed_reg), dir_reg(dir_reg), pot(pot), stroke(stroke), pot_min(pot_min), 
      pot_max(pot_max), act_max_vel(act_max_vel), pid(pid) {}
    
  float update_pos() {
    pos_mm = f_map(pot.read_analog_raw(), pot_min, pot_max, 0, stroke);
    return pos_mm;
  }

  float get_pos() {
    return pos_mm;
  }
  
  void sendI2CCommand(byte address, byte operationRegister, byte value){      // send command using I2C pin protocol for (MDO4 motor driver)
    Wire.beginTransmission(address);    // begin transmission with our selected driver
    Wire.write(operationRegister);      // enter the desired register
    Wire.write(value);                  // send the data to the register 
    byte error = Wire.endTransmission(false);
    if (error != 0) {
      // Handle error (e.g., print error message or retry)
      Serial.print("I2C Transmission Error: ");
      Serial.println(error);
      Wire.endTransmission(true); // End transmission and release the I2C bus
      Wire.begin(); // Restart the I2C bus
    }
  }
  
  void set_speed(int speed) {
    // Convert speed (in mm/s) to PWM value (0-255)
    // Speed is clamped to max velocity
    speed = constrain(speed, -act_max_vel, act_max_vel);
    int act_speed = map(abs(speed), 0, act_max_vel, 0, 255);
    sendI2CCommand(i2c_address, speed_reg, act_speed);
    sendI2CCommand(i2c_address, dir_reg, speed > 0 ? 1 : 2);
  }

  void tgt_ctrl(int tgt) {
    float tgt_error = tgt - pos_mm;
    float speed = pid.update(tgt_error);
    set_speed(speed);
  }

  void tgt_ctrl(int tgt, int other_pos) {
    float tgt_error = tgt - pos_mm;
    float rel_error = pos_mm - other_pos;
    float speed = pid.update(tgt_error, rel_error);
    set_speed(speed);
  }

  void vel_ctrl(int speed) {
    set_speed(speed);
  }

  void vel_ctrl(int speed, int other_pos, int hz) {
    float rel_error = pos_mm - other_pos;
    float vel_tgt =  pos_mm + speed / hz;
    tgt_ctrl(vel_tgt, rel_error);
  }

  void stop() {
    set_speed(0);
  }

  void resetPIDIntegral() {
    pid.resetIntegral();
  }
};

class Motor {
    OutPin dac1;
    OutPin dac2;

    int motor_max_vel; //rpm

public:
    Motor(OutPin dac1, OutPin dac2, int motor_max_vel) : dac1(dac1), dac2(dac2), motor_max_vel(motor_max_vel) {}

    void motor_ctrl(int signed_speed) {
        // Convert motor speeds (in rpm) to PWM values (0-255)
        // Constrain speeds to max velocity first
        signed_speed = constrain(signed_speed, -motor_max_vel, motor_max_vel);

        // Map the absolute speed values to PWM range
        int motor_speed = map(abs(signed_speed), 0, motor_max_vel, 0, 255);
        if (signed_speed > 0) {
            dac1.write_pwm_raw(motor_speed);
            dac2.write_pwm_raw(0);
        } else {
            dac1.write_pwm_raw(0);
            dac2.write_pwm_raw(abs(motor_speed));
        }
    }

    void stop() {
        motor_ctrl(0);
        // Write inhibit here high in the case that we use it later on
    }

};

#endif
