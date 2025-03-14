#include <Wire.h>

//TODO check pins
//DACs
const int dac11_pin = 3;
const int dac12_pin = 5;
const int dac21_pin = 6;
const int dac22_pin = 9;

//LEDs
const int ledr_pin = 2;
const int ledy_pin = 4;
const int ledg_pin = 7;
const int ledb_pin = 8;

//Actuators
char a1_i2c_address = 0xB0;
char a2_i2c_address = 0xB2;
char a3_i2c_address = 0xB4;

char speed_reg = 0x02;
char dir_reg = 0x00;

const int pot1_pin = A0;
const int pot2_pin = A1;
const int pot3_pin = A2;

//TODO implement servo logic
const int servo_pin = 10;

bool emergency_stop = false;
const unsigned long estop_timeout = 1000; // 1 second timeout
unsigned long last_message_time = 0;

// PWM motor speeds
int m1_speed = 0;
int m2_speed = 0;

// Actuator targets
int a1_speed = 0;
int a2_speed = 0;
int a3_speed = 0;

float a1_pos = 0;
float a2_pos = 0;
float a3_pos = 0;

int a12_tgt = -1;
int a3_tgt = -1;

// Actuator info
//TODO get real numbers
// Actuator stroke in mm
int a12_stroke = 300;
int a3_stroke = 250;

//TODO calibrate actuators
int a1_pot_min = 0;
int a1_pot_max = 1023;
int a2_pot_min = 0;
int a2_pot_max = 1023;
int a3_pot_min = 0;
int a3_pot_max = 1023;

int act_max_vel = 25; //mm/s
float act_threshold = .5; //mm
float act_error_factor = 1;

// Update rate
int update_rate = 50; //hz
unsigned long last_update_time = 0;

void setup(){
    Serial.begin(2000000);
    delay(100);
    Wire.begin();

    pinMode(dac11_pin, OUTPUT);
    pinMode(dac12_pin, OUTPUT);
    pinMode(dac21_pin, OUTPUT);
    pinMode(dac22_pin, OUTPUT);

    pinMode(ledr_pin, OUTPUT);
    pinMode(ledy_pin, OUTPUT);
    pinMode(ledg_pin, OUTPUT);
    pinMode(ledb_pin, OUTPUT);

    pinMode(servo_pin, OUTPUT);
}

void loop() {
    if (Serial.available()) {
        String inputString = Serial.readStringUntil('>');  // Read until '>'
        if (inputString.startsWith("<")) {                 // Ensure message starts with '<'
            inputString = inputString.substring(1);        // Remove the starting '<'
            char operation = inputString[0];
            inputString = inputString.substring(2);        // Remove operation type ie "M" or "B"
            if (inputString.endsWith(">")) {
                inputString = inputString.substring(0, inputString.length() - 1);  // Remove the ending '>'
            }

            switch (operation) {
                case 'M': {
                    int m1_speed = inputString.substring(0, inputString.indexOf(',')).toInt();
                    inputString = inputString.substring(inputString.indexOf(',') + 1);
                    int m2_speed = inputString.toInt();
                    //TODO convert to PWM`
                    break;
                }
                case 'A': {
                    int a12_tgt = inputString.substring(0, inputString.indexOf(',')).toInt();
                    inputString = inputString.substring(inputString.indexOf(',') + 1);
                    int a3_tgt = inputString.substring(0, inputString.indexOf(',')).toInt();
                    inputString = inputString.substring(inputString.indexOf(',') + 1);
                    int a1_speed = inputString.substring(0, inputString.indexOf(',')).toInt();
                    int a2_speed = a1_speed;
                    inputString = inputString.substring(inputString.indexOf(',') + 1);
                    int a3_speed = inputString.toInt();
                    //TODO convert to 'pwm'
                    break;
                }
            }
            last_message_time = millis(); // Update the last message time
            emergency_stop = false; // Reset emergency stop
        }
    }

    // Check if the time since the last message exceeds the timeout
    if (millis() - last_message_time > estop_timeout) {
        emergency_stop = true;
    }

    if (emergency_stop) {
        //TODO make sure it stops
        motor_ctrl(0, 0);
        actuator_vel_ctrl(a1_i2c_address, 0);
        actuator_vel_ctrl(a2_i2c_address, 0);
        actuator_vel_ctrl(a3_i2c_address, 0);
        break;
    }

    // Update at a hz rate
    unsigned long current_time = millis();
    if (current_time - last_update_time < 1000 / update_rate) {
        break;
    }
    last_update_time = current_time;

    //Run motors
    motor_ctrl(m1_speed, m2_speed);

=   //Get actuator feedback
    a1_pos = map(analogRead(pot1_pin), a1_pot_min, a1_pot_max, 0, a1_stroke);
    a2_pos = map(analogRead(pot2_pin), a2_pot_min, a2_pot_max, 0, a2_stroke);
    a3_pos = map(analogRead(pot3_pin), a3_pot_min, a3_pot_max, 0, a3_stroke);

    //TODO send actuator feedback
    
    //Actuator control
    //Velocity Control
    if (a3_tgt > 0) {
        float a3_error = a3_tgt - a3_pos;
        if (abs(a3_error) > act_threshold) {
            a3_speed = a3_error > 0 ? act_max_vel : -act_max_vel;
        } else {
            a3_speed = 0;
        }
    }
  

    if (a12_tgt > 0 ) {
        float a1_error = a12_tgt - a1_pos;
        float a2_error = a12_tgt - a2_pos;
        float a12_error = a1_pos - a2_pos;

        if (abs(a1_error) > act_threshold) {
            a1_speed = a1_error > 0 ? act_max_vel : -act_max_vel;
        } else {
            a1_speed = 0;
        }

        if (abs(a2_error) > act_threshold) {
            a2_speed = a2_error > 0 ? act_max_vel : -act_max_vel;
        } else {
            a2_speed = 0;
        }

        float factor = act_error_factor * a12_error;
        a1_speed -= factor;
        a2_speed += factor;
    }

    // Constrain all speeds
    a1_speed = constrain(a1_speed, -act_max_vel, act_max_vel);
    a2_speed = constrain(a2_speed, -act_max_vel, act_max_vel);
    a3_speed = constrain(a3_speed, -act_max_vel, act_max_vel);
    actuator_vel_ctrl(a3_i2c_address, a3_speed);
    actuator_vel_ctrl(a1_i2c_address, a1_speed);
    actuator_vel_ctrl(a2_i2c_address, a2_speed);
    
}

void motor_ctrl(int m1_speed, int m2_speed) {
    if (m1_speed > 0) {
        analogWrite(dac11_pin, m1_speed);
        analogWrite(dac12_pin, 0);
    } else {
        analogWrite(dac11_pin, 0);
        analogWrite(dac12_pin, abs(m1_speed));
    }

    if (m2_speed > 0) {
        analogWrite(dac21_pin, m2_speed);
        analogWrite(dac22_pin, 0);
    } else {
        analogWrite(dac21_pin, 0);
        analogWrite(dac22_pin, abs(m2_speed));
    }
}

void actuator_vel_ctrl(char i2c_address, int speed) {
    sendI2CCommand(i2c_address, speed_reg, abs(speed));
    sendI2CCommand(i2c_address, dir_reg, speed > 0 ? 1 : 2);
}

void sendI2CCommand(byte address, byte operationRegister, byte value){      // send command using I2C pin protocol for (MDO4 motor driver)
    Wire.beginTransmission(address);    // begin transmission with our selected driver
    Wire.write(operationRegister);      // enter the desired register
    Wire.write(value);                  // send the data to the register 
    Wire.endTransmission();
}


//TODO figure out actuator position control