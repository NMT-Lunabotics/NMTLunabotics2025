//#include "helpers.hpp"
//#include "main_bus.hpp"
//#include <Arduino_CAN.h>
#include <Wire.h>
bool emergency_stop = false;
//------------------------------------------
//big motors
//------------------------------------------
char motor1_address = 0x58;     // Motor 1 I2C address
char motor2_address = 0x58;     // Motor 2 I2C address  // TODO change address

const float rpm_to_pwm_constant = 2.5;  // Conversion between rpm and pwm values
//------------------------------------------
//bucket actuator
//------------------------------------------
const int bucket_speed_pin = 3;           // Bucket actuator speed, (pwm)
const int bucket_direction_pin = 4;       // Bucket actuator direction pin
const int bucket_potentiometer_pin = A0;  // Bucket actuator potentiometer adjustment pin

int bucket_max_speed = 240;  // Max bucket actuator speed, (6 mm/s)
int bucket_threshold = 1;
int bucket_min_pos = 17;   // Min bucket actuator position
int bucket_max_pos = 270;  // Max bucket actuator position

int bucket_stroke = 300;  // Stroke length, (mm):
int bucket_potMin = 34;   // Calibrated min potentiometer mapping range to stroke range
// TODO fix
int bucket_potMax = 945;     // Calibrated max potentiometer mapping range to stroke range
int bucket_target_pos = -1;  // (mm)
int bucket_target_vel = 1;   // Defualt velocity of bucket actuator and storage of saved velocity

int bucket_update_rate = 50;  // Update rate to update bucket actuator at, (hz)

//------------------------------------------
//dual actuators
//------------------------------------------ TODO change back to read_analog_raw(), if required
const int dual_speed_left = 3;            // Left dual actuator speed pin
const int dual_speed_right = 5;           // Right dual actuator speed pin
const int dual_direction_left = 4;        // Left dual actuator direction pin
const int dual_direction_right = 6;       // Right dual actuator direction pin
const int dual_potentiometer_left = A0;   // Left dual actuator potentiometer pin
const int dual_potentiometer_right = A1;  // Right dual actuator potentiometer pin

int dual_target_pos = -1;  // (mm)
int dual_target_vel = 1;   //Defualt velocity of bucket actuator and storage of saved velocity (mm/s)

int dual_max_speed = 200;  // (5 mm/s)
float dual_threshold = 1;  // in mm

int dual_stroke = 250;  // stroke length, in mm:
int dual_potMin = 34;   // Calibrated, pot val at min stroke
int dual_potMax = 945;  // Calibrated, pot val at max stroke

int dual_update_rate = 50;  //(hz)
int dual_max_error = 5;

int dual_error_factor = 12;

void setup() {
  Serial.begin(9600);  // Set baud rate to match Python script  2000000
  delay(100);
  Wire.begin(); // Start I2C communication

  // Set bucket actuator pins as output and potentiometer as analog input pin
  pinMode(bucket_speed_pin, OUTPUT);
  pinMode(bucket_direction_pin, OUTPUT);
  pinMode(bucket_potentiometer_pin, INPUT);

  // Set dual actuators pins as output and potentiometers as analog input pin
  pinMode(dual_speed_left, OUTPUT);
  pinMode(dual_speed_right, OUTPUT);
  pinMode(dual_direction_left, OUTPUT);
  pinMode(dual_direction_right, OUTPUT);
  pinMode(dual_potentiometer_left, INPUT);
  pinMode(dual_potentiometer_right, INPUT);
}

void loop() {
  if (Serial.available()) {
    String inputString = Serial.readStringUntil('>');  // Read until '>'
    if (inputString.startsWith("<")) {                 // Ensure message starts with '<'
      inputString = inputString.substring(1);          // Remove the starting '<'
      char operation = inputString[0];
      inputString = inputString.substring(2);  // Remove operation type ie "M" or "B"
      int commaIndex = inputString.indexOf(',');

      int elementCount = 0;  // Count number of inputs for actuators which allow multiple 1-2 values for input
      for (int i = 0; i < inputString.length(); i++) {
        if (inputString[i] == ',') continue;
        elementCount = elementCount + 1;
      }
      switch (operation) {
        case 'E':  //<E,stop/start>
          {
            if (inputString.substring(0, commaIndex).toInt() == 0) emergency_stop = false;
            else emergency_stop = true;
            break;
          }
        case 'M':  //<M,Motor1 speed,Motor2 speed>
          {
            //Serial.println("Recived Motor Command");
            int rpm_motor1 = inputString.substring(0, commaIndex).toInt();
            int rpm_motor2 = inputString.substring(commaIndex + 1).toInt();
            operateMotor(rpm_motor1, rpm_motor2);
            break;
          }
        case 'B':  //<B,position> or <B,position,velocity>
          {
            //Serial.println("Recived Bucket Actuator Command");
            bucket_target_pos = inputString.substring(0, commaIndex).toInt();
            if (elementCount >= 2) bucket_target_vel = inputString.substring(commaIndex + 1).toInt();
            break;
          }
        case 'L':  //<L,position> or <L,position,velocity>
          {
            //Serial.println("Recived Dual Actuator Command");
            dual_target_pos = inputString.substring(0, commaIndex).toInt();
            if (elementCount >= 2) dual_target_vel = inputString.substring(commaIndex + 1).toInt();
            break;
          }
        default:
          Serial.println("Error: Unknown operation!");
          break;
      }
    }
  }
  if (emergency_stop == true) { // Set motor 1 and 2 direction (0x00 command register) to instant stop
    sendI2CCommand(motor1_address, 0x00, 0);
    sendI2CCommand(motor2_address, 0x00, 0);
    return;
  }
  bucketActuatorUpdate(bucket_target_pos, bucket_target_vel);
  dualActuatorUpdate(dual_target_pos, dual_target_vel);
}

void operateMotor(int rpm_motor1, int rpm_motor2) {
  if (emergency_stop == true) return;
  // Convert RPM to PWM using the constant to get into (0-255) speed range for motor drivers
  int motor1_speed = constrain(abs(rpm_motor1) * rpm_to_pwm_constant, 0, 255);
  int motor2_speed = constrain(abs(rpm_motor2) * rpm_to_pwm_constant, 0, 255);

  // Set motor 1 and 2 speed (0x02 speed register)
  sendI2CCommand(motor1_address, 0x02, motor1_speed);
  sendI2CCommand(motor2_address, 0x02, motor2_speed);

  // Run motor 1 and 2 directions (0x00 command register) to forwards/reverse
  if (rpm_motor1 >= 0) sendI2CCommand(motor1_address, 0x00, 1);
  else sendI2CCommand(motor1_address, 0x00, 2); 
  if (rpm_motor2 >= 0) sendI2CCommand(motor2_address, 0x00, 1);
  else sendI2CCommand(motor2_address, 0x00, 2); 

  /*Serial.print("pwm_motor1: ");
    Serial.print(pwm_motor1);
    Serial.print('\t');
    Serial.print("pwm_motor2: ");
    Serial.println(pwm_motor2);*/
}

void bucketActuatorUpdate(int bucket_target_pos, int bucket_target_vel) {  //update bucket actuator until position is reached within threshold
  if (emergency_stop == true) return;
  int bucket_last_time;
  int speed;
  int current_time = millis();
  int dt = 1000 / bucket_update_rate;
  if (current_time - bucket_last_time < dt) return;
  bucket_last_time = current_time;

  int pos = map(analogRead(bucket_potentiometer_pin), bucket_potMin, bucket_potMax, 0, bucket_stroke);//TODO replace with replacment for potentiometer reader

  if (bucket_target_pos == -1) {
    speed = bucket_target_vel * 51;
  } else {
    int error = pos - bucket_target_pos;

    if (error > bucket_threshold) {
      speed = -bucket_max_speed;
    } else if (error < -bucket_threshold) {
      speed = bucket_max_speed;
    } else {
      speed = 1;
    }
  }

  speed = constrain(speed, -bucket_max_speed, bucket_max_speed);

  /*Serial.print("speed: ");
  Serial.print(speed);
  Serial.print('\t');
  Serial.print("pos: ");
  Serial.println(pos);*/

  if ((pos < bucket_max_pos || speed <= 0) && (pos > bucket_min_pos || speed >= 0)) set_speed(bucket_speed_pin, bucket_direction_pin, -speed, false);
  else set_speed(bucket_speed_pin, bucket_direction_pin, 0, false);
}

void dualActuatorUpdate(int target_pos, int target_vel) {
  if (emergency_stop == true) return;
  int speed_l = 0;
  int speed_r = 0;
  float dual_last_time;

  float current_time = millis();
  int dt = 1000 / dual_update_rate;
  if (current_time - dual_last_time < dt) return;
  dual_last_time = current_time;

  int pos_l = map(analogRead(dual_potentiometer_left), dual_potMin, dual_potMax, 0, dual_stroke);  //TODO replace with replacment for potentiometer reader 
  //map(dual_potentiometer_left.read_analog_raw(), dual_potMin, dual_potMax, 0, dual_stroke);
  int pos_r = map(analogRead(dual_potentiometer_right), dual_potMin, dual_potMax, 0, dual_stroke);  //TODO replace with replacment for potentiometer reader
  //map(dual_potentiometer_right.read_analog_raw(), dual_potMin, dual_potMax, 0, dual_stroke);
  int error_lr = pos_l - pos_r;
  int error_l = 0;
  int error_r = 0;

  bool doomsday = false;
  if (error_lr > dual_max_error) {
    doomsday = true;
    //Serial.println("Doomsday");
  }

  if (target_pos == -1) {
    speed_l = target_vel * 51;
    speed_r = target_vel * 51;
  } else {
    error_l = pos_l - target_pos;
    error_r = pos_r - target_pos;

    if (error_l > dual_threshold) {
      speed_l = -dual_max_speed;
    } else if (error_l < -dual_threshold) {
      speed_l = dual_max_speed;
    } else {
      speed_l = 0;
    }

    if (error_r > dual_threshold) {
      speed_r = -dual_max_speed;
    } else if (error_r < -dual_threshold) {
      speed_r = dual_max_speed;
    } else {
      speed_r = 0;
    }
  }

  int factor = dual_error_factor * error_lr;
  speed_l -= factor;
  speed_r += factor;

  speed_l = constrain(speed_l, -255, 255);
  speed_r = constrain(speed_r, -255, 255);

  /*Serial.print("speed_l: ");
  Serial.print(speed_l);
  Serial.print('\t');
  Serial.print("speed_r: ");
  Serial.print(speed_r);
  Serial.print('\t');
  Serial.print("pos_l: ");
  Serial.print(pos_l);
  Serial.print('\t');
  Serial.print("pos_r: ");
  Serial.print(pos_r);
  Serial.print('\t');
  Serial.print("factor: ");
  Serial.println(factor);*/

  set_speed(dual_speed_left, dual_direction_left, -speed_l, false);
  set_speed(dual_speed_right, dual_direction_right, -speed_r, false);
}

void set_speed(int speedPin, int dirPin, int signed_speed, bool invert_direction) {  //set speed of bucket and dual actuators
  if (emergency_stop == true) return;
  if (invert_direction) {
    digitalWrite(dirPin, signed_speed > 0); //TODO replace with replacment for actuator PWM pin direction
  } else {
    digitalWrite(dirPin, signed_speed < 0); //TODO replace with replacment for actuator PWM pin direction
  }
  analogWrite(speedPin, abs(signed_speed)); //TODO replace with replacment for actuator PWM pin speed
}

void sendI2CCommand(byte address, byte operationRegister, byte value){      // send command using I2C pin protocol for (MDO4 motor driver)
    Wire.beginTransmission(address);    // begin transmission with our selected driver
    Wire.write(operationRegister);      // enter the desired register
    Wire.write(value);                  // send the data to the register 
    Wire.endTransmission();
}
