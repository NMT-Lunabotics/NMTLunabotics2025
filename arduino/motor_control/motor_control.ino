// Updated to use the same serial protocol as system_control.ino

const int motor1_pwm_pin = 3;  // Motor 1 speed (PWM)
const int motor1_dir_pin1 = 4;  // Motor 1 direction pin 1
const int motor1_dir_pin2 = 5;  // Motor 1 direction pin 2

const int motor2_pwm_pin = 6;  // Motor 2 speed (PWM)
const int motor2_dir_pin1 = 7;  // Motor 2 direction pin 1
const int motor2_dir_pin2 = 8;  // Motor 2 direction pin 2

const float rpm_to_pwm_constant = 5;

int mL_speed = 0;
int mR_speed = 0;

int update_rate = 200; // Hz
unsigned long last_update_time = 0;
unsigned long current_time = 0;

void processMessage(byte* data, int length);
void set_speed(int pwm_pin, int dir_pin1, int dir_pin2, int speed);

void setup() {
  Serial.begin(115200);  // Set baud rate to match Python script

  // Set motor control pins as output
  pinMode(motor1_pwm_pin, OUTPUT);
  pinMode(motor1_dir_pin1, OUTPUT);
  pinMode(motor1_dir_pin2, OUTPUT);

  pinMode(motor2_pwm_pin, OUTPUT);
  pinMode(motor2_dir_pin1, OUTPUT);
  pinMode(motor2_dir_pin2, OUTPUT);
}

void loop() {
  // Protocol: 0x02 <length> <data...> 0x03
  if (Serial.available() > 0) {
    if (Serial.read() == 0x02) { // Start byte
      while (Serial.available() < 1) {} // Wait for length byte
      int length = Serial.read();
      while (Serial.available() < length + 1) {} // Wait for the entire message
      byte data[length];
      Serial.readBytes(data, length);
      while (Serial.available() < 1) {} // Wait for end byte
      if (Serial.read() == 0x03) { // End byte
        processMessage(data, length);
      } else {
        set_speed(motor1_pwm_pin, motor1_dir_pin1, motor1_dir_pin2, 0);
        set_speed(motor2_pwm_pin, motor2_dir_pin1, motor2_dir_pin2, 0);
        Serial.println("End byte not found");
      }
    }
  }

  current_time = millis();
  if (current_time - last_update_time >= 1000 / update_rate) {
    last_update_time = current_time;

    // Set motor speeds
    set_speed(motor1_pwm_pin, motor1_dir_pin1, motor1_dir_pin2, mL_speed);
    set_speed(motor2_pwm_pin, motor2_dir_pin1, motor2_dir_pin2, mR_speed);
  }

}

void set_speed(int pwm_pin, int dir_pin1, int dir_pin2, int speed) {
  int pwm_value = constrain(abs(speed) * rpm_to_pwm_constant, 0, 255);
  if (speed >= 0) {
    digitalWrite(dir_pin1, HIGH);
    digitalWrite(dir_pin2, LOW);
  } else {
    digitalWrite(dir_pin1, LOW);
    digitalWrite(dir_pin2, HIGH);
  }
  analogWrite(pwm_pin, pwm_value);
}

void processMessage(byte* data, int length) {
  char type = data[0];
  switch (type) {
    case 'M': { // Motor control
      mL_speed = (int8_t)data[1];
      mR_speed = (int8_t)data[2];
      break;
    }
    default:
      Serial.println("Unknown message type");
      break;
  }
}
