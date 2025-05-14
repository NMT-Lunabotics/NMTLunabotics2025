#include "helpers.hpp"
// #include <Servo.h>

// Debug mode flag
bool debug_mode = false;

//////// ACTUATORS ////////
// Driver 1 pins
#define DRV11_PWM_PIN 6
#define DRV11_DIR1_PIN 34
#define DRV11_DIR2_PIN 36
#define DRV12_PWM_PIN 7
#define DRV12_DIR1_PIN 38
#define DRV12_DIR2_PIN 40

// Driver 2 pins
#define DRV21_PWM_PIN 9
#define DRV21_DIR1_PIN 44
#define DRV21_DIR2_PIN 42
#define DRV22_PWM_PIN 8
#define DRV22_DIR1_PIN 46
#define DRV22_DIR2_PIN 48

// Actuator potentiometer read pins
// TODO verify
#define POTR_PIN A0
#define POTL_PIN A1
#define POTB_PIN A2

// Actuator info
// Actuator stroke in mm
#define ALR_STROKE 191
#define AB_STROKE 140

// Actuator Calibration
#define AL_POT_MIN 48
#define AL_POT_MAX 893
#define AR_POT_MIN 0
#define AR_POT_MAX 840
#define AB_POT_MIN 30
#define AB_POT_MAX 782

float act_max_vel = 25; //mm/s
// float act_fix_err = 3.0; // mm
float act_max_err = 5.0; // mm
float act_err_threshold = 1; // mm

// Actuator targets
int aL_speed = 0;
int aR_speed = 0;
int aB_speed = 0;

float aL_pos = 0;
float aR_pos = 0;
float aB_pos = 0;

float last_al_pos = 0;
float last_ar_pos = 0;

int aLR_tgt = -1;
int aB_tgt = 40;

float lr_err = 0;
float prev_err = 0;

//////// MOTORS ////////
// const int DACL1_PIN = 2;
// const int DACL2_PIN = 3;
// const int DACR1_PIN = 4;
// const int DACR2_PIN = 5;
// const int EN_PIN = 32; // Common for both motors

// int motor_max_vel = 30; //rpm

// // Motor speeds
// int mL_speed = 0;
// int mR_speed = 0;

//TODO implement servo logic
// #define SERVO_PIN 22

// Servo
// bool servo_state = false;

// LED's
// TODO implement
// #define LEDR_PIN 24
// #define LEDY_PIN 26
// #define LEDG_PIN 28
// #define LEDB_PIN 30

// bool led_r = false;
// bool led_y = false;
// bool led_g = false;
// bool led_b = false;

// Timing
int update_rate = 200; //hz
int feedback_rate = 10; //hz
int reset_int_rate = 10; //hz
unsigned long last_update_time = 0;
unsigned long last_feedback_time = 0;
unsigned long last_reset_int_time = 0;
unsigned long current_time = 0;
// const unsigned long estop_timeout = 1000; // 1 second timeout
// unsigned long last_message_time = 0;
// bool emergency_stop = false;
// bool doomsday = false;

// Set up PID controllers
PID pidL(2.2, 0.0022, 0.34, 2.0);
PID pidR(1.85, 0.0018, 0.31, 1.7);
PID pidB(3.0, 0.001, 0.4);
float vel_gain = 2.5;

// Set up actuators
int change_l_dir = 1; // Separate from invert: -1 for invert speed
int change_r_dir = 1;
PWM_Driver left_driver(DRV12_PWM_PIN, DRV12_DIR1_PIN, DRV12_DIR2_PIN, true);
Actuator act_left(left_driver, pidL, POTL_PIN, AL_POT_MIN, AL_POT_MAX, ALR_STROKE, act_max_vel);

PWM_Driver right_driver(DRV11_PWM_PIN, DRV11_DIR1_PIN, DRV11_DIR2_PIN, true);
Actuator act_right(right_driver, pidR, POTR_PIN, AR_POT_MIN, AR_POT_MAX, ALR_STROKE, act_max_vel);

PWM_Driver bucket_driver(DRV22_PWM_PIN, DRV22_DIR1_PIN, DRV22_DIR2_PIN, true);
Actuator act_bucket(bucket_driver, pidB, POTB_PIN, AB_POT_MIN, AB_POT_MAX, AB_STROKE, act_max_vel, 30, 110);

// Set up motors
// OutPin motor_left_dac1(DACL1_PIN);
// OutPin motor_left_dac2(DACL2_PIN);
// OutPin motor_right_dac1(DACR1_PIN);
// OutPin motor_right_dac2(DACR2_PIN);
// OutPin motor_enable(EN_PIN);
// Motor motor_left(motor_left_dac1, motor_left_dac2, motor_enable, motor_max_vel, false);
// Motor motor_right(motor_right_dac1, motor_right_dac2, motor_enable, motor_max_vel, true);

// Set up LEDs
// OutPin ledr_pin(LEDR_PIN);
// OutPin ledy_pin(LEDY_PIN);
// OutPin ledg_pin(LEDG_PIN);
// OutPin ledb_pin(LEDB_PIN);

// Set up servo
// Servo servo;

// void processMessage(byte* data, int length);
void stop_all();
void fault();

void setup(){
    Serial.begin(115200);
    Serial.flush();
    // servo.attach(SERVO_PIN); //THIS LINE BREAKS THE MOTORS
    // servo.write(0);
    // ledr_pin.write(1);
    // ledy_pin.write(1);
    // ledg_pin.write(0);
    // ledb_pin.write(0);
    stop_all();
    Serial.println("Giving you some time to connect and read debug messages");
    delay(2000);
    Serial.println("I'm done waiting. Let's go.");
    
    for (int i = 0; i < 10; i++) {
        aL_pos = act_left.update_pos();
        aR_pos = act_right.update_pos();
        aB_pos = act_bucket.update_pos();
    }
}

void loop() {
    current_time = millis();
    if (current_time - last_update_time >= 1000 / update_rate) {
        last_update_time = current_time;

        aL_pos = act_left.update_pos();
        aR_pos = act_right.update_pos();
        aB_pos = act_bucket.update_pos();
        Serial.println("Left: " + String(aL_pos) + " Right: " + String(aR_pos) + " Bucket: " + String(aB_pos));

        act_bucket.tgt_ctrl(40);

        lr_err = aL_pos - aR_pos;

        if (abs(lr_err) > act_err_threshold) {
            float factor = lr_err * vel_gain;
            float l_speed = (aL_speed - factor) * change_l_dir;
            float r_speed = (aR_speed + factor) * change_r_dir;
            act_left.vel_ctrl(l_speed);
            act_right.vel_ctrl(r_speed);
            float detected_vel_left = (aL_pos - last_al_pos);
            float detected_vel_right = (aR_pos - last_ar_pos);
            Serial.println("Left - detected vel: " + String(detected_vel_left)
                + " tgt vel: " + String(l_speed)
                + " Right - detected vel: " + String(detected_vel_right)
                + " tgt vel: " + String(r_speed));

            bool detected_dir_left = (detected_vel_left > 0);
            bool detected_dir_right = (detected_vel_right > 0);
            bool target_dir_left = (l_speed > 0);
            bool target_dir_right = (r_speed > 0);

            if (detected_dir_left != target_dir_left) {
                Serial.println("Left actuator direction mismatch. Changing direction");
                change_l_dir *= -1;
            }

            if (detected_dir_right != target_dir_right) {
                Serial.println("Right actuator direction mismatch. Changing direction");
                change_r_dir *= -1;
            }
        } else {
            Serial.println("Actuators within error threshold. Stopping");
            act_left.stop();
            act_right.stop();
        }

        if (abs(lr_err) > abs(prev_err) && abs(lr_err) > act_max_err) {
            stop_all();
            Serial.println("Actuators diverging");
        }

        last_al_pos = aL_pos;
        last_ar_pos = aR_pos;
        prev_err = lr_err;
    }
}

// void processMessage(byte* data, int length) {
//     char type = data[0];
//     if (debug_mode) {
//         Serial.print("Received message of type: ");
//         Serial.println(type);
//         Serial.print("Length: ");
//         Serial.println(length);
//         Serial.print("Data: ");
//         for (int i = 0; i < length; i++) {
//             Serial.print(data[i], HEX);
//             Serial.print(" ");
//         }
//         Serial.println();
//     }

//     switch (type) {
//         case 'A': { // Actuator control
//             aLR_tgt = (int16_t)((data[1] << 8) | data[2]);  // Adjusted index to skip the type byte
//             aB_tgt = (int16_t)((data[3] << 8) | data[4]);
//             aL_speed = -(int8_t)data[5];
//             aR_speed = aL_speed;
//             aB_speed = -(int8_t)data[6];
//             if (debug_mode) {
//                 Serial.print("Arm Position: ");
//                 Serial.println(aLR_tgt);
//                 Serial.print("Bucket Position: ");
//                 Serial.println(aB_tgt);
//                 Serial.print("Arm Velocity: ");
//                 Serial.println(aL_speed);
//                 Serial.print("Bucket Velocity: ");
//                 Serial.println(aB_speed);
//             }
//             break;
//         }
//         case 'M': { // Motor control
//             mL_speed = (int8_t)data[1];  // Adjusted index to skip the type byte
//             mR_speed = (int8_t)data[2];
//             if (debug_mode) {
//                 Serial.print("Left Speed: ");
//                 Serial.println(mL_speed);
//                 Serial.print("Right Speed: ");
//                 Serial.println(mR_speed);
//             }
//             break;
//         }
//         case 'S': { // Servo control
//             servo_state = data[1];  // Adjusted index to skip the type byte
//             if (debug_mode) {
//                 Serial.print("Servo State: ");
//                 Serial.println(servo_state);
//             }
//             break;
//         }
//         case 'L': { // LED control
//             led_r = data[1];  // Adjusted index to skip the type byte
//             led_y = data[2];
//             led_g = data[3];
//             led_b = data[4];
//             if (debug_mode) {
//                 Serial.print("Red: ");
//                 Serial.println(led_r);
//                 Serial.print("Yellow: ");
//                 Serial.println(led_y);
//                 Serial.print("Green: ");
//                 Serial.println(led_g);
//                 Serial.print("Blue: ");
//                 Serial.println(led_b);
//             }
//             break;
//         }
//         default:
//             Serial.println("Unknown message type");
//             break;
//     }
// }

void stop_all() {
    act_left.stop();
    act_right.stop();
    act_bucket.stop();
    // motor_left.stop();
    // motor_right.stop();
}

// void fault(String msg) {
//     while (true) {
//         stop_all();
//         ledr_pin.write(1);
//         delay(500);
//         ledr_pin.write(0);
//         delay(500);
//         Serial.println("Critical Error: " + msg + " - Reset arduino to continue.");
//     }
// }