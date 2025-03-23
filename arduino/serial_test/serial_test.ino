#include <Wire.h>

void setup() {
    Serial.begin(115200);
    Serial.println("Starting serial test...");
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
}

void processMessage(byte* data, int length) {
    char type = data[0];
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

    switch (type) {
        case 'A': { // Actuator control
            int16_t arm_pos = (int16_t)((data[1] << 8) | data[2]);  // Adjusted index to skip the type byte
            int16_t bucket_pos = (int16_t)((data[3] << 8) | data[4]);
            int arm_vel = (int8_t)data[5];
            int bucket_vel = (int8_t)data[6];
            Serial.print("Arm Position: ");
            Serial.println(arm_pos);
            Serial.print("Bucket Position: ");
            Serial.println(bucket_pos);
            Serial.print("Arm Velocity: ");
            Serial.println(arm_vel);
            Serial.print("Bucket Velocity: ");
            Serial.println(bucket_vel);
            break;
        }
        case 'M': { // Motor control
            int left_speed = (int8_t)data[1];  // Adjusted index to skip the type byte
            int right_speed = (int8_t)data[2];
            Serial.print("Left Speed: ");
            Serial.println(left_speed);
            Serial.print("Right Speed: ");
            Serial.println(right_speed);
            break;
        }
        case 'S': { // Servo control
            bool servo_state = data[1];  // Adjusted index to skip the type byte
            Serial.print("Servo State: ");
            Serial.println(servo_state);
            break;
        }
        case 'L': { // LED control
            int red = data[1];  // Adjusted index to skip the type byte
            int yellow = data[2];
            int green = data[3];
            int blue = data[4];
            Serial.print("Red: ");
            Serial.println(red);
            Serial.print("Yellow: ");
            Serial.println(yellow);
            Serial.print("Green: ");
            Serial.println(green);
            Serial.print("Blue: ");
            Serial.println(blue);
            break;
        }
        default:
            Serial.println("Unknown message type");
            break;
    }
}