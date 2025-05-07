bool debug_mode = false;

bool led_r = false;
bool led_y = false;
bool led_g = false;
bool led_b = false;

int led_pin_r = 10;  // Example pin for red LED
int led_pin_y = 11;  // Example pin for yellow LED
int led_pin_g = 12;  // Example pin for green LED
int led_pin_b = 13;  // Example pin for blue LED

void setup() {
    Serial.begin(115200);
    Serial.flush();

    // Set LED pins as outputs
    pinMode(led_pin_r, OUTPUT);
    pinMode(led_pin_y, OUTPUT);
    pinMode(led_pin_g, OUTPUT);
    pinMode(led_pin_b, OUTPUT);

    // Turn off all LEDs initially
    digitalWrite(led_pin_r, LOW);
    digitalWrite(led_pin_y, LOW);
    digitalWrite(led_pin_g, LOW);
    digitalWrite(led_pin_b, LOW);
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

    // Continuously update LED states
    digitalWrite(led_pin_r, led_r ? HIGH : LOW);
    digitalWrite(led_pin_y, led_y ? HIGH : LOW);
    digitalWrite(led_pin_g, led_g ? HIGH : LOW);
    digitalWrite(led_pin_b, led_b ? HIGH : LOW);
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
