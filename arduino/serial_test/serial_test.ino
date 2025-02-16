void setup() {
    // Start the serial communication
    Serial.begin(9600);
    while (!Serial) {
        ; // Wait for the serial port to connect. Needed for native USB port only
    }
}

void loop() {
    // Check if data is available to read
    if (Serial.available() > 0) {
        // Read the incoming byte
        char incomingByte = Serial.read();
        // Print the incoming byte to the serial monitor
        Serial.print(incomingByte);
    }
}