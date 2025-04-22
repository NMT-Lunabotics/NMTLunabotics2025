void setup() {
    // Initialize serial communication at 9600 baud
    Serial.begin(9600);
    pinMode(9, OUTPUT);
}

void loop() {
    for (int dutyCycle = 0; dutyCycle <= 255; dutyCycle++) {
        analogWrite(9, dutyCycle);
        delay(10); // Wait for 10ms
    }
    for (int dutyCycle = 255; dutyCycle >= 0; dutyCycle--) {
        analogWrite(9, dutyCycle);
        delay(10); // Wait for 10ms
    }
}