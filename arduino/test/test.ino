void setup() {
    // Initialize serial communication at 9600 baud
    Serial.begin(9600);

    // Set pins 3, 5, 6, and 9 as output
    pinMode(3, OUTPUT);
    pinMode(5, OUTPUT);
    pinMode(6, OUTPUT);
    pinMode(9, OUTPUT);

    // Set pins 3, 5, 6, and 9 high
    digitalWrite(3, HIGH);
    digitalWrite(5, HIGH);
    digitalWrite(6, HIGH);
    digitalWrite(9, HIGH);
}

void loop() {

}