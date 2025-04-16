void setup() {
    // Initialize serial communication at 9600 baud
    Serial.begin(9600);
}

void loop() {
    // Read analog values from A0, A1, and A2
    int valueA0 = analogRead(A0);
    int valueA1 = analogRead(A1);
    int valueA2 = analogRead(A2);

    // Print the values to the Serial Monitor
    Serial.print("A0: ");
    Serial.print(valueA0);
    Serial.print(" | A1: ");
    Serial.print(valueA1);
    Serial.print(" | A2: ");
    Serial.println(valueA2);

    // Add a small delay to avoid flooding the Serial Monitor
    delay(500);
}