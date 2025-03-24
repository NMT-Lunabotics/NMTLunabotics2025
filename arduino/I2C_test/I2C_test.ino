void setup() {
  Serial.begin(9600); // Start the serial communication
}

void loop() {
  int sensorValueA0 = analogRead(A0); // Read the value from analog pin A0
  int sensorValueA1 = analogRead(A1); // Read the value from analog pin A1

  Serial.print("A0: ");
  Serial.println(sensorValueA0); // Print the value of A0
  // Serial.print(" A1: ");
  // Serial.println(sensorValueA1); // Print the value of A1

  delay(1000); // Wait for a second before reading again
}