void setup() {
  // Initialize serial communication at 9600 baud
  Serial.begin(9600);

  // Configure A0 and A1 as input pins
  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
}

void loop() {
  // Read analog values from A0 and A1
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

  // Wait for 500 milliseconds before the next reading
  delay(500);
}