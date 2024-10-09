int motor1_speed = 0;
int motor2_speed = 0;
bool receiving = false;
String inputString = "";

void setup() {
  Serial.begin(9600);  // Set baud rate (must match the Python script)
}

void loop() {
  while (Serial.available()) {
    char inChar = (char)Serial.read();

    if (inChar == '<') {
      // Start of message
      inputString = "";
      receiving = true;
    } else if (inChar == '>') {
      // End of message
      if (receiving) {
        processInputString(inputString);
        receiving = false;
      }
    } else if (receiving) {
      inputString += inChar;  // Collect characters inside the frame
    }
  }
}

void processInputString(String data) {
  // Extract motor speeds from the received string
  int commaIndex = data.indexOf(',');
  if (commaIndex > 0) {
    motor1_speed = data.substring(0, commaIndex).toInt();   // Get motor 1 speed
    motor2_speed = data.substring(commaIndex + 1).toInt();  // Get motor 2 speed

    // Add your motor control code here

    // Optionally, send a confirmation message back
    Serial.print("Motor1: ");
    Serial.print(motor1_speed);
    Serial.print(", Motor2: ");
    Serial.println(motor2_speed);
  }
}

