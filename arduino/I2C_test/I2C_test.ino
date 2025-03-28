#include <Wire.h>

void scanI2C() {
  Serial.println("Scanning for I2C devices...");
  for (byte address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    byte error = Wire.endTransmission();

    if (error == 0) {
      Serial.print("I2C device found at address 0x");
      if (address < 16) Serial.print("0");
      Serial.println(address, HEX);
    } else if (error == 4) {
      Serial.print("Unknown error at address 0x");
      if (address < 16) Serial.print("0");
      Serial.println(address, HEX);
    }
  }
  Serial.println("Scan complete.");
}

void setup(){
  Serial.begin(115200);
  Wire.begin();
}

void loop() {
  scanI2C(); // Call the I2C scanning function

  delay(1000); // Wait for a second before reading again
}