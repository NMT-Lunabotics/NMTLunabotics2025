#include <Wire.h>

#define ADDRESS             0x58                       // Address of MD04 driver
#define CMDBYTE             0x00                       // Command byte
#define SPEEDBYTE           0x02                       // Byte to write to speed register

#define POTPIN             A0                         // Potentiometer pin

void setup(){
  Serial.begin(9600);                             
  delay(100);

  pinMode(POTPIN, INPUT);                              // Set potentiometer pin as input
  
  Wire.begin();                                        // Start I2C connection

  sendData(SPEEDBYTE, 255);
  sendData(CMDBYTE, 1);
  delay(8000);


  Serial.print("max:");
  Serial.println(analogRead(POTPIN));                 // Print potentiometer value

  sendData(SPEEDBYTE, 255);
  sendData(CMDBYTE, 2);
  delay(8000);

  Serial.print("min:");
  Serial.println(analogRead(POTPIN));                 // Print potentiometer value
}

void loop(){
 
}

void sendData(byte commandRegister, byte value){       // Send data through I2C communication
  Wire.beginTransmission(ADDRESS);                     // Start transmission of data to MD04 motor driver address
    Wire.write(commandRegister);                       // Select used register
    Wire.write(value);                                 // Send data to register 
  Wire.endTransmission();   
}
