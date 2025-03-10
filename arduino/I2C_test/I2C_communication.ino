#include <Wire.h>

#define ADDRESS             0x58                       // Address of MD04 driver
#define CMDBYTE             0x00                       // Command byte
#define SPEEDBYTE           0x02                       // Byte to write to speed register
void setup(){
  Serial.begin(19200);                             
  delay(100);
  
  Wire.begin();                                        // Start I2C connection
}

void loop(){
 sendData(SPEEDBYTE, 255);                             // Send speed data
    sendData(CMDBYTE, 1);                              // Send Run Direction Command     
}

void sendData(byte commandRegister, byte value){       // Send data through I2C communication
  Wire.beginTransmission(ADDRESS);                     // Start transmission of data to MD04 motor driver address
    Wire.write(commandRegister);                       // Select used register
    Wire.write(value);                                 // Send data to register 
  Wire.endTransmission();   
}
