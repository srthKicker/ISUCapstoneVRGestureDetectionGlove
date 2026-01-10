#include <Arduino.h>
#include <Wire.h>

const int MPU_ADDR = 0x68; //Only 2 I2C addresses available 
//Default SDA and SCL pins 
const int SDA = 21;
const int SCL = 22;

// Function declarations
//Reads the byte from a piece of memory over I2C
byte readRegister(uint8_t reg) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 1, true);
  return Wire.read();
}

/*
  Setup will 
*/
void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22);
  int result = myFunction(2, 3);
}

void loop() {
  // put your main code here, to run repeatedly:
}

// put function definitions here:
int myFunction(int x, int y) {
  return x + y;
}