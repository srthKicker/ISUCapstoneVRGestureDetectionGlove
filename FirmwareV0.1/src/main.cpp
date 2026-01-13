#include <Arduino.h>
#include <Wire.h>
#include <MadgwickAHRS.h>

  /*
  Constant declarations
  */
const uint8_t MPU_ADDR = 0x68; //Only 2 I2C addresses available 
const int SDA_Pin = 21; //Currently on default I2C pins
const int SCL_Pin = 22;
const int I2CBusSpeed = 100000; // Start with 100kHz, could go to 400kHz if bottlenecking is an issue
//Registers (change for different IMU)
const uint8_t powerReg = 0x6B;
const uint8_t accStart = 0x3B;
const uint8_t gyroStart = 0x43;

  /*
  Variable declarations
  */
//Filter object
Madgwick madg;


  /*
  Function declarations
  */
//Reads the byte from a piece of memory over I2C
byte readByte(uint8_t addr, uint8_t reg) {
  Wire.beginTransmission(addr); //Set an address to connect to
  Wire.write(reg); //Set a register to read/write
  Wire.endTransmission(false); //Prep to read
  Wire.requestFrom(addr, 1, 0);
  return Wire.read();
}

//Reads 16 bit int from the memory location (generally used for sensor value)
int16_t readInt16(uint8_t addr, uint8_t reg) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(addr, 2, 0);
  return Wire.read() << 8 | Wire.read(); // Put significant byte 1st
} 

//Writes to the specified location over I2C
void writeRegister(int addr, uint8_t reg, uint8_t value){
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission(true);
}

  /*
    Setup will initialize communication
  */
void setup() {
  Serial.begin(115200); //Set baud rate
  Wire.begin(SDA_Pin, SCL_Pin);
  Wire.setClock(I2CBusSpeed); //Sets the clock rate
  //Wake up the IMU MPU6050
  writeRegister(MPU_ADDR, powerReg, 0);
}

void loop() {
  int16_t ax = readInt16(MPU_ADDR, accStart);
  int16_t ay = readInt16(MPU_ADDR, accStart+2);
  int16_t az = readInt16(MPU_ADDR, accStart+4);
  int16_t gx = readInt16(MPU_ADDR, gyroStart);
  int16_t gy = readInt16(MPU_ADDR, gyroStart+2);
  int16_t gz = readInt16(MPU_ADDR, gyroStart+4);

  Serial.printf("\n\nAccel: ax=%6d  ay=%6d  az=%6d\n", ax, ay, az);
  Serial.printf("Gyro : gx=%6d  gy=%6d  gz=%6d\n", gx, gy, gz);
  delay(500);
}
