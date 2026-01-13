#include <Arduino.h>
#include <Wire.h>
#include <MadgwickAHRS.h>

  /*
  Constant declarations
  */
const uint8_t MPU_ADDR = 0x68; //Only 2 I2C addresses available 
const int SDA_Pin = 21; //Currently on default I2C pins
const int SCL_Pin = 22;
const int I2CBusSpeed = 400000; // Start with 100kHz, could go to 400kHz if bottlenecking is an issue
//Registers (change for different IMU)
const uint8_t powerReg = 0x6B;
const uint8_t accStart = 0x3B;
const uint8_t gyroStart = 0x43;
const float accSensitivityFactor = 16384.0f;
const float gyroSensitivityFactor = 131.0f;
const float madgRate = 500.0f; //Hz rate of polling

  /*
  Variable declarations
  */
//Filter object                
Madgwick madg;
//Timestamps
unsigned long lastTime;
unsigned long currentTime;

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
  //madg.begin(madgRate);

  currentTime = millis();
}

void loop() {
  
  float ax = readInt16(MPU_ADDR, accStart) / accSensitivityFactor;
  float ay = readInt16(MPU_ADDR, accStart+2) / accSensitivityFactor;
  float az = readInt16(MPU_ADDR, accStart+4) / accSensitivityFactor;
  float gx = (readInt16(MPU_ADDR, gyroStart) / gyroSensitivityFactor) * DEG_TO_RAD;
  float gy = (readInt16(MPU_ADDR, gyroStart+2) / gyroSensitivityFactor) * DEG_TO_RAD;
  float gz = (readInt16(MPU_ADDR, gyroStart+4) / gyroSensitivityFactor) * DEG_TO_RAD;
  //Mark down time between reads.
  lastTime = currentTime;
  currentTime = millis();
  //Serial.printf("\n\nAccel: ax=%6.3f  ay=%6.3f  az=%6.3f\n", ax, ay, az);
  //Serial.printf("Gyro : gx=%6.3f  gy=%6.3f  gz=%6.3f\n\n", gx, gy, gz);

  madg.updateIMU(gx,gy,gz,ax,ay,az, currentTime, lastTime);
  lastTime = currentTime;
Serial.printf("Roll: %6f, Pitch: %6f, TimeBetween: %10d\n", madg.getRoll(), madg.getPitch(), (currentTime-lastTime));
  //Yaw only works with magnetometer
  //Serial.printf("Roll: %6f, Pitch: %6f, Yaw: %6f, TimeBetween: %6f\n", madg.getRoll(), madg.getPitch(), madg.getYaw(), (currentTime-lastTime));
  //delay(1000/madgRate); Removed to see how fast the ESP32 can go
}
