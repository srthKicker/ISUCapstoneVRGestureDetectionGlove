#include <Arduino.h>
#include <Wire.h>

const int MPU_ADDR = 0x68;
// Function declarations
//Reads the byte from a piece of memory over I2C
byte readRegister(uint8_t reg) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 1, true);
  return Wire.read();
}



void setup() {
  Serial.begin(115200);
  Wire.begin(21,22);

  //To initialize the mpu6050
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B); //Power management
  Wire.write(0); //Write to 0 to wake up the sensor
  Wire.endTransmission(true);

  //Let's find the gyroscope and accelerometer ranges
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1A);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 3, true);
  byte cfig = Wire.read();
  byte gcfig = Wire.read();
  byte acfig = Wire.read();

  Serial.println("MPU-6050 Configuration Bytes:");
  Serial.print("  CONFIG     (0x1A): 0x");
  Serial.println(cfig, HEX);

  Serial.print("  GYRO CONFIG (0x1B): 0x");
  Serial.println(gcfig, HEX);

  Serial.print("  ACCEL CONFIG (0x1C): 0x");
  Serial.println(acfig, HEX);

};


void loop() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B); // Starting register for accelerometer data
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 14, true); // Request 14 bytes

  int16_t ax = Wire.read() << 8 | Wire.read();
  int16_t ay = Wire.read() << 8 | Wire.read();
  int16_t az = Wire.read() << 8 | Wire.read();
  int16_t temp = Wire.read() << 8 | Wire.read(); // Raw temperature
  int16_t gx = Wire.read() << 8 | Wire.read();
  int16_t gy = Wire.read() << 8 | Wire.read();
  int16_t gz = Wire.read() << 8 | Wire.read();
  
  byte gcfig = readRegister(0x1B);
  byte acfig = readRegister(0x1C);
  byte cnfig = readRegister(0x1A);
  Serial.println("MPU-6050 Configuration Bytes:");
  Serial.print("  CONFIG     (0x1A): 0x");
  Serial.println(cnfig, HEX);

  Serial.print("  GYRO CONFIG (0x1B): 0x");
  Serial.println(gcfig, HEX);

  Serial.print("  ACCEL CONFIG (0x1C): 0x");
  Serial.println(acfig, HEX);



  Serial.print("Accel: ");
  Serial.print(ax); Serial.print(" ");
  Serial.print(ay); Serial.print(" ");
  Serial.print(az); Serial.print(" | Gyro: ");
  Serial.print(gx); Serial.print(" ");
  Serial.print(gy); Serial.print(" ");
  Serial.print(gz); Serial.println();

  delay(500);

};