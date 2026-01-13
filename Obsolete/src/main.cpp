#include <Arduino.h>
#include <Wire.h>

const int MPU_ADDR = 0x68;

void setup() {
  Serial.begin(115200);
  Wire.begin(21,22);

  //To initialize the mpu6050
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B); //Power management register
  Wire.write(0); //Write to 0 to wake up the sensor
  Wire.endTransmission(true);

}
/*
  Note: This is a test bit of code, and the goal is 
  to be able to test the accuracy and speed of the
  madgwick filter on the ESP32 
*/
void loop(){

  Wire.beginTransmission(MPU_ADDR); //Connect to mpu
  Wire.write(0x3B); //Accelerometer starting register in mpu 6050
  //Accelerometer has 6 bytes total, 2 each for X, Y, and Z axis
  Wire.endTransmission(false); //Restarts for read or something
  //14 bytes will give us all of accelerometer, temp, and gyroscope
  Wire.requestFrom(MPU_ADDR, 14, true);
  //Since wire.read returns one byte at a time, this is how we have
  //to retrieve information from the buffer
  int16_t ax = Wire.read() << 8 | Wire.read();
  int16_t ay = Wire.read() << 8 | Wire.read();
  int16_t az = Wire.read() << 8 | Wire.read();
  int16_t temp = Wire.read() << 8 | Wire.read(); // Raw temperature
  int16_t gx = Wire.read() << 8 | Wire.read();
  int16_t gy = Wire.read() << 8 | Wire.read();
  int16_t gz = Wire.read() << 8 | Wire.read();

  Serial.print("Accelerometer: ");
  Serial.print(ax); Serial.print(", ");
  Serial.print(ay); Serial.print(", ");
  Serial.print(az); Serial.print(", | Temp: ");
  Serial.print(temp); Serial.print(", | Gyroscope: ");
  Serial.print(gx); Serial.print(", ");
  Serial.print(gy); Serial.print(", ");
  Serial.print(gz); Serial.println();

  delay(2000); //Slows down loop for testing purposes
}
