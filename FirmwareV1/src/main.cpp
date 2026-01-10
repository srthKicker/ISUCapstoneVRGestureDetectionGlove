#include <Arduino.h>
#include <Wire.h>
#include <I2Cdev.h>
#include <MPU6050.h>
#include <Quaternion.h>

/*
Constant declarations
*/
const int MPU_ADDR = 0x68; //Only 2 I2C addresses available 
const int SDA_Pin = 21; //Currently on default I2C pins
const int SCL_Pin = 22;
const int I2CBusSpeed = 400000; // Start with 100kHz, could go to 400kHz if bottlenecking is an issue
/*
Variable declarations
*/
MPU6050 mpu; //The first processor we will test (will add more for the 5 fingers)
bool mpuIsConnected; //Probably will add more as well for the other fingers

/*
Function declarations
*/
//Reads the byte from a piece of memory over I2C
byte readRegister(uint8_t reg) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 1, true);
  return Wire.read();
}

/*
  Setup will initialize communication, 
*/
void setup() {
  Serial.begin(115200);
  Wire.begin(SDA_Pin, SCL_Pin);
  Wire.setClock(I2CBusSpeed); //Sets the clock rate
  mpu.initialize();

  //Tests the connection
  if(mpu.testConnection()){
    Serial.println("MPU initialized successfully!");
    mpuIsConnected = true;
  }
  else{ //If it doesnt initialize, stay here forever. Use vTaskDelete on RTOS
    Serial.println("MPU FAILED to initialize.");
    while(true){
      Serial.println("Failed to intialize MPU... Too bad, so sad.");
      delay(500); //Stops all other operations, but we want that.
    }
  }

}

void loop() {

}
