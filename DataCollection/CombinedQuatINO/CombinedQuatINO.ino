#include <MPU6500_WE.h>
#include <MPU9250_WE.h>
#include <MadgwickAHRS.h>

const int csPin[6] = {22,17,16,4,26,27};  // Chip Select Pin
const int mosiPin = 23;  // "MOSI" Pin
const int misoPin = 19;  // "MISO" Pin
const int sckPin = 18;  // SCK Pin
bool useSPI = true;    // SPI use flag

struct Quaternion {
  float w, x, y, z;
};

MPU9250_WE MPU9250 = MPU9250_WE(&SPI, csPin[0], mosiPin, misoPin, sckPin, useSPI);
MPU6500_WE MPU6500_1 = MPU6500_WE(&SPI, csPin[1], mosiPin, misoPin, sckPin, useSPI);
MPU6500_WE MPU6500_2 = MPU6500_WE(&SPI, csPin[2], mosiPin, misoPin, sckPin, useSPI);
MPU6500_WE MPU6500_3 = MPU6500_WE(&SPI, csPin[3], mosiPin, misoPin, sckPin, useSPI);
MPU6500_WE MPU6500_4 = MPU6500_WE(&SPI, csPin[4], mosiPin, misoPin, sckPin, useSPI);
MPU6500_WE MPU6500_5 = MPU6500_WE(&SPI, csPin[5], mosiPin, misoPin, sckPin, useSPI);

Madgwick madgwick9250;
Madgwick madgwick6500[5];

Quaternion q9250;
Quaternion q6500[5];

// Conjugate of a unit quaternion = its inverse
Quaternion quatInverse(Quaternion q) {
  return {q.w, -q.x, -q.y, -q.z};
}

// Hamilton product: a * b
Quaternion quatMultiply(Quaternion a, Quaternion b) {
  return {
    a.w*b.w - a.x*b.x - a.y*b.y - a.z*b.z,
    a.w*b.x + a.x*b.w + a.y*b.z - a.z*b.y,
    a.w*b.y - a.x*b.z + a.y*b.w + a.z*b.x,
    a.w*b.z + a.x*b.y - a.y*b.x + a.z*b.w
  };
}




void printVector(const char* label, float x, float y, float z) {
  Serial.println(label);
  Serial.print(x); Serial.print("   ");
  Serial.print(y); Serial.print("   ");
  Serial.println(z);
}

void printIMU(
  const char* imuName,
  float ax, float ay, float az, float resultantG,
  float gx, float gy, float gz,
  float temp,
  bool hasMag = false,
  float mx = 0, float my = 0, float mz = 0
) {
  Serial.println(imuName);

  printVector("Acceleration in g (x,y,z):", ax, ay, az);
  Serial.print("Resultant g: ");
  Serial.println(resultantG);

  printVector("Gyroscope data in degrees/s:", gx, gy, gz);

  if (hasMag) {
    printVector("Magnetometer data in µTesla:", mx, my, mz);
  }

  Serial.print("Temperature in °C: ");
  Serial.println(temp);

  Serial.println("********************************************");
}

void printQuat(const Quaternion& quat) {
  Serial.printf(
    "%.6f,%.6f,%.6f,%.6f",
    quat.w,
    quat.x,
    quat.y,
    quat.z
  );
}


void setup() {
  Serial.begin(500000);
  delay(200);
//Check MPU Connections:
  if(!MPU9250.init()){
    Serial.println("MPU9250 does not respond");
  }
  else{
    Serial.println("MPU9250 is connected");
  }
  if(!MPU9250.initMagnetometer()){
    Serial.println("Magnetometer does not respond");
  }
  else{
    Serial.println("Magnetometer is connected");
  }
  if(!MPU6500_1.init()){
    Serial.println("MPU6500_1 does not respond");
  }
  else{
    Serial.println("MPU6500_1 is connected");
  }
  if(!MPU6500_2.init()){
    Serial.println("MPU6500_2 does not respond");
  }
  else{
    Serial.println("MPU6500_2 is connected");
  }
  if(!MPU6500_3.init()){
    Serial.println("MPU6500_3 does not respond");
  }
  else{
    Serial.println("MPU6500_3 is connected");
  }
  if(!MPU6500_4.init()){
    Serial.println("MPU6500_4 does not respond");
  }
  else{
    Serial.println("MPU6500_4 is connected");
  }
  if(!MPU6500_5.init()){
    Serial.println("MPU6500_5 does not respond");
  }
    else{
    Serial.println("MPU6500_5 is connected");
    }

  Serial.println("Position MPU's flat and don't move it - calibrating...");
  delay(1000);
  MPU9250.autoOffsets();
  MPU6500_1.autoOffsets();
  MPU6500_2.autoOffsets();
  MPU6500_3.autoOffsets();
  MPU6500_4.autoOffsets();
  MPU6500_5.autoOffsets();
  Serial.println("Done!");

  MPU9250.enableGyrDLPF();
  MPU9250.setGyrDLPF(MPU9250_DLPF_6);
  MPU9250.setSampleRateDivider(5);
  MPU9250.setGyrRange(MPU9250_GYRO_RANGE_250);
  MPU9250.setAccRange(MPU9250_ACC_RANGE_2G);
  MPU9250.enableAccDLPF(true);
  MPU9250.setAccDLPF(MPU9250_DLPF_6);
  MPU9250.setMagOpMode(AK8963_CONT_MODE_100HZ);

  MPU6500_1.enableGyrDLPF();
  MPU6500_1.setGyrDLPF(MPU6500_DLPF_6);
  MPU6500_1.setSampleRateDivider(5);
  MPU6500_1.setGyrRange(MPU6500_GYRO_RANGE_250);
  MPU6500_1.setAccRange(MPU6500_ACC_RANGE_2G);
  MPU6500_1.enableAccDLPF(true);
  MPU6500_1.setAccDLPF(MPU6500_DLPF_6);

  MPU6500_2.enableGyrDLPF();
  MPU6500_2.setGyrDLPF(MPU6500_DLPF_6);
  MPU6500_2.setSampleRateDivider(5);
  MPU6500_2.setGyrRange(MPU6500_GYRO_RANGE_250);
  MPU6500_2.setAccRange(MPU6500_ACC_RANGE_2G);
  MPU6500_2.enableAccDLPF(true);
  MPU6500_2.setAccDLPF(MPU6500_DLPF_6);

  MPU6500_3.enableGyrDLPF();
  MPU6500_3.setGyrDLPF(MPU6500_DLPF_6);
  MPU6500_3.setSampleRateDivider(5);
  MPU6500_3.setGyrRange(MPU6500_GYRO_RANGE_250);
  MPU6500_3.setAccRange(MPU6500_ACC_RANGE_2G);
  MPU6500_3.enableAccDLPF(true);
  MPU6500_3.setAccDLPF(MPU6500_DLPF_6);

  MPU6500_4.enableGyrDLPF();
  MPU6500_4.setGyrDLPF(MPU6500_DLPF_6);
  MPU6500_4.setSampleRateDivider(5);
  MPU6500_4.setGyrRange(MPU6500_GYRO_RANGE_250);
  MPU6500_4.setAccRange(MPU6500_ACC_RANGE_2G);
  MPU6500_4.enableAccDLPF(true);
  MPU6500_4.setAccDLPF(MPU6500_DLPF_6);

  MPU6500_5.enableGyrDLPF();
  MPU6500_5.setGyrDLPF(MPU6500_DLPF_6);
  MPU6500_5.setSampleRateDivider(5);
  MPU6500_5.setGyrRange(MPU6500_GYRO_RANGE_250);
  MPU6500_5.setAccRange(MPU6500_ACC_RANGE_2G);
  MPU6500_5.enableAccDLPF(true);
  MPU6500_5.setAccDLPF(MPU6500_DLPF_6);

  // Initialize Madgwick filters with sample rate
  // Sample rate = 1000 / (1 + divider) = 1000 / 6 ≈ 166 Hz
  madgwick9250.begin(166);
  for (int i = 0; i < 5; i++) {
    madgwick6500[i].begin(166);
  }

  delay(200);
}

void loop() {
  // put your main code here, to run repeatedly:

  xyzFloat gValue_9250 = MPU9250.getGValues();
  xyzFloat gyr_9250 = MPU9250.getGyrValues();
  xyzFloat magValue = MPU9250.getMagValues(); 
  float temp = MPU9250.getTemperature();
  float resultantG = MPU9250.getResultantG(gValue_9250);

  xyzFloat gValue_1 = MPU6500_1.getGValues();
  xyzFloat gyr_1 = MPU6500_1.getGyrValues();
  float temp_1 = MPU6500_1.getTemperature();
  float resultantG_1 = MPU6500_1.getResultantG(gValue_1);

  xyzFloat gValue_2 = MPU6500_2.getGValues();
  xyzFloat gyr_2 = MPU6500_2.getGyrValues();
  float temp_2 = MPU6500_2.getTemperature();
  float resultantG_2 = MPU6500_2.getResultantG(gValue_2);
  
  xyzFloat gValue_3 = MPU6500_3.getGValues();
  xyzFloat gyr_3 = MPU6500_3.getGyrValues();
  float temp_3 = MPU6500_3.getTemperature();
  float resultantG_3 = MPU6500_3.getResultantG(gValue_3);

  xyzFloat gValue_4 = MPU6500_4.getGValues();
  xyzFloat gyr_4 = MPU6500_4.getGyrValues();
  float temp_4 = MPU6500_4.getTemperature();
  float resultantG_4 = MPU6500_4.getResultantG(gValue_4);

  xyzFloat gValue_5 = MPU6500_5.getGValues();
  xyzFloat gyr_5 = MPU6500_5.getGyrValues();
  float temp_5 = MPU6500_5.getTemperature();
  float resultantG_5 = MPU6500_5.getResultantG(gValue_5);



  madgwick9250.update(gyr_9250.x, gyr_9250.y, gyr_9250.z, gValue_9250.x, gValue_9250.y, gValue_9250.z, magValue.x, magValue.y, magValue.z);
  

  madgwick6500[0].updateIMU(gyr_1.x, gyr_1.y, gyr_1.z, gValue_1.x, gValue_1.y, gValue_1.z);
  madgwick6500[1].updateIMU(gyr_2.x, gyr_2.y, gyr_2.z, gValue_2.x, gValue_2.y, gValue_2.z);
  madgwick6500[2].updateIMU(gyr_3.x, gyr_3.y, gyr_3.z, gValue_3.x, gValue_3.y, gValue_3.z);
  madgwick6500[3].updateIMU(gyr_4.x, gyr_4.y, gyr_4.z, gValue_4.x, gValue_4.y, gValue_4.z);
  madgwick6500[4].updateIMU(gyr_5.x, gyr_5.y, gyr_5.z, gValue_5.x, gValue_5.y, gValue_5.z);


  // Get wrist quaternion (global frame)
  q9250 = {
    madgwick9250.q0Getter(),
    madgwick9250.q1Getter(),
    madgwick9250.q2Getter(),
    madgwick9250.q3Getter()
  };

  // Get finger quaternions (global frame)
  for (int i = 0; i < 5; i++) {
    q6500[i] = {
      madgwick6500[i].q0Getter(),
      madgwick6500[i].q1Getter(),
      madgwick6500[i].q2Getter(),
      madgwick6500[i].q3Getter()
    };
  }

  // Compute wrist-relative finger quaternions:
  // q_relative = inverse(q_wrist) * q_finger
  // This removes wrist rotation so each finger is independent of the wrist.
  Quaternion q_wrist_inv = quatInverse(q9250);
  Quaternion q_rel[5];
  for (int i = 0; i < 5; i++) {
    q_rel[i] = quatMultiply(q_wrist_inv, q6500[i]);
  }

  // Send: wrist (global), then 5 fingers (wrist-relative)
  printQuat(q9250);
  Serial.print(",");
  for (int i = 0; i < 5; i++) {
    printQuat(q_rel[i]);
    if (i < 4) Serial.print(",");
  }
  Serial.println();
  delay(10); 

}



