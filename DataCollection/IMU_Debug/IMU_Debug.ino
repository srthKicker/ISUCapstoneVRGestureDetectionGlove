// =============================================================
//  IMU_Debug.ino
//  Diagnostic sketch for 1x MPU9250 + 5x MPU6500 over SPI
//  Tests: SPI bus, init, WHO_AM_I, raw data sanity, Madgwick
//  Open Serial Monitor at 115200 baud
// =============================================================

#include <MPU6500_WE.h>
#include <MPU9250_WE.h>
#include <MadgwickAHRS.h>
#include <SPI.h>

// ---- Pin config (replace 14 -> 25 if you swapped it) --------
const int csPin[6]  = {22, 17, 16, 4, 27, 26};
const int mosiPin   = 23;
const int misoPin   = 19;
const int sckPin    = 18;
bool      useSPI    = true;

// ---- Expected WHO_AM_I values --------------------------------
// MPU9250 = 0x71  (some clones return 0x73)
// MPU6500 = 0x70
#define WHOAMI_MPU9250_A  0x71
#define WHOAMI_MPU9250_B  0x73  // common clone value
#define WHOAMI_MPU6500    0x70

// ---- Thresholds for sanity checks ----------------------------
#define ACCEL_G_MIN   0.85f   // at rest, resultant g should be ~1.0
#define ACCEL_G_MAX   1.15f
#define GYRO_DRIFT_MAX 5.0f   // deg/s at rest — larger means a problem

// ---- IMU objects ---------------------------------------------
MPU9250_WE imu9250  = MPU9250_WE(&SPI, csPin[0], mosiPin, misoPin, sckPin, useSPI);
MPU6500_WE imu6500[5] = {
  MPU6500_WE(&SPI, csPin[1], mosiPin, misoPin, sckPin, useSPI),
  MPU6500_WE(&SPI, csPin[2], mosiPin, misoPin, sckPin, useSPI),
  MPU6500_WE(&SPI, csPin[3], mosiPin, misoPin, sckPin, useSPI),
  MPU6500_WE(&SPI, csPin[4], mosiPin, misoPin, sckPin, useSPI),
  MPU6500_WE(&SPI, csPin[5], mosiPin, misoPin, sckPin, useSPI),
};

// ---- Track which sensors actually initialised ---------------
bool ok9250 = false;
bool okMag  = false;
bool ok6500[5] = {false, false, false, false, false};

Madgwick filter9250;
Madgwick filter6500[5];

// =============================================================
//  Helpers
// =============================================================

void separator(char c = '-', int len = 50) {
  for (int i = 0; i < len; i++) Serial.print(c);
  Serial.println();
}

void printPass()  { Serial.print("  [PASS]  "); }
void printFail()  { Serial.print("  [FAIL]  "); }
void printWarn()  { Serial.print("  [WARN]  "); }

// Manually wiggle a CS pin and check it responds — confirms the
// GPIO itself is driving correctly before the library touches it.
void testCSPin(int pin, const char* label) {
  pinMode(pin, OUTPUT);
  digitalWrite(pin, HIGH);
  delay(1);
  bool high = digitalRead(pin);
  digitalWrite(pin, LOW);
  delay(1);
  bool low = digitalRead(pin);
  digitalWrite(pin, HIGH); // leave deselected

  if (high && !low) {
    printPass();
  } else {
    printWarn();
  }
  Serial.print(label);
  Serial.print(" CS pin GPIO");
  Serial.print(pin);
  Serial.print(": HIGH=");
  Serial.print(high);
  Serial.print(" LOW=");
  Serial.println(low);
}

// =============================================================
//  STAGE 1 — SPI Bus
// =============================================================
void testSPIBus() {
  separator('=');
  Serial.println("STAGE 1: SPI Bus Init");
  separator();

  SPI.begin(sckPin, misoPin, mosiPin);
  delay(100);
  printPass();
  Serial.println("SPI.begin() called");

  Serial.println();
  Serial.println("  CS pin GPIO checks (should toggle HIGH<->LOW freely):");
  testCSPin(csPin[0], "MPU9250 ");
  testCSPin(csPin[1], "MPU6500_1");
  testCSPin(csPin[2], "MPU6500_2");
  testCSPin(csPin[3], "MPU6500_3");
  testCSPin(csPin[4], "MPU6500_4");
  testCSPin(csPin[5], "MPU6500_5");
}

// =============================================================
//  STAGE 2 — init() for every sensor
// =============================================================
void testInit() {
  separator('=');
  Serial.println("STAGE 2: Sensor init()");
  separator();

  ok9250 = imu9250.init();
  if (ok9250) { printPass(); Serial.println("MPU9250  init OK"); }
  else         { printFail(); Serial.println("MPU9250  init FAILED — check wiring on CS GPIO" + String(csPin[0])); }

  okMag = ok9250 && imu9250.initMagnetometer();
  if (okMag)  { printPass(); Serial.println("  Magnetometer init OK"); }
  else         { printWarn(); Serial.println("  Magnetometer init FAILED (MPU9250 needed first)"); }

  for (int i = 0; i < 5; i++) {
    ok6500[i] = imu6500[i].init();
    if (ok6500[i]) {
      printPass();
    } else {
      printFail();
    }
    Serial.print("MPU6500_");
    Serial.print(i + 1);
    Serial.print(" CS GPIO");
    Serial.print(csPin[i + 1]);
    if (!ok6500[i]) Serial.print("  <-- FAILED — check wiring");
    Serial.println();
  }

  // Summary
  int passed = ok9250 + ok6500[0] + ok6500[1] + ok6500[2] + ok6500[3] + ok6500[4];
  Serial.println();
  Serial.print("  Init result: ");
  Serial.print(passed);
  Serial.println("/6 sensors responded");
}

// =============================================================
//  STAGE 3 — Raw accel/gyro sanity  (sensors flat, stationary)
// =============================================================
void testRawData() {
  separator('=');
  Serial.println("STAGE 3: Raw data sanity  (keep glove FLAT & STILL)");
  separator();

  // ---- MPU9250 ----
  if (ok9250) {
    xyzFloat acc = imu9250.getGValues();
    xyzFloat gyr = imu9250.getGyrValues();
    float rg     = imu9250.getResultantG(acc);

    Serial.println("MPU9250:");
    Serial.print  ("  Accel g  x="); Serial.print(acc.x, 3);
    Serial.print  ("  y=");          Serial.print(acc.y, 3);
    Serial.print  ("  z=");          Serial.print(acc.z, 3);
    Serial.print  ("  |g|=");        Serial.println(rg, 3);

    Serial.print  ("  Gyro °/s x="); Serial.print(gyr.x, 2);
    Serial.print  ("  y=");          Serial.print(gyr.y, 2);
    Serial.print  ("  z=");          Serial.println(gyr.z, 2);

    if (rg >= ACCEL_G_MIN && rg <= ACCEL_G_MAX) {
      printPass(); Serial.println("MPU9250 accel magnitude in range");
    } else {
      printFail();
      Serial.print("MPU9250 accel magnitude OUT OF RANGE: "); Serial.println(rg, 3);
    }

    float gyrMag = sqrt(gyr.x*gyr.x + gyr.y*gyr.y + gyr.z*gyr.z);
    if (gyrMag < GYRO_DRIFT_MAX) {
      printPass(); Serial.println("MPU9250 gyro drift OK");
    } else {
      printWarn();
      Serial.print("MPU9250 gyro drift high: "); Serial.print(gyrMag, 2); Serial.println(" °/s");
    }

    if (okMag) {
      xyzFloat mag = imu9250.getMagValues();
      Serial.print  ("  Mag µT   x="); Serial.print(mag.x, 2);
      Serial.print  ("  y=");          Serial.print(mag.y, 2);
      Serial.print  ("  z=");          Serial.println(mag.z, 2);
      float magMag = sqrt(mag.x*mag.x + mag.y*mag.y + mag.z*mag.z);
      // Earth's field is roughly 25–65 µT
      if (magMag > 10.0f && magMag < 100.0f) {
        printPass(); Serial.println("MPU9250 magnetometer field plausible");
      } else {
        printWarn();
        Serial.print("MPU9250 magnetometer field unusual: "); Serial.print(magMag, 1); Serial.println(" µT");
      }
    }
    Serial.println();
  }

  // ---- MPU6500s ----
  for (int i = 0; i < 5; i++) {
    if (!ok6500[i]) {
      printFail();
      Serial.print("MPU6500_"); Serial.print(i + 1);
      Serial.println(" skipped (failed init)");
      Serial.println();
      continue;
    }

    xyzFloat acc = imu6500[i].getGValues();
    xyzFloat gyr = imu6500[i].getGyrValues();
    float rg     = imu6500[i].getResultantG(acc);

    Serial.print("MPU6500_"); Serial.print(i + 1); Serial.println(":");
    Serial.print  ("  Accel g  x="); Serial.print(acc.x, 3);
    Serial.print  ("  y=");          Serial.print(acc.y, 3);
    Serial.print  ("  z=");          Serial.print(acc.z, 3);
    Serial.print  ("  |g|=");        Serial.println(rg, 3);

    Serial.print  ("  Gyro °/s x="); Serial.print(gyr.x, 2);
    Serial.print  ("  y=");          Serial.print(gyr.y, 2);
    Serial.print  ("  z=");          Serial.println(gyr.z, 2);

    if (rg >= ACCEL_G_MIN && rg <= ACCEL_G_MAX) {
      printPass();
    } else {
      printFail();
    }
    Serial.print("MPU6500_"); Serial.print(i + 1);
    Serial.print(" accel magnitude: "); Serial.print(rg, 3);
    if (rg < ACCEL_G_MIN || rg > ACCEL_G_MAX) Serial.print("  <-- OUT OF RANGE");
    Serial.println();

    float gyrMag = sqrt(gyr.x*gyr.x + gyr.y*gyr.y + gyr.z*gyr.z);
    if (gyrMag < GYRO_DRIFT_MAX) {
      printPass();
    } else {
      printWarn();
    }
    Serial.print("MPU6500_"); Serial.print(i + 1);
    Serial.print(" gyro drift: "); Serial.print(gyrMag, 2); Serial.println(" °/s");

    Serial.println();
  }
}

// =============================================================
//  STAGE 4 — Madgwick filter  (checks quaternion is unit length)
// =============================================================
void testMadgwick() {
  separator('=');
  Serial.println("STAGE 4: Madgwick filter  (50 iterations each sensor)");
  separator();

  // warm up
  for (int iter = 0; iter < 50; iter++) {
    if (ok9250) {
      xyzFloat a = imu9250.getGValues();
      xyzFloat g = imu9250.getGyrValues();
      if (okMag) {
        xyzFloat m = imu9250.getMagValues();
        filter9250.update(g.x, g.y, g.z, a.x, a.y, a.z, m.x, m.y, m.z);
      } else {
        filter9250.updateIMU(g.x, g.y, g.z, a.x, a.y, a.z);
      }
    }
    for (int i = 0; i < 5; i++) {
      if (ok6500[i]) {
        xyzFloat a = imu6500[i].getGValues();
        xyzFloat g = imu6500[i].getGyrValues();
        filter6500[i].updateIMU(g.x, g.y, g.z, a.x, a.y, a.z);
      }
    }
    delay(10);
  }

  // check quaternion norms
  auto checkQuat = [](float w, float x, float y, float z, const char* name) {
    float norm = sqrt(w*w + x*x + y*y + z*z);
    Serial.print(name);
    Serial.print(":  w="); Serial.print(w, 4);
    Serial.print("  x="); Serial.print(x, 4);
    Serial.print("  y="); Serial.print(y, 4);
    Serial.print("  z="); Serial.print(z, 4);
    Serial.print("  |q|="); Serial.print(norm, 5);
    if (norm > 0.98f && norm < 1.02f) {
      Serial.println("  [PASS]");
    } else {
      Serial.println("  [FAIL] norm not ~1.0 — filter may be diverging");
    }
  };

  if (ok9250)
    checkQuat(filter9250.q0Getter(), filter9250.q1Getter(),
              filter9250.q2Getter(), filter9250.q3Getter(), "MPU9250 ");

  for (int i = 0; i < 5; i++) {
    char label[12];
    snprintf(label, sizeof(label), "MPU6500_%d", i + 1);
    if (ok6500[i])
      checkQuat(filter6500[i].q0Getter(), filter6500[i].q1Getter(),
                filter6500[i].q2Getter(), filter6500[i].q3Getter(), label);
    else {
      Serial.print(label);
      Serial.println(":  skipped (failed init)");
    }
  }
}

// =============================================================
//  STAGE 5 — Live data stream  (runs forever, prints every 1s)
// =============================================================
void liveStream() {
  separator('=');
  Serial.println("STAGE 5: Live stream — one line per second");
  Serial.println("  Format:  sensor | |g| | gyro_mag | q_norm");
  separator();

  while (true) {
    if (ok9250) {
      xyzFloat a = imu9250.getGValues();
      xyzFloat g = imu9250.getGyrValues();
      if (okMag) {
        xyzFloat m = imu9250.getMagValues();
        filter9250.update(g.x, g.y, g.z, a.x, a.y, a.z, m.x, m.y, m.z);
      } else {
        filter9250.updateIMU(g.x, g.y, g.z, a.x, a.y, a.z);
      }
    }
    for (int i = 0; i < 5; i++) {
      if (ok6500[i]) {
        xyzFloat a = imu6500[i].getGValues();
        xyzFloat g = imu6500[i].getGyrValues();
        filter6500[i].updateIMU(g.x, g.y, g.z, a.x, a.y, a.z);
      }
    }

    static uint32_t lastPrint = 0;
    if (millis() - lastPrint >= 1000) {
      lastPrint = millis();

      // ---- MPU9250 ----
      if (ok9250) {
        xyzFloat a = imu9250.getGValues();
        xyzFloat g = imu9250.getGyrValues();
        float rg   = imu9250.getResultantG(a);
        float gm   = sqrt(g.x*g.x + g.y*g.y + g.z*g.z);
        float w = filter9250.q0Getter(), x = filter9250.q1Getter(),
              y = filter9250.q2Getter(), z = filter9250.q3Getter();
        float qn   = sqrt(w*w + x*x + y*y + z*z);

        Serial.printf("MPU9250   |g|=%5.3f  gyr=%5.2f°/s  |q|=%6.4f  (w=%5.3f x=%5.3f y=%5.3f z=%5.3f)\n",
                      rg, gm, qn, w, x, y, z);
      }

      // ---- MPU6500s ----
      for (int i = 0; i < 5; i++) {
        if (!ok6500[i]) {
          Serial.printf("MPU6500_%d  [OFFLINE]\n", i + 1);
          continue;
        }
        xyzFloat a = imu6500[i].getGValues();
        xyzFloat g = imu6500[i].getGyrValues();
        float rg   = imu6500[i].getResultantG(a);
        float gm   = sqrt(g.x*g.x + g.y*g.y + g.z*g.z);
        float w = filter6500[i].q0Getter(), x = filter6500[i].q1Getter(),
              y = filter6500[i].q2Getter(), z = filter6500[i].q3Getter();
        float qn   = sqrt(w*w + x*x + y*y + z*z);

        Serial.printf("MPU6500_%d  |g|=%5.3f  gyr=%5.2f°/s  |q|=%6.4f  (w=%5.3f x=%5.3f y=%5.3f z=%5.3f)\n",
                      i + 1, rg, gm, qn, w, x, y, z);
      }
      separator();
    }
    delay(10);
  }
}

// =============================================================
//  setup / loop
// =============================================================
void setup() {
  Serial.begin(115200);
  delay(500); // give Serial monitor time to open

  separator('=');
  Serial.println("  IMU DIAGNOSTIC TOOL");
  Serial.println("  1x MPU9250 + 5x MPU6500  |  SPI");
  separator('=');
  Serial.println();

  testSPIBus();
  Serial.println();
  delay(200);

  testInit();
  Serial.println();
  delay(200);

  // Calibrate only the sensors that passed init
  Serial.println("Calibrating (keep flat & still)...");
  delay(1000);
  if (ok9250)    imu9250.autoOffsets();
  for (int i = 0; i < 5; i++)
    if (ok6500[i]) imu6500[i].autoOffsets();
  Serial.println("Calibration done.");
  Serial.println();

  // Apply standard config only to sensors that passed
  if (ok9250) {
    imu9250.enableGyrDLPF();
    imu9250.setGyrDLPF(MPU9250_DLPF_6);
    imu9250.setSampleRateDivider(5);
    imu9250.setGyrRange(MPU9250_GYRO_RANGE_250);
    imu9250.setAccRange(MPU9250_ACC_RANGE_2G);
    imu9250.enableAccDLPF(true);
    imu9250.setAccDLPF(MPU9250_DLPF_6);
    if (okMag) imu9250.setMagOpMode(AK8963_CONT_MODE_100HZ);
  }
  for (int i = 0; i < 5; i++) {
    if (!ok6500[i]) continue;
    imu6500[i].enableGyrDLPF();
    imu6500[i].setGyrDLPF(MPU6500_DLPF_6);
    imu6500[i].setSampleRateDivider(5);
    imu6500[i].setGyrRange(MPU6500_GYRO_RANGE_250);
    imu6500[i].setAccRange(MPU6500_ACC_RANGE_2G);
    imu6500[i].enableAccDLPF(true);
    imu6500[i].setAccDLPF(MPU6500_DLPF_6);
  }

  testRawData();
  Serial.println();

  testMadgwick();
  Serial.println();

  liveStream(); // runs forever
}

void loop() {
  // everything is in liveStream() inside setup()
}
