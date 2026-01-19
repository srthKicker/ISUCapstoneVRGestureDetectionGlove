BHI360 ESP32 Driver (ESP-IDF)
ESP-IDF application for Bosch BHI360 smart sensor. Loads firmware via I²C, enables Game Rotation Vector (quaternion output), and polls/parses FIFO data. Prints unit quaternions via UART at ~100 Hz.
​



#Note for Seth
# the golden command for seth $$$$ :-P $$$$
source ~/esp/esp-idf/export.sh 
then idf.py works in my terminal :D





Hardware Setup
ESP32 GPIO21 → BHI360 SDA (0x28, 100 kHz, <10cm wires)
ESP32 GPIO22 → BHI360 SCL
3.3V/5V level shifter (TXS0108E) recommended
Internal pullups OK for prototype; add 4.7kΩ external for production
Bosch_Shuttle3_BHI360.fw.h required (firmware binary)
Files
main.c: App entrypoint. I²C init, BHI360 firmware upload/boot, FIFO polling + quaternion callback.

bhi360_i2c.h: Function prototypes + bhi360_cntxt_t struct for dynamic port/address.

i2c.c: BHI360 API wrappers (bhi360_i2c_read/write/delay_us) using ESP-IDF driver + context pointer.

Build & Flash instructions
idf.py set-target esp32  (or esp32s3 if we need to)
idf.py build flash monitor


CMakeLists.txt explaination
cmake_minimum_required(VERSION 3.16)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(bhi360-esp32)
CMakeLists.txt (components/bhi360_i2c/CMakeLists.txt)
text
idf_component_register(SRCS "i2c.c"
                       INCLUDE_DIRS "."
                       REQUIRES driver bhi360)
Move i2c.c/h to components/bhi360_i2c/ or register in main/CMakeLists.txt. Add Bosch BHI360_SensorAPI as component dir.

Sensor Config
#define BHI360_VIRTUAL_SENSOR_ID 37 → Game Rotation Vector (accel/gyro only)

Scale /16384.0f → 16-bit fixed-point to float quaternion
​

Callback triggers on FIFO events; bhi360_set_virtual_sensor(37,1,20,0,&dev); to enable.

Output
text
Uploading firmware...
BHI360 ready - polling for rotation vector...
Rotation Vector: w=0.987 i=-0.012 j=0.045 k=0.156

Status (needs testing)
    I²C context switching ready
    100 kHz safe for level shifters
    Firmware + FIFO parsing completed