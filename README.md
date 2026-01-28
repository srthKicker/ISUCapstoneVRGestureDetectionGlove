#ARGH

Branch to test driver editing to remove COINES dependance

Currently using https://github.com/Dmivaka/STM32-HAL-BHI360 which is easier to adapt to ESP-IDF

That is the baseline for the drivers, hopefully will take less time to implement.

BHI360 + ESP32 Interface (Work in Progress)
This repository contains in‑progress ESP‑IDF firmware for communicating with the Bosch BHI360 smart sensor hub using the modern ESP‑IDF I²C master API. The project focuses on establishing reliable I²C communication, loading firmware into the BHI360, enabling virtual sensors, and parsing FIFO data — currently demonstrated with the Game Rotation Vector virtual sensor.


Current Features
Modern ESP‑IDF i2c_master_bus + device‑handle API

Clean I²C abstraction layer (bhi360_i2c_read, bhi360_i2c_write, bhi360_delay_us)

BHI360 initialization using Bosch’s BHY2 driver

Firmware upload to RAM (Bosch_Shuttle3_BHI360.fw.h)

Virtual sensor configuration (Game Rotation Vector, ID 37)

FIFO polling + callback‑based quaternion parsing

Debug logging for I²C bring‑up and sensor validation

Required Hardware
1. Bosch BHI360 Sensor
Any board exposing 1.8 V I²C, such as:

Bosch Shuttle Board

Custom breakout

Third‑party carrier

2. ESP32 Development Board
Any ESP32 variant

Current development targets the standard ESP32 (ESP32‑WROOM)

3. Bidirectional Level Shifter (Required)
The BHI360 uses 1.8 V I²C, while the ESP32 uses 3.3 V.
A proper bidirectional level shifter (e.g., BSS138‑based) is required for:

SDA

SCL

4. Power Requirements
BHI360 must be powered from a stable 1.8 V supply

Do not connect it directly to the ESP32’s 3.3 V rail

Repository Structure
All code is in the FirmwareV0.11 directory, inside that is where you can initialize ESP-IDF to run the program
/FirmwareV0.11/main
    main.c                 # ESP32 application entry point
    i2c.c                  # Modern ESP-IDF I2C read/write wrappers
    bhi360_i2c.h           # I2C abstraction + context struct
FirmwareV0.11/components/bhi360
    bhi3.h                 # Bosch driver definitions
    Also includes other driver code.
FirmwareV0.11/components/bhi360/firmware
    Bosch_Shuttle3_BHI360.fw.h   # Firmware for the BHI360 shuttle board. This directory includes other versions for other sensors.
    (source BOSCH)

Getting Started
1. Wire the hardware
BHI360 → Level Shifter → ESP32

Ensure correct pull‑ups (internal pull‑ups currently enabled in code)

Provide a clean 1.8 V supply to the sensor

2. Build and flash
Code
idf.py build
idf.py flash monitor
3. Expected output
Successful I²C probe (CHIP_ID = 0xC8)

Firmware upload messages

FIFO callback printing quaternion values

What the Code Currently Does
Initializes I²C using the modern ESP‑IDF bus/device API

Reads the BHI360 chip ID to verify communication

Uploads firmware to RAM and boots the sensor

Registers a FIFO callback (rot_vec_cb)

Enables the Game Rotation Vector virtual sensor at 100 Hz

Continuously polls FIFO and prints quaternion data

