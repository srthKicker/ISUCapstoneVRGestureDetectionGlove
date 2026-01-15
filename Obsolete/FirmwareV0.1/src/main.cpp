#include <Arduino.h>
#include <Wire.h>

// Bosch BHY2 headers (adjust paths as needed)
#include "bhy2.h"
#include "bhy2_hif.h"
#include "bhy2_parse.h"

/*
    Seth Kiker: This is AI slop code, complete garbage for now, just wanted
    to try something to see if it'd work (it didnt) so ignore for now. I'll
    be back to this later.

    Note: If you want the MPU6050 version, just use the oldMPU6050Main.txt file, I kept it there.
*/


// I2C config
static constexpr uint8_t BHI360_I2C_ADDR = 0x28;   // Check your board (0x28 or 0x29)
static constexpr int SDA_PIN = 21;
static constexpr int SCL_PIN = 22;
static constexpr uint32_t I2C_FREQ = 100000;

// BHY2 globals
static bhy2_dev bhy2;
static uint8_t fifo_work_buffer[2048];

// Forward declarations
static int8_t i2c_read(uint8_t reg, uint8_t *data, uint16_t len, void *intf_ptr);
static int8_t i2c_write(uint8_t reg, const uint8_t *data, uint16_t len, void *intf_ptr);
static void game_rv_callback(const bhy2_fifo_parse_data_info *info, void *ref);
static void check_rslt(const char *msg, int8_t rslt);

// Firmware blobs (you must provide these)
extern const uint8_t bhi360_ram_patch[];
extern const uint32_t bhi360_ram_patch_size;
extern const uint8_t bhi360_firmware[];
extern const uint32_t bhi360_firmware_size;

void setup()
{
    Serial.begin(115200);
    delay(1000);
    Serial.println("BHI360 bring-up starting...");

    // --- I2C init ---
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN, I2C_FREQ);

    // --- BHY2 device init ---
    memset(&bhy2, 0, sizeof(bhy2));
    bhy2.intf = BHY2_I2C_INTERFACE;
    bhy2.intf_ptr = (void *)&BHI360_I2C_ADDR;
    bhy2.read = i2c_read;
    bhy2.write = i2c_write;
    bhy2.delay_us = [](uint32_t period, void *intf_ptr) {
        (void)intf_ptr;
        delayMicroseconds(period);
    };

    int8_t rslt = bhy2_init(&bhy2);
    check_rslt("bhy2_init", rslt);

    // --- Upload RAM patch ---
    rslt = bhy2_patch_upload(bhi360_ram_patch, bhi360_ram_patch_size, &bhy2);
    check_rslt("bhy2_patch_upload", rslt);

    // --- Upload firmware ---
    rslt = bhy2_firmware_upload(bhi360_firmware, bhi360_firmware_size, &bhy2);
    check_rslt("bhy2_firmware_upload", rslt);

    // --- Boot firmware ---
    rslt = bhy2_boot_firmware(&bhy2);
    check_rslt("bhy2_boot_firmware", rslt);

    // --- Verify firmware running ---
    uint8_t status = 0;
    rslt = bhy2_get_boot_status(&status, &bhy2);
    check_rslt("bhy2_get_boot_status", rslt);
    Serial.printf("Boot status: 0x%02X\n", status);

    // --- Register Game Rotation Vector callback ---
    rslt = bhy2_register_fifo_parse_callback(BHY2_SENSOR_ID_GAME_ROTATION_VECTOR,
                                             game_rv_callback, nullptr, &bhy2);
    check_rslt("bhy2_register_fifo_parse_callback", rslt);

    // --- Enable Game Rotation Vector virtual sensor ---
    float odr_hz = 100.0f; // desired ODR
    rslt = bhy2_set_virt_sensor_cfg(BHY2_SENSOR_ID_GAME_ROTATION_VECTOR,
                                    BHY2_SENSOR_ENABLE, odr_hz, 0, &bhy2);
    check_rslt("bhy2_set_virt_sensor_cfg", rslt);

    Serial.println("BHI360 initialized, GameRV enabled.");
}

void loop()
{
    // Simple polling loop; you can switch to interrupt-driven later
    uint8_t fifo_buffer[512];
    uint16_t fifo_len = sizeof(fifo_buffer);

    int8_t rslt = bhy2_hif_read_fifo(fifo_buffer, &fifo_len, &bhy2);
    if (rslt == BHY2_OK && fifo_len > 0)
    {
        bhy2_parse_fifo(fifo_buffer, fifo_len, &bhy2);
    }

    delay(10);
}

// --- I2C read/write wrappers ---

static int8_t i2c_read(uint8_t reg, uint8_t *data, uint16_t len, void *intf_ptr)
{
    uint8_t addr = *(uint8_t *)intf_ptr;

    Wire.beginTransmission(addr);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0)
        return BHY2_E_COM_FAILED;

    uint16_t read = Wire.requestFrom((int)addr, (int)len);
    if (read != len)
        return BHY2_E_COM_FAIL;

    for (uint16_t i = 0; i < len; i++)
        data[i] = Wire.read();

    return BHY2_OK;
}

static int8_t i2c_write(uint8_t reg, const uint8_t *data, uint16_t len, void *intf_ptr)
{
    uint8_t addr = *(uint8_t *)intf_ptr;

    Wire.beginTransmission(addr);
    Wire.write(reg);
    for (uint16_t i = 0; i < len; i++)
        Wire.write(data[i]);
    if (Wire.endTransmission(true) != 0)
        return BHY2_E_COM_FAIL;

    return BHY2_OK;
}

// --- Game Rotation Vector callback ---

static void game_rv_callback(const bhy2_fifo_parse_data_info *info, void *ref)
{
    (void)ref;

    const bhy2_data_quaternion *q =
        reinterpret_cast<const bhy2_data_quaternion *>(info->data_ptr);

    // Scale factor from Bosch docs (typically 2^14)
    constexpr float SCALE = 1.0f / 16384.0f;

    float x = q->x * SCALE;
    float y = q->y * SCALE;
    float z = q->z * SCALE;
    float w = q->w * SCALE;

    Serial.printf("GameRV: w=%.3f x=%.3f y=%.3f z=%.3f\n", w, x, y, z);
}

// --- Helper for error checking ---

static void check_rslt(const char *msg, int8_t rslt)
{
    if (rslt != BHY2_OK)
    {
        Serial.printf("%s failed, rslt=%d\n", msg, rslt);
        while (true)
        {
            delay(1000);
        }
    }
}
