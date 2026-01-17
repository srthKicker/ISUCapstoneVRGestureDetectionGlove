Initial test of FreeRTOS on the ESP32 board.

To compile: idf.py build
To flash to board: idf.py flash
To rebuild (say you change some CMakeLists.txt or something)
idf.py reconfigure

Sorry, i don't know if there is a GUI option here

To add:
Below is the function in bhi360.h. I have to make a read function, a write function, and a few more. 
int8_t bhi360_init(
    enum bhi360_intf intf,           // BHI360_SPI_INTF or BHI360_I2C_INTF
    bhi360_read_fptr_t read,         // Your read function
    bhi360_write_fptr_t write,       // Your write function
    bhi360_delay_us_fptr_t delay_us, // Microsecond delay function
    uint32_t read_write_len,         // Max transfer size
    void *intf_ptr,                  // Optional context pointer
    struct bhi360_dev *dev           // Device handle (output)
);

Seemingly important functions to look at: 
bhi360_upload_firmware_to_ram()
bhi360_boot_from_ram()
bhi360_get_and_process_fifo()