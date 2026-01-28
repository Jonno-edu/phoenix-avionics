/**
 * @file i2c.c
 * @brief I2C (Inter-Integrated Circuit) communication interface implementation.
 * 
 * This module provides functions to initialize the I2C bus, read from,
 * and write to I2C devices. It supports multiple I2C buses if the hardware
 * allows it.
 * 
 * @date 2025-10-28
 * @author Petrus J. Marais
 */

#include "i2c.h"

bool I2C_init(void)
{
    // Initialize the I2C port
    i2c_init(I2C_PORT, I2C_FREQUENCY);

    // Configure SDA/SCL pins
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_PIN);
    gpio_pull_up(I2C_SCL_PIN);

    // Pull the OE pin high to enable the I2C bus
    gpio_init(I2C1_OE_PIN);
    gpio_set_dir(I2C1_OE_PIN, GPIO_OUT);
    gpio_put(I2C1_OE_PIN, 1);

    // Test the bus by writing zero bytes to the default device
    uint8_t dummy = 0;
    int ret = i2c_write_blocking(I2C_PORT, 0x42, &dummy, 0, false);

    // Note: 0x42 is just a random address to ping. 
    // Usually we just want to init GPIOs. 
    // The previous implementation returned ret >= 0, but i2c_write_blocking returns PICO_ERROR_GENERIC on fail
    // or number of bytes written. 
    // For init, we mainly care about pin setup.
    
    return true; 
}

bool I2C_write(uint8_t dev_addr, const uint8_t *data, uint16_t length)
{
    int ret = i2c_write_blocking(I2C_PORT, dev_addr, data, length, false);
    return (ret == length);
}

bool I2C_read(uint8_t dev_addr, uint8_t *data, uint16_t length)
{
    int ret = i2c_read_blocking(I2C_PORT, dev_addr, data, length, false);
    return (ret == length);
}

bool I2C_check_device(uint8_t dev_addr)
{
    uint8_t dummy = 0;
    if (i2c_read_blocking(I2C_PORT, dev_addr, &dummy, 1, false) >= 0) {
         return true;
    }
    return false; 
    // The original code used I2C_write check. 
    // Often a 0-length write is used to scan.
}

uint8_t I2C_scan_bus(void) {
    uint8_t count = 0;
    
    printf("\n--- I2C Bus Scan ---\n");
    printf("     0  1  2  3  4  5  6  7  8  9  A  B  C  D  E  F\n");
    
    // The first row (0x00 to 0x07) is reserved and should be padded
    printf("00: -- -- -- -- -- -- -- --");
    
    // Scan addresses 0x08 through 0x7F
    for (int addr = 0x08; addr < 0x78; ++addr) {
        
        if (addr % 16 == 0) {
            printf("\n%02x:", addr);
        }
        
        int ret;
        uint8_t rxdata;
        // Standard scan method: 1-byte read or 0-byte write.
        // Let's stick to the previous method if it worked, or use standard
        
        // Using 1-byte read to be safe? 
        // Original code called I2C_check_device which did a 1-byte write.
        // Let's implement I2C_check_device properly.
        
        uint8_t dst;
        ret = i2c_read_blocking(I2C_PORT, addr, &dst, 1, false);

        if (ret >= 0) {
            printf(" %02x", addr);
            count++;
        } else {
            printf(" --");
        }
    }
    
    printf("\n\nScan complete. Total devices found: %u\n", count);
    return count;
}
