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

    return (ret >= 0);
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
    if (!I2C_write(dev_addr, &dummy, 1)) {
        return false;
    }
    return true;
}

uint8_t I2C_scan_bus(void) {
    uint8_t count = 0;
    
    printf("\n--- I2C Bus Scan ---\n");
    printf("     0  1  2  3  4  5  6  7  8  9  A  B  C  D  E  F\n");
    
    // The first row (0x00 to 0x07) is reserved and should be padded
    printf("00: -- -- -- -- -- -- -- --");
    
    // Scan addresses 0x08 through 0x7F (valid 7-bit addresses are 0x08-0x77)
    // The loop goes up to 0x7F to complete the last row printing, even though 
    // checks usually stop at 0x77 to avoid conflicting with reserved addresses (0x78-0x7F)
    for (int addr = 0x08; addr <= 0x7F; ++addr) {
        
        // Start a new line every 16 addresses (i.e., when addr % 16 == 0)
        // Since we start at 0x08, the first line is padded.
        if (addr % 16 == 0) {
            printf("\n%02x:", addr);
        }
        
        // Check if a device is present at this address
        if (I2C_check_device(addr)) {
            printf(" %02x", addr);
            count++;
        } else {
            printf(" --");
        }
    }
    
    printf("\n\nScan complete. Total devices found: %u\n", count);
    return count;
}
