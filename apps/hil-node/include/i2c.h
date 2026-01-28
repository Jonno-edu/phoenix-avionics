/**
 * @file i2c.h
 * @brief I2C (Inter-Integrated Circuit) communication interface.
 * 
 * This module provides functions to initialize the I2C bus, read from,
 * and write to I2C devices. It supports multiple I2C buses if the hardware
 * allows it.
 * 
 * @date 2025-10-28
 * @author Petrus J. Marais
 */

#ifndef I2C_H
#define I2C_H

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"

#define RP2350 /**< Define if compiling for RP2350 */
// #define pico2W /**< Define if compiling for Pico W board */

/** I2C bus configuration for RP2350 */
#ifdef RP2350
    /**< Default I2C port for Raspberry Pi Pico */
    #define I2C_PORT i2c1 // (i2c0 for pico2W | i2c1 for RP2350 OBC)

    /**< I2C1 output enable pin */
    // #define I2C1_OE_PIN 15 // OBCv1.0 uses GPIO15
    #define I2C1_OE_PIN 15 // OBCv2.0 uses GPIO46 // OBCv1.0 uses GPIO 15

    /** Default SDA and SCL pins */
    #define I2C_SDA_PIN 6 // (0 for pico2W | 6 for RP2350 OBC)
    #define I2C_SCL_PIN 7 // (1 for pico2W | 7 for RP2350 OBC)

    /** Default I2C frequency */
    #define I2C_FREQUENCY 400000 // 400 kHz
#endif // RP2350

/** I2C bus configuration for RP2350 */
#ifdef pico2W
    /**< Default I2C port for Raspberry Pi Pico */
    #define I2C_PORT i2c0

    /** Default SDA and SCL pins */
    #define I2C_SDA_PIN 0
    #define I2C_SCL_PIN 1

    /** Default I2C frequency */
    #define I2C_FREQUENCY 400000 // 400 kHz
#endif // RP2350

/**
 * @brief Initialize the I2C bus.
 * 
 * This function configures the I2C interface for communication.
 * It should be called once during system initialization.
 * 
 * @return `true` if initialization was successful, `false` otherwise.
 */
bool I2C_init(void);

/**
 * @brief Write data to an I2C device.
 *
 * @param dev_addr 7-bit I2C device address.
 * @param data Pointer to data buffer to be sent.
 * @param length Number of bytes to write.
 * @return true if transmission was successful, false otherwise.
 */
bool I2C_write(uint8_t dev_addr, const uint8_t *data, uint16_t length);

/**
 * @brief Read data from an I2C device.
 *
 * @param dev_addr 7-bit I2C device address.
 * @param data Pointer to buffer where received data will be stored.
 * @param length Number of bytes to read.
 * @return true if reception was successful, false otherwise.
 */
bool I2C_read(uint8_t dev_addr, uint8_t *data, uint16_t length);

/**
 * @brief Check if an I2C device is connected.
 *
 * @param dev_addr 7-bit I2C device address.
 * @return `true` if device acknowledges, otherwise `false`.
 */
bool I2C_check_device(uint8_t dev_addr);

/**
 * @brief Scans the entire I2C bus (addresses 0x08 to 0x77) and prints found devices.
 *
 *  @return The total number of I2C devices found.
 */
uint8_t I2C_scan_bus(void);

#endif /* I2C_H */

