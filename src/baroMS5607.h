/**
 * @file baroMS5607.h
 * @brief Interface for MS5607 barometric pressure sensor.
 *
 * This module provides functions to read atmospheric pressure, temperature
 * (if supported), and compute altitude using the barometric formula.
 */

#ifndef BAROMS5607_H
#define BAROMS5607_H

#include <stdbool.h>
#include <stdint.h>
#include "i2c.h"

#define BAROMS5607_I2C_ADDRESS          0x77  /**< I2C address for MS5607 barometric sensor - with CS low*/

/** @name MS5607 I2C read lengths */
#define BAROMS5607_I2C_PROM_READ_LENGTH 2     /**< Number of bytes to read for PROM coefficients. */
#define BAROMS5607_I2C_ADC_READ_LENGTH  3     /**< Number of bytes to read for ADC result. */

/**
 * @brief MS5607 command definitions.
 */
enum BAROMS5607_Commands {
    BAROMS5607_CMD_RESET       = 0x1E, /**< Reset command */
    BAROMS5607_CMD_CONVERT_D1  = 0x48, /**< Pressure conversion command (OSR=4096, typical=D1)*/
    BAROMS5607_CMD_CONVERT_D2  = 0x58, /**< Temperature conversion command (OSR=4096, typical=D2)*/
    BAROMS5607_CMD_ADC_READ    = 0x00,  /**< ADC read command */
    BAROMS5607_CMD_PROM_READ_C1 = 0xA2, /**< PROM read command for coefficient C1 */
    BAROMS5607_CMD_PROM_READ_C2 = 0xA4, /**< PROM read command for coefficient C2 */
    BAROMS5607_CMD_PROM_READ_C3 = 0xA6, /**< PROM read command for coefficient C3 */
    BAROMS5607_CMD_PROM_READ_C4 = 0xA8, /**< PROM read command for coefficient C4 */    
    BAROMS5607_CMD_PROM_READ_C5 = 0xAA, /**< PROM read command for coefficient C5 */
    BAROMS5607_CMD_PROM_READ_C6 = 0xAC  /**< PROM read command for coefficient C6 */
};

/**
 * @brief Oversampling rate options for MS5607.
 */
typedef enum {
    BAROMS5607_OSR_256  = 0x00, /**< Oversampling rate 256 */
    BAROMS5607_OSR_512  = 0x02, /**< Oversampling rate 512 */
    BAROMS5607_OSR_1024 = 0x04, /**< Oversampling rate 1024 */
    BAROMS5607_OSR_2048 = 0x06, /**< Oversampling rate 2048 */
    BAROMS5607_OSR_4096 = 0x08  /**< Oversampling rate 4096 */
} BAROMS5607_OSR;

/**
 * @brief Structure to store barometric sensor readings.
 */
typedef struct {
    uint32_t D1;            /**< Raw pressure ADC value */
    uint32_t D2;            /**< Raw temperature ADC value */
    int64_t dT;             /**< Difference between actual and reference temperature */
    int32_t TEMP;           /**< Computed temperature in hundredths of degrees Celsius */
    int64_t OFF;            /**< Offset at actual temperature */
    int64_t SENS;           /**< Sensitivity at actual temperature */
    int32_t P;              /**< Temperature compensated pressure in hundredths of mBar (Pascals)*/
} BAROMS5607_Data_raw;

/**
 * @brief Structure to store calibration coefficients.
 */
typedef struct {
    uint16_t C1; /**< Pressure sensitivity - SENS*/
    uint16_t C2; /**< Pressure offset - OFF*/
    uint16_t C3; /**< Temperature coefficient of pressure sensitivity - TCS*/
    uint16_t C4; /**< Temperature coefficient of pressure offset - TCO*/
    uint16_t C5; /**< Reference temperature - TREF*/
    uint16_t C6; /**< Temperature coefficient of the temperature - TEMPSENS*/
} BAROMS5607_CalibrationData;

/**
 * @brief Write data to the barometer sensor.
 *
 * @param[in] data Pointer to data buffer to write.
 * @param[in] length Number of bytes to write.
 * @return `true` if write was successful.
 */
bool BAROMS5607_write(const uint8_t *data, uint16_t length);

/**
 * @brief Read data from the barometer sensor.
 * 
 * @param[out] data Pointer to buffer to store read data.
 * @param[in] length Number of bytes to read.
 * @return `true` if read was successful.
 */
bool BAROMS5607_read(uint8_t *data, uint16_t length);

/**
 * @brief Check connectivity to the barometer sensor.
 *
 * @return `true` if sensor is detected.
 */
bool BAROMS5607_isConnected(void);

/**
 * @brief Reset the barometer sensor.
 *
 * @return `true` if successful.
 */
bool BAROMS5607_reset(void);

/**
 * @brief Read calibration data from the barometer sensor.
 *
 * @param[out] calibData Pointer to a BAROMS5607_CalibrationData structure to store the coefficients.
 * @return `true` if successful.
 */
bool BAROMS5607_readCalibrationData(BAROMS5607_CalibrationData *calibData);

/**
 * @brief Initialize the barometer sensor.
 *
 * @return `true` if successful.
 */
bool BAROMS5607_init(void);

bool BAROMS5607_readADC(uint32_t *adc_out);

/**
 * 
 */
bool BAROMS5607_ConversionD1(void);

bool BAROMS5607_ConversionD2(void);

bool BAROMS5607_poll(void);

bool BAROMS5607_getPressure_raw(int32_t *pressure);

bool BAROMS5607_getPressure(double *pressure);

bool BAROMS5607_getTemp_raw(int32_t *temp);

bool BAROMS5607_getTemp (double *temp);

bool baro_init(void);

#endif /* BAROMS5607_H */
