/**
 * @file baroMS5607.c
 * @brief MS5607 barometric pressure sensor interface.
 * 
 * Provides functions to initialize the sensor, read pressure and temperature,
 * and compute altitude using the barometric formula.
 * 
 * @date 2025-10-29
 * @author Petrus J. Marais
 */

#include "baroMS5607.h"
#include <stdio.h>
#include <stdint.h>
#include <inttypes.h>
#include "pico/stdlib.h"
#include "math.h"

BAROMS5607_CalibrationData calibData;
BAROMS5607_Data_raw rawData;

bool BAROMS5607_write(const uint8_t *data, uint16_t length)
{
    return I2C_write(BAROMS5607_I2C_ADDRESS, data, length);
}

bool BAROMS5607_read(uint8_t *data, uint16_t length)
{
    return I2C_read(BAROMS5607_I2C_ADDRESS, data, length);
}

bool BAROMS5607_isConnected(void)
{
    uint8_t cmd = BAROMS5607_CMD_PROM_READ_C1; // safe "small" command for presence check
    if (!BAROMS5607_write(&cmd, 1)) {
        return false;
    }
    return true;
}

bool BAROMS5607_reset(void)
{
    uint8_t reg = BAROMS5607_CMD_RESET;
    if (!BAROMS5607_write(&reg, 1)) { // Send reset command
        return false;
    }
    return true;
}

bool BAROMS5607_readCalibrationData(BAROMS5607_CalibrationData *calibData)
{
    if (!calibData) return false;
    uint8_t buf[2];

    uint8_t cmd = BAROMS5607_CMD_PROM_READ_C1;
    if (!BAROMS5607_write(&cmd, 1)) return false;
    if (!BAROMS5607_read(buf, sizeof(buf))) return false;
    calibData->C1 = (buf[0] << 8) | buf[1];

    cmd = BAROMS5607_CMD_PROM_READ_C2;
    if (!BAROMS5607_write(&cmd, 1)) return false;
    if (!BAROMS5607_read(buf, sizeof(buf))) return false;
    calibData->C2 = (buf[0] << 8) | buf[1];

    cmd = BAROMS5607_CMD_PROM_READ_C3;
    if (!BAROMS5607_write(&cmd, 1)) return false;
    if (!BAROMS5607_read(buf, sizeof(buf))) return false;
    calibData->C3 = (buf[0] << 8) | buf[1];

    cmd = BAROMS5607_CMD_PROM_READ_C4;
    if (!BAROMS5607_write(&cmd, 1)) return false;
    if (!BAROMS5607_read(buf, sizeof(buf))) return false;
    calibData->C4 = (buf[0] << 8) | buf[1];

    cmd = BAROMS5607_CMD_PROM_READ_C5;
    if (!BAROMS5607_write(&cmd, 1)) return false;
    if (!BAROMS5607_read(buf, sizeof(buf))) return false;
    calibData->C5 = (buf[0] << 8) | buf[1];

    cmd = BAROMS5607_CMD_PROM_READ_C6;
    if (!BAROMS5607_write(&cmd, 1)) return false;
    if (!BAROMS5607_read(buf, sizeof(buf))) return false;
    calibData->C6 = (buf[0] << 8) | buf[1];

    return true;
}

bool BAROMS5607_init()
{
    // Check if barometer is connected
    if (!BAROMS5607_isConnected()) {
        printf("Barometer not connected!\n");
        return false;
    }

    // Reset the barometer to load calibration PROM into the internal registers
    if (!BAROMS5607_reset()) {
        printf("Barometer reset failed!\n");
        return false;
    }

    sleep_ms(10); // Wait for reset to complete

    if (!BAROMS5607_readCalibrationData(&calibData)) {
        printf("Failed to read barometer calibration data!\n");
        return false;
    }

    // Print calibration data for debugging
    // printf("MS5607 Calibration Data:\n");
    // printf("C1: %u\n", calibData.C1);
    // printf("C2: %u\n", calibData.C2);
    // printf("C3: %u\n", calibData.C3);
    // printf("C4: %u\n", calibData.C4);   
    // printf("C5: %u\n", calibData.C5);
    // printf("C6: %u\n", calibData.C6);

    return true;
}

bool BAROMS5607_readADC(uint32_t *adc_out)
{
    // first write ADC read command (0x00) then read 3 bytes
    uint8_t cmd = BAROMS5607_CMD_ADC_READ; // usually 0x00
    if (!BAROMS5607_write(&cmd, 1)) {
        return false;
    }

    uint8_t buf[3];
    if (!BAROMS5607_read(buf, 3)) {
        return false;
    }

    // MSB first
    *adc_out = ((uint32_t)buf[0] << 16) | ((uint32_t)buf[1] << 8) | (uint32_t)buf[2];
    return true;
}

bool BAROMS5607_ConversionD1(void)
{
    uint8_t cmd = BAROMS5607_CMD_CONVERT_D1;
    if (!BAROMS5607_write(&cmd, 1)){
        printf("Failed to write D1 conversion command.\n");
        return false;
    }

    sleep_ms(10);

    if (!BAROMS5607_readADC(&rawData.D1)){
        printf("Failed to read D1 conversion.\n");
        return false;
    }

    printf("D1: %" PRIu32 "\n", rawData.D1);

    return true;
}

bool BAROMS5607_ConversionD2(void)
{
    uint8_t cmd = BAROMS5607_CMD_CONVERT_D2;
    if (!BAROMS5607_write(&cmd, 1)){
        printf("Failed to write D2 conversion command.\n");
        return false;
    }

    sleep_ms(10);

    if (!BAROMS5607_readADC(&rawData.D2)){
        printf("Failed to read D2 conversion.\n");
        return false;
    }

    printf("D2: %" PRIu32 "\n", rawData.D2);

    return true;
}

bool BAROMS5607_poll(void)
{
    if (!BAROMS5607_ConversionD1()){
        printf("D1 Conversion failed!\n");
        return false;
    }

    if (!BAROMS5607_ConversionD2()){
        printf("D2 Conversion failed!\n");
        return false;
    }

    // use 64-bit intermediates as recommended by datasheet
    int32_t dT = (int32_t)rawData.D2 - ((int32_t)calibData.C5 << 8); // C5 * 2^8
    rawData.dT = dT;

    printf("C5: %u\n", calibData.C5);
    printf("D2: %" PRIu32 "\n", rawData.D2);
    printf("dT: %" PRId64 "\n", rawData.dT);

    // TEMP in 0.01 deg C
    int64_t TEMP = 2000 + (((int64_t)dT * calibData.C6) >> 23);
    rawData.TEMP = (int32_t)TEMP;

    // OFF and SENS
    int64_t OFF = ((int64_t)calibData.C2 << 17) + (((int64_t)calibData.C4 * dT) >> 6); // C2*2^17 + (C4*dT)/2^6
    int64_t SENS = ((int64_t)calibData.C1 << 16) + (((int64_t)calibData.C3 * dT) >> 7); // C1*2^16 + (C3*dT)/2^7

    // Second order compensation (recommended by datasheet for low temps)
    int64_t T2 = 0, OFF2 = 0, SENS2 = 0;
    if (TEMP < 2000) { // < 20.00 °C
        int64_t temp_diff = TEMP - 2000;
        T2 = (temp_diff * temp_diff) >> 1; // = ((TEMP-2000)^2) / 2
        OFF2 = 5 * (temp_diff * temp_diff) >> 1; // 5 * ((TEMP-2000)^2) / 2
        SENS2 = 5 * (temp_diff * temp_diff) >> 2; // 5 * ((TEMP-2000)^2) / 4

        if (TEMP < -1500) { // < -15.00 °C extra compensation
            int64_t t3 = (TEMP + 1500) * (TEMP + 1500);
            OFF2 += 7 * t3;
            SENS2 += (11 * t3) >> 1;
        }
    }

    TEMP -= T2;
    OFF -= OFF2;
    SENS -= SENS2;

    // Pressure
    int64_t pressure_intermediate = (((int64_t)rawData.D1 * SENS) >> 21) - OFF; // D1 * SENS / 2^21 - OFF
    int32_t P = (int32_t)(pressure_intermediate >> 15); // /2^15
    rawData.P = P;

    // rawData.TEMP already in 0.01 deg C; rawData.P in Pa
    return true;
}


bool BAROMS5607_getPressure_raw(int32_t *pressure)
{
    *pressure = rawData.P;
    return true;
}

bool BAROMS5607_getPressure(double *pressure)
{
    *pressure = (double)rawData.P * 0.001; // in kPa

    return true;
}

bool BAROMS5607_getTemp_raw(int32_t *temp)
{
    *temp = rawData.TEMP;

    return true;
}

bool BAROMS5607_getTemp(double *temp)
{
    *temp = (double)rawData.TEMP * 0.01;

    return true;
}

bool baro_init(void) {
     // Initialize the I2C bus 
    if (!I2C_init()) { 
        printf("I2C initialization failed!\n"); 
        return false;
    }
    
    printf("I2C initialized.\n");

    sleep_ms(1000);

    I2C_scan_bus();

    // MS5607 Barometer test
    if (!BAROMS5607_init()) {
        printf("Barometer initialization failed!\n");
        return false;
    }
    
    return true;
}
