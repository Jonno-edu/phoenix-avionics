#include <stdio.h>
#include <string.h>
#include <math.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "pico/i2c_slave.h"
#include "pico/sync.h"
#include "pico/util/queue.h"
#include "hil_protocol.h"      
#include "sensor_config.h"     

// --- HARDWARE CONFIGURATION ---
#define I2C_PORT      i2c1
#define PIN_SDA       6
#define PIN_SCL       7
#define I2C_OE_PIN    15

// --- PROTOCOL IDs ---
#define SYNC_BYTE     0xAA
#define PKT_TYPE_FAST 0x11
#define PKT_TYPE_SLOW 0x22

// --- PACKET STRUCTURES ---
// 1. FAST PACKET (IMU) - 1kHz
typedef struct __attribute__((packed)) {
    uint8_t  sync;          // 0xAA
    uint8_t  type;          // 0x11
    float    accel[3];      // m/s2
    float    gyro[3];       // rad/s
    float    mag[3];        // uT
} FastPacket_t;

// 2. SLOW PACKET (GPS/Baro/Temp) - 50Hz
typedef struct __attribute__((packed)) {
    uint8_t  sync;          // 0xAA
    uint8_t  type;          // 0x22
    double   lat;           // deg
    double   lon;           // deg
    float    alt;           // m
    float    vel[3];        // m/s
    float    press;         // Pa
    float    temp_baro;     // C
    float    temp_stack;    // C
} SlowPacket_t;

// 3. UNION FOR QUEUE
typedef union {
    uint8_t type;       
    FastPacket_t fast;
    SlowPacket_t slow;
} HilPacket_u;

// --- GLOBALS ---
volatile packet_imu_t  reg_imu;
volatile packet_baro_t reg_baro;
volatile packet_gps_t  reg_gps;
volatile packet_mag_t  reg_mag;
volatile packet_temp_t reg_temp;

critical_section_t crit_sec;
queue_t pkt_queue; // FIFO Buffer (Depth 512)

// Shadows for I2C safety
static packet_imu_t  shadow_imu;
static packet_baro_t shadow_baro;
static packet_gps_t  shadow_gps;
static packet_mag_t  shadow_mag;
static packet_temp_t shadow_temp;

// Stats
volatile uint32_t processed_count = 0;
uint32_t rx_count = 0;

// =============================================================================
// I2C SLAVE HANDLER (IRQ)
// =============================================================================
static void i2c_slave_handler(i2c_inst_t *i2c, i2c_slave_event_t event) {
    static uint8_t active_reg = 0;
    static uint8_t byte_idx = 0;

    switch (event) {
        case I2C_SLAVE_RECEIVE:
            active_reg = i2c_read_byte_raw(i2c);
            byte_idx = 0;
            critical_section_enter_blocking(&crit_sec);
            switch (active_reg) {
                case HIL_REG_IMU:   memcpy(&shadow_imu, (void*)&reg_imu, sizeof(packet_imu_t));   break;
                case HIL_REG_BARO1: memcpy(&shadow_baro, (void*)&reg_baro, sizeof(packet_baro_t)); break;
                case HIL_REG_GPS:   memcpy(&shadow_gps, (void*)&reg_gps, sizeof(packet_gps_t));    break;
                case HIL_REG_MAG:   memcpy(&shadow_mag, (void*)&reg_mag, sizeof(packet_mag_t));    break;
                case HIL_REG_TEMP:  memcpy(&shadow_temp, (void*)&reg_temp, sizeof(packet_temp_t)); break;
            }
            critical_section_exit(&crit_sec);
            break;

        case I2C_SLAVE_REQUEST:
            {
                uint8_t val = 0x00;
                uint8_t *ptr = NULL;
                size_t size = 0;
                switch (active_reg) {
                    case HIL_REG_IMU:   ptr = (uint8_t*)&shadow_imu; size = sizeof(packet_imu_t);   break;
                    case HIL_REG_BARO1: ptr = (uint8_t*)&shadow_baro; size = sizeof(packet_baro_t); break;
                    case HIL_REG_GPS:   ptr = (uint8_t*)&shadow_gps; size = sizeof(packet_gps_t);   break;
                    case HIL_REG_MAG:   ptr = (uint8_t*)&shadow_mag; size = sizeof(packet_mag_t);   break;
                    case HIL_REG_TEMP:  ptr = (uint8_t*)&shadow_temp; size = sizeof(packet_temp_t); break;
                }
                if (ptr && byte_idx < size) val = ptr[byte_idx++];
                i2c_write_byte_raw(i2c, val);
            }
            break;
        case I2C_SLAVE_FINISH: byte_idx = 0; break;
    }
}

// =============================================================================
// SIMULATION UPDATE (1kHz Timer)
// =============================================================================
bool simulation_tick(struct repeating_timer *t) {
    HilPacket_u pkt;
    
    // Process ONE packet per tick (1kHz pacing)
    if (queue_try_remove(&pkt_queue, &pkt)) {
        critical_section_enter_blocking(&crit_sec);
        
        uint32_t now_ms = to_ms_since_boot(get_absolute_time());

        if (pkt.fast.type == PKT_TYPE_FAST) {
            reg_imu.seq++;
            reg_imu.time_ms = now_ms;
            reg_mag.seq++;
            reg_mag.time_ms = now_ms;

            reg_imu.accel[0] = (int16_t)(pkt.fast.accel[0] / SENSOR_IMU_ACCEL_SENSITIVITY);
            reg_imu.accel[1] = (int16_t)(pkt.fast.accel[1] / SENSOR_IMU_ACCEL_SENSITIVITY);
            reg_imu.accel[2] = (int16_t)(pkt.fast.accel[2] / SENSOR_IMU_ACCEL_SENSITIVITY);

            reg_imu.gyro[0] = (int16_t)((pkt.fast.gyro[0]) / SENSOR_IMU_GYRO_SENSITIVITY);
            reg_imu.gyro[1] = (int16_t)((pkt.fast.gyro[1]) / SENSOR_IMU_GYRO_SENSITIVITY);
            reg_imu.gyro[2] = (int16_t)((pkt.fast.gyro[2]) / SENSOR_IMU_GYRO_SENSITIVITY);

            reg_mag.mag[0] = (int16_t)(pkt.fast.mag[0] / SENSOR_MAG_SENSITIVITY);
            reg_mag.mag[1] = (int16_t)(pkt.fast.mag[1] / SENSOR_MAG_SENSITIVITY);
            reg_mag.mag[2] = (int16_t)(pkt.fast.mag[2] / SENSOR_MAG_SENSITIVITY);
        }
        else if (pkt.fast.type == PKT_TYPE_SLOW) {
            reg_gps.seq++;
            reg_gps.time_ms = now_ms;
            reg_baro.seq++;
            reg_baro.time_ms = now_ms;
            reg_temp.seq++;
            reg_temp.time_ms = now_ms;

            reg_gps.lat = (int32_t)(pkt.slow.lat / SENSOR_GPS_LAT_LON_SENSITIVITY);
            reg_gps.lon = (int32_t)(pkt.slow.lon / SENSOR_GPS_LAT_LON_SENSITIVITY);
            reg_gps.alt = (int32_t)(pkt.slow.alt / SENSOR_GPS_ALTITUDE_SENSITIVITY);
            reg_gps.vel_n = (int32_t)(pkt.slow.vel[0] / SENSOR_GPS_VELOCITY_SENSITIVITY);
            reg_gps.vel_e = (int32_t)(pkt.slow.vel[1] / SENSOR_GPS_VELOCITY_SENSITIVITY);
            reg_gps.vel_d = (int32_t)(pkt.slow.vel[2] / SENSOR_GPS_VELOCITY_SENSITIVITY);
            
            reg_baro.pressure_pa = (uint32_t)(pkt.slow.press / SENSOR_BMP581_SENSITIVITY);
            reg_baro.temp_c = (int32_t)(pkt.slow.temp_baro / SENSOR_BMP581_TEMP_SENSITIVITY);
            reg_temp.temp_c = (int16_t)(pkt.slow.temp_stack / SENSOR_TEMP_SENSITIVITY);
        }
        
        critical_section_exit(&crit_sec);
        processed_count++;

        // Add some debug for first non-zero data
        static bool first_accel = true;
        if (first_accel && reg_imu.accel[0] != 0) {
            printf("[HIL] DEBUG: First Accel received: %d\n", reg_imu.accel[0]);
            first_accel = false;
        }
    }
    return true;
}

// =============================================================================
// MAIN
// =============================================================================
int main() {
    stdio_init_all();
    
    // Hardware Init
    gpio_init(I2C_OE_PIN); gpio_set_dir(I2C_OE_PIN, GPIO_OUT); gpio_put(I2C_OE_PIN, 1);
    gpio_init(PIN_SDA); gpio_set_function(PIN_SDA, GPIO_FUNC_I2C); gpio_pull_up(PIN_SDA);
    gpio_init(PIN_SCL); gpio_set_function(PIN_SCL, GPIO_FUNC_I2C); gpio_pull_up(PIN_SCL);
    i2c_init(I2C_PORT, HIL_I2C_BAUDRATE);
    i2c_slave_init(I2C_PORT, HIL_SLAVE_ADDR, i2c_slave_handler);
    critical_section_init(&crit_sec);
    
    // Init Queue (Depth 512)
    queue_init(&pkt_queue, sizeof(HilPacket_u), 512);

    struct repeating_timer timer;
    add_repeating_timer_us(-1000, simulation_tick, NULL, &timer);

    printf("[HIL] Split-Mode Started. Queue Depth: 512.\n");

    // Parser State
    int state = 0; 
    uint8_t pkt_type = 0;
    int pkt_idx = 0;
    int target_len = 0;
    
    union { FastPacket_t fast; SlowPacket_t slow; } incoming;
    uint8_t *in_bytes = (uint8_t*)&incoming;
    uint32_t last_log = 0;
    uint32_t last_debug_log = 0;

    while (1) {
        int ch = getchar_timeout_us(0);
        if (ch == PICO_ERROR_TIMEOUT) continue;
        uint8_t b = (uint8_t)ch;

        if (state == 0) {
            if (b == SYNC_BYTE) state = 1;
        } 
        else if (state == 1) {
            if (b == PKT_TYPE_FAST) {
                pkt_type = PKT_TYPE_FAST;
                target_len = sizeof(FastPacket_t);
                incoming.fast.sync = SYNC_BYTE;
                incoming.fast.type = PKT_TYPE_FAST;
                pkt_idx = 2; 
                state = 2;
            }
            else if (b == PKT_TYPE_SLOW) {
                pkt_type = PKT_TYPE_SLOW;
                target_len = sizeof(SlowPacket_t);
                incoming.slow.sync = SYNC_BYTE;
                incoming.slow.type = PKT_TYPE_SLOW;
                pkt_idx = 2;
                state = 2;
            }
            else { state = 0; } 
        }
        else if (state == 2) {
            in_bytes[pkt_idx++] = b;
            if (pkt_idx >= target_len) {
                HilPacket_u final_pkt;
                if (pkt_type == PKT_TYPE_FAST) memcpy(&final_pkt.fast, &incoming.fast, sizeof(FastPacket_t));
                else memcpy(&final_pkt.slow, &incoming.slow, sizeof(SlowPacket_t));
                
                if (queue_try_add(&pkt_queue, &final_pkt)) {
                    rx_count++;
                }
                state = 0;
            }
        }
        
        // Log Stats for Adaptive Python Pacer
        uint32_t now = to_ms_since_boot(get_absolute_time());
        if (now - last_log > 1000) {
            uint lvl = queue_get_level(&pkt_queue);
            // Print critical format for python parser: "Buf: X/Y"
            printf("[HIL] Buf: %u/512 | Rx: %lu | Proc: %lu\n", lvl, rx_count, processed_count);
            rx_count = 0; processed_count = 0;
            last_log = now;
        }

        // Log Debug Sensor Data every 5 seconds
        if (now - last_debug_log > 5000) {
            packet_imu_t snap_imu;
            packet_gps_t snap_gps;
            
            critical_section_enter_blocking(&crit_sec);
            snap_imu = reg_imu;
            snap_gps = reg_gps;
            critical_section_exit(&crit_sec);

            printf("[HIL] SENSORS (Active): Acc: %d,%d,%d | GPS: %ld,%ld (x1e7) Alt: %ld\n", 
                snap_imu.accel[0], snap_imu.accel[1], snap_imu.accel[2],
                (long)snap_gps.lat, (long)snap_gps.lon, (long)snap_gps.alt);
            
            last_debug_log = now;
        }
    }
}
