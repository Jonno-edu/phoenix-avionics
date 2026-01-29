#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "pico/i2c_slave.h"
#include "hardware/sync.h"
#include "hardware/timer.h"

// --- CONFIGURATION ---
#define I2C_PORT       i2c1
#define I2C_SLAVE_ADDR 0x55
#define PIN_SDA        6
#define PIN_SCL        7
#define I2C_BAUD       400000   // 400kHz (Stable)

// CRITICAL: Pin 15 enables the level shifter on your board
#define I2C_OE_PIN     15       

// BUFFER SETTINGS
#define BUF_SIZE       16384 
#define LOW_WATERMARK  1500  

// --- PACKET PROTOCOL (Must match OBC) ---
#pragma pack(push, 1)

// 1. IMU Packet (Type 0x01)
typedef struct { 
    uint16_t seq; 
    uint32_t time_ms; 
    int16_t  accel_x; 
} packet_imu_t;

// 2. Baro Packet (Type 0x02)
typedef struct { 
    uint16_t seq; 
    uint32_t time_ms; 
    int32_t  pressure; 
} packet_baro_t;

// 3. GPS Packet (Type 0x03)
typedef struct { 
    uint16_t seq; 
    uint32_t time_ms; 
    int32_t  lat; 
} packet_gps_t;

#pragma pack(pop)

// --- GLOBAL STATE ---
volatile packet_imu_t  reg_imu;
volatile packet_baro_t reg_baro;
volatile packet_gps_t  reg_gps;

// FIFO
uint8_t rx_buf[BUF_SIZE];
volatile int head = 0;
volatile int tail = 0;
volatile bool tick_pending = false;

// Stats
volatile uint32_t stats_imu_count = 0;
volatile uint32_t stats_fifo_max = 0;
const uint8_t REQ_CMD[] = {0xDE, 0xAD, 0xBE, 0xEF};

// --- I2C SLAVE HANDLER (Renamed to match main call) ---
static void i2c_slave_handler(i2c_inst_t *i2c, i2c_slave_event_t event) {
    static uint8_t active_reg = 0;
    static uint8_t byte_idx = 0;
    uint8_t *ptr = NULL;
    size_t size = 0;

    switch (event) {
        case I2C_SLAVE_RECEIVE: 
            // Master writes register address
            active_reg = i2c_read_byte_raw(i2c);
            byte_idx = 0;
            break;

        case I2C_SLAVE_REQUEST: 
            // Master wants data
            if (active_reg == 0x01) { ptr = (uint8_t*)&reg_imu; size = sizeof(packet_imu_t); }
            else if (active_reg == 0x02) { ptr = (uint8_t*)&reg_baro; size = sizeof(packet_baro_t); }
            else if (active_reg == 0x03) { ptr = (uint8_t*)&reg_gps; size = sizeof(packet_gps_t); }

            if (ptr && byte_idx < size) {
                i2c_write_byte_raw(i2c, ptr[byte_idx++]);
            } else {
                i2c_write_byte_raw(i2c, 0x00); // Pad with zeros if over-read
            }
            break;

        case I2C_SLAVE_FINISH: 
            byte_idx = 0; 
            break;
    }
}

// --- 1kHz TIMER ---
bool timer_callback(struct repeating_timer *t) {
    tick_pending = true;
    return true;
}

// --- PARSER ---
int parse_next_packet() {
    int len = (head >= tail) ? (head - tail) : (BUF_SIZE - tail + head);
    if (len < 4) return 0;

    // 1. Sync Search
    if (rx_buf[tail] != 0xAA || rx_buf[(tail+1)%BUF_SIZE] != 0xBB) {
        tail = (tail + 1) % BUF_SIZE;
        return 0;
    }

    // 2. Identify Type
    uint8_t type = rx_buf[(tail+2)%BUF_SIZE];
    int pkt_size = 0;

    if (type == 0x01) pkt_size = 3 + sizeof(packet_imu_t) + 1;
    else if (type == 0x02) pkt_size = 3 + sizeof(packet_baro_t) + 1;
    else if (type == 0x03) pkt_size = 3 + sizeof(packet_gps_t) + 1;
    else { 
        tail = (tail + 2) % BUF_SIZE; 
        return 0; 
    }

    if (len < pkt_size) return 0; // Wait for full packet

    // 3. Extract Data
    int payload_start = (tail + 3) % BUF_SIZE;
    
    if (type == 0x01) {
        uint8_t *dst = (uint8_t*)&reg_imu;
        for(int i=0; i<sizeof(packet_imu_t); i++) dst[i] = rx_buf[(payload_start + i) % BUF_SIZE];
        tail = (tail + pkt_size) % BUF_SIZE;
        stats_imu_count++;
        return 1; // Step Complete (IMU Updated)
    } 
    else if (type == 0x02) {
        uint8_t *dst = (uint8_t*)&reg_baro;
        for(int i=0; i<sizeof(packet_baro_t); i++) dst[i] = rx_buf[(payload_start + i) % BUF_SIZE];
        tail = (tail + pkt_size) % BUF_SIZE;
        return 0; // Keep going
    } 
    else if (type == 0x03) {
        uint8_t *dst = (uint8_t*)&reg_gps;
        for(int i=0; i<sizeof(packet_gps_t); i++) dst[i] = rx_buf[(payload_start + i) % BUF_SIZE];
        tail = (tail + pkt_size) % BUF_SIZE;
        return 0; // Keep going
    }
    return 0;
}

int main() {
    stdio_init_all();
    
    // 1. I2C Initialization
    i2c_init(I2C_PORT, I2C_BAUD);
    gpio_set_function(PIN_SDA, GPIO_FUNC_I2C);
    gpio_set_function(PIN_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(PIN_SDA);
    gpio_pull_up(PIN_SCL);

    // 2. Enable Bus Buffer (CRITICAL for Connection)
    gpio_init(I2C_OE_PIN);
    gpio_set_dir(I2C_OE_PIN, GPIO_OUT);
    gpio_put(I2C_OE_PIN, 1); 

    // 3. Start Slave Logic (Corrected Function Name)
    i2c_slave_init(I2C_PORT, I2C_SLAVE_ADDR, i2c_slave_handler);

    // 4. Start 1kHz Physics Timer
    struct repeating_timer timer;
    add_repeating_timer_us(-1000, timer_callback, NULL, &timer);

    sleep_ms(2000); 

    // Timing Variables
    uint32_t last_req_time = 0;
    uint32_t last_log_time = 0;

    while (1) {
        // A. USB INGESTION
        int c = getchar_timeout_us(0);
        while (c >= 0) {
            rx_buf[head] = (uint8_t)c;
            head = (head + 1) % BUF_SIZE;
            if (head == tail) tail = (tail + 1) % BUF_SIZE;
            c = getchar_timeout_us(0);
        }

        // B. PHYSICS STEP (Drain FIFO)
        if (tick_pending) {
            int attempts = 0;
            while(attempts < 5) {
                if (parse_next_packet() == 1) {
                    tick_pending = false; 
                    break; 
                }
                // Stop if buffer is empty
                int len = (head >= tail) ? (head - tail) : (BUF_SIZE - tail + head);
                if (len < 4) break;
                attempts++;
            }
        }

        // C. FLOW CONTROL (Fill FIFO)
        int bytes_in_fifo = (head >= tail) ? (head - tail) : (BUF_SIZE - tail + head);
        uint32_t now = to_ms_since_boot(get_absolute_time());

        if (bytes_in_fifo < LOW_WATERMARK && (now - last_req_time > 20)) {
            fwrite(REQ_CMD, 1, 4, stdout);
            fflush(stdout);
            last_req_time = now;
        }
        
        // D. LOGGING
        if (now - last_log_time > 1000) {
            printf("[HIL] Hz: %4d | FIFO: %4d | IMU_Seq: %d\n", 
                   stats_imu_count, bytes_in_fifo, reg_imu.seq);
            stats_imu_count = 0;
            last_log_time = now;
        }
    }
}