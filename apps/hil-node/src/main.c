#include <stdio.h>
#include <math.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "pico/i2c_slave.h"
#include "hil_protocol.h"

// --- CONFIGURATION ---
#define I2C_PORT       i2c1
#define PIN_SDA        6
#define PIN_SCL        7

// CHECK YOUR BOARD VERSION FOR OE PIN!
// Phoenix OBC v1.0 usually uses GPIO 15
// Phoenix OBC v2.0 usually uses GPIO 46
#define I2C_OE_PIN     15


// --- GLOBAL STATE ---
// This holds the "Live" simulation data
static struct {
    fast_packet_t   fast;
    medium_packet_t medium;
    slow_packet_t   slow;
} sensor_state;

// I2C State Machine Variables
static uint8_t active_reg = 0x00;
static uint8_t byte_index = 0;

// --- INTERRUPT HANDLER ---
static void i2c_slave_handler(i2c_inst_t *i2c, i2c_slave_event_t event) {
    switch (event) {
        case I2C_SLAVE_RECEIVE: 
            // Master wrote a byte -> It is the Register Address
            active_reg = i2c_read_byte_raw(i2c);
            byte_index = 0;
            break;

        case I2C_SLAVE_REQUEST: { // <--- ADDED BRACE HERE
            // Master wants data -> Serve from the Active Register
            uint8_t val = 0;
            uint8_t *ptr = NULL;
            size_t size = 0;

            if (active_reg == HIL_REG_FAST) {
                ptr = (uint8_t*)&sensor_state.fast;
                size = sizeof(fast_packet_t);
            } else if (active_reg == HIL_REG_MEDIUM) {
                ptr = (uint8_t*)&sensor_state.medium;
                size = sizeof(medium_packet_t);
            } else if (active_reg == HIL_REG_SLOW) {
                ptr = (uint8_t*)&sensor_state.slow;
                size = sizeof(slow_packet_t);
            }

            if (ptr && byte_index < size) {
                val = ptr[byte_index];
                byte_index++; 
            } else {
                val = 0; // Padding if master reads too much
            }
            
            i2c_write_byte_raw(i2c, val);
            break;
        } // <--- ADDED CLOSING BRACE HERE

        case I2C_SLAVE_FINISH:
            byte_index = 0;
            break;
    }
}

int main() {
    stdio_init_all();
    
    // 1. Hardware Init
    i2c_init(I2C_PORT, HIL_I2C_BAUDRATE);
    gpio_set_function(PIN_SDA, GPIO_FUNC_I2C);
    gpio_set_function(PIN_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(PIN_SDA);
    gpio_pull_up(PIN_SCL);

    // 2. Enable Bus Buffers
    gpio_init(I2C_OE_PIN);
    gpio_set_dir(I2C_OE_PIN, GPIO_OUT);
    gpio_put(I2C_OE_PIN, 1);

    // 3. Start Slave Mode
    i2c_slave_init(I2C_PORT, HIL_I2C_SLAVE_ADDR, i2c_slave_handler);
    
    printf("\n--- HIL NODE (SLAVE) RUNNING @ 1MHz ---\n");

    // 4. Simulation Loop (Generates dummy data)
    uint32_t t = 0;
    while(1) {
        // Update FAST (1kHz)
        sensor_state.fast.sequence_id++;
        sensor_state.fast.accel[0] = (int16_t)(1000 * sin(t * 0.1)); // Sine wave X
        sensor_state.fast.accel[1] = (int16_t)(1000 * cos(t * 0.1)); // Cosine wave Y

        // Update MEDIUM (100Hz)
        if (t % 10 == 0) {
            sensor_state.medium.baro_pres = 101325 + (t % 50);
            sensor_state.medium.temp_stack = 35 * 128; // 35 degrees fixed
        }

        // Update SLOW (10Hz)
        if (t % 100 == 0) {
            sensor_state.slow.lat = -33924870 + (t % 1000);
            sensor_state.slow.lon = 18424055;
            sensor_state.slow.alt = 150000; // 150m
        }

        t++;
        sleep_us(1000); // 1ms Loop time
    }
}