// usb_console.c
#include "core/usb_console.h"
#include <stdio.h>
#include "FreeRTOS.h"
#include "task.h"

#if !PICO_BUILD
    #include <unistd.h>
    #include <stdlib.h>
    #include <fcntl.h>
    #include <string.h>
    #include <errno.h>
#else
    #include <pico/stdlib.h>
    #include <hardware/uart.h>
#endif

// Circular buffer
static volatile uint8_t rx_buffer[CONSOLE_BUFFER_SIZE];
static volatile uint16_t write_index = 0;
static volatile uint16_t read_index = 0;

#if !PICO_BUILD
// SIL: PTY (pseudo-terminal) for virtual serial port
static int pty_master_fd = -1;
static char pty_slave_name[256];
#endif

#if PICO_BUILD
// USB RX callback - drain ALL available data into buffer
static void on_usb_rx(void *param) {
    (void)param;
    
    int ch;
    while ((ch = getchar_timeout_us(0)) != PICO_ERROR_TIMEOUT) {
        // Direct all USB input to the console buffer (for Packet Parser)
        console_buffer_push((uint8_t)ch);
    }
}
#endif

void console_init(void) {
#if PICO_BUILD
    stdio_set_chars_available_callback(on_usb_rx, NULL);
#else
    // SIL: Open a pseudo-terminal pair
    pty_master_fd = posix_openpt(O_RDWR | O_NOCTTY);
    
    if (pty_master_fd < 0) {
        printf("ERROR: Failed to open PTY: %s\n", strerror(errno));
        return;
    }
    
    if (grantpt(pty_master_fd) < 0) {
        printf("ERROR: grantpt failed: %s\n", strerror(errno));
        close(pty_master_fd);
        return;
    }
    
    if (unlockpt(pty_master_fd) < 0) {
        printf("ERROR: unlockpt failed: %s\n", strerror(errno));
        close(pty_master_fd);
        return;
    }
    
    if (ptsname_r(pty_master_fd, pty_slave_name, sizeof(pty_slave_name)) != 0) {
        printf("ERROR: ptsname_r failed: %s\n", strerror(errno));
        close(pty_master_fd);
        return;
    }
    
    // Set non-blocking mode
    int flags = fcntl(pty_master_fd, F_GETFL, 0);
    fcntl(pty_master_fd, F_SETFL, flags | O_NONBLOCK);
    
    printf("\n========================================\n");
    printf("SIL Virtual Serial Port Created\n");
    printf("========================================\n");
    printf("Port: %s\n", pty_slave_name);
    printf("\nConnect your Python scripts to this port.\n");
    printf("Example: python3 test.py --port %s\n", pty_slave_name);
    printf("Or use: screen %s 115200\n", pty_slave_name);
    printf("========================================\n\n");
#endif
}

void console_send(const uint8_t *data, uint16_t len) {
#if PICO_BUILD
    // Always send to USB CDC for testing/monitoring
    // uart_write_blocking is not for USB CDC. 
    // USB CDC usually uses stdio or tud_cdc_write.
    // Assuming putchar_raw loop for now if direct write isn't available via easier API.
    for(uint16_t i=0; i<len; i++) {
        putchar_raw(data[i]);
    }
#else
    // SIL: Write to PTY (virtual serial port)
    if (pty_master_fd >= 0) {
        ssize_t written = write(pty_master_fd, data, len);
        if (written < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
            // Only log errors that aren't just "buffer full"
            printf("[SIL] PTY write error: %s\n", strerror(errno));
        }
    }
    
    // Also echo to stdout for debugging (hex dump for larger blocks might be noisy, but byte by byte is too)
    // printf("[SIL TX] "); 
    // for(int i=0; i<len; i++) printf("%02X ", data[i]);
    // printf("\n");
#endif
}
// NOTE: This function is designed to be called from ISR context ONLY.
// Do NOT call from task context without adding interrupt disable guards.
void console_buffer_push(uint8_t byte) {
    rx_buffer[write_index] = byte;
    write_index = (write_index + 1) % CONSOLE_BUFFER_SIZE;
    
    // If buffer full, overwrite oldest data (move read_index)
    if (write_index == read_index) {
        read_index = (read_index + 1) % CONSOLE_BUFFER_SIZE;
    }
}
    
bool console_bytes_available(void) {
    taskENTER_CRITICAL();
    bool available = (read_index != write_index);
    taskEXIT_CRITICAL();
    return available;
}

uint8_t console_read_byte(void) {
    taskENTER_CRITICAL();
    if (read_index == write_index) {
        taskEXIT_CRITICAL();
        return 0;
    }
    
    uint8_t byte = rx_buffer[read_index];
    read_index = (read_index + 1) % CONSOLE_BUFFER_SIZE;
    taskEXIT_CRITICAL();
    return byte;
}

#if !PICO_BUILD
// SIL: High-priority task that simulates UART interrupt by polling PTY
void vConsoleRxTask(void *pvParameters) {
    (void)pvParameters;
    
    uint8_t buffer[64];
    
    printf("[SIL] Console RX task started - polling virtual serial port\n");
    
    while (1) {
        if (pty_master_fd < 0) {
            // PTY not initialized, wait and retry
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }
        
        // Non-blocking read from PTY
        ssize_t n = read(pty_master_fd, buffer, sizeof(buffer));
        
        if (n > 0) {
            // Push all received bytes into circular buffer
            // This simulates the UART interrupt pushing data
            for (ssize_t i = 0; i < n; i++) {
                if (debug_cli_is_enabled()) {
                    debug_cli_process_char(buffer[i]);
                } else {
                    console_buffer_push(buffer[i]);
                }
            }
            
            // Optional: debug output
            // printf("[SIL] Received %zd bytes\n", n);
        } else if (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
            // Real error (not just "no data available")
            printf("[SIL] PTY read error: %s\n", strerror(errno));
        }
        
        // Poll every 5ms (~200Hz)
        // This is the key difference from hardware interrupts
        // Good enough for RS485 at 115200 baud
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}
#endif