// phoenix_icd.h
// Single-file Phoenix ICD (combined protocol definitions)
#ifndef PHOENIX_ICD_H
#define PHOENIX_ICD_H

#include <stdint.h>

// Version
#define PHOENIX_PROTOCOL_VERSION_MAJOR  1
#define PHOENIX_PROTOCOL_VERSION_MINOR  0

// PACKED_STRUCT
#if defined(__GNUC__)
    #define PACKED_STRUCT __attribute__((packed))
#else
    #define PACKED_STRUCT
#endif

#define DBG_BUFFER_SIZE  128 // TODO move?

// ============================================================================
// DEVICE ADDRESSES
// ============================================================================
#define ADDR_OBC             0x01
#define ADDR_EPS             0x02
#define ADDR_TRACKING_RADIO  0x03
#define ADDR_GSE             0xF0

// COMMON IDs (supported by ALL nodes)
#define TC_COMMON_RESET         0x00
#define TLM_COMMON_IDENT        0x00
#define EVENT_COMMON_LOG        0x01
#define EVENT_OBC_NORB_STREAM   0x02  // Payload: [uint8_t topic_id][uint32_t timestamp_ms][raw struct bytes]

#define LOG_LEVEL_ERROR         0
#define LOG_LEVEL_WARN          1
#define LOG_LEVEL_INFO          2
#define LOG_LEVEL_DEBUG         3

// EPS
#define TC_EPS_POWER            0x01
#define TLM_EPS_POWER           0x01
#define TLM_EPS_MEASURE         0x02

// RADIO
#define TC_RADIO_BEACON         0x02
#define TLM_RADIO_DATA          0x03
#define TLM_RADIO_STATUS        0x04

// OBC
#define TC_OBC_TELEM_RATE       0x05
#define TC_OBC_LOG_LEVEL        0x06
#define TC_OBC_SIM_MODE         0x08
#define TC_OBC_AVIONICS_MODE    0x09
#define TC_OBC_ENABLE_TUNNEL    0x0A
#define TC_OBC_FLIGHT_STATE     0x0B
#define TC_OBC_NORB_SUBSCRIBE   0x0C  // Payload: NorbSubscribePayload_t

#define TLM_OBC_SENSOR_DATA        0x02
#define TLM_OBC_HIGH_SPEED_STREAM  0x0B
#define TLM_OBC_NORB_STREAM        0x0C  // Payload: [uint8_t topic_id][raw struct bytes]

// TELEMETRY PAYLOAD STRUCTURES
typedef struct {
    uint8_t  topic_id;   // topic_id_t value (0..TOPIC_COUNT-1)
    uint16_t rate_ms;    // streaming interval in ms; 0 = unsubscribe
} PACKED_STRUCT NorbSubscribePayload_t;

typedef struct {
    uint8_t  node_type;
    uint8_t  interface_version;
    uint8_t  firmware_major;
    uint8_t  firmware_minor;
    uint16_t uptime_seconds;
    uint16_t uptime_milliseconds;
} PACKED_STRUCT TlmIdentificationPayload_t;

typedef struct {
    uint32_t time;
    int16_t  accel_x;
    int16_t  accel_y;
    int16_t  accel_z;
} PACKED_STRUCT TlmSensorData_t;

// EPS Payloads
typedef enum {
    POWER_OFF = 0,
    POWER_ON  = 1
} PowerSelect_t;

typedef enum {
    EPS_LINE_3V3_1 = 0,
    EPS_LINE_3V3_2,
    EPS_LINE_3V3_3,
    EPS_LINE_5V_1,
    EPS_LINE_5V_2,
    EPS_LINE_5V_3,
    EPS_LINE_12V,
    EPS_LINE_COUNT
} EpsLineIndex_t;

typedef struct {
    uint8_t rail_3v3_1 : 1;
    uint8_t rail_3v3_2 : 1;
    uint8_t rail_3v3_3 : 1;
    uint8_t rail_5v_1  : 1;
    uint8_t rail_5v_2  : 1;
    uint8_t rail_5v_3  : 1;
    uint8_t rail_12v   : 1;
    uint8_t reserved   : 1;
    uint8_t fault_flags;
} PACKED_STRUCT EpsPowerStatus_t;

typedef EpsPowerStatus_t EpsPowerSetCmd_t;

typedef struct {
    uint16_t batt_voltage_mv;
    uint16_t batt_current_ma;
    uint16_t current_3v3_1_ma;
    uint16_t current_5v_1_ma;
    uint16_t current_12v_ma;
} PACKED_STRUCT EpsMeasurements_t;

// Radio payload
typedef struct {
    uint16_t runtime_seconds;
    uint16_t runtime_milliseconds;
    uint8_t  avionics_state;
    uint8_t  apogee_counter;
    uint16_t battery_current;
    int32_t  est_lat;
    int32_t  est_lon;
    int32_t  est_alt;
    int16_t  est_vel_n;
    int16_t  est_vel_e;
    int16_t  est_vel_d;
    uint32_t baro_pressure;
    int32_t  baro_temp;
    int16_t  temp_stack;
    int16_t  accel_x;
    int16_t  accel_y;
    int16_t  accel_z;
    int16_t  gyro_x;
    int16_t  gyro_y;
    int16_t  gyro_z;
    int16_t  mag_x;
    int16_t  mag_y;
    int16_t  mag_z;
    int32_t  gps_lat;
    int32_t  gps_lon;
    int32_t  gps_alt;
    int32_t  gps_vel_n;
    int32_t  gps_vel_e;
    int32_t  gps_vel_d;
    uint32_t gps_itow;
    uint16_t gps_week;
    uint8_t  time_valid_flags;
    uint8_t  apogee_votes;

} PACKED_STRUCT TlmTrackingBeaconPayload_t;

// EVENT PAYLOAD STRUCTURES
typedef struct {
    bool error : 1;
    bool warn  : 1;
    bool info  : 1;
    bool debug : 1;
} PACKED_STRUCT EventLogConfig_t;

#endif // PHOENIX_ICD_H
