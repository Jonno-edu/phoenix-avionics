#pragma once

// Telecommand IDs  (MSG_TYPE_TELECOMMAND namespace)
#define ID_CMD_RESET              0b00001
#define ID_CMD_SLEEP              0b00010
#define ID_CMD_TRACKING_BEACON    0b00011
#define ID_CMD_WAKE               0b00100
#define ID_CMD_SET_TELEM_RATE     0b00101
#define ID_CMD_SET_LOG_LEVEL      0b00110
#define ID_CMD_SET_SIM_STATE      0b01000
#define ID_CMD_GET_PENDING_MSG    0b10000

// Telemetry IDs  (MSG_TYPE_TLM_REQ / MSG_TYPE_TLM_RESP namespace)
#define ID_TLM_IDENTIFICATION     0b00001
#define ID_TLM_SENSOR_DATA        0b00010
#define ID_TLM_TRACKING_BEACON    0b00011
