#pragma once
#include <stdint.h>
#include "tctlm.h"   // for CommsInterfaceId_t

// RS485 node addresses
#define ADDR_OBC             0x01
#define ADDR_TRACKING_RADIO  0x03
// Future: #define ADDR_24GHZ_RADIO  0x04
// Future: #define ADDR_EPS          0x02

// CommsInterfaceId_t mappings (matches CommsInterfaceId_t typedef in tctlm.h)
#define COMMS_IFACE_RS485    ((CommsInterfaceId_t)0x01)
#define COMMS_IFACE_USB      ((CommsInterfaceId_t)0x02)
