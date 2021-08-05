#ifndef __RTCDATA_H
#define __RTCDATA_H

#include <Arduino.h>
#include <rtc_memory.h>
#include "debug.h"
#include "config.h"

// The ESP8266 RTC memory is arranged into blocks of 4 bytes. The access methods read and write 4 bytes at a time,
// so the RTC data structure should be padded to a 4-byte multiple.

typedef struct _RTCDATA
{
    uint8_t valid;           // 1 bytes
    uint8_t channel;         // 1 byte,   2 in total
    uint8_t ap_mac[6];       // 6 bytes,  8 in total
    uint8_t ds1820addr[8];   // 8 bytes, 16 in total
    uint8_t loopsBeforeScan; // 1 byte,  17 total
    uint8_t fuelGaugeLoops;  // 1 byte,  18 total
    uint8_t padding[2];      // 1 byte,  20 total
} RtcData;

#define DSVALID 0x01
#define APVALID 0x02

#endif
