#ifndef __MAIN_H
#define __MAIN_H

#include <Arduino.h>
#include "debug.h"
#include "config.h"

#include "rtcdata.h"

extern RtcMemory rtcMemory;
extern RtcData *rtcData;
extern unsigned long starttime;
extern char clientId[64];

#endif