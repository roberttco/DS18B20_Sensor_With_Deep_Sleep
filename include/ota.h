#ifndef __OTA_H
#define __OTA_H

#include <Arduino.h>
#include <ESP8266httpUpdate.h>

#include "debug.h"
#include "config.h"

#include "rtcdata.h"

#ifndef FW_VERSION
#define FW_VERSION 0
#endif

#ifndef FW_SERVER_IP
#define FW_SERVER_IP "192.168.2.5"
#endif

#ifndef FW_SERVER_PORT
#define FW_SERVER_PORT 80
#endif

#ifndef FW_URL_ROOT
#define FW_URL_ROOT "/firmware/" // requires trailing /
#endif

void checkForUpdates(String mac);

#endif
