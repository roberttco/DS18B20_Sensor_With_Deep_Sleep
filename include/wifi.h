#ifndef __WIFI_H
#define __WIFI_H

#include <Arduino.h>
#include <ESP8266WiFi.h>
#include "debug.h"
#include "config.h"

#include "rtcdata.h"

#ifndef WIFI_PASSWORD
#define WIFI_PASSWORD "UbKNUJakLBLpOh"
#endif

#ifndef WIFI_SSID
#define WIFI_SSID "qsvMIMt8Fm6NV3"
#endif

extern unsigned long wificonnecttime;

boolean ConnectToWiFi();

#ifdef SCAN_SSIDS
void ScanSsidsAndSend();
#endif

#endif