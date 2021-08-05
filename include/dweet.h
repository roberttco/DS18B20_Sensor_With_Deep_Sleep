#ifndef __DWEET_H
#define __DWEET_H

#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>
#include "debug.h"
#include "config.h"
#include "wifi.h"

#ifndef DWEET_SERVER
#define DWEET_SERVER "dweet.io"
#endif

#ifndef DWEET_PORT
#define DWEET_PORT 80
#endif

#ifndef DWEET_URL
#define DWEET_URL "/dweet/quietly/for/" // need trailing /
#endif

#ifndef DWEET_THING
#define DWEET_THING "c440d18e-4fa7-4eb7-bafb-499b19de97ab"
#endif

//boolean SendDweet(char *buf);
boolean DWEET_ConnectAndSend(const char *buf);

#endif // __DWEET_H
