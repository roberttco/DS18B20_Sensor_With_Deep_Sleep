#ifndef __UTIL_H
#define __UTIL_H

#include <Arduino.h>

String oneWireAddressToString(uint8_t *mac, boolean hex);
String macAddressToString(uint8_t *mac);

#endif