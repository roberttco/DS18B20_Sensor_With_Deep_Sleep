#ifndef __MQTT_H
#define __MQTT_H

#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>

#include "debug.h"
#include "config.h"
#include "wifi.h"

#ifndef MQTT_BROKER
#define MQTT_BROKER "192.168.2.6"
#endif

#ifndef MQTT_PORT
#define MQTT_PORT 24552
#endif

#ifndef MQTT_USERNAME
#define MQTT_USERNAME ""
#endif

#ifndef MQTT_PASSWORD
#define MQTT_PASSWORD ""
#endif

#ifndef MQTT_TOPIC
#define MQTT_TOPIC "7133egyptian/out/sensors/sensor/json"
#endif

//extern WiFiClient wifiClient;
//extern PubSubClient mqttClient;

void MQTT_ConnectAndSend(char *topic, char *buf);

#endif // __MQTT_H
