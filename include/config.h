#ifndef __CONFIG_H
#define __CONFIG_H

#ifndef FW_VERSION
#define FW_VERSION 40
#endif

// sensor 1
// Device is an ESP-07
// Choose Generic ESP8266 with 1MB RAM (FS:none, OTA:502KB)

// NOW defined in platformio.ini
//#define DALLAS_GPIO 13
//#define MQTT_TOPIC "7133egyptian/out/sensors/sensor1/json"
//#define SSID_TOPIC "7133egyptian/out/sensors/sensor1/ssidscan"
//#define DWEET_THING "ebd1425c-9026-452b-8178-7201bd977b55"

// NOW defined in platformio.ini
//#define DALLAS_GPIO 13
//#define MQTT_TOPIC "7133egyptian/out/sensors/sensor2/json"
//#define SSID_TOPIC "7133egyptian/out/sensors/sensor2/ssidscan"
//#define DWEET_THING "c440d18e-4fa7-4eb7-bafb-499b19de97ab"

#define CLIENT_ID_TEMPLATE "Sensor%s"

#ifndef DEEP_SLEEP_TIME_SECONDS
#define DEEP_SLEEP_TIME_SECONDS 300
#endif

//#define USE_DWEET          // comment out to NOT dweet results
//#define USE_MQTT           // comment out to not use MQTT
#define MQTT_BROKER "192.168.2.6"
#define MQTT_PORT 24552

#ifndef SSID_RESCANLOOPS
#define SSID_RESCANLOOPS 12 // how many measurements loops between SSID scans
#endif

// battery measurement defines

#ifndef VCC_MEASURE_MODE
#define VCC_MEASURE_MODE 0 // 0=use TOUT, 1=measure the internal VCC
#endif

#ifndef ADC_TO_VOLTS_M
#if VCC_MEASURE_MODE == 0
#error You must define ADC_TO_VOLTS_M if VCC_MEASURE_MODE=0
#endif
#endif

#ifndef ADC_TO_VOLTS_B
#if VCC_MEASURE_MODE == 0
#error You must define ADC_TO_VOLTS_B if VCC_MEASURE_MODE=0
#endif
#endif

// #define CALIBRATE_VCC
// defined in platformio.ini now - #define VCC_CORRECTION 0.338

#ifndef VCC_CORRECTION
#define VCC_CORRECTION 0
#endif

#ifndef VCC_CUTOFF
#define VCC_CUTOFF 0
#endif

#endif