#include "main.h"
#include "config.h"

#include <ESP8266WiFi.h>
#include <DS18B20.h>

#include "util.h"
#include "wifi.h"
#include "dweet.h"
#include "ota.h"
#include "mqtt.h"

// #########################
// # DS18B20 variables
// #########################
// Use the DS18B20 miltiple example to scan the bus and get the address
DS18B20 ds(DALLAS_GPIO);
uint8_t ds18b20address[8]; // = {40,216,131,138,6,0,0,237};
uint8_t selected;

RtcMemory rtcMemory;

// #########################
// # Misc variables
// #########################
char buf[1024];
char clientId[64];
float tempc = 0;
double ucvdd = 0;
double vdd = 0;

RtcData *rtcData = NULL;
String myMac;

unsigned long starttime;
unsigned long sleepTimeSeconds = DEEP_SLEEP_TIME_SECONDS;

#if VCC_MEASUREMODE == 1
// make it so the A/D converter reads VDD
ADC_MODE(ADC_VCC);
#else
ADC_MODE(ADC_TOUT)
#endif
    
void setup()
{
    starttime = millis();

    Serial.begin(115200);
    while (!Serial);

    DEBUG_PRINTLN("\n------- STARTING --------\n");

    // start with WiFi turned off
    // https://www.bakke.online/index.php/2017/05/21/reducing-wifi-power-consumption-on-esp8266-part-2/
    WiFi.mode(WIFI_OFF);
    WiFi.forceSleepBegin();
    delay(1);

    if (rtcMemory.begin())
    {
        DEBUG_PRINTLN("RTC memory library initialization done!");
    }
    else
    {
        DEBUG_PRINTLN("No previous RTC memory data found. The memory is reset to zeros!");
    }

    // Get the data
    rtcData = rtcMemory.getData<RtcData>();

    // sanity check
    if (rtcData->fuelGaugeLoops > ADC_FILTER_SIZE)
    {
        rtcData->fuelGaugeLoops = 0;
    }

    if (rtcData->loopsBeforeScan > SSID_RESCANLOOPS)
    {
        rtcData->loopsBeforeScan = 0;
    }

#if DEBUG == 1
    Serial.printf("RTC Data:\n    Valid: 0x%X\n    Channel: %d\n    ap_mac: %s\n    ds1820addr: %s\n    Loops before scan: %i\n    Fuel gauge loops: %i\n",
                  rtcData->valid,
                  rtcData->channel,
                  macAddressToString(rtcData->ap_mac).c_str(),
                  oneWireAddressToString(rtcData->ds1820addr,false).c_str(),
                  rtcData->loopsBeforeScan,
                  rtcData->fuelGaugeLoops);
#endif

    // if gpio 13 is low, then wipe the RTC data and force a rediscovery of the AP and the DS18B20 address
    int clear_a = digitalRead(13);
    int clear_b = digitalRead(14);
    if ((clear_a == 0) && (clear_b == 0))
    {
        DEBUG_PRINTLN("Clearing RTC data structure.");
        memset(rtcData, 0, sizeof(RtcData));

        // after clearing the RTC memory, wait 10 secoinds and reboot so there is time to
        // reconnect the sensor wire.
        delay(10000);
        ESP.restart();
    }

#if DEBUG == 1
    Serial.printf("ET: %li, Setup() complete\n",millis() - starttime);
#endif
}

void loop()
{
    double adcavg = -1;

    if (rtcData->fuelGaugeLoops >= FUEL_GAUGE_LOOPS)
    {
#if DEBUG == 1
    Serial.printf ("Calculating ADC average over %d samples.\n",ADC_FILTER_SIZE);
#endif

#if VCC_MEASUREMODE == 1
        // get the battery voltage
        ucvdd = ESP.getVcc() / 1000.0;
        vdd = ucvdd + VCC_CORRECTION;
#else
        // TODO: get average vdd value by averaging ADC values then converting to voltage
#if ADC_FILTER_SIZE == 0
        adcavg = (double)analogRead(A0);
        
#else
        long adcsum = 0L;
        for (int filter = 0; filter < ADC_FILTER_SIZE; filter++)
        {
            adcsum += analogRead(A0);
            if (0 != ADC_FILTER_PERIOD)
            {
                delay(ADC_FILTER_PERIOD);
            }
        }
        adcavg = (double)adcsum / (double)(ADC_FILTER_SIZE);
#endif
        Serial.println(adcavg); // used for serial plotting in Arduino IDE
    
        ucvdd = adcavg / 1000.0;
        vdd = ucvdd + VCC_CORRECTION;
#endif
        rtcData->fuelGaugeLoops = 0;
    }
    else
    {
        rtcData->fuelGaugeLoops += 1;
    }

#ifdef CALIBRATE_VCC
    DEBUG_PRINTLN("Take VCC reading now - you have 5 seconds.");
    //delay here to make it easier to read VCC with a volt meter to calibrate the VCC correction value
    delay(5000);
#endif

    if (rtcData != NULL && ((rtcData->valid & DSVALID) == DSVALID))
    {
        DEBUG_PRINTLN("Using stored DS18B0 address.");
        selected = ds.select(rtcData->ds1820addr);
    }
    else
    {
        DEBUG_PRINTLN("Getting DS18B0 address (and storing it for next time).");
        while (selected == 0 && ds.selectNext())
        {
#if DEBUG == 1
            switch (ds.getFamilyCode())
            {
            case MODEL_DS18S20:
                Serial.println("Model: DS18S20/DS1820");
                break;
            case MODEL_DS1822:
                Serial.println("Model: DS1822");
                break;
            case MODEL_DS18B20:
                Serial.println("Model: DS18B20");
                break;
            default:
                Serial.println("Unrecognized Device");
                break;
            }
#endif

            uint8_t ds18b20address[8];
            ds.getAddress(ds18b20address);
            selected = 1;

#if DEBUG == 1
    Serial.printf("ET: %li, DS18B20 address retrieved - ",millis() - starttime);
    DEBUG_PRINTLN(oneWireAddressToString(ds18b20address,false));
#endif

            rtcData->valid |= DSVALID;
            memcpy(rtcData->ds1820addr, ds18b20address, 8); // Copy the DS18B20 address to memory
            break;
        }
    }

    // assign the client ID using the DS18B20 address - which will be unique to the sensor
    sprintf(clientId,CLIENT_ID_TEMPLATE,oneWireAddressToString(rtcData->ds1820addr,true).c_str());

    if (selected != 0)
    {
        tempc = ds.getTempC();
    }
    else
    {
        tempc = 255;
    }

#if DEBUG == 1
    Serial.printf("ET: %li, Tempc: %f, ADC: %.4f, UcVdd: %f, Vdd: %f\n",millis() - starttime,tempc,adcavg,ucvdd,vdd);
#endif

    // Now send out the measurements

#if USE_DWEET || USE_MQTT
    int connectRetries = 1; // retry once with a wifi wakeup in the middle

    while (connectRetries > 0)
    {
        if (ConnectToWiFi())
        {
            // successfully connected
            uint8_t mac[6];
            WiFi.macAddress(mac);
            myMac = macAddressToString(mac);

            // Write current connection info back to RTC
            rtcData->channel = WiFi.channel();
            rtcData->valid |= APVALID;
            memcpy(rtcData->ap_mac, WiFi.BSSID(), 6); // Copy 6 bytes of BSSID (AP's MAC address)

#if DEBUG == 1
    Serial.printf("ET: %li, Connected to WiFi\n",millis() - starttime);
#endif
            break;
        }
        else
        {
            // wait 10 second between retries.
#if DEBUG == 1
            Serial.printf("ET: %li, WiFi connect failed.  Waiting 10 seconds.\n",millis() - starttime);
#endif
            delay(10000);
        }

        connectRetries--;
    }

    if (connectRetries == 0)
    {
#if DEBUG == 1
        Serial.printf("ET: %li, Connect retries exhausted.  Clearing RTC data and sleeping for 5 minutes.\n",millis() - starttime);
#endif
       
        // clear RTC data in case there is something amiss with the info - get a fresh start.
        memset(rtcData, 0, sizeof(RtcData));
        sleepTimeSeconds = 300;
    }
    else
    {
        connectRetries = 5;
        long rssi = WiFi.RSSI();
        while ((rssi > 0) && (connectRetries > 0))
        {
            connectRetries--;
            delay(50);
        }

        if (connectRetries <= 0)
        {
            rssi = 0;
        }

#if DEBUG == 1
        Serial.printf("ET: %li, Got RSSI (%ld)\n",millis() - starttime,rssi);
#endif

        double tempf = tempc * 1.8 + 32;

        snprintf(buf, sizeof(buf), "{ \"wifi\": { \"ssid\":\"%s\",\"connecttime\":\"%ld\", \"rssi\":\"%ld\",\"mac\":\"%s\" },"
                                   "\"power\" : { \"adc\" : \"%.4f\", \"ucvdd\": \"%.4f\", \"vdd\": \"%.4f\", \"sleeptimeseconds\" : \"%ld\", \"ssidscanloop\" : \"%d\", \"fuelgaugeloop\" : \"%d\"},"
                                   "\"environment\" : { \"tempf\":\"%f\", \"tempc\":\"%f\", \"humidity\":\"%f\", \"pressure_hpa\":\"%f\", \"dewpoint\" : \"%f\" },"
                                   "\"firmware\" : { \"version\":\"%d\",\"date\":\"%s %s\",\"sleepmode\":\"%d\" }}",

                 WIFI_SSID,
                 (millis() - starttime),
                 rssi,
                 myMac.c_str(),

                 adcavg,
                 ucvdd,
                 vdd,
                 sleepTimeSeconds,
                 rtcData->loopsBeforeScan,
                 rtcData->fuelGaugeLoops,

                 tempf,
                 tempc,
                 0.0,
                 0.0,
                 0.0,

                 FW_VERSION,
                 __DATE__,
                 __TIME__,
#if DEEP_SLEEP == 1
                 1
#else
                 0
#endif
                );

#if DEBUG == 1
        Serial.printf("ET: %li, JSON buffer:\n%s\n",millis() - starttime,buf);
#endif
        
        if (WiFi.status() == WL_CONNECTED)
        {
#ifdef USE_DWEET
            DWEET_ConnectAndSend(buf);
#if DEBUG == 1
            Serial.printf("ET: %li, JSON sent to dweet.io\n",millis() - starttime);
#endif
#endif

#ifdef USE_MQTT
            MQTT_ConnectAndSend(MQTT_TOPIC, buf);

#if DEBUG == 1
            Serial.printf("ET: %li, JSON sent to MQTT broker\n",millis() - starttime);
#endif

#ifdef SCAN_SSIDS
            if (rtcData->loopsBeforeScan >= SSID_RESCANLOOPS)
            {
                ScanSsidsAndSend();
                
#if DEBUG == 1
                Serial.printf("ET: %li, SSIDs scanned\n",millis() - starttime);
#endif
                rtcData->loopsBeforeScan = 0;
            }
            else
            {
                rtcData->loopsBeforeScan += 1;
            }
#endif
#endif

            checkForUpdates(myMac);

#if DEBUG == 1
            Serial.printf("ET: %li, Updates checked\n",millis() - starttime);
#endif

        }
    }

    rtcMemory.save();

    DEBUG_PRINTLN("\n=====\nTime to sleep...");
    WiFi.disconnect(true);
    delay(1);
#endif

    if (vdd < VCC_CUTOFF)
    {
        // if Vdd < 3 volts then go to sleep for 1 hour hoping for some sun
        DEBUG_PRINTLN("Sleeping for 20 minutes because Vdd is too low - hoping for some sun.");
        sleepTimeSeconds = 1200;
    }
    else
    {
        sleepTimeSeconds = DEEP_SLEEP_TIME_SECONDS;
    }

    // WAKE_RF_DISABLED to keep the WiFi radio disabled when we wake up
#if DEBUG == 1
    Serial.printf("ET: %li, Sleeping for %li seconds.\n", millis() - starttime,sleepTimeSeconds);
#endif

#if DEEP_SLEEP == 1
    ESP.deepSleep(sleepTimeSeconds * 1e6, WAKE_RF_DISABLED);
#else
    delay(DEEP_SLEEP_TIME_SECONDS * 1000);
    starttime = millis();
#endif
}
