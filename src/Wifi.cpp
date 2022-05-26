#include "wifi.h"
#include "main.h"
#include "mqtt.h"
#include "util.h"

// #########################
// # WIFI variables
// #########################
const char *password = WIFI_PASSWORD;
const char *ssid = WIFI_SSID;

int status = WL_IDLE_STATUS;
int reconnectLoops = 0;

#ifdef DEBUG

const char *StatusToString(int status)
{
    switch (status)
    {
    case WL_CONNECTED:
        return "Connected";
    case WL_NO_SHIELD:
        return "No network hardware";
    case WL_IDLE_STATUS:
        return "Waiting for connection";
    case WL_NO_SSID_AVAIL:
        return "No ssid available";
    case WL_SCAN_COMPLETED:
        return "WiFi scan completed";
    case WL_CONNECT_FAILED:
        return "Connection failed";
    case WL_CONNECTION_LOST:
        return "Connection lost";
    case WL_DISCONNECTED:
        return "Disconnected";
    default:
        return "Unknown WiFi status";
    }
}

#endif

boolean ConnectToWiFi()
{
    boolean rval = false;

    // the vlaue will be zero if the data isn't valid
    boolean apvalid = ((rtcData->valid & APVALID) == APVALID);

    if (apvalid == true)
    {
        // The RTC data was good, make a quick connection
        DEBUG_PRINTLN("Connecting to AP using stored AP channel and MAC");
        WiFi.begin(ssid, password, rtcData->channel, rtcData->ap_mac, true);
    }
    else
    {
        // The RTC data was not valid, so make a regular connection
        DEBUG_PRINTLN("Connecting to AP by discovering AP channel and MAC");
        WiFi.begin(ssid, password);
    }
    delay(50);

    int wifiStatus = WiFi.status();
    int retries = 0;

    while (wifiStatus != WL_CONNECTED)
    {
#ifdef DEBUG
        // Serial.printf("ET: %li, Retries: %i, WiFi connect status: %i\n",millis() - starttime, retries, wifiStatus);
#endif
        retries++;
        if (retries == 400) // 400 * 50ms = 2 seconds
        {
            DEBUG_PRINTLN("No connection after 2 seconds, powercycling the WiFi radio.");
            WiFi.disconnect();
            delay(10);
            WiFi.forceSleepBegin();
            delay(100);
            WiFi.forceSleepWake();
            delay(10);
            WiFi.begin(ssid, ssid);
            continue;
        }

        if (retries == 1000) // 1000 * 50ms = 5 seconds
        {
            DEBUG_PRINTLN("No connection after 5 seconds, turing off WiFi radio and sleeping for 1 minute.");
            WiFi.disconnect(true);
            delay(1);
            WiFi.mode(WIFI_OFF);
            delay(1);
            WiFi.forceSleepBegin();
            delay(10);

            // set the AP valid flag to 0 in the RTC data so next go-round, the AP is rediscovered
            rtcData->valid = (rtcData->valid & DSVALID);
            rtcMemory.save();

            Serial.flush();

            ESP.deepSleep(60000000, WAKE_RF_DISABLED); // waling from this sleep restarts entire system - the next line shouldn't be called.
            ESP.reset();
            break;
        }
        delay(50);

        wifiStatus = WiFi.status();
    }

    if (wifiStatus == WL_CONNECTED)
    {
        rval = true;
#ifdef DEBUG
        Serial.printf("Connected to %s.  Assigned IP:", ssid);
        Serial.println(WiFi.localIP());
#endif
    }

    return rval;
}

#ifdef SCAN_SSIDS
void ScanSsidsAndSend()
{
    char jsonBuffer[1200];
    int bufferRemain = 1200; // keep track of space to avoid overrunning memory;

    DEBUG_PRINTLN("Starting WiFi SSID Scan");

    // WiFi.scanNetworks will return the number of networks found
    int n = WiFi.scanNetworks();

    DEBUG_PRINT(n);
    DEBUG_PRINTLN(" networks found");

    if (n == 0)
    {
        strcpy(jsonBuffer, "{\"ap\":[]}");
        bufferRemain -= 9;
    }
    else
    {
        strcpy(jsonBuffer, "{\"ap\":[");
        bufferRemain -= 7;

        char ssidstr[53];

        int last = min(20, n);

        for (int i = 0; i < min(20, n); ++i)
        {
            // Print SSID and RSSI for THE FIRST 20 networks found
            snprintf(ssidstr, sizeof(ssidstr), "{\"n\":\"%s\",\"p\":\"%d\"}", WiFi.SSID(i).c_str(), WiFi.RSSI(i));

            if (i + 1 < last)
            {
                strcat(ssidstr, ",");
            }

            int ssidstrlen = strlen(ssidstr);
            // if there isn't enough space left in the buffer for the clocing brace, then end here
            if (bufferRemain - ssidstrlen > 4)
            {
                strcat(jsonBuffer, ssidstr);
                bufferRemain -= ssidstrlen;
            }
            else
            {
                strcat(jsonBuffer, "\"Truncated results\"");
                bufferRemain -= 19;
                break;
            }
        }
        strcat(jsonBuffer, "]}");
    }

    DEBUG_PRINTLN(jsonBuffer);

    MQTT_ConnectAndSend(SSID_TOPIC, jsonBuffer);
}
#endif