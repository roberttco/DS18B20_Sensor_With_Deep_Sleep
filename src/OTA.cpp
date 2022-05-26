
#include "ota.h"
#include "main.h"

#ifdef DEBUG

void update_started()
{
    Serial.println("CALLBACK:  HTTP update process started");
}

void update_finished()
{
    Serial.println("CALLBACK:  HTTP update process finished");
}

void update_progress(int cur, int total)
{
    Serial.printf("CALLBACK:  HTTP update process at %d of %d bytes...\n", cur, total);
}

void update_error(int err)
{
    Serial.printf("CALLBACK:  HTTP update fatal error code %d\n", err);
}

#endif

void checkForUpdates(String mac)
{
    String fwURL = String(FW_URL_ROOT);
    fwURL.concat(mac); // fwURL = /<root>/<mac>

    String fwVersionURL = fwURL;
    fwVersionURL.concat("/version.txt"); // fwVersionURL = /<root>/<mac>/version.txt

#ifdef DEBUG
    Serial.printf("Getting firmware version from server at http://%s:%d%s\n", FW_SERVER_IP, FW_SERVER_PORT, fwVersionURL.c_str());
#endif

    WiFiClient wifiClient;
    HTTPClient httpClient;

    httpClient.begin(wifiClient, FW_SERVER_IP, FW_SERVER_PORT, fwVersionURL);

    int httpCode = httpClient.GET();
    if (httpCode == 200)
    {
        String newFWVersion = httpClient.getString();
        int newVersion = newFWVersion.toInt();

#ifdef DEBUG
        Serial.printf("Current FW version = %d, Available FW version = %d\n", FW_VERSION, newVersion);
#endif
        if (newVersion != FW_VERSION)
        {
            String fwImageURL = fwURL;
            fwImageURL.concat("/firmware.bin"); // fwImageURL = /<root>/<mac>/firmware.bin

#ifdef DEBUG
            Serial.printf("Getting firmware from server at http://%s:%d%s\n", FW_SERVER_IP, FW_SERVER_PORT, fwImageURL.c_str());

            ESPhttpUpdate.setLedPin(LED_BUILTIN, LOW);
            ESPhttpUpdate.onStart(update_started);
            ESPhttpUpdate.onEnd(update_finished);
            ESPhttpUpdate.onProgress(update_progress);
            ESPhttpUpdate.onError(update_error);
#endif

            WiFiClient wifiClient;
            t_httpUpdate_return ret = ESPhttpUpdate.update(wifiClient, FW_SERVER_IP, FW_SERVER_PORT, fwImageURL);

            if (ret == HTTP_UPDATE_OK)
            {
                DEBUG_PRINTLN("Clearing RTC data structure.");
                memset(rtcData, 0, sizeof(RtcData));
                rtcData->loopsBeforeScan = 0;
                rtcMemory.save();

                // after clearing the RTC memory, wait 10 seconds and reboot so there is time to
                // reconnect the sensor wire.
                delay(10000);
            }

#ifdef DEBUG
            switch (ret)
            {
            case HTTP_UPDATE_FAILED:
                Serial.printf("HTTP_UPDATE_FAILD Error (%d): %s\n", ESPhttpUpdate.getLastError(), ESPhttpUpdate.getLastErrorString().c_str());
                break;

            case HTTP_UPDATE_NO_UPDATES:
                Serial.println("HTTP_UPDATE_NO_UPDATES");
                break;

            case HTTP_UPDATE_OK:
                Serial.println("HTTP_UPDATE_OK");
                break;
            }
#endif
        }
        else
        {
            DEBUG_PRINTLN("Already on latest version");
        }
    }
    else
    {
        DEBUG_PRINT("Firmware version check failed, got HTTP response code ");
        DEBUG_PRINTLN(httpCode);
    }
    httpClient.end();
}
