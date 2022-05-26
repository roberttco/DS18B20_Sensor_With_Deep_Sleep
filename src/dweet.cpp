#include "dweet.h"

boolean DWEET_ConnectAndSend(const char *buf)
{
    boolean rval = false;
    if (WiFi.status() == WL_CONNECTED)
    {
        DEBUG_PRINTLN("Sending to dweet.io");

        WiFiClient wifiClient;
        HTTPClient http; // Declare object of class HTTPClient

        String dweetUrl = String(DWEET_URL);
        DEBUG_PRINT("Dweet URL = ");
        DEBUG_PRINTLN(dweetUrl);

        dweetUrl.concat(DWEET_THING);
        DEBUG_PRINT("Dweet URL = ");
        DEBUG_PRINTLN(dweetUrl.c_str());

        http.begin(wifiClient, DWEET_SERVER, DWEET_PORT, dweetUrl.c_str()); // Specify request destination
        http.addHeader("Content-Type", "application/json");                 // Specify content-type header

#ifdef DEBUG
        Serial.printf("HTTP result code = %u\n", http.POST(buf));
#endif
        // No need to end if the system is being restarted because of deep sleep
        http.end(); // Close connection

        rval = true;
    }
    else
    {
        DEBUG_PRINTLN("Not connected to WiFi - cant send to dweet.io");
    }

    return rval;
}
