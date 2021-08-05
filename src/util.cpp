#include <Arduino.h>

String oneWireAddressToString(uint8_t *mac, boolean hex=false)
{
    char result[33];

    if (hex == false)
    {
        snprintf(result, sizeof(result), "%d %d %d %d %d %d %d %d",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5], mac[6], mac[7]);
    }
    else
    {
        snprintf(result, sizeof(result), "%02X%02X%02X%02X%02X%02X%02X%02X",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5], mac[6], mac[7]);
    }

    return String(result);
}

String macAddressToString(uint8_t *mac)
{
    char result[14];

    snprintf(result, sizeof(result), "%02X%02X%02X%02X%02X%02X",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

    return String(result);
}